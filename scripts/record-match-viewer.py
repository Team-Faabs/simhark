#!/usr/bin/env python3
"""Record a simhark match viewer run to an MP4 file."""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import json
import os
import re
import shutil
import signal
import socket
import ssl
import struct
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path


VIEWER_URL = "http://127.0.0.1:8315/debug-big?team=yellow"
DEFAULT_WIDTH = 1920
DEFAULT_HEIGHT = 1080
DEFAULT_FPS = 30


class DevToolsSocket:
    def __init__(self, ws_url: str) -> None:
        parsed = urllib.parse.urlparse(ws_url)
        if parsed.scheme not in {"ws", "wss"}:
            raise ValueError(f"unsupported DevTools WebSocket URL: {ws_url}")
        self._ssl = parsed.scheme == "wss"
        self._host = parsed.hostname or "127.0.0.1"
        self._port = parsed.port or (443 if self._ssl else 80)
        self._path = parsed.path or "/"
        if parsed.query:
            self._path += f"?{parsed.query}"
        self._next_id = 1
        self._sock = self._connect()

    def _connect(self) -> socket.socket:
        raw = socket.create_connection((self._host, self._port), timeout=10)
        sock = ssl.create_default_context().wrap_socket(raw, server_hostname=self._host) if self._ssl else raw
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (
            f"GET {self._path} HTTP/1.1\r\n"
            f"Host: {self._host}:{self._port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n"
            "\r\n"
        )
        sock.sendall(request.encode("ascii"))
        response = b""
        while b"\r\n\r\n" not in response:
            chunk = sock.recv(4096)
            if not chunk:
                raise RuntimeError("DevTools WebSocket closed during handshake")
            response += chunk
        if b" 101 " not in response.split(b"\r\n", 1)[0]:
            raise RuntimeError(f"DevTools WebSocket handshake failed: {response[:120]!r}")
        sock.settimeout(1)
        return sock

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass

    def send(self, method: str, params: dict | None = None) -> int:
        message_id = self._next_id
        self._next_id += 1
        payload = {"id": message_id, "method": method}
        if params is not None:
            payload["params"] = params
        self._send_frame(json.dumps(payload, separators=(",", ":")).encode("utf-8"))
        return message_id

    def recv(self) -> dict | None:
        opcode, payload = self._recv_frame()
        if opcode == 8:
            return None
        if opcode != 1:
            return {}
        return json.loads(payload.decode("utf-8"))

    def _send_frame(self, payload: bytes) -> None:
        mask_key = os.urandom(4)
        length = len(payload)
        header = bytearray([0x81])
        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header.extend([0x80 | 126, *struct.pack("!H", length)])
        else:
            header.extend([0x80 | 127, *struct.pack("!Q", length)])
        masked = bytes(byte ^ mask_key[i % 4] for i, byte in enumerate(payload))
        self._sock.sendall(bytes(header) + mask_key + masked)

    def _recv_exact(self, size: int) -> bytes:
        data = b""
        while len(data) < size:
            chunk = self._sock.recv(size - len(data))
            if not chunk:
                raise RuntimeError("DevTools WebSocket closed")
            data += chunk
        return data

    def _recv_frame(self) -> tuple[int, bytes]:
        header = self._recv_exact(2)
        opcode = header[0] & 0x0F
        masked = bool(header[1] & 0x80)
        length = header[1] & 0x7F
        if length == 126:
            length = struct.unpack("!H", self._recv_exact(2))[0]
        elif length == 127:
            length = struct.unpack("!Q", self._recv_exact(8))[0]
        mask_key = self._recv_exact(4) if masked else b""
        payload = self._recv_exact(length)
        if masked:
            payload = bytes(byte ^ mask_key[i % 4] for i, byte in enumerate(payload))
        return opcode, payload


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run match-sim, open the viewer in headless Chromium, and record it.")
    parser.add_argument("--yellow", default="bangka", help="yellow team kind passed to match-sim")
    parser.add_argument("--blue", default="ungabunga", help="blue team kind passed to match-sim")
    parser.add_argument("--name", help="recording filename stem; timestamp is still prepended")
    parser.add_argument("--rec-dir", default="recordings", help="directory for the MP4 output")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH, help=argparse.SUPPRESS)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT, help=argparse.SUPPRESS)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help=argparse.SUPPRESS)
    return parser.parse_args()


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def require_tool(names: list[str], label: str) -> str:
    for name in names:
        path = shutil.which(name)
        if path:
            return path
    joined = ", ".join(names)
    raise SystemExit(f"missing {label}; tried: {joined}")


def slug(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "-", value.strip()).strip("-")
    return cleaned or "match"


def output_path(args: argparse.Namespace, root: Path) -> Path:
    rec_dir = Path(args.rec_dir)
    if not rec_dir.is_absolute():
        rec_dir = root / rec_dir
    rec_dir.mkdir(parents=True, exist_ok=True)
    timestamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
    raw_name = args.name or f"{args.yellow}-vs-{args.blue}"
    if raw_name.lower().endswith(".mp4"):
        raw_name = raw_name[:-4]
    name = slug(raw_name)
    return rec_dir / f"{timestamp}-{name}.mp4"


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def wait_http(url: str, timeout: float, process: subprocess.Popen | None = None) -> None:
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError(f"process exited with status {process.returncode} before {url} was available")
        try:
            with urllib.request.urlopen(url, timeout=1) as response:
                if response.status < 500:
                    return
        except Exception as err:  # noqa: BLE001
            last_error = err
        time.sleep(0.25)
    raise RuntimeError(f"timed out waiting for {url}: {last_error}")


def devtools_json(port: int, path: str, data: bytes | None = None) -> object:
    url = f"http://127.0.0.1:{port}{path}"
    request = urllib.request.Request(url, data=data, method="PUT" if data is not None else "GET")
    with urllib.request.urlopen(request, timeout=5) as response:
        return json.loads(response.read().decode("utf-8"))


def open_devtools_page(port: int, url: str) -> str:
    encoded = urllib.parse.quote(url, safe="")
    try:
        page = devtools_json(port, f"/json/new?{encoded}", data=b"")
    except urllib.error.HTTPError:
        pages = devtools_json(port, "/json/list")
        if not isinstance(pages, list) or not pages:
            raise RuntimeError("Chromium DevTools exposed no pages")
        page = pages[0]
    if not isinstance(page, dict) or "webSocketDebuggerUrl" not in page:
        raise RuntimeError(f"unexpected DevTools page response: {page!r}")
    return str(page["webSocketDebuggerUrl"])


def start_chromium(chromium: str, port: int, profile: Path, width: int, height: int) -> subprocess.Popen:
    return subprocess.Popen(
        [
            chromium,
            "--headless=new",
            f"--remote-debugging-port={port}",
            "--remote-debugging-address=127.0.0.1",
            f"--user-data-dir={profile}",
            "--no-first-run",
            "--no-default-browser-check",
            "--disable-background-timer-throttling",
            "--disable-renderer-backgrounding",
            "--disable-dev-shm-usage",
            f"--window-size={width},{height}",
            "about:blank",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def start_match(root: Path, yellow: str, blue: str) -> subprocess.Popen:
    command = [
        "cargo",
        "r",
        "-r",
        "-p",
        "match-runner",
        "--bin",
        "match-sim",
        "--config",
        ".cargo/local.toml",
        "-F",
        "viewer,viewer-debug",
        "--",
        "--yellow",
        yellow,
        "--blue",
        blue,
        "--viewer",
    ]
    return subprocess.Popen(command, cwd=root)


def stop_process(process: subprocess.Popen, timeout: float = 5) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def record_frames(ws_url: str, frame_dir: Path, match: subprocess.Popen, args: argparse.Namespace) -> int:
    cdp = DevToolsSocket(ws_url)
    frame_count = 0
    try:
        cdp.send("Page.enable")
        cdp.send(
            "Emulation.setDeviceMetricsOverride",
            {
                "width": args.width,
                "height": args.height,
                "deviceScaleFactor": 1,
                "mobile": False,
                "screenWidth": args.width,
                "screenHeight": args.height,
            },
        )
        cdp.send("Page.navigate", {"url": VIEWER_URL})
        cdp.send(
            "Page.startScreencast",
            {
                "format": "png",
                "maxWidth": args.width,
                "maxHeight": args.height,
                "everyNthFrame": 1,
            },
        )
        while match.poll() is None:
            try:
                message = cdp.recv()
            except socket.timeout:
                continue
            if not message or message.get("method") != "Page.screencastFrame":
                continue
            params = message.get("params", {})
            session_id = params.get("sessionId")
            data = params.get("data")
            if isinstance(data, str):
                frame_count += 1
                (frame_dir / f"frame_{frame_count:06d}.png").write_bytes(base64.b64decode(data))
            if session_id is not None:
                cdp.send("Page.screencastFrameAck", {"sessionId": session_id})
    finally:
        try:
            cdp.send("Page.stopScreencast")
        except Exception:  # noqa: BLE001
            pass
        cdp.close()
    return frame_count


def encode_mp4(ffmpeg: str, frame_dir: Path, out: Path, fps: int) -> None:
    subprocess.run(
        [
            ffmpeg,
            "-y",
            "-hide_banner",
            "-loglevel",
            "error",
            "-framerate",
            str(fps),
            "-i",
            str(frame_dir / "frame_%06d.png"),
            "-c:v",
            "libx264",
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(out),
        ],
        check=True,
    )


def main() -> int:
    args = parse_args()
    root = repo_root()
    chromium = require_tool(
        [
            os.environ.get("CHROME", ""),
            os.environ.get("CHROMIUM", ""),
            "chromium",
            "chromium-browser",
            "google-chrome",
            "google-chrome-stable",
        ],
        "Chromium/Chrome",
    )
    ffmpeg = require_tool(["ffmpeg"], "ffmpeg")
    out = output_path(args, root)

    match: subprocess.Popen | None = None
    browser: subprocess.Popen | None = None
    interrupted = False

    def handle_signal(signum: int, _frame: object) -> None:
        nonlocal interrupted
        interrupted = True
        if match and match.poll() is None:
            match.send_signal(signum)

    old_int = signal.signal(signal.SIGINT, handle_signal)
    old_term = signal.signal(signal.SIGTERM, handle_signal)

    with tempfile.TemporaryDirectory(prefix="simhark-record-") as temp:
        temp_dir = Path(temp)
        profile = temp_dir / "chromium-profile"
        frames = temp_dir / "frames"
        frames.mkdir()
        port = free_port()

        try:
            match = start_match(root, args.yellow, args.blue)
            wait_http(VIEWER_URL, timeout=300, process=match)

            browser = start_chromium(chromium, port, profile, args.width, args.height)
            wait_http(f"http://127.0.0.1:{port}/json/version", timeout=20, process=browser)
            ws_url = open_devtools_page(port, VIEWER_URL)

            print(f"recording {VIEWER_URL} -> {out}", file=sys.stderr)
            frame_count = record_frames(ws_url, frames, match, args)
            rc = match.wait()
            if interrupted:
                print("recording interrupted; encoding captured frames", file=sys.stderr)
            elif rc != 0:
                print(f"match runner exited with status {rc}; encoding captured frames", file=sys.stderr)

            if frame_count == 0:
                raise RuntimeError("captured no frames from Chromium")
            encode_mp4(ffmpeg, frames, out, args.fps)
            print(f"wrote {out} ({frame_count} frames)", file=sys.stderr)
            return rc if rc != 0 and not out.exists() else 0
        finally:
            signal.signal(signal.SIGINT, old_int)
            signal.signal(signal.SIGTERM, old_term)
            if browser:
                stop_process(browser)
            if match:
                stop_process(match)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as err:  # noqa: BLE001
        print(f"error: {err}", file=sys.stderr)
        raise SystemExit(1)
