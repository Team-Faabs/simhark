import { useRef, useEffect, useCallback, useState } from "react";
import type { PointerEvent } from "react";
import type {
  BallTrajectory,
  FieldConfig,
  DebugOverlay,
  RobotInputInfo,
  RobotDebugInfo,
  RobotState,
  ViewerDebugSnapshot,
  ViewerFrame,
} from "../hooks/useViewerSocket";

interface FieldCanvasProps {
  frame: ViewerFrame | null;
  debugTeamFilter?: "Blue" | "Yellow" | null;
  showDebugOverlays?: boolean;
}

const FIELD_GREEN_LIGHT = "#1a5c34";
const FIELD_GREEN_DARK = "#0d3320";
const LINE_COLOR = "#ffffff";
const LINE_GLOW_COLOR = "rgba(255, 255, 255, 0.15)";
const BLUE_COLOR = "#3b82f6";
const YELLOW_COLOR = "#f59e0b";
const BALL_TRAJECTORY_COLOR = "#ff4fb8";
const PADDING = 36;

const WORLD_COLORS = [
  { base: "#38bdf8", light: "#bae6fd", dark: "#0369a1" },
  { base: "#fb7185", light: "#fecdd3", dark: "#be123c" },
  { base: "#a78bfa", light: "#ddd6fe", dark: "#6d28d9" },
  { base: "#34d399", light: "#bbf7d0", dark: "#047857" },
  { base: "#fbbf24", light: "#fef3c7", dark: "#b45309" },
  { base: "#f472b6", light: "#fbcfe8", dark: "#be185d" },
  { base: "#2dd4bf", light: "#ccfbf1", dark: "#0f766e" },
  { base: "#c084fc", light: "#f3e8ff", dark: "#7e22ce" },
  { base: "#f97316", light: "#fed7aa", dark: "#c2410c" },
  { base: "#22c55e", light: "#dcfce7", dark: "#15803d" },
  { base: "#60a5fa", light: "#dbeafe", dark: "#1d4ed8" },
  { base: "#e879f9", light: "#fae8ff", dark: "#a21caf" },
];

type WorldTint = (typeof WORLD_COLORS)[number];
type RobotHitTarget = {
  x: number;
  y: number;
  radius: number;
  worldId: number;
  team: "Blue" | "Yellow";
  id: number;
  input: string | null;
};
type RobotHover = RobotHitTarget & { mouseX: number; mouseY: number };

export default function FieldCanvas({
  frame,
  debugTeamFilter = null,
  showDebugOverlays = false,
}: FieldCanvasProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const containerRef = useRef<HTMLDivElement>(null);
  const frameRef = useRef<ViewerFrame | null>(frame);
  const hitTargetsRef = useRef<RobotHitTarget[]>([]);
  const [hover, setHover] = useState<RobotHover | null>(null);

  // Keep latest frame in a ref so the resize observer can redraw without
  // re-binding.
  frameRef.current = frame;

  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;

    const dpr = window.devicePixelRatio || 1;
    const rect = container.getBoundingClientRect();
    canvas.width = Math.max(1, Math.floor(rect.width * dpr));
    canvas.height = Math.max(1, Math.floor(rect.height * dpr));
    canvas.style.width = `${rect.width}px`;
    canvas.style.height = `${rect.height}px`;

    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    const w = rect.width;
    const h = rect.height;

    ctx.fillStyle = "#070d15";
    ctx.fillRect(0, 0, w, h);

    const snapshot = frameRef.current;
    hitTargetsRef.current = [];
    if (!snapshot) {
      ctx.fillStyle = "#475569";
      ctx.font = '14px "Inter", system-ui';
      ctx.textAlign = "center";
      ctx.textBaseline = "middle";
      ctx.fillText("Waiting for simulation frames…", w / 2, h / 2);
      return;
    }

    const field = snapshot.field;
    const robotRadius = snapshot.robot_radius;
    const ballRadius = snapshot.ball_radius;

    drawField(ctx, w, h, field);

    const fieldLength = field.field_length;
    const fieldWidth = field.field_width;
    const halfBoundsX =
      fieldLength / 2 + field.margin_goal_line + field.goal_depth + 0.2;
    const halfBoundsY = fieldWidth / 2 + field.margin_touch_line + 0.2;
    const scale = Math.min(
      (w - 2 * PADDING) / (halfBoundsX * 2),
      (h - 2 * PADDING) / (halfBoundsY * 2)
    );
    const offsetX = w / 2;
    const offsetY = h / 2;
    const toCanvas = (fx: number, fy: number): [number, number] => [
      offsetX + fx * scale,
      offsetY - fy * scale,
    ];

    drawFieldLines(ctx, toCanvas, scale, field);
    drawCoordinateGuides(ctx, toCanvas, scale, field, w, h);

    const visibleStates = snapshot.states?.length ? snapshot.states : [snapshot.state];
    const worldOpacity = visibleStates.length > 1 ? 0.55 : 1;
    const debugSnapshot = snapshot.debug ?? null;
    const inputLookup = robotInputLookup(snapshot.robot_inputs ?? []);
    const debugRobots =
      visibleStates.length === 1
        ? robotDebugLookup(debugSnapshot, debugTeamFilter)
        : null;
    if (showDebugOverlays && visibleStates.length === 1 && debugSnapshot) {
      drawDebugOverlays(ctx, toCanvas, scale, field, debugSnapshot, debugTeamFilter);
    }
    if (showDebugOverlays && visibleStates.length === 1 && snapshot.ball_trajectory) {
      drawBallTrajectory(ctx, toCanvas, scale, snapshot.ball_trajectory);
    }

    for (const state of visibleStates) {
      const tint = worldTint(state.world_id);
      ctx.save();
      ctx.globalAlpha = worldOpacity;

      for (const robot of state.blue_robots) {
        if (!robot.is_on) continue;
        const debug = debugRobots?.get(robotDebugKey(robot));
        drawRobot(
          ctx,
          toCanvas,
          scale,
          robotRadius,
          robot,
          tint,
          BLUE_COLOR,
          debug?.color
        );
        const [x, y] = toCanvas(robot.x, robot.y);
        hitTargetsRef.current.push({
          x,
          y,
          radius: Math.max(robotRadius * scale, 8),
          worldId: state.world_id,
          team: robot.team,
          id: robot.id,
          input:
            debug?.message ??
            debug?.task ??
            inputLookup.get(robotInputKey(state.world_id, robot.team, robot.id)) ??
            null,
        });
      }
      for (const robot of state.yellow_robots) {
        if (!robot.is_on) continue;
        const debug = debugRobots?.get(robotDebugKey(robot));
        drawRobot(
          ctx,
          toCanvas,
          scale,
          robotRadius,
          robot,
          tint,
          YELLOW_COLOR,
          debug?.color
        );
        const [x, y] = toCanvas(robot.x, robot.y);
        hitTargetsRef.current.push({
          x,
          y,
          radius: Math.max(robotRadius * scale, 8),
          worldId: state.world_id,
          team: robot.team,
          id: robot.id,
          input:
            debug?.message ??
            debug?.task ??
            inputLookup.get(robotInputKey(state.world_id, robot.team, robot.id)) ??
            null,
        });
      }

      drawBall(ctx, toCanvas, scale, ballRadius, state.ball, tint);
      ctx.restore();
    }

    ctx.fillStyle = "rgba(71, 85, 105, 0.7)";
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.fillText(
      `${fieldLength.toFixed(2)}m × ${fieldWidth.toFixed(2)}m`,
      PADDING,
      h - 10
    );
  }, []);

  useEffect(() => {
    draw();
  }, [frame, draw]);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    const observer = new ResizeObserver(() => draw());
    observer.observe(container);
    return () => observer.disconnect();
  }, [draw]);

  const handlePointerMove = useCallback((event: PointerEvent<HTMLDivElement>) => {
    const rect = event.currentTarget.getBoundingClientRect();
    const mouseX = event.clientX - rect.left;
    const mouseY = event.clientY - rect.top;
    let target: RobotHitTarget | null = null;
    for (let index = hitTargetsRef.current.length - 1; index >= 0; index -= 1) {
      const candidate = hitTargetsRef.current[index];
      if (Math.hypot(candidate.x - mouseX, candidate.y - mouseY) <= candidate.radius + 6) {
        target = candidate;
        break;
      }
    }
    setHover(target ? { ...target, mouseX, mouseY } : null);
  }, []);

  return (
    <div
      ref={containerRef}
      className="w-full h-full relative"
      onPointerMove={handlePointerMove}
      onPointerLeave={() => setHover(null)}
    >
      <canvas ref={canvasRef} className="absolute inset-0" />
      {hover && (
        <div
          className="pointer-events-none absolute z-10 max-w-96 rounded-md border border-slate-600/70 bg-slate-950/92 px-2.5 py-2 text-[11px] text-slate-100 shadow-xl"
          style={{
            left: Math.min(hover.mouseX + 14, Math.max(12, (containerRef.current?.clientWidth ?? 0) - 340)),
            top: Math.min(hover.mouseY + 14, Math.max(12, (containerRef.current?.clientHeight ?? 0) - 120)),
          }}
        >
          <div className="mb-1 flex items-center gap-2 font-mono text-[10px]">
            <span className={hover.team === "Blue" ? "text-blue-300" : "text-amber-300"}>
              {hover.team} {hover.id}
            </span>
            <span className="text-slate-500">world {hover.worldId}</span>
          </div>
          <div className="max-h-24 overflow-hidden break-words font-mono leading-snug text-slate-300">
            {hover.input ?? "No recorded input for this robot in this frame"}
          </div>
        </div>
      )}
    </div>
  );
}

function robotInputLookup(inputs: RobotInputInfo[]): Map<string, string> {
  const lookup = new Map<string, string>();
  for (const input of inputs) {
    lookup.set(robotInputKey(input.world_id, input.team, input.id), input.input);
  }
  return lookup;
}

function robotInputKey(worldId: number, team: "Blue" | "Yellow", id: number): string {
  return `${worldId}:${team}:${id}`;
}

function drawField(
  ctx: CanvasRenderingContext2D,
  w: number,
  h: number,
  field: FieldConfig
) {
  const gradient = ctx.createLinearGradient(0, 0, w, h);
  gradient.addColorStop(0, FIELD_GREEN_DARK);
  gradient.addColorStop(0.5, FIELD_GREEN_LIGHT);
  gradient.addColorStop(1, FIELD_GREEN_DARK);
  ctx.fillStyle = gradient;
  ctx.fillRect(0, 0, w, h);
  // Suppress unused-arg warning for `field`; the param is kept for parity
  // with the original implementation should we need it later.
  void field;
}

function drawFieldLines(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  field: FieldConfig
) {
  const drawGlowLine = (
    x1: number,
    y1: number,
    x2: number,
    y2: number
  ) => {
    ctx.strokeStyle = LINE_GLOW_COLOR;
    ctx.lineWidth = 6;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
    ctx.strokeStyle = LINE_COLOR;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(x2, y2);
    ctx.stroke();
  };

  const drawGlowArc = (
    cx: number,
    cy: number,
    r: number,
    a1: number,
    a2: number
  ) => {
    ctx.strokeStyle = LINE_GLOW_COLOR;
    ctx.lineWidth = 6;
    ctx.beginPath();
    ctx.arc(cx, cy, r, a1, a2);
    ctx.stroke();
    ctx.strokeStyle = LINE_COLOR;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(cx, cy, r, a1, a2);
    ctx.stroke();
  };

  const drawGlowRect = (
    rx: number,
    ry: number,
    rw: number,
    rh: number
  ) => {
    ctx.strokeStyle = LINE_GLOW_COLOR;
    ctx.lineWidth = 6;
    ctx.strokeRect(rx, ry, rw, rh);
    ctx.strokeStyle = LINE_COLOR;
    ctx.lineWidth = 1.5;
    ctx.strokeRect(rx, ry, rw, rh);
  };

  const fieldLength = field.field_length;
  const fieldWidth = field.field_width;
  const [lx, ty] = toCanvas(-fieldLength / 2, fieldWidth / 2);
  drawGlowRect(lx, ty, fieldLength * scale, fieldWidth * scale);

  const [cx1, cy1] = toCanvas(0, fieldWidth / 2);
  const [cx2, cy2] = toCanvas(0, -fieldWidth / 2);
  drawGlowLine(cx1, cy1, cx2, cy2);

  const [cc, ccY] = toCanvas(0, 0);
  drawGlowArc(cc, ccY, field.field_center_radius * scale, 0, Math.PI * 2);

  // Penalty boxes
  const penaltyW = field.penalty_depth * scale;
  const penaltyH = field.penalty_width * scale;
  const [plx, ply] = toCanvas(-fieldLength / 2, field.penalty_width / 2);
  drawGlowRect(plx, ply, penaltyW, penaltyH);
  const [prx, pry] = toCanvas(
    fieldLength / 2 - field.penalty_depth,
    field.penalty_width / 2
  );
  drawGlowRect(prx, pry, penaltyW, penaltyH);

  // Goals
  const goalW = field.goal_depth * scale;
  const goalH = field.goal_width * scale;
  const [glx, gly] = toCanvas(
    -fieldLength / 2 - field.goal_depth,
    field.goal_width / 2
  );
  drawGlowRect(glx, gly, goalW, goalH);
  const [grx, gry] = toCanvas(fieldLength / 2, field.goal_width / 2);
  drawGlowRect(grx, gry, goalW, goalH);
}

function drawCoordinateGuides(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  field: FieldConfig,
  w: number,
  h: number
) {
  const fieldLength = field.field_length;
  const fieldWidth = field.field_width;
  const halfX = fieldLength / 2;
  const halfY = fieldWidth / 2;
  const labelColor = "rgba(226, 232, 240, 0.74)";
  const tickColor = "rgba(226, 232, 240, 0.46)";
  const axisColor = "rgba(56, 189, 248, 0.42)";
  const majorStep = chooseGuideStep(fieldLength, scale);
  const minorStep = majorStep / 2;

  ctx.save();
  ctx.font = '10px "JetBrains Mono", monospace';
  ctx.lineCap = "round";
  ctx.lineJoin = "round";

  const [left, top] = toCanvas(-halfX, halfY);
  const [right, bottom] = toCanvas(halfX, -halfY);
  const [, centerY] = toCanvas(0, 0);
  const [centerX] = toCanvas(0, 0);

  ctx.strokeStyle = axisColor;
  ctx.lineWidth = 1;
  ctx.setLineDash([6, 7]);
  ctx.beginPath();
  ctx.moveTo(left, centerY);
  ctx.lineTo(right, centerY);
  ctx.moveTo(centerX, top);
  ctx.lineTo(centerX, bottom);
  ctx.stroke();
  ctx.setLineDash([]);

  ctx.strokeStyle = tickColor;
  ctx.lineWidth = 1;
  for (
    let x = Math.ceil(-halfX / minorStep) * minorStep;
    x <= halfX + 1e-6;
    x += minorStep
  ) {
    const isMajor = isMultipleOfStep(x, majorStep);
    const [px] = toCanvas(x, 0);
    const tick = isMajor ? 7 : 4;
    ctx.beginPath();
    ctx.moveTo(px, bottom + 2);
    ctx.lineTo(px, bottom + 2 + tick);
    ctx.moveTo(px, top - 2);
    ctx.lineTo(px, top - 2 - tick);
    ctx.stroke();

    if (isMajor) {
      ctx.fillStyle = labelColor;
      ctx.textAlign = "center";
      ctx.textBaseline = "top";
      ctx.fillText(formatGuideNumber(x), px, Math.min(h - 16, bottom + 12));
    }
  }

  for (
    let y = Math.ceil(-halfY / minorStep) * minorStep;
    y <= halfY + 1e-6;
    y += minorStep
  ) {
    const isMajor = isMultipleOfStep(y, majorStep);
    const [, py] = toCanvas(0, y);
    const tick = isMajor ? 7 : 4;
    ctx.beginPath();
    ctx.moveTo(left - 2, py);
    ctx.lineTo(left - 2 - tick, py);
    ctx.moveTo(right + 2, py);
    ctx.lineTo(right + 2 + tick, py);
    ctx.stroke();

    if (isMajor) {
      ctx.fillStyle = labelColor;
      ctx.textAlign = "right";
      ctx.textBaseline = "middle";
      ctx.fillText(formatGuideNumber(y), Math.max(16, left - 12), py);
    }
  }

  ctx.fillStyle = labelColor;
  ctx.textAlign = "left";
  ctx.textBaseline = "top";
  ctx.fillText("x/y meters", Math.max(10, left), Math.min(h - 18, bottom + 28));

  drawAngleCompass(ctx, w, h);
  ctx.restore();
}

function drawAngleCompass(ctx: CanvasRenderingContext2D, w: number, h: number) {
  const radius = Math.min(38, Math.max(28, Math.min(w, h) * 0.055));
  const cx = w - PADDING - radius - 10;
  const cy = PADDING + radius + 8;
  if (cx < radius + 12 || cy < radius + 12) return;

  ctx.save();
  ctx.fillStyle = "rgba(2, 6, 23, 0.48)";
  ctx.strokeStyle = "rgba(148, 163, 184, 0.38)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(cx, cy, radius + 18, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  ctx.strokeStyle = "rgba(226, 232, 240, 0.54)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.arc(cx, cy, radius, 0, Math.PI * 2);
  ctx.stroke();

  const directions = [
    { degrees: 0, label: "0°" },
    { degrees: 90, label: "90°" },
    { degrees: 180, label: "180°" },
    { degrees: 270, label: "270°" },
  ];

  ctx.font = '10px "JetBrains Mono", monospace';
  for (const direction of directions) {
    const angle = (direction.degrees * Math.PI) / 180;
    const dx = Math.cos(angle);
    const dy = -Math.sin(angle);
    const tipX = cx + dx * radius;
    const tipY = cy + dy * radius;

    ctx.strokeStyle =
      direction.degrees === 0 ? "rgba(34, 211, 238, 0.9)" : "rgba(226, 232, 240, 0.58)";
    ctx.fillStyle = ctx.strokeStyle;
    ctx.lineWidth = direction.degrees === 0 ? 2 : 1.4;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(tipX, tipY, direction.degrees === 0 ? 3.2 : 2.4, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = "rgba(226, 232, 240, 0.86)";
    ctx.textAlign =
      direction.degrees === 0
        ? "left"
        : direction.degrees === 180
          ? "right"
          : "center";
    ctx.textBaseline =
      direction.degrees === 90
        ? "bottom"
        : direction.degrees === 270
          ? "top"
          : "middle";
    ctx.fillText(
      direction.label,
      cx + dx * (radius + 8),
      cy + dy * (radius + 8)
    );
  }

  ctx.fillStyle = "rgba(148, 163, 184, 0.76)";
  ctx.font = '9px "JetBrains Mono", monospace';
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("angle", cx, cy);
  ctx.restore();
}

function chooseGuideStep(fieldLength: number, scale: number): number {
  const minimumLabelSpacingPx = 44;
  const candidates = [0.5, 1, 2, 5];
  return (
    candidates.find((candidate) => candidate * scale >= minimumLabelSpacingPx) ??
    Math.max(1, Math.ceil(fieldLength / 8))
  );
}

function isMultipleOfStep(value: number, step: number): boolean {
  return Math.abs(value / step - Math.round(value / step)) < 1e-6;
}

function formatGuideNumber(value: number): string {
  const rounded = Math.abs(value) < 1e-6 ? 0 : value;
  return Number.isInteger(rounded) ? rounded.toFixed(0) : rounded.toFixed(1);
}

function drawBallTrajectory(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  trajectory: BallTrajectory
) {
  if (trajectory.points.length < 2) return;

  const points = trajectory.points.map((point) => toCanvas(point.x, point.y));

  ctx.save();
  ctx.lineCap = "round";
  ctx.lineJoin = "round";
  ctx.strokeStyle = "rgba(255, 79, 184, 0.16)";
  ctx.lineWidth = Math.max(10, scale * 0.055);
  ctx.beginPath();
  for (const [index, [x, y]] of points.entries()) {
    if (index === 0) {
      ctx.moveTo(x, y);
    } else {
      ctx.lineTo(x, y);
    }
  }
  ctx.stroke();

  ctx.strokeStyle = "rgba(255, 79, 184, 0.82)";
  ctx.lineWidth = Math.max(2.5, scale * 0.018);
  ctx.setLineDash([Math.max(8, scale * 0.06), Math.max(5, scale * 0.035)]);
  ctx.beginPath();
  for (const [index, [x, y]] of points.entries()) {
    if (index === 0) {
      ctx.moveTo(x, y);
    } else {
      ctx.lineTo(x, y);
    }
  }
  ctx.stroke();
  ctx.setLineDash([]);

  const [endX, endY] = points[points.length - 1];
  ctx.fillStyle = BALL_TRAJECTORY_COLOR;
  ctx.beginPath();
  ctx.arc(endX, endY, Math.max(4, scale * 0.025), 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "rgba(255, 255, 255, 0.8)";
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.restore();
}

function drawRobot(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  robotRadius: number,
  robot: RobotState,
  tint: WorldTint,
  teamColor: string,
  debugColor?: string
) {
  const [rx, ry] = toCanvas(robot.x, robot.y);
  const r = Math.max(robotRadius * scale, 8);
  const debugTint = debugColor ? tintFromHexColor(debugColor) : null;
  const bodyTint = debugTint ?? tint;
  const flatDebugColor = debugColor && !debugTint ? debugColor.trim() : null;

  const glowGrad = ctx.createRadialGradient(rx, ry, r, rx, ry, r * 2.5);
  glowGrad.addColorStop(0, hexToRgba(bodyTint.base, 0.24));
  glowGrad.addColorStop(1, "rgba(0,0,0,0)");
  ctx.fillStyle = glowGrad;
  ctx.beginPath();
  ctx.arc(rx, ry, r * 2.5, 0, Math.PI * 2);
  ctx.fill();

  ctx.beginPath();
  ctx.arc(rx, ry, r, 0, Math.PI * 2);
  if (flatDebugColor) {
    ctx.fillStyle = flatDebugColor;
  } else {
    const bodyGrad = ctx.createRadialGradient(
      rx - r * 0.3,
      ry - r * 0.3,
      0,
      rx,
      ry,
      r
    );
    bodyGrad.addColorStop(0, bodyTint.light);
    bodyGrad.addColorStop(0.55, bodyTint.base);
    bodyGrad.addColorStop(1, bodyTint.dark);
    ctx.fillStyle = bodyGrad;
  }
  ctx.fill();

  ctx.strokeStyle = "rgba(2, 6, 23, 0.82)";
  ctx.lineWidth = 3.2;
  ctx.stroke();
  ctx.strokeStyle = teamColor;
  ctx.lineWidth = 2;
  ctx.stroke();

  const headingAngle = -robot.orientation;
  ctx.beginPath();
  ctx.arc(rx, ry, r * 0.62, headingAngle - 0.72, headingAngle + 0.72);
  ctx.strokeStyle = "rgba(255,255,255,0.8)";
  ctx.lineWidth = 2.2;
  ctx.lineCap = "round";
  ctx.stroke();

  const dirLen = r + 8;
  const dx = Math.cos(robot.orientation) * dirLen;
  const dy = -Math.sin(robot.orientation) * dirLen;
  ctx.beginPath();
  ctx.moveTo(rx, ry);
  ctx.lineTo(rx + dx, ry + dy);
  ctx.strokeStyle = teamColor;
  ctx.lineWidth = 3;
  ctx.lineCap = "round";
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(rx + dx, ry + dy, 3, 0, Math.PI * 2);
  ctx.fillStyle = "#ffffff";
  ctx.fill();
  ctx.strokeStyle = teamColor;
  ctx.lineWidth = 1.5;
  ctx.stroke();

  const fontSize = Math.max(r * 0.85, 9);
  ctx.font = `bold ${fontSize}px "Inter", system-ui`;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.strokeStyle = "rgba(0, 0, 0, 0.7)";
  ctx.lineWidth = 3;
  ctx.lineJoin = "round";
  ctx.strokeText(String(robot.id), rx, ry);
  ctx.fillStyle = "#ffffff";
  ctx.fillText(String(robot.id), rx, ry);
}

function drawDebugOverlays(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  field: FieldConfig,
  debug: ViewerDebugSnapshot,
  teamFilter: "Blue" | "Yellow" | null
) {
  for (const overlay of debug.overlays ?? []) {
    if (teamFilter && overlay.team !== teamFilter) continue;
    if (overlay.kind === "holo_robot") {
      drawHoloRobot(ctx, toCanvas, scale, overlay);
    } else {
      drawKickLine(ctx, toCanvas, scale, field, overlay);
    }
  }
}

function drawHoloRobot(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  overlay: Extract<DebugOverlay, { kind: "holo_robot" }>
) {
  const [x, y] = toCanvas(overlay.x, overlay.y);
  const r = Math.max(0.09 * scale, 9);
  const color = normalizeHexColor(overlay.color) ?? overlay.color;

  ctx.save();
  ctx.setLineDash([6, 4]);
  ctx.lineWidth = 2.2;
  ctx.strokeStyle = color;
  ctx.fillStyle = hexToRgba(normalizeHexColor(overlay.color) ?? "#ffffff", 0.12);
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.setLineDash([]);

  if (typeof overlay.orientation === "number") {
    const dx = Math.cos(overlay.orientation) * (r + 7);
    const dy = -Math.sin(overlay.orientation) * (r + 7);
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + dx, y + dy);
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.2;
    ctx.lineCap = "round";
    ctx.stroke();
  }

  ctx.font = 'bold 10px "JetBrains Mono", monospace';
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.strokeStyle = "rgba(2, 6, 23, 0.85)";
  ctx.lineWidth = 3;
  const label = `${overlay.id}`;
  ctx.strokeText(label, x, y);
  ctx.fillStyle = "#ffffff";
  ctx.fillText(label, x, y);

  if (overlay.label) {
    const text = `${overlay.team[0]}${overlay.id} ${overlay.label}`;
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textBaseline = "top";
    ctx.strokeStyle = "rgba(2, 6, 23, 0.85)";
    ctx.lineWidth = 3;
    ctx.strokeText(text, x, y + r + 5);
    ctx.fillStyle = "#ffffff";
    ctx.fillText(text, x, y + r + 5);
  }
  ctx.restore();
}

function drawKickLine(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  field: FieldConfig,
  overlay: Extract<DebugOverlay, { kind: "kick_line" }>
) {
  const end = rayFieldIntersection(
    overlay.from_x,
    overlay.from_y,
    overlay.angle,
    field
  );
  const [x1, y1] = toCanvas(overlay.from_x, overlay.from_y);
  const [x2, y2] = toCanvas(end.x, end.y);
  const color = normalizeHexColor(overlay.color) ?? overlay.color;

  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = Math.max(2, scale * 0.01);
  ctx.setLineDash([10, 6]);
  ctx.beginPath();
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
  ctx.setLineDash([]);

  const angle = Math.atan2(y2 - y1, x2 - x1);
  const head = 10;
  ctx.beginPath();
  ctx.moveTo(x2, y2);
  ctx.lineTo(
    x2 - Math.cos(angle - 0.45) * head,
    y2 - Math.sin(angle - 0.45) * head
  );
  ctx.lineTo(
    x2 - Math.cos(angle + 0.45) * head,
    y2 - Math.sin(angle + 0.45) * head
  );
  ctx.closePath();
  ctx.fill();

  ctx.beginPath();
  ctx.arc(x2, y2, 4, 0, Math.PI * 2);
  ctx.fillStyle = "#ffffff";
  ctx.fill();
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.stroke();

  if (overlay.label) {
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.strokeStyle = "rgba(2, 6, 23, 0.85)";
    ctx.lineWidth = 3;
    const text = `${overlay.team[0]}${overlay.id} ${overlay.label}`;
    ctx.strokeText(text, x2 + 6, y2 - 4);
    ctx.fillStyle = "#ffffff";
    ctx.fillText(text, x2 + 6, y2 - 4);
  }
  ctx.restore();
}

function drawBall(
  ctx: CanvasRenderingContext2D,
  toCanvas: (x: number, y: number) => [number, number],
  scale: number,
  ballRadius: number,
  ball: { x: number; y: number; z: number },
  tint: WorldTint
) {
  const [bx, by] = toCanvas(ball.x, ball.y);
  const r = Math.max(ballRadius * scale * 1.8, 5);

  const glowGrad = ctx.createRadialGradient(bx, by, r * 0.5, bx, by, r * 4);
  glowGrad.addColorStop(0, hexToRgba(tint.base, 0.42));
  glowGrad.addColorStop(1, hexToRgba(tint.base, 0));
  ctx.fillStyle = glowGrad;
  ctx.beginPath();
  ctx.arc(bx, by, r * 4, 0, Math.PI * 2);
  ctx.fill();

  const ballGrad = ctx.createRadialGradient(
    bx - r * 0.3,
    by - r * 0.3,
    0,
    bx,
    by,
    r
  );
  ballGrad.addColorStop(0, tint.light);
  ballGrad.addColorStop(0.65, tint.base);
  ballGrad.addColorStop(1, tint.dark);
  ctx.beginPath();
  ctx.arc(bx, by, r, 0, Math.PI * 2);
  ctx.fillStyle = ballGrad;
  ctx.fill();
  ctx.strokeStyle = "rgba(255, 255, 255, 0.85)";
  ctx.lineWidth = 1.4;
  ctx.stroke();

  if (ball.z > ballRadius * 1.5) {
    ctx.fillStyle = "rgba(255,255,255,0.9)";
    ctx.font = '11px "JetBrains Mono", monospace';
    ctx.textAlign = "left";
    ctx.textBaseline = "bottom";
    ctx.fillText(`${ball.z.toFixed(2)}m`, bx + r + 4, by - r - 2);
  }
}

function worldTint(worldId: number): WorldTint {
  return WORLD_COLORS[positiveModulo(worldId, WORLD_COLORS.length)];
}

function robotDebugLookup(
  debug: ViewerDebugSnapshot | null,
  teamFilter: "Blue" | "Yellow" | null
): Map<string, RobotDebugInfo> | null {
  if (!debug?.robots.length) return null;
  return new Map(
    debug.robots
      .filter((robot) => !teamFilter || robot.team === teamFilter)
      .map((robot) => [robotDebugKey(robot), robot])
  );
}

function robotDebugKey(robot: { team: "Blue" | "Yellow"; id: number }): string {
  return `${robot.team}:${robot.id}`;
}

function positiveModulo(value: number, divisor: number): number {
  return ((value % divisor) + divisor) % divisor;
}

function tintFromHexColor(color: string): WorldTint | null {
  const hex = normalizeHexColor(color);
  if (!hex) return null;

  const { r, g, b } = rgbFromHex(hex);
  const light = mixRgb({ r, g, b }, { r: 255, g: 255, b: 255 }, 0.58);
  const dark = mixRgb({ r, g, b }, { r: 2, g: 6, b: 23 }, 0.42);
  return {
    base: hex,
    light: rgbToHex(light),
    dark: rgbToHex(dark),
  };
}

function normalizeHexColor(color: string): string | null {
  const trimmed = color.trim();
  const match = trimmed.match(/^#?([0-9a-fA-F]{6})$/);
  return match ? `#${match[1].toLowerCase()}` : null;
}

function rayFieldIntersection(
  x: number,
  y: number,
  angle: number,
  field: FieldConfig
): { x: number; y: number } {
  const dx = Math.cos(angle);
  const dy = Math.sin(angle);
  const halfX = field.field_length / 2;
  const halfY = field.field_width / 2;
  const candidates: number[] = [];
  const eps = 1e-9;

  if (Math.abs(dx) > eps) {
    candidates.push((halfX - x) / dx, (-halfX - x) / dx);
  }
  if (Math.abs(dy) > eps) {
    candidates.push((halfY - y) / dy, (-halfY - y) / dy);
  }

  const distance =
    candidates
      .filter((t) => t > 0)
      .map((t) => ({ t, px: x + dx * t, py: y + dy * t }))
      .filter(({ px, py }) => px >= -halfX - 1e-6 && px <= halfX + 1e-6 && py >= -halfY - 1e-6 && py <= halfY + 1e-6)
      .sort((left, right) => left.t - right.t)[0]?.t ?? 3.0;

  return { x: x + dx * distance, y: y + dy * distance };
}

function hexToRgba(hex: string, alpha: number): string {
  const { r, g, b } = rgbFromHex(hex);
  return `rgba(${r}, ${g}, ${b}, ${alpha})`;
}

function rgbFromHex(hex: string): { r: number; g: number; b: number } {
  const normalized = hex.replace("#", "");
  const value = Number.parseInt(normalized, 16);
  const r = (value >> 16) & 255;
  const g = (value >> 8) & 255;
  const b = value & 255;
  return { r, g, b };
}

function mixRgb(
  first: { r: number; g: number; b: number },
  second: { r: number; g: number; b: number },
  amount: number
): { r: number; g: number; b: number } {
  return {
    r: Math.round(first.r + (second.r - first.r) * amount),
    g: Math.round(first.g + (second.g - first.g) * amount),
    b: Math.round(first.b + (second.b - first.b) * amount),
  };
}

function rgbToHex({ r, g, b }: { r: number; g: number; b: number }): string {
  return `#${[r, g, b]
    .map((value) => value.toString(16).padStart(2, "0"))
    .join("")}`;
}
