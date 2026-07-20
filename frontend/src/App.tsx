import { useCallback, useEffect, useRef } from "react";
import type { ReactNode } from "react";
import { useViewerSocket } from "./hooks/useViewerSocket";
import FieldCanvas from "./components/FieldCanvas";
import StatsPanel from "./components/StatsPanel";
import GameStatePanel from "./components/GameStatePanel";
import WorldSelector from "./components/WorldSelector";
import ControlPanel from "./components/ControlPanel";
import TestPanel from "./components/TestPanel";
import DebugPanel from "./components/DebugPanel";
import ReplayPanel from "./components/ReplayPanel";
import type { GameStateInfo, GoalSummary, ReplayEvent } from "./hooks/useViewerSocket";

declare global {
  interface Window {
    __SIMHARK_WS_PORT__?: number;
  }
}

const FALLBACK_WS_PORT = 8316;
type ViewerRoute = "default" | "debug" | "debug-big";
type DebugTeamFilter = "Blue" | "Yellow" | null;

function resolveWsPort(): number {
  if (typeof window !== "undefined" && typeof window.__SIMHARK_WS_PORT__ === "number") {
    return window.__SIMHARK_WS_PORT__;
  }
  return FALLBACK_WS_PORT;
}

function resolveViewerRoute(): ViewerRoute {
  if (typeof window === "undefined") return "default";
  switch (window.location.pathname) {
    case "/debug":
      return "debug";
    case "/debug-big":
      return "debug-big";
    default:
      return "default";
  }
}

function resolveDebugTeamFilter(): DebugTeamFilter {
  if (typeof window === "undefined") return null;
  const team = new URLSearchParams(window.location.search).get("team");
  switch (team?.toLowerCase()) {
    case "blue":
      return "Blue";
    case "yellow":
      return "Yellow";
    default:
      return null;
  }
}

export default function App() {
  const wsPort = resolveWsPort();
  const route = resolveViewerRoute();
  const debugTeam = resolveDebugTeamFilter();
  const {
    frame,
    connected,
    selectWorld,
    selectWorlds,
    sendControl,
    setSpeed,
    stepReplay,
    skipReplay,
    seekReplay,
    scrubReplay,
    flushReplayScrub,
  } =
    useViewerSocket(wsPort);
  const control = frame?.control ?? { web_enabled: false, running: true, speed: 1 };
  const selectedWorlds = frame?.selected_worlds ?? [frame?.selected_world ?? 0];
  const teamNames = replayTeamNames(frame?.events ?? [], frame?.replay?.frame_index ?? 0);
  const gameState = frame?.game_state
    ? {
        ...frame.game_state,
        blue_name: frame.game_state.blue_name ?? teamNames.blue,
        yellow_name: frame.game_state.yellow_name ?? teamNames.yellow,
      }
    : teamNames.blue || teamNames.yellow
      ? {
          command: "UNKNOWN",
          command_counter: 0,
          stage: null,
          state_counts: {},
          blue_name: teamNames.blue,
          yellow_name: teamNames.yellow,
        }
      : null;
  const showDebug = route !== "default";
  const spaceHoldRef = useRef<{
    startedAt: number;
    wasRunning: boolean;
    restoreSpeed: number | null;
  } | null>(null);
  const skipReplaySeconds = useCallback(
    (seconds: number) => {
      if (!frame?.replay?.enabled) return;
      const deltaFrames = Math.round(seconds * Math.max(1, frame.replay.base_speed));
      skipReplay(deltaFrames);
    },
    [frame?.replay?.enabled, frame?.replay?.base_speed, skipReplay]
  );

  useEffect(() => {
    if (!frame?.replay?.enabled) return;
    const onKeyDown = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null;
      if (target && ["INPUT", "SELECT", "TEXTAREA", "BUTTON"].includes(target.tagName)) return;
      if (event.code === "Comma" || event.code === "ArrowLeft") {
        event.preventDefault();
        stepReplay(-replayStepSize(event));
        return;
      }
      if (event.code === "Period" || event.code === "ArrowRight") {
        event.preventDefault();
        stepReplay(replayStepSize(event));
        return;
      }
      if (event.code === "KeyJ") {
        event.preventDefault();
        skipReplaySeconds(-10);
        return;
      }
      if (event.code === "KeyL") {
        event.preventDefault();
        skipReplaySeconds(10);
        return;
      }
      if (event.code !== "Space" || event.repeat) return;
      event.preventDefault();
      if (control.running) {
        spaceHoldRef.current = {
          startedAt: performance.now(),
          wasRunning: true,
          restoreSpeed: control.speed,
        };
        setSpeed(Math.min(4, Math.max(0.1, control.speed * 2)));
        return;
      }
      spaceHoldRef.current = {
        startedAt: performance.now(),
        wasRunning: false,
        restoreSpeed: null,
      };
      sendControl("start");
    };
    const onKeyUp = (event: KeyboardEvent) => {
      if (event.code !== "Space") return;
      const hold = spaceHoldRef.current;
      spaceHoldRef.current = null;
      if (!hold) return;
      event.preventDefault();
      if (hold.wasRunning) {
        if (hold.restoreSpeed !== null) setSpeed(hold.restoreSpeed);
        if (performance.now() - hold.startedAt < 180) {
          sendControl("pause");
        }
      }
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
  }, [
    frame?.replay?.enabled,
    control.running,
    control.speed,
    sendControl,
    setSpeed,
    skipReplaySeconds,
    stepReplay,
  ]);

  if (route === "debug-big") {
    return (
      <AppShell
        connected={connected}
        gameState={gameState}
        goals={frame?.goals ?? defaultGoals}
      >
        <div className="flex-1 grid min-h-0 gap-2 p-2 grid-cols-[minmax(0,1fr)_minmax(420px,0.95fr)]">
          <div className="min-w-0 glass-panel overflow-hidden panel-accent">
            <FieldCanvas
              frame={frame}
              debugTeamFilter={debugTeam}
              showDebugOverlays
              onSeekReplay={seekReplay}
              onScrubReplay={scrubReplay}
              onScrubReplayEnd={flushReplayScrub}
            />
          </div>
          <div className="min-w-0 glass-panel panel-accent overflow-hidden flex flex-col">
            <ControlPanel control={control} onSend={sendControl} onSpeed={setSpeed} />
            <ReplayPanel
              replay={frame?.replay ?? null}
              events={frame?.events ?? []}
              onSeek={seekReplay}
              onScrub={scrubReplay}
              onScrubEnd={flushReplayScrub}
              onSkipSeconds={skipReplaySeconds}
            />
            <div className="shrink-0 grid grid-cols-2 border-b border-slate-700/30">
              <GameStatePanel
                gameState={gameState}
                goals={frame?.goals ?? { blue: 0, yellow: 0, blue_active: false, yellow_active: false }}
              />
              <StatsPanel frame={frame} />
            </div>
            <div className="flex-1 min-h-0">
              <DebugPanel
                debug={frame?.debug ?? null}
                teamFilter={debugTeam}
                variant="big"
              />
            </div>
          </div>
        </div>
      </AppShell>
    );
  }

  return (
    <AppShell
      connected={connected}
      gameState={gameState}
      goals={frame?.goals ?? defaultGoals}
    >
      <div className="flex-1 flex min-h-0 gap-2 p-2">
        <div className="flex-1 min-w-0">
          <div className="h-full glass-panel overflow-hidden panel-accent">
            <FieldCanvas
              frame={frame}
              debugTeamFilter={debugTeam}
              showDebugOverlays={showDebug}
              onSeekReplay={seekReplay}
              onScrubReplay={scrubReplay}
              onScrubReplayEnd={flushReplayScrub}
            />
          </div>
        </div>

        <div className="w-88 shrink-0 glass-panel panel-accent flex flex-col overflow-y-auto overflow-x-hidden">
          <ControlPanel control={control} onSend={sendControl} onSpeed={setSpeed} />
          <ReplayPanel
            replay={frame?.replay ?? null}
            events={frame?.events ?? []}
            onSeek={seekReplay}
            onScrub={scrubReplay}
            onScrubEnd={flushReplayScrub}
            onSkipSeconds={skipReplaySeconds}
          />
          <WorldSelector
            worldCount={frame?.world_count ?? 0}
            selected={selectedWorlds}
            suite={frame?.test_suite ?? null}
            onSelect={selectWorlds}
          />
          <TestPanel
            suite={frame?.test_suite ?? null}
            selectedWorld={frame?.selected_world ?? 0}
            onSelect={selectWorld}
          />
          <GameStatePanel
            gameState={gameState}
            goals={frame?.goals ?? { blue: 0, yellow: 0, blue_active: false, yellow_active: false }}
          />
          {showDebug && (
            <DebugPanel debug={frame?.debug ?? null} teamFilter={debugTeam} />
          )}
          <StatsPanel frame={frame} />
        </div>
      </div>
    </AppShell>
  );
}

function replayStepSize(event: KeyboardEvent): number {
  if (event.ctrlKey) return 100;
  if (event.shiftKey) return 10;
  return 1;
}

function replayTeamNames(events: ReplayEvent[], frameIndex: number): {
  blue: string | null;
  yellow: string | null;
} {
  let event: ReplayEvent | null = null;
  for (let index = events.length - 1; index >= 0; index -= 1) {
    const candidate = events[index];
    if (candidate.kind === "referee" && candidate.frame <= frameIndex) {
      event = candidate;
      break;
    }
  }
  const details = event?.details;
  if (!details) return { blue: null, yellow: null };
  return {
    blue: replayDetailValue(details, "blue"),
    yellow: replayDetailValue(details, "yellow"),
  };
}

function replayDetailValue(details: string, key: string): string | null {
  const prefix = `${key} `;
  const line = details.split("\n").find((candidate) => candidate.startsWith(prefix));
  const value = line?.slice(prefix.length).trim();
  return value || null;
}

function AppShell({
  connected,
  gameState,
  goals,
  children,
}: {
  connected: boolean;
  gameState: GameStateInfo | null;
  goals: GoalSummary;
  children: ReactNode;
}) {
  return (
    <div className="h-full flex flex-col bg-dot-pattern text-slate-100">
      <header className="grid grid-cols-[minmax(220px,1fr)_minmax(300px,1.2fr)_minmax(180px,1fr)] items-center gap-4 px-5 py-2.5 bg-slate-900/80 backdrop-blur-xl border-b border-slate-700/40 shrink-0 relative">
        <div className="absolute bottom-0 left-0 right-0 h-px bg-linear-to-r from-transparent via-cyan-500/30 to-transparent" />

        <div className="flex min-w-0 items-center gap-4">
          <div className="flex items-center gap-2.5">
            <div className="w-7 h-7 rounded-lg bg-linear-to-br from-cyan-500 to-blue-600 flex items-center justify-center shadow-lg shadow-cyan-500/20">
              <svg
                className="w-4 h-4 text-white"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2.5"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <path d="M13 2L3 14h9l-1 8 10-12h-9l1-8z" />
              </svg>
            </div>
            <h1 className="text-lg font-bold tracking-tight">
              <span className="text-cyan-400">sim</span>
              <span className="text-slate-200">hark</span>
            </h1>
          </div>
          <div className="h-4 w-px bg-slate-700/60" />
          <span className="text-xs text-slate-500 font-mono tracking-wide">
            parallel SSL simulator
          </span>
        </div>

        <HeaderScoreboard gameState={gameState} goals={goals} />

        <div className="ml-auto flex items-center gap-3 px-3 py-1.5 rounded-lg bg-slate-800/50 border border-slate-700/30">
          <span
            className={`inline-block w-2 h-2 rounded-full transition-all duration-300 ${
              connected
                ? "bg-emerald-400 shadow-[0_0_6px_rgba(52,211,153,0.6)] animate-pulse-dot"
                : "bg-red-500 shadow-[0_0_6px_rgba(239,68,68,0.4)]"
            }`}
          />
          <span className="text-xs font-mono text-slate-400">
            {connected ? "LIVE" : "OFFLINE"}
          </span>
        </div>
      </header>
      {children}
    </div>
  );
}

const defaultGoals: GoalSummary = {
  blue: 0,
  yellow: 0,
  blue_active: false,
  yellow_active: false,
};

function HeaderScoreboard({
  gameState,
  goals,
}: {
  gameState: GameStateInfo | null;
  goals: GoalSummary;
}) {
  return (
    <div className="relative z-10 grid min-w-0 grid-cols-[minmax(0,1fr)_auto_minmax(0,1fr)] items-center gap-3">
      <HeaderTeam
        side="blue"
        name={gameState?.blue_name ?? "Blue"}
        score={goals.blue}
        active={goals.blue_active}
      />
      <div className="font-mono text-[11px] font-semibold text-slate-500">VS</div>
      <HeaderTeam
        side="yellow"
        name={gameState?.yellow_name ?? "Yellow"}
        score={goals.yellow}
        active={goals.yellow_active}
      />
    </div>
  );
}

function HeaderTeam({
  side,
  name,
  score,
  active,
}: {
  side: "blue" | "yellow";
  name: string;
  score: number;
  active: boolean;
}) {
  const isBlue = side === "blue";
  return (
    <div
      className={[
        "flex min-w-0 items-center gap-2 rounded-md border px-2.5 py-1",
        isBlue
          ? "border-blue-400/35 bg-blue-500/10"
          : "border-amber-400/35 bg-amber-500/10",
        active ? "ring-2 ring-emerald-300/70" : "",
      ].join(" ")}
    >
      <span
        className={[
          "h-2.5 w-2.5 shrink-0 rounded-full",
          isBlue ? "bg-blue-400" : "bg-amber-300",
        ].join(" ")}
      />
      <span className="min-w-0 flex-1 truncate text-sm font-semibold text-slate-100">
        {name}
      </span>
      <span
        className={[
          "shrink-0 font-mono text-2xl font-bold leading-none",
          isBlue ? "text-blue-200" : "text-amber-200",
        ].join(" ")}
      >
        {score}
      </span>
    </div>
  );
}
