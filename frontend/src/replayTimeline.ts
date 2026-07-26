import type { GameStateInfo, ReplayEvent, ReplayStatus } from "./hooks/useViewerSocket";

export interface ReplayPhase {
  startFrame: number;
  endFrame: number;
  label: string;
  raw: string;
  isTimeout: boolean;
  isInactive: boolean;
}

const PHASE_COLORS = [
  "#38bdf8",
  "#34d399",
  "#fbbf24",
  "#fb7185",
  "#a78bfa",
  "#2dd4bf",
];

export function replayPhases(events: ReplayEvent[], replay: ReplayStatus | null): ReplayPhase[] {
  if (!replay?.enabled || replay.frame_count <= 0) return [];
  const phaseEvents = events
    .filter((event) => event.kind === "referee")
    .slice()
    .sort((left, right) => left.frame - right.frame);
  const maxFrame = Math.max(0, replay.frame_count - 1);
  const phases: ReplayPhase[] = [];

  if (phaseEvents.length === 0) {
    return [{
      startFrame: 0,
      endFrame: maxFrame,
      label: "Replay",
      raw: "REPLAY",
      isTimeout: false,
      isInactive: false,
    }];
  }

  if (phaseEvents[0].frame > 0) {
    phases.push({
      startFrame: 0,
      endFrame: Math.min(maxFrame, phaseEvents[0].frame - 1),
      label: "Warmup",
      raw: "WARMUP",
      isTimeout: false,
      isInactive: false,
    });
  }

  for (let index = 0; index < phaseEvents.length; index += 1) {
    const event = phaseEvents[index];
    const next = phaseEvents[index + 1];
    const startFrame = clampFrame(event.frame, maxFrame);
    const endFrame = next ? clampFrame(next.frame - 1, maxFrame) : maxFrame;
    if (endFrame < startFrame) continue;
    phases.push({
      startFrame,
      endFrame,
      label: phaseLabel(event.label),
      raw: event.label,
      isTimeout: isTimeoutPhase(event.label),
      isInactive: isInactivePhase(event.label),
    });
  }

  return phases;
}

export function currentReplayPhase(
  events: ReplayEvent[],
  replay: ReplayStatus | null,
  gameState: GameStateInfo | null
): ReplayPhase | null {
  const phases = replayPhases(events, replay);
  const current = replay?.frame_index ?? 0;
  return (
    phases.find((phase) => current >= phase.startFrame && current <= phase.endFrame) ??
    (gameState
      ? {
          startFrame: current,
          endFrame: current,
          label: phaseLabel(gameState.command),
          raw: gameState.command,
          isTimeout: isTimeoutPhase(gameState.command),
          isInactive: isInactivePhase(gameState.command),
        }
      : null)
  );
}

export function nextActivePhaseFrame(events: ReplayEvent[], replay: ReplayStatus | null): number | null {
  if (!replay?.enabled) return null;
  const phases = replayPhases(events, replay);
  const current = replay.frame_index;
  const currentPhase = phases.find((phase) => current >= phase.startFrame && current <= phase.endFrame);
  if (!currentPhase?.isInactive) return null;
  const next = phases.find((phase) => phase.startFrame > current && !phase.isInactive);
  return next?.startFrame ?? null;
}

export function phaseColor(phase: ReplayPhase, index: number): string {
  if (phase.isInactive) return "#64748b";
  const label = phase.raw.toUpperCase();
  if (label.includes("BALL_PLACEMENT")) return "#22c55e";
  if (label.includes("KICKOFF") || label.includes("KICK_OFF")) return "#f59e0b";
  if (label.includes("STOP") || label.includes("HALT")) return "#94a3b8";
  if (label.includes("FORCE_START") || label.includes("RUNNING")) return "#38bdf8";
  return PHASE_COLORS[index % PHASE_COLORS.length];
}

export function replayEventColor(kind: ReplayEvent["kind"]): string {
  switch (kind) {
    case "goal_blue":
      return "#60a5fa";
    case "goal_yellow":
      return "#fbbf24";
    case "foul":
      return "#fb7185";
    case "referee":
      return "#a78bfa";
    default:
      return "#2dd4bf";
  }
}

export function isGoalEvent(kind: ReplayEvent["kind"]): boolean {
  return kind === "goal_blue" || kind === "goal_yellow";
}

export function phaseLabel(command: string): string {
  const normalized = command
    .replace(/^COMMAND_/, "")
    .replace(/_/g, " ")
    .trim()
    .toLowerCase();
  if (!normalized) return "Unknown";
  return normalized.replace(/\b\w/g, (letter) => letter.toUpperCase());
}

export function formatReplayTime(frameIndex: number, tickHz: number): string {
  const seconds = frameIndex / Math.max(1, tickHz);
  const wholeSeconds = Math.floor(seconds);
  const minutes = Math.floor(wholeSeconds / 60);
  const secs = wholeSeconds % 60;
  return `${minutes}:${secs.toString().padStart(2, "0")}`;
}

function isTimeoutPhase(command: string): boolean {
  return command.toUpperCase().includes("TIMEOUT");
}

function isInactivePhase(command: string): boolean {
  const label = command.toUpperCase();
  return label.includes("TIMEOUT") || label.includes("HALT") || label.includes("STOP");
}

function clampFrame(frame: number, maxFrame: number): number {
  return Math.max(0, Math.min(maxFrame, Math.floor(frame)));
}
