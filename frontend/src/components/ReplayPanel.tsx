import { useCallback, useRef } from "react";
import type { PointerEvent } from "react";
import type { ReplayEvent, ReplayStatus } from "../hooks/useViewerSocket";
import {
  currentReplayPhase,
  formatReplayTime,
  isGoalEvent,
  nextActivePhaseFrame,
  phaseColor,
  replayPhases,
} from "../replayTimeline";

interface ReplayPanelProps {
  replay: ReplayStatus | null;
  events: ReplayEvent[];
  onSeek?: (frameIndex: number) => void;
  onScrub?: (frameIndex: number) => void;
  onScrubEnd?: (frameIndex?: number) => void;
  onSkipSeconds?: (seconds: number) => void;
}

type EventStyle = {
  color: string;
  bgColor: string;
  bg: string;
  border: string;
  text: string;
  icon: "goal" | "foul" | "referee" | "event";
};

export default function ReplayPanel({
  replay,
  events,
  onSeek,
  onScrub,
  onScrubEnd,
  onSkipSeconds,
}: ReplayPanelProps) {
  const scrubFrameRef = useRef<number | null>(null);
  const seekFromPointer = useCallback(
    (event: PointerEvent<HTMLDivElement>) => {
      if ((!onScrub && !onSeek) || !replay?.enabled || replay.frame_count <= 0) return;
      const rect = event.currentTarget.getBoundingClientRect();
      const ratio = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width));
      const frameIndex = Math.round(ratio * (replay.frame_count - 1));
      if (scrubFrameRef.current === frameIndex) return;
      scrubFrameRef.current = frameIndex;
      (onScrub ?? onSeek)?.(frameIndex);
    },
    [onScrub, onSeek, replay?.enabled, replay?.frame_count]
  );
  const handlePointerDown = useCallback(
    (event: PointerEvent<HTMLDivElement>) => {
      event.preventDefault();
      event.currentTarget.setPointerCapture(event.pointerId);
      scrubFrameRef.current = null;
      seekFromPointer(event);
    },
    [seekFromPointer]
  );
  const handlePointerMove = useCallback(
    (event: PointerEvent<HTMLDivElement>) => {
      if (!event.currentTarget.hasPointerCapture(event.pointerId)) return;
      event.preventDefault();
      seekFromPointer(event);
    },
    [seekFromPointer]
  );
  const handlePointerEnd = useCallback((event: PointerEvent<HTMLDivElement>) => {
    if (event.currentTarget.hasPointerCapture(event.pointerId)) {
      event.currentTarget.releasePointerCapture(event.pointerId);
    }
    const finalFrame = scrubFrameRef.current ?? undefined;
    scrubFrameRef.current = null;
    onScrubEnd?.(finalFrame);
  }, [onScrubEnd]);

  if (!replay?.enabled) return null;
  const currentFrame = replay.frame_index + 1;
  const progress = replay.frame_count > 0 ? currentFrame / replay.frame_count : 0;
  const phases = replayPhases(events, replay);
  const currentPhase = currentReplayPhase(events, replay, null);
  const nextActiveFrame = nextActivePhaseFrame(events, replay);

  return (
    <div className="px-3 py-2.5 border-b border-slate-700/30 space-y-2.5">
      <div className="flex items-center justify-between">
        <h2 className="text-[10px] font-semibold text-cyan-400/80 uppercase tracking-[0.15em]">
          Replay
        </h2>
        <span className="font-mono text-[10px] text-slate-400">
          {currentFrame}/{replay.frame_count}
        </span>
      </div>
      <div className="flex items-center justify-between gap-2 rounded-md border border-slate-700/35 bg-slate-900/35 px-2 py-1.5">
        <div className="min-w-0">
          <div className="truncate text-xs font-semibold text-slate-100">
            {currentPhase?.label ?? "Replay"}
          </div>
          <div className="font-mono text-[10px] text-slate-500">
            {formatReplayTime(replay.frame_index, replay.base_speed)}
          </div>
        </div>
        <div className="flex shrink-0 items-center gap-1">
          <button
            type="button"
            onClick={() => onSkipSeconds?.(-10)}
            className="rounded border border-slate-700/60 bg-slate-950/50 px-2 py-1 font-mono text-[10px] text-slate-300 transition hover:border-cyan-300/70 hover:text-cyan-100"
          >
            -10s
          </button>
          <button
            type="button"
            onClick={() => onSkipSeconds?.(10)}
            className="rounded border border-slate-700/60 bg-slate-950/50 px-2 py-1 font-mono text-[10px] text-slate-300 transition hover:border-cyan-300/70 hover:text-cyan-100"
          >
            +10s
          </button>
          {nextActiveFrame !== null && (
            <button
              type="button"
              onClick={() => onSeek?.(nextActiveFrame)}
              className="rounded border border-slate-700/60 bg-slate-950/50 px-2 py-1 text-[10px] font-semibold uppercase tracking-[0.1em] text-slate-300 transition hover:border-cyan-300/70 hover:text-cyan-100"
            >
              Skip {currentPhase?.label ?? "inactive"}
            </button>
          )}
        </div>
      </div>
      <div
        className="relative h-4 cursor-ew-resize touch-none rounded-full"
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerEnd}
        onPointerCancel={handlePointerEnd}
        role="slider"
        aria-label="Scrub replay timeline"
        aria-valuemin={0}
        aria-valuemax={Math.max(0, replay.frame_count - 1)}
        aria-valuenow={replay.frame_index}
      >
        <div className="absolute left-0 right-0 top-1/2 h-1 -translate-y-1/2 overflow-hidden rounded-full bg-slate-800 shadow-inner">
          {phases.map((phase, index) => {
            const left = replay.frame_count > 1
              ? (phase.startFrame / (replay.frame_count - 1)) * 100
              : 0;
            const width = replay.frame_count > 1
              ? ((phase.endFrame - phase.startFrame + 1) / replay.frame_count) * 100
              : 100;
            return (
              <div
                key={`${phase.startFrame}-${phase.raw}-${index}`}
                className="absolute inset-y-0"
                title={phase.label}
                style={{
                  left: `${Math.max(0, Math.min(100, left))}%`,
                  width: `${Math.max(0.5, Math.min(100, width))}%`,
                  backgroundColor: phaseColor(phase, index),
                  opacity: phase.isInactive ? 0.3 : 0.42,
                }}
              />
            );
          })}
          <div
            className="relative h-full rounded-full bg-[#ff174f]"
            style={{ width: `${Math.max(0, Math.min(1, progress)) * 100}%` }}
          />
        </div>
        {events.map((event, index) => {
          const style = eventStyle(event.kind);
          const left = replay.frame_count > 1
            ? (event.frame / (replay.frame_count - 1)) * 100
            : 0;
          return (
            <span
              key={`${event.frame}-${event.kind}-${index}-marker`}
              title={`${event.label} - frame ${event.frame}`}
              className={[
                "pointer-events-none absolute top-1/2 -translate-x-1/2 -translate-y-1/2 border border-slate-950/70 shadow",
                isGoalEvent(event.kind)
                  ? "h-4 w-1.5 rounded-sm ring-1 ring-white/75"
                  : "h-3 w-1 rounded-full",
              ].join(" ")}
              style={{ left: `${Math.max(0, Math.min(100, left))}%`, backgroundColor: style.color }}
            />
          );
        })}
      </div>
      <div className="max-h-52 space-y-1 overflow-y-auto pr-0.5">
        {events.length > 0 ? (
          events.map((event, index) => {
            const style = eventStyle(event.kind);
            const isPast = event.frame <= replay.frame_index;
            return (
              <button
                key={`${event.frame}-${event.kind}-${index}`}
                type="button"
                onClick={() => onSeek?.(event.frame)}
                className={[
                  "w-full rounded-md border px-2 py-1.5 text-left text-[11px] transition",
                  "hover:border-cyan-300/60 hover:bg-slate-800/80 focus:outline-none focus:ring-1 focus:ring-cyan-300/70",
                  isPast ? style.bg : "bg-slate-900/35",
                ].join(" ")}
                style={{ borderColor: isPast ? style.border : "rgba(51, 65, 85, 0.55)" }}
              >
                <div className="flex items-center justify-between gap-2">
                  <span className="flex min-w-0 items-center gap-2">
                    <span
                      className="flex h-5 w-5 shrink-0 items-center justify-center rounded border"
                      style={{
                        borderColor: style.border,
                        backgroundColor: style.bgColor,
                        color: style.color,
                      }}
                    >
                      <EventIcon icon={style.icon} />
                    </span>
                    <span className={`truncate ${style.text}`}>{event.label}</span>
                  </span>
                  <span className="shrink-0 font-mono text-[10px] text-slate-500">
                    f{event.frame}
                  </span>
                </div>
                {event.details && (
                  <div className="mt-0.5 break-words pl-7 text-[10px] leading-snug text-slate-400">
                    {event.details}
                  </div>
                )}
              </button>
            );
          })
        ) : (
          <div className="rounded-md border border-slate-700/35 bg-slate-900/35 px-2 py-1.5 text-[11px] text-slate-500">
            No replay events
          </div>
        )}
      </div>
    </div>
  );
}

function eventStyle(kind: ReplayEvent["kind"]): EventStyle {
  switch (kind) {
    case "goal_blue":
      return {
        color: "#60a5fa",
        bgColor: "rgba(59, 130, 246, 0.12)",
        bg: "bg-blue-500/10",
        border: "rgba(96, 165, 250, 0.45)",
        text: "text-blue-100",
        icon: "goal",
      };
    case "goal_yellow":
      return {
        color: "#fbbf24",
        bgColor: "rgba(245, 158, 11, 0.12)",
        bg: "bg-amber-500/10",
        border: "rgba(251, 191, 36, 0.45)",
        text: "text-amber-100",
        icon: "goal",
      };
    case "foul":
      return {
        color: "#fb7185",
        bgColor: "rgba(244, 63, 94, 0.12)",
        bg: "bg-rose-500/10",
        border: "rgba(251, 113, 133, 0.45)",
        text: "text-rose-100",
        icon: "foul",
      };
    case "referee":
      return {
        color: "#a78bfa",
        bgColor: "rgba(139, 92, 246, 0.12)",
        bg: "bg-violet-500/10",
        border: "rgba(167, 139, 250, 0.45)",
        text: "text-violet-100",
        icon: "referee",
      };
    default:
      return {
        color: "#2dd4bf",
        bgColor: "rgba(20, 184, 166, 0.12)",
        bg: "bg-teal-500/10",
        border: "rgba(45, 212, 191, 0.45)",
        text: "text-teal-100",
        icon: "event",
      };
  }
}

function EventIcon({ icon }: { icon: EventStyle["icon"] }) {
  if (icon === "goal") {
    return (
      <svg viewBox="0 0 16 16" className="h-3.5 w-3.5" aria-hidden="true">
        <path d="M3 13V4h10v9" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" strokeLinejoin="round" />
        <path d="M5.5 13V6.5h5V13M3 8h10" fill="none" stroke="currentColor" strokeWidth="1.2" strokeLinecap="round" />
      </svg>
    );
  }
  if (icon === "foul") {
    return (
      <svg viewBox="0 0 16 16" className="h-3.5 w-3.5" aria-hidden="true">
        <path d="M8 2.5l6 10.5H2L8 2.5z" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinejoin="round" />
        <path d="M8 6v3.2M8 11.7h.01" fill="none" stroke="currentColor" strokeWidth="1.6" strokeLinecap="round" />
      </svg>
    );
  }
  if (icon === "referee") {
    return (
      <svg viewBox="0 0 16 16" className="h-3.5 w-3.5" aria-hidden="true">
        <path d="M3 3.5h8l2 2-2 2H3V3.5zM3 7.5V13" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
      </svg>
    );
  }
  return (
    <svg viewBox="0 0 16 16" className="h-3.5 w-3.5" aria-hidden="true">
      <circle cx="8" cy="8" r="4.5" fill="none" stroke="currentColor" strokeWidth="1.5" />
      <path d="M8 5.5v3l2 1.5" fill="none" stroke="currentColor" strokeWidth="1.4" strokeLinecap="round" strokeLinejoin="round" />
    </svg>
  );
}
