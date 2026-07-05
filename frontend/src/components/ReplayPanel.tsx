import type { ReplayEvent, ReplayStatus } from "../hooks/useViewerSocket";

interface ReplayPanelProps {
  replay: ReplayStatus | null;
  events: ReplayEvent[];
}

export default function ReplayPanel({ replay, events }: ReplayPanelProps) {
  if (!replay?.enabled) return null;
  const currentFrame = replay.frame_index + 1;
  const progress = replay.frame_count > 0 ? currentFrame / replay.frame_count : 0;

  return (
    <div className="px-3 py-2.5 border-b border-slate-700/30 space-y-2">
      <div className="flex items-center justify-between">
        <h2 className="text-[10px] font-semibold text-cyan-400/80 uppercase tracking-[0.15em]">
          Replay
        </h2>
        <span className="font-mono text-[10px] text-slate-400">
          {currentFrame}/{replay.frame_count}
        </span>
      </div>
      <div className="h-1.5 overflow-hidden rounded-full bg-slate-800">
        <div
          className="h-full bg-cyan-400"
          style={{ width: `${Math.max(0, Math.min(1, progress)) * 100}%` }}
        />
      </div>
      <div className="max-h-52 space-y-1 overflow-y-auto pr-0.5">
        {events.length > 0 ? (
          events.map((event, index) => (
            <div
              key={`${event.frame}-${event.kind}-${index}`}
              className={[
                "rounded-md border px-2 py-1.5 text-[11px]",
                event.frame <= replay.frame_index
                  ? "border-cyan-500/30 bg-cyan-500/10"
                  : "border-slate-700/35 bg-slate-900/35",
              ].join(" ")}
            >
              <div className="flex items-center justify-between gap-2">
                <span className="truncate text-slate-100">{event.label}</span>
                <span className="shrink-0 font-mono text-[10px] text-slate-500">
                  f{event.frame}
                </span>
              </div>
              {event.details && (
                <div className="mt-0.5 break-words text-[10px] leading-snug text-slate-400">
                  {event.details}
                </div>
              )}
            </div>
          ))
        ) : (
          <div className="rounded-md border border-slate-700/35 bg-slate-900/35 px-2 py-1.5 text-[11px] text-slate-500">
            No replay events
          </div>
        )}
      </div>
    </div>
  );
}
