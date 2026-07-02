import type { ViewerDebugSnapshot } from "../hooks/useViewerSocket";

interface DebugPanelProps {
  debug: ViewerDebugSnapshot | null;
  teamFilter?: "Blue" | "Yellow" | null;
  variant?: "sidebar" | "big";
}

export default function DebugPanel({
  debug,
  teamFilter = null,
  variant = "sidebar",
}: DebugPanelProps) {
  const big = variant === "big";
  const robots = (debug?.robots ?? [])
    .filter((robot) => !teamFilter || robot.team === teamFilter)
    .sort((left, right) => {
      if (left.team !== right.team) return left.team.localeCompare(right.team);
      return left.id - right.id;
    });

  return (
    <div className={panelClass(big)}>
      <h2 className="text-[10px] font-semibold text-cyan-400/80 uppercase tracking-[0.15em] flex items-center gap-1.5">
        <svg
          className="w-3 h-3"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <path d="M8 6h13" />
          <path d="M8 12h13" />
          <path d="M8 18h13" />
          <path d="M3 6h.01" />
          <path d="M3 12h.01" />
          <path d="M3 18h.01" />
        </svg>
        Debug
      </h2>

      <div className={strategyClass(big)}>
        <div className="flex items-center justify-between gap-2 mb-1">
          <div className="text-[9px] uppercase tracking-[0.15em] text-slate-500">
            Strategy
          </div>
          {debug && !teamFilter && (
            <div className="font-mono text-[9px] text-slate-500">
              world {debug.world_id}
            </div>
          )}
          {debug && teamFilter && (
            <div className="font-mono text-[9px] text-slate-500">
              {teamFilter.toLowerCase()} · world {debug.world_id}
            </div>
          )}
        </div>
        <div className={big ? "text-[11px] leading-snug text-slate-200 break-words whitespace-pre-wrap" : "text-xs leading-snug text-slate-200 break-words whitespace-pre-wrap"}>
          {debug?.strategy || "No strategy message"}
        </div>
      </div>

      <div className={robotsClass(big)}>
        {robots.length > 0 ? (
          robots.map((robot) => (
            <div
              key={`${robot.team}-${robot.id}`}
              className={robotCardClass(big)}
            >
              <div className="flex items-center gap-2 min-w-0">
                <span
                  className="w-2.5 h-2.5 rounded-full border border-white/50 shrink-0"
                  style={{ backgroundColor: robot.color }}
                />
                <span className={teamClass(robot.team)}>
                  {robot.team} {robot.id}
                </span>
                <span className="min-w-0 truncate font-mono text-[10px] text-slate-200">
                  {robot.task}
                </span>
              </div>
              {robot.message && (
                <div className={messageClass(big)}>
                  {robot.message}
                </div>
              )}
            </div>
          ))
        ) : (
          <div className="rounded-lg bg-slate-900/40 border border-slate-700/30 px-2.5 py-2 text-[11px] text-slate-500">
            {teamFilter
              ? `No ${teamFilter.toLowerCase()} robot debug messages`
              : "No robot debug messages"}
          </div>
        )}
      </div>
    </div>
  );
}

function panelClass(big: boolean): string {
  return big
    ? "h-full min-h-0 px-3 py-2.5 space-y-2 overflow-hidden flex flex-col"
    : "px-3 py-2.5 border-b border-slate-700/30 space-y-2 shrink-0 overflow-hidden";
}

function strategyClass(big: boolean): string {
  return [
    "rounded-lg bg-slate-900/40 border border-slate-700/30 px-2.5",
    big ? "py-1.5 shrink-0" : "py-2",
  ].join(" ");
}

function robotsClass(big: boolean): string {
  return big
    ? "grid flex-1 min-h-0 content-start gap-1.5 overflow-y-auto overflow-x-hidden pr-0.5 [grid-template-columns:repeat(auto-fit,minmax(230px,1fr))] auto-rows-min"
    : "space-y-1.5 max-h-72 overflow-y-auto pr-0.5";
}

function robotCardClass(big: boolean): string {
  return [
    "rounded-lg bg-slate-900/40 border border-slate-700/30 px-2.5 min-w-0",
    big ? "py-1.5 min-h-0" : "py-2",
  ].join(" ");
}

function messageClass(big: boolean): string {
  return big
    ? "mt-1 text-[10px] leading-snug text-slate-400 break-words"
    : "mt-1.5 text-[11px] leading-snug text-slate-400 break-words";
}

function teamClass(team: "Blue" | "Yellow"): string {
  return [
    "shrink-0 font-mono text-[10px] font-semibold",
    team === "Blue" ? "text-blue-300" : "text-amber-300",
  ].join(" ");
}
