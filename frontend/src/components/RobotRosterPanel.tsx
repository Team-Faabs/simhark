import type { RobotState, ViewerFrame } from "../hooks/useViewerSocket";

interface RobotRosterPanelProps {
  frame: ViewerFrame | null;
  onSetPresence: (
    worldId: number,
    team: "Blue" | "Yellow",
    id: number,
    present: boolean
  ) => void;
}

export default function RobotRosterPanel({
  frame,
  onSetPresence,
}: RobotRosterPanelProps) {
  if (frame?.replay.enabled) {
    return null;
  }

  const state = frame?.state;

  return (
    <div className="px-3 py-2.5 border-b border-slate-700/30 space-y-2 shrink-0">
      <div className="flex items-center justify-between gap-2">
        <h2 className="text-[10px] font-semibold text-cyan-400/80 uppercase tracking-[0.15em] flex items-center gap-1.5">
          <svg
            className="w-3 h-3"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <circle cx="12" cy="8" r="4" />
            <path d="M4 21a8 8 0 0 1 16 0" />
          </svg>
          Robots
        </h2>
        <span className="text-[9px] text-slate-500">click to add/remove</span>
      </div>

      <TeamRoster
        team="Blue"
        robots={state?.blue_robots ?? []}
        worldId={state?.world_id ?? 0}
        disabled={!state}
        onSetPresence={onSetPresence}
      />
      <TeamRoster
        team="Yellow"
        robots={state?.yellow_robots ?? []}
        worldId={state?.world_id ?? 0}
        disabled={!state}
        onSetPresence={onSetPresence}
      />
    </div>
  );
}

function TeamRoster({
  team,
  robots,
  worldId,
  disabled,
  onSetPresence,
}: {
  team: "Blue" | "Yellow";
  robots: RobotState[];
  worldId: number;
  disabled: boolean;
  onSetPresence: RobotRosterPanelProps["onSetPresence"];
}) {
  const active = robots.filter((robot) => robot.is_on).length;
  const activeClass =
    team === "Blue"
      ? "border-blue-400/70 bg-blue-500/25 text-blue-100 hover:bg-blue-500/35"
      : "border-amber-400/70 bg-amber-500/25 text-amber-100 hover:bg-amber-500/35";

  return (
    <div className="rounded-lg bg-slate-900/40 border border-slate-700/30 px-2 py-1.5">
      <div className="flex items-center justify-between mb-1.5">
        <span
          className={`text-[10px] font-semibold ${
            team === "Blue" ? "text-blue-400" : "text-amber-400"
          }`}
        >
          {team}
        </span>
        <span className="font-mono text-[9px] text-slate-500">
          {active}/{robots.length} on field
        </span>
      </div>
      <div className="flex flex-wrap gap-1">
        {robots.map((robot) => {
          const action = robot.is_on ? "Remove" : "Add";
          return (
            <button
              key={robot.id}
              type="button"
              aria-label={`${action} ${team} robot ${robot.id}`}
              aria-pressed={robot.is_on}
              title={`${action} ${team} robot ${robot.id}`}
              disabled={disabled}
              onClick={() => onSetPresence(worldId, team, robot.id, !robot.is_on)}
              className={`h-6 min-w-6 rounded-md border px-1.5 font-mono text-[10px] transition disabled:cursor-not-allowed disabled:opacity-40 ${
                robot.is_on
                  ? activeClass
                  : "border-slate-700/60 bg-slate-950/40 text-slate-500 hover:border-slate-500 hover:text-slate-300"
              }`}
            >
              {robot.id}
            </button>
          );
        })}
        {!robots.length && <span className="text-[10px] text-slate-600">—</span>}
      </div>
    </div>
  );
}
