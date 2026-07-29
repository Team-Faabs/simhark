import type { ControlSnapshot } from "../hooks/useViewerSocket";

interface ControlPanelProps {
  control: ControlSnapshot;
  onSend: (action: "start" | "stop" | "restart" | "pause") => void;
  onSpeed: (speed: number) => void;
}

export default function ControlPanel({ control, onSend, onSpeed }: ControlPanelProps) {
  if (!control.web_enabled) {
    return null;
  }

  return (
    <div className="px-3 py-2.5 border-b border-slate-700/30 space-y-2">
      <div className="flex items-center justify-between">
        <h2 className="text-[10px] font-semibold text-cyan-400/80 uppercase tracking-[0.15em] flex items-center gap-1.5">
          <svg
            className="w-3 h-3"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
          >
            <polygon points="5 3 19 12 5 21 5 3" />
          </svg>
          Simulator
        </h2>
        <span
          className={`inline-flex items-center gap-1.5 text-[10px] font-mono uppercase tracking-wide px-1.5 py-0.5 rounded-md border ${
            control.running
              ? "bg-emerald-500/20 text-emerald-200 border-emerald-400/40"
              : "bg-slate-700/40 text-slate-300 border-slate-600/40"
          }`}
        >
          <span
            className={`w-1.5 h-1.5 rounded-full ${
              control.running
                ? "bg-emerald-400 animate-pulse-dot"
                : "bg-slate-500"
            }`}
          />
          {control.running ? "running" : "paused"}
        </span>
      </div>

      <div className="grid grid-cols-2 gap-2">
        <button
          onClick={() => onSend("start")}
          disabled={control.running}
          className="flex items-center justify-center gap-1.5 py-2 rounded-lg text-xs font-semibold transition bg-emerald-600/80 hover:bg-emerald-600 text-white disabled:opacity-40 disabled:cursor-not-allowed"
        >
          <svg className="w-3.5 h-3.5" viewBox="0 0 24 24" fill="currentColor">
            <polygon points="5 3 19 12 5 21 5 3" />
          </svg>
          Start
        </button>
        <button
          onClick={() => onSend("pause")}
          disabled={!control.running}
          className="flex items-center justify-center gap-1.5 py-2 rounded-lg text-xs font-semibold transition bg-slate-700/60 hover:bg-slate-700/80 text-white disabled:opacity-40 disabled:cursor-not-allowed"
        >
          <svg className="w-3.5 h-3.5" viewBox="0 0 24 24" fill="currentColor">
            <rect x="6" y="5" width="4" height="14" rx="1" />
            <rect x="14" y="5" width="4" height="14" rx="1" />
          </svg>
          Pause
        </button>
        <button
          onClick={() => onSend("stop")}
          className="flex items-center justify-center gap-1.5 py-2 rounded-lg text-xs font-semibold transition bg-red-600/80 hover:bg-red-600 text-white disabled:opacity-40 disabled:cursor-not-allowed"
        >
          <svg className="w-3.5 h-3.5" viewBox="0 0 24 24" fill="currentColor">
            <rect x="6" y="6" width="12" height="12" rx="1" />
          </svg>
          Stop
        </button>
        <button
          onClick={() => onSend("restart")}
          className="flex items-center justify-center gap-1.5 py-2 rounded-lg text-xs font-semibold transition bg-cyan-600/80 hover:bg-cyan-600 text-white"
        >
          <svg
            className="w-3.5 h-3.5"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2.5"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 12a9 9 0 1 1-3-6.7" />
            <path d="M21 3v6h-6" />
          </svg>
          Restart
        </button>
      </div>

      <div className="flex items-center gap-2">
        <span className="text-[10px] uppercase tracking-[0.15em] text-slate-500">
          Speed
        </span>
        <select
          value={nearestSpeed(control.speed)}
          onChange={(event) => onSpeed(Number(event.target.value))}
          className="bg-slate-900/60 border border-slate-700/40 rounded-md text-xs text-slate-100 font-mono px-2 py-1.5 flex-1 focus:outline-none focus:border-cyan-500/60"
        >
          <option value={0.1}>0.1x</option>
          <option value={0.25}>0.25x</option>
          <option value={0.5}>0.5x</option>
          <option value={1}>1x</option>
          <option value={2}>2x</option>
          <option value={4}>4x</option>
        </select>
      </div>
    </div>
  );
}

function nearestSpeed(speed: number): number {
  const speeds = [0.1, 0.25, 0.5, 1, 2, 4];
  return speeds.reduce((best, candidate) =>
    Math.abs(candidate - speed) < Math.abs(best - speed) ? candidate : best
  );
}
