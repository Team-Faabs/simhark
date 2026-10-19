import type { ChangeEvent } from "react";
import type { TestSuiteSnapshot } from "../hooks/useViewerSocket";

interface WorldSelectorProps {
  worldCount: number;
  worldNames?: string[];
  selected: number[];
  suite: TestSuiteSnapshot | null;
  onSelect: (indexes: number[] | "all") => void;
}

export default function WorldSelector({
  worldCount,
  worldNames = [],
  selected,
  suite,
  onSelect,
}: WorldSelectorProps) {
  const safeCount = Math.max(worldCount, 1);
  const disabled = safeCount <= 1;
  const selectedSet = new Set(selected);
  const allSelected = selected.length >= safeCount;
  const failedWorlds =
    suite?.tests.filter((test) => test.outcome === "failed").map((test) => test.world_id) ?? [];
  const passedWorlds =
    suite?.tests.filter((test) => test.outcome === "passed").map((test) => test.world_id) ?? [];
  const value = selected.map(String);

  const handleChange = (event: ChangeEvent<HTMLSelectElement>) => {
    const values = Array.from(event.target.selectedOptions, (option) => option.value);
    const worlds = values.map(Number).filter(Number.isFinite);
    onSelect(worlds.length > 0 ? worlds : [0]);
  };

  return (
    <div className="px-3 py-2.5 border-b border-slate-700/30 shrink-0 overflow-hidden">
      <h2 className="text-[10px] font-semibold text-cyan-400/80 uppercase tracking-[0.15em] mb-2 flex items-center gap-1.5">
        <svg
          className="w-3 h-3"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
        >
          <rect x="3" y="3" width="7" height="7" />
          <rect x="14" y="3" width="7" height="7" />
          <rect x="3" y="14" width="7" height="7" />
          <rect x="14" y="14" width="7" height="7" />
        </svg>
        World
      </h2>
      <div className="grid grid-cols-3 gap-1.5 mb-2">
        <button
          type="button"
          onClick={() => onSelect("all")}
          disabled={disabled}
          className={quickButtonClass(allSelected)}
        >
          All
        </button>
        <button
          type="button"
          onClick={() => onSelect(failedWorlds)}
          disabled={!suite || failedWorlds.length === 0}
          className={quickButtonClass(false)}
        >
          Failed
        </button>
        <button
          type="button"
          onClick={() => onSelect(passedWorlds)}
          disabled={!suite || passedWorlds.length === 0}
          className={quickButtonClass(false)}
        >
          Passed
        </button>
      </div>
      <div className="flex items-start gap-2">
        <select
          multiple
          value={value}
          onChange={handleChange}
          disabled={disabled}
          className="h-28 min-w-0 flex-1 rounded-md border border-slate-700/40 bg-slate-900/60 px-2 py-1.5 font-mono text-xs text-slate-100 focus:border-cyan-500/60 focus:outline-none disabled:opacity-60"
        >
          {Array.from({ length: safeCount }, (_, i) => (
            <option key={i} value={i}>
              {worldNames[i] || `world ${i}`}
            </option>
          ))}
        </select>
        <span className="w-14 shrink-0 pt-1.5 text-right font-mono text-[10px] text-slate-500">
          {selectedSet.size}/{safeCount}
        </span>
      </div>
    </div>
  );
}

function quickButtonClass(active: boolean): string {
  return [
    "rounded-md border px-2 py-1.5 text-[10px] font-mono uppercase tracking-wide transition",
    active
      ? "border-cyan-400/70 bg-cyan-500/20 text-cyan-100"
      : "border-slate-700/40 bg-slate-900/50 text-slate-300 hover:bg-slate-800/70",
    "disabled:opacity-40 disabled:cursor-not-allowed disabled:hover:bg-slate-900/50",
  ].join(" ");
}
