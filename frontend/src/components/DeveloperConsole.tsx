import { useEffect, useMemo, useState } from "react";
import {
  SchemaRenderer,
  mergeObjectDefaults,
  type FormPart,
  type JsonObject,
  type RegistryEntry,
  type RegistryPart,
  type RendererSchema,
  type RendererValues,
  type RobotOption,
} from "@faabs/schema-renderer";
import type {
  DeveloperRequest,
  DeveloperSnapshot,
  ViewerFrame,
} from "../hooks/useViewerSocket";

interface DeveloperConsoleProps {
  developer: DeveloperSnapshot;
  frame: ViewerFrame | null;
  onRequest: (request: DeveloperRequest) => void;
  view: "dev-match" | "ai-lab";
}

interface Selection {
  kind: string;
  entry: RegistryEntry;
  form: FormPart;
}

export default function DeveloperConsole({
  developer,
  frame,
  onRequest,
  view,
}: DeveloperConsoleProps) {
  const schema = developer.schema;

  if (view === "dev-match" && schema.matchControls) {
    return (
      <DevMatchConsole
        controls={schema.matchControls}
        results={developer.results}
        onRequest={onRequest}
      />
    );
  }

  if (view === "ai-lab" && schema.matchControls) {
    return (
      <AiLabConsole
        controls={schema.matchControls}
        results={developer.results}
        frame={frame}
        onRequest={onRequest}
      />
    );
  }

  return (
    <RegistryConsole
      schema={schema}
      results={developer.results}
      frame={frame}
      onRequest={onRequest}
    />
  );
}

function RegistryConsole({
  schema,
  results,
  frame,
  onRequest,
}: {
  schema: RendererSchema;
  results: DeveloperSnapshot["results"];
  frame: ViewerFrame | null;
  onRequest: (request: DeveloperRequest) => void;
}) {
  const [modeId, setModeId] = useState(
    schema.initialModeId ?? schema.modes?.[0]?.id ?? ""
  );
  const [tabId, setTabId] = useState(
    schema.initialTabId ?? schema.tabs[0]?.id ?? ""
  );
  const [values, setValues] = useState<RendererValues>({});
  const [selection, setSelection] = useState<Selection | null>(() =>
    firstSelection(schema, tabId)
  );
  const robots = useMemo(
    () => robotOptions(frame, modeId),
    [frame?.state, modeId]
  );

  useEffect(() => {
    const nextMode = schema.modes?.some((mode) => mode.id === modeId)
      ? modeId
      : schema.initialModeId ?? schema.modes?.[0]?.id ?? "";
    if (nextMode !== modeId) setModeId(nextMode);
  }, [modeId, schema]);

  useEffect(() => {
    if (!selection || !modeId) return;
    const timer = window.setTimeout(() => {
      onRequest({
        action: "activate",
        target: modeId,
        kind: singular(selection.kind),
        entry: selection.entry.id,
        config: sectionValues(selection.form, "config", values),
        params: sectionValues(selection.form, "params", values),
      });
    }, 80);
    return () => window.clearTimeout(timer);
  }, [modeId, onRequest, selection, values]);

  const result = results[modeId];

  function selectMode(nextMode: string) {
    if (modeId && modeId !== nextMode) {
      onRequest({ action: "disable", target: modeId });
    }
    setModeId(nextMode);
  }

  function selectTab(nextTab: string) {
    setTabId(nextTab);
    const nextSelection = firstSelection(schema, nextTab);
    setSelection(nextSelection);
    if (!nextSelection && modeId) {
      onRequest({ action: "disable", target: modeId });
    }
  }

  function selectEntry(registryId: string, entryId: string) {
    setSelection(findSelection(schema, registryId, entryId));
  }

  return (
    <div className="developer-console">
      <SchemaRenderer
        schema={schema}
        values={values}
        onValuesChange={setValues}
        modeId={modeId}
        onModeChange={selectMode}
        tabId={tabId}
        onTabChange={selectTab}
        onRegistryEntryChange={selectEntry}
        robots={robots}
        density="compact"
        theme="simhark"
        className="developer-schema-renderer"
        renderFooter={() => (
          <div className="developer-footer">
            <div
              className={[
                "developer-status",
                result?.ok === false ? "developer-status-error" : "",
              ].join(" ")}
            >
              <span />
              {result?.message ?? "Select an entry to start direct drive"}
            </div>
            <button
              type="button"
              className="developer-stop"
              disabled={!modeId}
              onClick={() =>
                modeId && onRequest({ action: "disable", target: modeId })
              }
            >
              Stop direct drive
            </button>
          </div>
        )}
      />
    </div>
  );
}

function DevMatchConsole({
  controls,
  results,
  onRequest,
}: {
  controls: NonNullable<DeveloperSnapshot["schema"]["matchControls"]>;
  results: DeveloperSnapshot["results"];
  onRequest: (request: DeveloperRequest) => void;
}) {
  return (
    <div className="developer-console match-dev-console">
      <div className="match-dev-heading">
        <div>
          <span className="match-dev-live-dot" />
          Dev match
        </div>
        <small>No time limit</small>
      </div>

      <div className="match-dev-content">
        <p className="match-dev-intro">
          Swap controllers without resetting the running world or score.
        </p>
        <AiSelect
          team="blue"
          current={controls.blueAi}
          choices={controls.availableAis}
          result={results.blue}
          onRequest={onRequest}
        />
        <AiSelect
          team="yellow"
          current={controls.yellowAi}
          choices={controls.availableAis}
          result={results.yellow}
          onRequest={onRequest}
        />
        <MatchSetting controls={controls} result={results["ball-recovery"]} onRequest={onRequest} />
      </div>
    </div>
  );
}

function AiSelect({
  team,
  current,
  choices,
  result,
  onRequest,
}: {
  team: "blue" | "yellow";
  current: string | null;
  choices: Array<{ id: string; label: string }>;
  result: DeveloperSnapshot["results"][string] | undefined;
  onRequest: (request: DeveloperRequest) => void;
}) {
  return (
    <label className="match-dev-ai-row">
      <span>
        <i className={`match-dev-team-dot match-dev-team-dot-${team}`} />
        {team} AI
      </span>
      <select
        value={current ?? ""}
        disabled={!current}
        onChange={(event) =>
          onRequest({
            action: "switch_ai",
            target: team,
            ai: event.target.value,
          })
        }
      >
        {!current && <option value="">No active AI</option>}
        {choices.map((ai) => (
          <option value={ai.id} key={ai.id}>
            {ai.label}
          </option>
        ))}
      </select>
      {result && (
        <small className={result.ok ? "" : "match-dev-error"}>
          {result.message}
        </small>
      )}
    </label>
  );
}

function MatchSetting({
  controls,
  result,
  onRequest,
}: {
  controls: NonNullable<DeveloperSnapshot["schema"]["matchControls"]>;
  result: DeveloperSnapshot["results"][string] | undefined;
  onRequest: (request: DeveloperRequest) => void;
}) {
  return (
    <div className="match-dev-setting-wrap">
      <button
        type="button"
        role="switch"
        aria-checked={controls.teleportBallOnNoProgress}
        className="match-dev-setting"
        onClick={() =>
          onRequest({
            action: "set_ball_recovery",
            target: "ball-recovery",
            enabled: !controls.teleportBallOnNoProgress,
          })
        }
      >
        <span>
          <strong>Recover stalled ball</strong>
          <small>Teleport to centre after sustained no progress.</small>
        </span>
        <i className={controls.teleportBallOnNoProgress ? "match-dev-switch-on" : ""}>
          <b />
        </i>
      </button>

      {result && (
        <small className={result.ok ? "match-dev-result" : "match-dev-result match-dev-error"}>
          {result.message}
        </small>
      )}
    </div>
  );
}

function AiLabConsole({
  controls,
  results,
  frame,
  onRequest,
}: {
  controls: NonNullable<DeveloperSnapshot["schema"]["matchControls"]>;
  results: DeveloperSnapshot["results"];
  frame: ViewerFrame | null;
  onRequest: (request: DeveloperRequest) => void;
}) {
  const targets = [
    controls.blueDeveloperSchema
      ? { id: "blue" as const, label: "Blue team", schema: controls.blueDeveloperSchema }
      : null,
    controls.yellowDeveloperSchema
      ? {
          id: "yellow" as const,
          label: "Yellow team",
          schema: controls.yellowDeveloperSchema,
        }
      : null,
  ].filter((target): target is NonNullable<typeof target> => target !== null);

  if (targets.length === 0) {
    return (
      <div className="developer-console ai-lab-empty">
        <div>
          <strong>Dehumanized AI Lab</strong>
          <p>
            No team is currently using Dehumanized. Select it in the Dev Match
            tab, then return here.
          </p>
        </div>
      </div>
    );
  }

  const initial = targets[0];
  const schema: RendererSchema = {
    ...initial.schema,
    modes: targets.map(({ id, label }) => ({
      id,
      label,
      description: `Invoke against the ${label.toLowerCase()} Dehumanized controller`,
      icon: "pulse",
    })),
    initialModeId: initial.id,
  };

  return (
    <RegistryConsole
      schema={schema}
      results={results}
      frame={frame}
      onRequest={onRequest}
    />
  );
}

function firstSelection(schema: RendererSchema, tabId: string): Selection | null {
  const tab = schema.tabs.find((candidate) => candidate.id === tabId);
  if (!tab || tab.source.kind !== "inline" || tab.source.part.kind !== "registry") {
    return null;
  }
  const registry = tab.source.part;
  const entry =
    registry.entries.find((candidate) => candidate.id === registry.initialEntryId) ??
    registry.entries[0];
  return entry ? selectionFromEntry(registry, entry) : null;
}

function findSelection(
  schema: RendererSchema,
  registryId: string,
  entryId: string
): Selection | null {
  for (const tab of schema.tabs) {
    if (tab.source.kind !== "inline" || tab.source.part.kind !== "registry") {
      continue;
    }
    const registry = tab.source.part;
    if (registry.id !== registryId) continue;
    const entry = registry.entries.find((candidate) => candidate.id === entryId);
    return entry ? selectionFromEntry(registry, entry) : null;
  }
  return null;
}

function selectionFromEntry(
  registry: RegistryPart,
  entry: RegistryEntry
): Selection | null {
  if (entry.source.kind !== "inline" || entry.source.part.kind !== "form") {
    return null;
  }
  return {
    kind: registry.id,
    entry,
    form: entry.source.part,
  };
}

function sectionValues(
  form: FormPart,
  sectionId: string,
  values: RendererValues
): JsonObject {
  const section = form.sections.find((candidate) => candidate.id === sectionId);
  if (!section) return {};
  return mergeObjectDefaults(
    section.schema,
    values[form.id]?.[section.id],
    section.initialValue
  );
}

function singular(kind: string): string {
  return kind.endsWith("s") ? kind.slice(0, -1) : kind;
}

function robotOptions(frame: ViewerFrame | null, modeId: string): RobotOption[] {
  if (!frame) return [];
  const robots =
    modeId === "yellow" ? frame.state.yellow_robots : frame.state.blue_robots;
  return robots
    .filter((robot) => robot.is_on)
    .map((robot) => ({
      value: `R${robot.id}`,
      label: `Robot ${robot.id}`,
      team: "own" as const,
      detail: robot.infrared ? "ball detected" : undefined,
    }));
}
