import { useEffect, useRef, useState, useCallback } from "react";

export interface FieldConfig {
  field_length: number;
  field_width: number;
  field_line_width: number;
  field_center_radius: number;
  penalty_width: number;
  penalty_depth: number;
  margin_touch_line: number;
  margin_goal_line: number;
  goal_depth: number;
  goal_width: number;
  goal_height: number;
}

export interface BallState {
  x: number;
  y: number;
  z: number;
  vx: number;
  vy: number;
  vz: number;
}

export interface BallTrajectory {
  world_id: number;
  points: Array<{ x: number; y: number }>;
  stop_time: number;
}

export interface RobotState {
  id: number;
  team: "Blue" | "Yellow";
  x: number;
  y: number;
  z: number;
  orientation: number;
  vx: number;
  vy: number;
  vz: number;
  v_angular: number;
  infrared: boolean;
  dribbler_on: boolean;
  kick_status: "NoKick" | "FlatKick" | "ChipKick";
  is_on: boolean;
  wheel_speeds: [number, number, number, number];
}

export interface WorldState {
  world_id: number;
  sim_time: number;
  frame: number;
  ball: BallState;
  blue_robots: RobotState[];
  yellow_robots: RobotState[];
  goal_blue: boolean;
  goal_yellow: boolean;
}

export interface GameStateInfo {
  command: string;
  command_counter: number;
  stage: string | null;
  blue_name: string | null;
  yellow_name: string | null;
  state_counts: Record<string, number>;
}

export interface GoalSummary {
  blue: number;
  yellow: number;
  blue_active: boolean;
  yellow_active: boolean;
}

export interface ControlSnapshot {
  web_enabled: boolean;
  running: boolean;
  speed: number;
}

export interface ReplayStatus {
  enabled: boolean;
  frame_index: number;
  frame_count: number;
  base_speed: number;
}

export interface ReplayEvent {
  frame: number;
  sim_time: number;
  world_id: number | null;
  kind: "goal_blue" | "goal_yellow" | "foul" | "referee" | "custom";
  label: string;
  details: string | null;
}

export interface RobotInputInfo {
  world_id: number;
  team: "Blue" | "Yellow";
  id: number;
  input: string;
}

export interface TestStatus {
  world_id: number;
  path: string[];
  name: string;
  outcome: "running" | "passed" | "failed" | "timed_out";
  frame: number;
  message: string | null;
}

export interface TestSuiteSnapshot {
  passed: number;
  failed: number;
  timed_out: number;
  running: number;
  tests: TestStatus[];
}

export interface RobotDebugInfo {
  team: "Blue" | "Yellow";
  id: number;
  task: string;
  color: string;
  message: string | null;
}

export type DebugOverlay =
  | {
      kind: "holo_robot";
      team: "Blue" | "Yellow";
      id: number;
      x: number;
      y: number;
      orientation: number | null;
      color: string;
      label: string | null;
    }
  | {
      kind: "kick_line";
      team: "Blue" | "Yellow";
      id: number;
      from_x: number;
      from_y: number;
      angle: number;
      color: string;
      label: string | null;
    };

export interface ViewerDebugSnapshot {
  world_id: number;
  strategy: string | null;
  robots: RobotDebugInfo[];
  overlays?: DebugOverlay[];
}

export interface DeveloperResult {
  target: string;
  entry: string | null;
  ok: boolean;
  message: string;
}

export interface DeveloperSnapshot {
  schema: import("@dehumanized/schema-renderer").RendererSchema;
  results: Record<string, DeveloperResult>;
}

export type DeveloperRequest =
  | {
      action: "activate";
      target: string;
      kind: string;
      entry: string;
      config: import("@dehumanized/schema-renderer").JsonObject;
      params: import("@dehumanized/schema-renderer").JsonObject;
    }
  | {
      action: "disable";
      target: string;
    };

export interface ViewerFrame {
  world_count: number;
  selected_world: number;
  selected_worlds?: number[];
  field: FieldConfig;
  robot_radius: number;
  ball_radius: number;
  ball_trajectory: BallTrajectory | null;
  state: WorldState;
  states?: WorldState[];
  game_state: GameStateInfo | null;
  test_suite: TestSuiteSnapshot | null;
  goals: GoalSummary;
  control: ControlSnapshot;
  replay: ReplayStatus;
  events: ReplayEvent[];
  robot_inputs: RobotInputInfo[];
  debug?: ViewerDebugSnapshot | null;
  developer?: DeveloperSnapshot | null;
}

const RECONNECT_DELAY_MS = 1000;
const SCRUB_SEND_INTERVAL_MS = 40;

export function useViewerSocket(wsPort: number) {
  const [frame, setFrame] = useState<ViewerFrame | null>(null);
  const [connected, setConnected] = useState(false);
  const socketRef = useRef<WebSocket | null>(null);
  const reconnectTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const scrubFrameRef = useRef<number | null>(null);
  const scrubTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);

  const flushScrubReplay = useCallback(() => {
    scrubTimerRef.current = null;
    const frameIndex = scrubFrameRef.current;
    scrubFrameRef.current = null;
    const socket = socketRef.current;
    if (frameIndex === null || !socket || socket.readyState !== WebSocket.OPEN) return;
    if (socket.bufferedAmount > 0) {
      scrubFrameRef.current = frameIndex;
      scrubTimerRef.current = setTimeout(flushScrubReplay, SCRUB_SEND_INTERVAL_MS);
      return;
    }
    socket.send(`replay:seek:${frameIndex}`);
  }, []);

  const connect = useCallback(() => {
    if (socketRef.current?.readyState === WebSocket.OPEN) return;
    const protocol = window.location.protocol === "https:" ? "wss" : "ws";
    const host = window.location.hostname || "localhost";
    const url = `${protocol}://${host}:${wsPort}`;

    try {
      const socket = new WebSocket(url);
      socketRef.current = socket;

      socket.addEventListener("open", () => setConnected(true));

      socket.addEventListener("message", (event) => {
        try {
          const data: ViewerFrame = JSON.parse(event.data);
          setFrame(data);
        } catch (err) {
          console.error("failed to parse viewer frame", err);
        }
      });

      socket.addEventListener("close", () => {
        setConnected(false);
        socketRef.current = null;
        reconnectTimerRef.current = setTimeout(connect, RECONNECT_DELAY_MS);
      });

      socket.addEventListener("error", () => {
        setConnected(false);
      });
    } catch (err) {
      console.error("failed to open WebSocket", err);
      reconnectTimerRef.current = setTimeout(connect, RECONNECT_DELAY_MS);
    }
  }, [wsPort]);

  useEffect(() => {
    connect();
    return () => {
      if (reconnectTimerRef.current) clearTimeout(reconnectTimerRef.current);
      if (scrubTimerRef.current) clearTimeout(scrubTimerRef.current);
      socketRef.current?.close();
    };
  }, [connect]);

  const selectWorld = useCallback((index: number) => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(`world:${index}`);
    }
  }, []);

  const selectWorlds = useCallback((indexes: number[] | "all") => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      if (indexes === "all") {
        socket.send("worlds:all");
      } else {
        socket.send(`worlds:${indexes.join(",")}`);
      }
    }
  }, []);

  const sendControl = useCallback((action: "start" | "stop" | "restart" | "pause") => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(`control:${action}`);
    }
  }, []);

  const setSpeed = useCallback((speed: number) => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(`speed:${speed}`);
    }
  }, []);

  const moveRobot = useCallback(
    (
      worldId: number,
      team: "Blue" | "Yellow",
      id: number,
      x: number,
      y: number
    ) => {
      const socket = socketRef.current;
      if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(`robot:move:${worldId}:${team}:${id}:${x}:${y}`);
      }
    },
    []
  );

  const setRobotPresence = useCallback(
    (worldId: number, team: "Blue" | "Yellow", id: number, present: boolean) => {
      const socket = socketRef.current;
      if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(`robot:presence:${worldId}:${team}:${id}:${present}`);
      }
    },
    []
  );

  const moveBall = useCallback((worldId: number, x: number, y: number) => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(`ball:move:${worldId}:${x}:${y}`);
    }
  }, []);

  const sendDeveloperRequest = useCallback((request: DeveloperRequest) => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(`developer:${JSON.stringify(request)}`);
    }
  }, []);

  const stepReplay = useCallback((delta: number) => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(`replay:step:${delta}`);
    }
  }, []);

  const skipReplay = useCallback((deltaFrames: number) => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(`replay:skip:${Math.trunc(deltaFrames)}`);
    }
  }, []);

  const seekReplay = useCallback((frameIndex: number) => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(`replay:seek:${Math.max(0, Math.floor(frameIndex))}`);
    }
  }, []);

  const scrubReplay = useCallback(
    (frameIndex: number) => {
      scrubFrameRef.current = Math.max(0, Math.floor(frameIndex));
      if (scrubTimerRef.current) return;
      scrubTimerRef.current = setTimeout(flushScrubReplay, SCRUB_SEND_INTERVAL_MS);
    },
    [flushScrubReplay]
  );

  const flushReplayScrub = useCallback(
    (frameIndex?: number) => {
      if (scrubTimerRef.current) {
        clearTimeout(scrubTimerRef.current);
        scrubTimerRef.current = null;
      }
      scrubFrameRef.current = null;
      if (typeof frameIndex === "number") {
        seekReplay(frameIndex);
      }
    },
    [seekReplay]
  );

  return {
    frame,
    connected,
    selectWorld,
    selectWorlds,
    sendControl,
    setSpeed,
    moveRobot,
    setRobotPresence,
    moveBall,
    sendDeveloperRequest,
    stepReplay,
    skipReplay,
    seekReplay,
    scrubReplay,
    flushReplayScrub,
  };
}
