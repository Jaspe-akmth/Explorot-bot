#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import os
import random
import threading
import time
from collections import deque
from typing import Deque, Dict, Iterable, List, Optional, Tuple

from flask import Flask, send_from_directory
from flask_socketio import SocketIO

try:
    from gpiozero import LineSensor
except Exception:
    LineSensor = None

import robot_utils

HOST = "0.0.0.0"
PORT = 8765

WORLD_W = 4.0
WORLD_H = 4.0

LINEAR_SPEED = 0.15      # m/s estimate from odometry integration
ANGULAR_SPEED = 1.22     # rad/s estimate from odometry integration
MAX_SENSOR_RANGE = 0.50  # ultrasonic useful range
MANUAL_SPEED = 100
ALGO_SPEED = 100

# Optional line sensors (very short range / binary cues).
LINE_SENSOR_PINS = {"left": 19, "center": 16, "right": 20}
LINE_SENSOR_RELATIVE_ANGLE = {
    "left": -math.radians(34.0),
    "center": 0.0,
    "right": math.radians(34.0),
}
LINE_SENSOR_MARK_RANGE_M = 0.07
LINE_SENSOR_YAW_CORR_RAD_S = math.radians(18.0)
USE_LINE_SENSORS_FOR_MAP = True

# Occupancy grid on the server side, coarse on purpose because ultrasonic is noisy.
CELL_M = 0.10
GRID_W = int(math.ceil(WORLD_W / CELL_M)) + 1
GRID_H = int(math.ceil(WORLD_H / CELL_M)) + 1

# Log-odds occupancy parameters.
# Free evidence is intentionally stronger so that one clear scan already opens a traversable corridor.
LOG_ODDS_MIN = -4.0
LOG_ODDS_MAX = 4.0
LOG_ODDS_FREE = -0.85
LOG_ODDS_OCC = 1.30
UNKNOWN_BAND_LOW = 0.47
UNKNOWN_BAND_HIGH = 0.62

# Simple robot geometry / safety.
ROBOT_RADIUS_M = 0.10
PATH_OBSTACLE_MARGIN_CELLS = 0
SAFE_FRONT_CLEARANCE_M = 0.22
SLOW_FRONT_CLEARANCE_M = 0.30
WAYPOINT_REACHED_M = 0.07
ANGLE_TOL = math.radians(12.0)

# Exploration pacing. Continuous motion like the earlier random walk; speed is lowered slightly while an algorithm runs.
INITIAL_SCAN_ANGLE = math.radians(210.0)
LOCAL_SCAN_ANGLE = math.radians(55.0)
BLOCKED_SCAN_ANGLE = math.radians(90.0)
SEARCH_SWEEP_ANGLE = math.radians(340.0)
SEARCH_MOVE_DURATION_S = 1.25
SEARCH_MIN_SCORE = 0.10
WAYPOINTS_BEFORE_RESCAN = 8
WALL_FOLLOW_DURATION_S = 2.6
AVOID_TURN_BASE_S = 0.55
AVOID_TURN_CORNER_BONUS_S = 0.22
BACKOUT_DURATION_S = 0.60
DANGER_FRONT_CLEARANCE_M = 0.14

DEFAULT_EXPLORATION_CONFIG = {
    "safe_front_clearance_m": SAFE_FRONT_CLEARANCE_M,
    "slow_front_clearance_m": SLOW_FRONT_CLEARANCE_M,
    "danger_front_clearance_m": DANGER_FRONT_CLEARANCE_M,
    "algo_speed": ALGO_SPEED,
    "wall_follow_duration_s": WALL_FOLLOW_DURATION_S,
    "avoid_turn_base_s": AVOID_TURN_BASE_S,
    "backout_duration_s": BACKOUT_DURATION_S,
}

app = Flask(__name__)
app.config["SECRET_KEY"] = "secret!"
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode="threading",
    logger=True,
    engineio_logger=True,
)

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

robot_lock = threading.Lock()
robot: Optional[robot_utils.Robot] = None
background_started = False
line_sensors = {"left": None, "center": None, "right": None}

pose = {
    "x": WORLD_W / 2,
    "y": WORLD_H / 2,
    "angle": 0.0,
}

motion_state = {
    "mode": "stop",
    "updated_at": time.time(),
}

state = {
    "robot_available": False,
    "sensor_available": False,
    "line_sensor_available": False,
    "line_sensors": {"left": False, "center": False, "right": False},
    "distance_m": None,
    "last_error": None,
}

# Server-side map used by the exploration algorithms.
occupancy: Dict[Tuple[int, int], float] = {}

exploration = {
    "enabled": False,
    "mode": "idle",          # idle, random, bfs, dfs
    "phase": "idle",         # idle, scan, plan, turn, move, move_pause, blocked
    "scan_anchor": None,
    "scan_accum": 0.0,
    "scan_prev_angle": None,
    "scan_goal": 0.0,
    "did_initial_scan": False,
    "path": [],
    "path_index": 0,
    "waypoint": None,
    "target": None,
    "turn_dir": None,
    "turn_until": 0.0,
    "move_until": 0.0,
    "pause_until": 0.0,
    "waypoints_since_scan": 0,
    "nudge_until": 0.0,
    "no_path_count": 0,
    "recovery_side": None,
    "wall_follow_until": 0.0,
    "blocked_context": None,
    "blocked_attempts": 0,
    "blocked_resume_phase": "move",
    "backup_until": 0.0,
    "backup_count": 0,
    "search_best_angle": None,
    "search_best_score": -1e9,
    "search_target_angle": None,
    "search_move_until": 0.0,
    "search_reason": None,
    "stats": {
        "visited_frontiers": 0,
        "replans": 0,
    },
}


@app.route("/", methods=["GET"])
def index():
    return send_from_directory(BASE_DIR, "frontend_robot.html")


@app.route("/simulation", methods=["GET"])
def simulation_page():
    for filename in ("simulator_exploration_demo_v9.html", "simulator_exploration_demo_v8.html", "simulator_exploration_demo.html"):
        path = os.path.join(BASE_DIR, filename)
        if os.path.exists(path):
            return send_from_directory(BASE_DIR, filename)
    return "simulateur introuvable", 404


def current_exploration_config() -> Dict[str, float]:
    return {
        "safe_front_clearance_m": SAFE_FRONT_CLEARANCE_M,
        "slow_front_clearance_m": SLOW_FRONT_CLEARANCE_M,
        "danger_front_clearance_m": DANGER_FRONT_CLEARANCE_M,
        "algo_speed": ALGO_SPEED,
        "wall_follow_duration_s": WALL_FOLLOW_DURATION_S,
        "avoid_turn_base_s": AVOID_TURN_BASE_S,
        "backout_duration_s": BACKOUT_DURATION_S,
    }


def emit_exploration_config() -> None:
    socketio.emit("explorationConfig", current_exploration_config())


def apply_exploration_config(data: Optional[Dict]) -> Dict[str, float]:
    global SAFE_FRONT_CLEARANCE_M, SLOW_FRONT_CLEARANCE_M, DANGER_FRONT_CLEARANCE_M
    global ALGO_SPEED, WALL_FOLLOW_DURATION_S, AVOID_TURN_BASE_S, BACKOUT_DURATION_S

    data = data or {}

    def _num(name: str, fallback: float) -> float:
        raw = data.get(name, fallback)
        try:
            return float(raw)
        except Exception:
            return float(fallback)

    SAFE_FRONT_CLEARANCE_M = clamp(_num("safe_front_clearance_m", SAFE_FRONT_CLEARANCE_M), 0.03, 0.25)
    SLOW_FRONT_CLEARANCE_M = clamp(_num("slow_front_clearance_m", SLOW_FRONT_CLEARANCE_M), SAFE_FRONT_CLEARANCE_M + 0.01, 0.45)
    DANGER_FRONT_CLEARANCE_M = clamp(_num("danger_front_clearance_m", DANGER_FRONT_CLEARANCE_M), 0.02, SAFE_FRONT_CLEARANCE_M)
    ALGO_SPEED = int(clamp(_num("algo_speed", ALGO_SPEED), 40, 100))
    WALL_FOLLOW_DURATION_S = clamp(_num("wall_follow_duration_s", WALL_FOLLOW_DURATION_S), 0.6, 8.0)
    AVOID_TURN_BASE_S = clamp(_num("avoid_turn_base_s", AVOID_TURN_BASE_S), 0.12, 1.8)
    BACKOUT_DURATION_S = clamp(_num("backout_duration_s", BACKOUT_DURATION_S), 0.12, 1.8)

    if robot is not None:
        if exploration.get("enabled"):
            set_algo_speed()
        else:
            set_manual_speed()

    return current_exploration_config()


def reset_exploration_config_values() -> Dict[str, float]:
    return apply_exploration_config(DEFAULT_EXPLORATION_CONFIG)


def clamp(v: float, a: float, b: float) -> float:
    return max(a, min(b, v))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def signed_angle_diff(target: float, current: float) -> float:
    return normalize_angle(target - current)


def apply_robot_speed(speed: int) -> None:
    global robot
    if robot is None:
        return
    try:
        robot.speed = int(clamp(speed, 0, 100))
    except Exception:
        pass


def set_manual_speed() -> None:
    apply_robot_speed(MANUAL_SPEED)


def set_algo_speed() -> None:
    apply_robot_speed(ALGO_SPEED)


def set_motion(mode: str) -> None:
    motion_state["mode"] = mode
    motion_state["updated_at"] = time.time()


def update_pose(dt: float) -> None:
    mode = motion_state["mode"]

    if mode == "forward":
        ls = state.get("line_sensors", {})
        if ls.get("left") and not ls.get("right"):
            pose["angle"] += LINE_SENSOR_YAW_CORR_RAD_S * dt
        elif ls.get("right") and not ls.get("left"):
            pose["angle"] -= LINE_SENSOR_YAW_CORR_RAD_S * dt
        pose["x"] += math.sin(pose["angle"]) * LINEAR_SPEED * dt
        pose["y"] += math.cos(pose["angle"]) * LINEAR_SPEED * dt
    elif mode == "backward":
        pose["x"] -= math.sin(pose["angle"]) * LINEAR_SPEED * dt
        pose["y"] -= math.cos(pose["angle"]) * LINEAR_SPEED * dt
    elif mode == "left":
        pose["angle"] -= ANGULAR_SPEED * dt
    elif mode == "right":
        pose["angle"] += ANGULAR_SPEED * dt

    pose["angle"] = normalize_angle(pose["angle"])
    pose["x"] = clamp(float(pose["x"]), 0.0, WORLD_W)
    pose["y"] = clamp(float(pose["y"]), 0.0, WORLD_H)


def get_sensor_distance_safe() -> Optional[float]:
    global robot
    if robot is None:
        return None

    try:
        distance = robot.get_sensor_distance()
        if distance is None:
            return None
        return float(distance)
    except Exception:
        return None


def setup_line_sensors() -> bool:
    global line_sensors
    if LineSensor is None:
        return False

    ok = False
    for name, pin in LINE_SENSOR_PINS.items():
        if line_sensors.get(name) is not None:
            ok = True
            continue
        try:
            line_sensors[name] = LineSensor(pin)
            ok = True
        except Exception:
            line_sensors[name] = None
    return ok


def close_line_sensors() -> None:
    global line_sensors
    for name, sensor in list(line_sensors.items()):
        if sensor is None:
            continue
        try:
            sensor.close()
        except Exception:
            pass
        line_sensors[name] = None


def read_line_sensors_safe() -> Optional[Dict[str, bool]]:
    if all(sensor is None for sensor in line_sensors.values()):
        return None

    readings: Dict[str, bool] = {}
    got_any = False
    for name, sensor in line_sensors.items():
        if sensor is None:
            readings[name] = False
            continue
        try:
            readings[name] = bool(sensor.line_detected)
            got_any = True
        except Exception:
            readings[name] = False
    return readings if got_any else None


def line_bias_radians() -> float:
    ls = state.get("line_sensors", {})
    bias = 0.0
    if ls.get("left") and not ls.get("right"):
        bias += math.radians(12.0)
    if ls.get("right") and not ls.get("left"):
        bias -= math.radians(12.0)
    if ls.get("center"):
        if ls.get("left") and not ls.get("right"):
            bias += math.radians(8.0)
        elif ls.get("right") and not ls.get("left"):
            bias -= math.radians(8.0)
    return clamp(bias, -math.radians(20.0), math.radians(20.0))


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def sigmoid(x: float) -> float:
    if x >= 0:
        z = math.exp(-x)
        return 1.0 / (1.0 + z)
    z = math.exp(x)
    return z / (1.0 + z)


def world_to_grid(x: float, y: float) -> Tuple[int, int]:
    gx = int(math.floor(clamp(x, 0.0, WORLD_W - 1e-9) / CELL_M))
    gy = int(math.floor(clamp(y, 0.0, WORLD_H - 1e-9) / CELL_M))
    return gx, gy


def grid_to_world(gx: int, gy: int) -> Tuple[float, float]:
    return gx * CELL_M, gy * CELL_M


def in_bounds(gx: int, gy: int) -> bool:
    return 0 <= gx < GRID_W and 0 <= gy < GRID_H


def log_odds_to_prob(lo: float) -> float:
    return sigmoid(lo)


def get_log_odds(gx: int, gy: int) -> float:
    return occupancy.get((gx, gy), 0.0)


def get_prob(gx: int, gy: int) -> float:
    return log_odds_to_prob(get_log_odds(gx, gy))


def set_log_odds(gx: int, gy: int, delta: float) -> None:
    if not in_bounds(gx, gy):
        return
    key = (gx, gy)
    occupancy[key] = clamp(occupancy.get(key, 0.0) + delta, LOG_ODDS_MIN, LOG_ODDS_MAX)


def classify_cell(gx: int, gy: int) -> str:
    p = get_prob(gx, gy)
    if p >= UNKNOWN_BAND_HIGH:
        return "occupied"
    if p <= UNKNOWN_BAND_LOW:
        return "free"
    return "unknown"


def neighbors4(gx: int, gy: int) -> Iterable[Tuple[int, int]]:
    yield gx + 1, gy
    yield gx - 1, gy
    yield gx, gy + 1
    yield gx, gy - 1


def neighbors8(gx: int, gy: int) -> Iterable[Tuple[int, int]]:
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            yield gx + dx, gy + dy


def is_walkable(gx: int, gy: int) -> bool:
    if not in_bounds(gx, gy):
        return False
    if classify_cell(gx, gy) != "free":
        return False

    for dx in range(-PATH_OBSTACLE_MARGIN_CELLS, PATH_OBSTACLE_MARGIN_CELLS + 1):
        for dy in range(-PATH_OBSTACLE_MARGIN_CELLS, PATH_OBSTACLE_MARGIN_CELLS + 1):
            nx, ny = gx + dx, gy + dy
            if not in_bounds(nx, ny):
                return False
            if classify_cell(nx, ny) == "occupied":
                return False
    return True


def is_frontier(gx: int, gy: int) -> bool:
    if classify_cell(gx, gy) != "free":
        return False
    for nx, ny in neighbors4(gx, gy):
        if in_bounds(nx, ny) and classify_cell(nx, ny) == "unknown":
            return True
    return False


def mark_robot_footprint_free() -> None:
    rgx, rgy = world_to_grid(pose["x"], pose["y"])
    rad = max(1, int(math.ceil(ROBOT_RADIUS_M / CELL_M)))
    for dx in range(-rad, rad + 1):
        for dy in range(-rad, rad + 1):
            gx, gy = rgx + dx, rgy + dy
            if in_bounds(gx, gy):
                set_log_odds(gx, gy, LOG_ODDS_FREE * 0.8)


def update_occupancy_from_sensor(distance_m: Optional[float]) -> None:
    if distance_m is None:
        return

    dist = clamp(float(distance_m), 0.0, MAX_SENSOR_RANGE)
    step = CELL_M * 0.5
    hit = dist < MAX_SENSOR_RANGE - 0.02
    angle = pose["angle"]

    mark_robot_footprint_free()

    r = step
    while r < dist:
        x = pose["x"] + math.sin(angle) * r
        y = pose["y"] + math.cos(angle) * r
        gx, gy = world_to_grid(x, y)
        if not in_bounds(gx, gy):
            break
        set_log_odds(gx, gy, LOG_ODDS_FREE)
        r += step

    if hit:
        ox = pose["x"] + math.sin(angle) * dist
        oy = pose["y"] + math.cos(angle) * dist
        gx, gy = world_to_grid(ox, oy)
        weights = {
            (0, 0): 1.00,
            (1, 0): 0.65, (-1, 0): 0.65,
            (0, 1): 0.65, (0, -1): 0.65,
            (1, 1): 0.40, (1, -1): 0.40,
            (-1, 1): 0.40, (-1, -1): 0.40,
        }
        for (dx, dy), w in weights.items():
            set_log_odds(gx + dx, gy + dy, LOG_ODDS_OCC * w)


def update_occupancy_from_line_sensors(readings: Optional[Dict[str, bool]]) -> None:
    if not USE_LINE_SENSORS_FOR_MAP or not readings:
        return

    mark_robot_footprint_free()
    weights = {
        (0, 0): 1.00,
        (1, 0): 0.70, (-1, 0): 0.70,
        (0, 1): 0.70, (0, -1): 0.70,
    }

    for name, active in readings.items():
        if not active:
            continue
        rel = LINE_SENSOR_RELATIVE_ANGLE[name]
        ang = pose["angle"] + rel
        ox = pose["x"] + math.sin(ang) * LINE_SENSOR_MARK_RANGE_M
        oy = pose["y"] + math.cos(ang) * LINE_SENSOR_MARK_RANGE_M
        gx, gy = world_to_grid(ox, oy)
        for (dx, dy), w in weights.items():
            set_log_odds(gx + dx, gy + dy, LOG_ODDS_OCC * 0.9 * w)



def path_length_from_parent(parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]], end: Tuple[int, int]) -> int:
    n = 0
    cur: Optional[Tuple[int, int]] = end
    while cur is not None:
        n += 1
        cur = parent.get(cur)
    return n


def angle_to_cell(cell: Tuple[int, int]) -> float:
    wx0, wy0 = grid_to_world(*cell)
    wx = wx0 + CELL_M * 0.5
    wy = wy0 + CELL_M * 0.5
    return math.atan2(wx - pose["x"], wy - pose["y"])


def frontier_candidate_score(
    parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]],
    cell: Tuple[int, int],
) -> float:
    path_len = path_length_from_parent(parent, cell)
    unknown_neighbors = sum(
        1 for nx, ny in neighbors8(*cell)
        if in_bounds(nx, ny) and classify_cell(nx, ny) == "unknown"
    )
    view_bonus = heading_unknown_bonus(angle_to_cell(cell), max_range=0.55)
    return unknown_neighbors * 10.0 + view_bonus * 8.0 - path_len


def scout_candidate_score(
    parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]],
    cell: Tuple[int, int],
) -> float:
    path_len = path_length_from_parent(parent, cell)
    unknown_neighbors = sum(
        1 for nx, ny in neighbors8(*cell)
        if in_bounds(nx, ny) and classify_cell(nx, ny) == "unknown"
    )
    free_neighbors = sum(
        1 for nx, ny in neighbors8(*cell)
        if in_bounds(nx, ny) and classify_cell(nx, ny) == "free"
    )
    view_bonus = heading_unknown_bonus(angle_to_cell(cell), max_range=0.70)
    return unknown_neighbors * 9.0 + free_neighbors * 0.6 + view_bonus * 10.0 - path_len * 0.85


def find_path_to_frontier(start: Tuple[int, int], strategy: str) -> List[Tuple[int, int]]:
    best_cell: Optional[Tuple[int, int]] = None
    best_score = -1e9
    if not in_bounds(*start):
        return []
    if classify_cell(*start) == "unknown":
        # Make the robot's own cell definitely free.
        set_log_odds(start[0], start[1], LOG_ODDS_FREE * 2.0)
    if not is_walkable(*start):
        return []

    parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
    visited = {start}

    if strategy == "bfs":
        fringe: Deque[Tuple[int, int]] = deque([start])
        while fringe:
            cell = fringe.popleft()
            if cell != start and is_frontier(*cell):
                score = frontier_candidate_score(parent, cell)
                if score > best_score:
                    best_score = score
                    best_cell = cell
            for nxt in ordered_neighbors(cell):
                if nxt not in visited and is_walkable(*nxt):
                    visited.add(nxt)
                    parent[nxt] = cell
                    fringe.append(nxt)
    else:
        fringe2: List[Tuple[int, int]] = [start]
        while fringe2:
            cell = fringe2.pop()
            if cell != start and is_frontier(*cell):
                score = frontier_candidate_score(parent, cell)
                if score > best_score:
                    best_score = score
                    best_cell = cell
                continue
            # Reverse order so the first logical neighbor is explored deeper first.
            nbs = list(ordered_neighbors(cell))
            nbs.reverse()
            for nxt in nbs:
                if nxt not in visited and is_walkable(*nxt):
                    visited.add(nxt)
                    parent[nxt] = cell
                    fringe2.append(nxt)
    if best_cell is not None:
        return reconstruct_path(parent, best_cell)
    return []


def find_path_to_promising_free_cell(start: Tuple[int, int], strategy: str) -> List[Tuple[int, int]]:
    if not in_bounds(*start) or not is_walkable(*start):
        return []

    parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start: None}
    visited = {start}
    best_cell: Optional[Tuple[int, int]] = None
    best_score = -1e9

    def consider(cell: Tuple[int, int]) -> None:
        nonlocal best_cell, best_score
        if cell == start:
            return
        score = scout_candidate_score(parent, cell)
        if score > best_score:
            best_score = score
            best_cell = cell

    if strategy == "bfs":
        fringe: Deque[Tuple[int, int]] = deque([start])
        while fringe:
            cell = fringe.popleft()
            consider(cell)
            for nxt in ordered_neighbors(cell):
                if nxt not in visited and is_walkable(*nxt):
                    visited.add(nxt)
                    parent[nxt] = cell
                    fringe.append(nxt)
    else:
        fringe2: List[Tuple[int, int]] = [start]
        while fringe2:
            cell = fringe2.pop()
            consider(cell)
            nbs = list(ordered_neighbors(cell))
            nbs.reverse()
            for nxt in nbs:
                if nxt not in visited and is_walkable(*nxt):
                    visited.add(nxt)
                    parent[nxt] = cell
                    fringe2.append(nxt)

    if best_cell is not None and best_score > SEARCH_MIN_SCORE:
        return reconstruct_path(parent, best_cell)
    return []


def reconstruct_path(parent: Dict[Tuple[int, int], Optional[Tuple[int, int]]], end: Tuple[int, int]) -> List[Tuple[int, int]]:
    path = [end]
    cur = end
    while parent[cur] is not None:
        cur = parent[cur]
        path.append(cur)
    path.reverse()
    return path


def ordered_neighbors(cell: Tuple[int, int]) -> Iterable[Tuple[int, int]]:
    gx, gy = cell
    # Bias the search toward the robot's current heading to reduce dithering.
    preferred = heading_preference_order()
    for dx, dy in preferred:
        nxt = gx + dx, gy + dy
        if in_bounds(*nxt):
            yield nxt


def heading_preference_order() -> List[Tuple[int, int]]:
    ang = pose["angle"]
    vx = math.sin(ang)
    vy = math.cos(ang)
    dirs = [
        (1, 0), (-1, 0), (0, 1), (0, -1),
    ]
    dirs.sort(key=lambda d: -(d[0] * vx + d[1] * vy))
    return dirs


def heading_unknown_bonus(angle: float, max_range: float = 0.45) -> float:
    score = 0.0
    seen = set()
    r = CELL_M
    while r <= max_range:
        x = pose["x"] + math.sin(angle) * r
        y = pose["y"] + math.cos(angle) * r
        gx, gy = world_to_grid(x, y)
        if not in_bounds(gx, gy):
            break
        if (gx, gy) in seen:
            r += CELL_M
            continue
        seen.add((gx, gy))
        cls = classify_cell(gx, gy)
        if cls == "unknown":
            score += 0.12
        elif cls == "free":
            score -= 0.005
        else:
            score -= 0.05
            break
        r += CELL_M
    return score


def current_search_heading_score() -> float:
    front = state.get("distance_m")
    ls = state.get("line_sensors", {})
    clearance = MAX_SENSOR_RANGE * 0.85 if front is None else float(front)
    score = clearance * 1.15 + 3.0 * heading_unknown_bonus(pose["angle"])
    if ls.get("center"):
        score -= 0.35
    if ls.get("left") and ls.get("right"):
        score -= 0.12
    return score


def start_search_sweep(reason: str = "explore") -> None:
    exploration["phase"] = "search_sweep"
    exploration["scan_anchor"] = pose["angle"]
    exploration["scan_prev_angle"] = pose["angle"]
    exploration["scan_accum"] = 0.0
    exploration["scan_goal"] = SEARCH_SWEEP_ANGLE
    exploration["search_best_angle"] = pose["angle"]
    exploration["search_best_score"] = current_search_heading_score()
    exploration["search_target_angle"] = None
    exploration["search_move_until"] = 0.0
    exploration["search_reason"] = reason
    command_right()


def handle_search_sweep_phase() -> bool:
    phase = exploration.get("phase")
    if phase not in {"search_sweep", "search_turn", "search_move"}:
        return False

    if phase == "search_sweep":
        prev = exploration.get("scan_prev_angle")
        cur = pose["angle"]
        if prev is None:
            exploration["scan_prev_angle"] = cur
        else:
            exploration["scan_accum"] += abs(signed_angle_diff(cur, prev))
            exploration["scan_prev_angle"] = cur

        score = current_search_heading_score()
        if score > float(exploration.get("search_best_score", -1e9)):
            exploration["search_best_score"] = score
            exploration["search_best_angle"] = cur

        if exploration["scan_accum"] >= exploration.get("scan_goal", SEARCH_SWEEP_ANGLE):
            command_stop()
            exploration["search_target_angle"] = exploration.get("search_best_angle", pose["angle"])
            exploration["phase"] = "search_turn"
            return True

        command_right()
        return True

    if phase == "search_turn":
        target_angle = exploration.get("search_target_angle")
        if target_angle is None:
            target_angle = exploration.get("search_best_angle", pose["angle"])
            exploration["search_target_angle"] = target_angle

        diff = signed_angle_diff(target_angle, pose["angle"])
        if abs(diff) <= ANGLE_TOL * 0.75:
            exploration["search_move_until"] = time.time() + SEARCH_MOVE_DURATION_S
            exploration["phase"] = "search_move"
            command_forward()
            return True

        if diff < 0:
            command_left()
        else:
            command_right()
        return True

    # search_move
    if immediate_blocked(0.005):
        start_blocked_recovery(exploration.get("mode") or "explore", resume_phase="search_move")
        return True

    if time.time() < exploration.get("search_move_until", 0.0):
        command_forward()
        return True

    command_stop()
    exploration["no_path_count"] = 0
    exploration["search_reason"] = None
    if exploration.get("mode") == "random":
        exploration["phase"] = "move"
    else:
        start_scan(LOCAL_SCAN_ANGLE)
    return True


def immediate_blocked(front_pad: float = 0.0) -> bool:
    front = state.get("distance_m")
    ls = state.get("line_sensors", {})
    if ls.get("center"):
        return True
    if front is not None and front < SAFE_FRONT_CLEARANCE_M + front_pad:
        return True
    return False


def pick_avoid_turn(preferred: Optional[str] = None) -> str:
    ls = state.get("line_sensors", {})
    front = state.get("distance_m")

    if preferred in {"left", "right"}:
        return preferred
    if ls.get("left") and not ls.get("right"):
        return "right"
    if ls.get("right") and not ls.get("left"):
        return "left"

    prev = exploration.get("turn_dir")
    if prev == "left":
        return "right"
    if prev == "right":
        return "left"

    # If we are very close and the side sensors give no clue, pick a deterministic side first.
    if front is not None and front < DANGER_FRONT_CLEARANCE_M:
        return "right"
    return random.choice(["left", "right"])


def start_blocked_recovery(context: str, preferred_dir: Optional[str] = None, resume_phase: str = "move") -> None:
    now = time.time()
    ls = state.get("line_sensors", {})
    front = state.get("distance_m")

    exploration["phase"] = "blocked"
    exploration["blocked_context"] = context
    exploration["blocked_resume_phase"] = resume_phase
    exploration["blocked_attempts"] = int(exploration.get("blocked_attempts", 0)) + 1

    turn_dir = pick_avoid_turn(preferred_dir)
    exploration["turn_dir"] = turn_dir

    extra = 0.0
    if ls.get("center") and (ls.get("left") or ls.get("right")):
        extra += AVOID_TURN_CORNER_BONUS_S
    if front is not None and front < DANGER_FRONT_CLEARANCE_M:
        extra += 0.12

    exploration["turn_until"] = now + AVOID_TURN_BASE_S + extra
    if turn_dir == "left":
        command_left()
    else:
        command_right()


def start_backup(context: str, resume_phase: str = "move", duration_s: Optional[float] = None) -> None:
    duration = BACKOUT_DURATION_S if duration_s is None else float(duration_s)
    exploration["phase"] = "backout"
    exploration["blocked_context"] = context
    exploration["blocked_resume_phase"] = resume_phase
    exploration["backup_count"] = int(exploration.get("backup_count", 0)) + 1
    exploration["backup_until"] = time.time() + duration
    command_backward()


def handle_blocked_phase() -> bool:
    if exploration.get("phase") != "blocked":
        return False

    now = time.time()
    if now < exploration.get("turn_until", 0.0):
        if exploration.get("turn_dir") == "left":
            command_left()
        else:
            command_right()
        return True

    if immediate_blocked(0.01):
        attempts = int(exploration.get("blocked_attempts", 0))
        if attempts < 2:
            other = "right" if exploration.get("turn_dir") == "left" else "left"
            start_blocked_recovery(
                exploration.get("blocked_context") or exploration.get("mode") or "random",
                preferred_dir=other,
                resume_phase=exploration.get("blocked_resume_phase") or "move",
            )
            return True

        if int(exploration.get("backup_count", 0)) < 1:
            start_backup(
                exploration.get("blocked_context") or exploration.get("mode") or "random",
                resume_phase=exploration.get("blocked_resume_phase") or "move",
            )
            return True

        # Still stuck after both turns + one reverse: escalate.
        exploration["blocked_attempts"] = 0
        exploration["backup_count"] = 0
        if (exploration.get("blocked_context") or exploration.get("mode")) == "random":
            start_wall_follow()
        else:
            start_wall_follow()
        return True

    exploration["blocked_attempts"] = 0
    exploration["backup_count"] = 0
    resume = exploration.get("blocked_resume_phase") or "move"
    if resume == "move":
        command_forward()
        exploration["phase"] = "move"
    else:
        exploration["phase"] = resume
    return True


def handle_backout_phase() -> bool:
    if exploration.get("phase") != "backout":
        return False

    if time.time() < exploration.get("backup_until", 0.0):
        command_backward()
        return True

    command_stop()
    exploration["blocked_attempts"] = 0
    preferred = pick_avoid_turn()
    start_blocked_recovery(
        exploration.get("blocked_context") or exploration.get("mode") or "random",
        preferred_dir=preferred,
        resume_phase=exploration.get("blocked_resume_phase") or "move",
    )
    return True


def choose_random_turn() -> None:
    start_blocked_recovery("random", resume_phase="move")


def choose_wall_follow_side() -> str:
    ls = state.get("line_sensors", {})
    if ls.get("left") and not ls.get("right"):
        return "left"
    if ls.get("right") and not ls.get("left"):
        return "right"
    return random.choice(["left", "right"])


def start_wall_follow(side: Optional[str] = None, duration_s: Optional[float] = None) -> None:
    duration = WALL_FOLLOW_DURATION_S if duration_s is None else float(duration_s)
    exploration["phase"] = "wall_follow"
    exploration["recovery_side"] = side or choose_wall_follow_side()
    exploration["wall_follow_until"] = time.time() + duration
    exploration["move_until"] = 0.0
    exploration["pause_until"] = 0.0
    exploration["path"] = []
    exploration["path_index"] = 0
    exploration["waypoint"] = None
    exploration["target"] = None


def step_wall_follow() -> None:
    side = exploration.get("recovery_side") or "left"
    now = time.time()
    ls = state.get("line_sensors", {})
    front = state.get("distance_m")

    if now >= exploration.get("wall_follow_until", 0.0):
        command_stop()
        if exploration.get("mode") == "random":
            start_search_sweep("wall_follow_exit")
        else:
            start_search_sweep("wall_follow_exit")
        return

    if ls.get("center") or (front is not None and front < SAFE_FRONT_CLEARANCE_M):
        if side == "left":
            command_right()
        else:
            command_left()
        return

    # If the chosen wall disappears, gently steer toward it to reacquire structure.
    if side == "left" and not ls.get("left"):
        command_left()
        return
    if side == "right" and not ls.get("right"):
        command_right()
        return

    # If the opposite side suddenly fires, move away from it.
    if side == "left" and ls.get("right") and not ls.get("left"):
        command_left()
        return
    if side == "right" and ls.get("left") and not ls.get("right"):
        command_right()
        return

    command_forward()
    exploration["phase"] = "move"


def start_scan(goal_angle: Optional[float] = None) -> None:
    if goal_angle is None:
        goal_angle = INITIAL_SCAN_ANGLE if not exploration["did_initial_scan"] else LOCAL_SCAN_ANGLE
    exploration["phase"] = "scan"
    exploration["scan_anchor"] = pose["angle"]
    exploration["scan_prev_angle"] = pose["angle"]
    exploration["scan_accum"] = 0.0
    exploration["scan_goal"] = goal_angle
    exploration["move_until"] = 0.0
    exploration["pause_until"] = 0.0
    set_motion("right")
    if robot is not None:
        robot.droite()


def handle_scan_phase() -> None:
    prev = exploration["scan_prev_angle"]
    cur = pose["angle"]
    if prev is None:
        exploration["scan_prev_angle"] = cur
        return

    delta = abs(signed_angle_diff(cur, prev))
    exploration["scan_accum"] += delta
    exploration["scan_prev_angle"] = cur

    if exploration["scan_accum"] >= exploration.get("scan_goal", LOCAL_SCAN_ANGLE):
        command_stop()
        exploration["phase"] = "plan"
        exploration["did_initial_scan"] = True


def command_stop() -> None:
    if motion_state["mode"] == "stop":
        return
    set_motion("stop")
    if robot is not None:
        robot.stop()


def command_forward() -> None:
    if motion_state["mode"] == "forward":
        return
    set_motion("forward")
    if robot is not None:
        robot.avance()


def command_left() -> None:
    if motion_state["mode"] == "left":
        return
    set_motion("left")
    if robot is not None:
        robot.gauche()


def command_right() -> None:
    if motion_state["mode"] == "right":
        return
    set_motion("right")
    if robot is not None:
        robot.droite()


def command_backward() -> None:
    if motion_state["mode"] == "backward":
        return
    set_motion("backward")
    if robot is not None:
        robot.recule()


def begin_next_waypoint() -> None:
    path = exploration["path"]
    idx = exploration["path_index"]
    if not path or idx >= len(path):
        exploration["waypoint"] = None
        exploration["target"] = None
        exploration["phase"] = "plan"
        return
    exploration["waypoint"] = path[idx]
    exploration["target"] = path[-1]
    exploration["phase"] = "turn"


def plan_frontier_path(strategy: str) -> None:
    start = world_to_grid(pose["x"], pose["y"])
    path = find_path_to_frontier(start, strategy)
    exploration["stats"]["replans"] += 1

    if len(path) < 2:
        scout_path = find_path_to_promising_free_cell(start, strategy)
        if len(scout_path) >= 2:
            exploration["no_path_count"] = max(0, int(exploration.get("no_path_count", 0)) - 1)
            exploration["search_reason"] = None
            exploration["path"] = scout_path
            exploration["path_index"] = 1
            exploration["waypoint"] = scout_path[1]
            exploration["target"] = scout_path[-1]
            exploration["waypoints_since_scan"] = 0
            exploration["move_until"] = 0.0
            exploration["pause_until"] = 0.0
            exploration["phase"] = "turn"
            return

        exploration["path"] = []
        exploration["path_index"] = 0
        exploration["waypoint"] = None
        exploration["target"] = None
        exploration["no_path_count"] += 1

        front = state["distance_m"]
        ls = state.get("line_sensors", {})

        if exploration["no_path_count"] == 1:
            start_scan(BLOCKED_SCAN_ANGLE)
            return

        if exploration["no_path_count"] <= 5:
            start_search_sweep(f"{strategy}_no_path")
            return

        if front is not None and front > SAFE_FRONT_CLEARANCE_M + 0.05 and not ls.get("center"):
            exploration["phase"] = "nudge"
            exploration["nudge_until"] = time.time() + 1.25
            command_forward()
            return

        start_wall_follow(duration_s=WALL_FOLLOW_DURATION_S * 1.45)
        return

    exploration["no_path_count"] = 0
    exploration["search_reason"] = None
    exploration["path"] = path
    exploration["path_index"] = 1
    exploration["waypoint"] = path[1]
    exploration["target"] = path[-1]
    exploration["waypoints_since_scan"] = 0
    exploration["move_until"] = 0.0
    exploration["pause_until"] = 0.0
    exploration["phase"] = "turn"


def drive_to_waypoint() -> None:
    waypoint = exploration["waypoint"]
    if waypoint is None:
        exploration["phase"] = "plan"
        return

    wx0, wy0 = grid_to_world(*waypoint)
    wx = wx0 + CELL_M * 0.5
    wy = wy0 + CELL_M * 0.5
    target_angle = math.atan2(wx - pose["x"], wy - pose["y"]) + line_bias_radians() * 0.65
    diff = signed_angle_diff(target_angle, pose["angle"])
    dist_wp = distance((pose["x"], pose["y"]), (wx, wy))
    front = state["distance_m"]

    ls = state.get("line_sensors", {})

    if ls.get("center") or (front is not None and front < SAFE_FRONT_CLEARANCE_M):
        command_stop()
        exploration["path"] = []
        exploration["path_index"] = 0
        exploration["waypoint"] = None
        exploration["target"] = None
        start_blocked_recovery(exploration.get("mode") or "frontier", resume_phase="scan")
        return

    if abs(diff) > ANGLE_TOL:
        exploration["move_until"] = 0.0
        exploration["pause_until"] = 0.0
        if diff < 0:
            command_left()
        else:
            command_right()
        exploration["phase"] = "turn"
        return

    if dist_wp <= WAYPOINT_REACHED_M:
        command_stop()
        if waypoint == exploration["target"]:
            exploration["stats"]["visited_frontiers"] += 1
            exploration["waypoints_since_scan"] = 0
            start_scan(LOCAL_SCAN_ANGLE)
            return

        exploration["path_index"] += 1
        exploration["waypoints_since_scan"] += 1
        if exploration["waypoints_since_scan"] >= WAYPOINTS_BEFORE_RESCAN:
            exploration["waypoints_since_scan"] = 0
            start_scan(LOCAL_SCAN_ANGLE)
            return
        begin_next_waypoint()
        return

    command_forward()
    exploration["phase"] = "move"


def step_random_exploration() -> None:
    front = state["distance_m"]
    ls = state.get("line_sensors", {})

    if handle_backout_phase():
        return
    if handle_blocked_phase():
        return
    if handle_search_sweep_phase():
        return

    if exploration.get("phase") == "wall_follow":
        step_wall_follow()
        return

    if front is None:
        command_stop()
        return

    if ls.get("center") or front < SAFE_FRONT_CLEARANCE_M:
        command_stop()
        start_search_sweep("random_blocked")
        return

    # Near a corner: bias away before the front sensor forces a hard stop.
    if front < SLOW_FRONT_CLEARANCE_M:
        if ls.get("left") and not ls.get("right"):
            command_right()
            exploration["phase"] = "turn"
            return
        if ls.get("right") and not ls.get("left"):
            command_left()
            exploration["phase"] = "turn"
            return
        start_search_sweep("random_slow")
        return

    if ls.get("left") and not ls.get("right"):
        command_right()
        exploration["phase"] = "turn"
        return
    if ls.get("right") and not ls.get("left"):
        command_left()
        exploration["phase"] = "turn"
        return

    command_forward()
    exploration["phase"] = "move"


def step_frontier_exploration(strategy: str) -> None:
    phase = exploration["phase"]

    if handle_backout_phase():
        return
    if handle_blocked_phase():
        return
    if handle_search_sweep_phase():
        return

    if phase == "idle":
        start_scan()
        return

    if phase == "scan":
        handle_scan_phase()
        return

    if phase == "plan":
        plan_frontier_path(strategy)
        return

    if phase == "nudge":
        front = state["distance_m"]
        ls = state.get("line_sensors", {})
        if time.time() < exploration["nudge_until"] and (front is None or front > SAFE_FRONT_CLEARANCE_M) and not ls.get("center"):
            command_forward()
            exploration["phase"] = "move"
            return
        command_stop()
        if immediate_blocked(0.01):
            start_blocked_recovery(strategy, resume_phase="search_move")
        else:
            start_search_sweep(f"{strategy}_nudge")
        return

    if phase == "wall_follow":
        step_wall_follow()
        return

    if phase in ("turn", "move", "move_pause"):
        drive_to_waypoint()
        return

    start_scan(LOCAL_SCAN_ANGLE)


def exploration_step() -> None:
    if not exploration["enabled"]:
        return

    mode = exploration["mode"]
    if mode == "random":
        step_random_exploration()
    elif mode == "bfs":
        step_frontier_exploration("bfs")
    elif mode == "dfs":
        step_frontier_exploration("dfs")


def explored_ratio() -> float:
    known = 0
    total = GRID_W * GRID_H
    for gx in range(GRID_W):
        for gy in range(GRID_H):
            if classify_cell(gx, gy) != "unknown":
                known += 1
    return known / max(1, total)


def emit_full_state() -> None:
    socketio.emit("robotPoseUpdate", {
        "x": pose["x"],
        "y": pose["y"],
        "angle": pose["angle"],
        "angle_deg": math.degrees(pose["angle"]),
        "mode": motion_state["mode"],
        "robot_available": state["robot_available"],
        "sensor_available": state["sensor_available"],
        "line_sensor_available": state["line_sensor_available"],
        "line_sensors": state["line_sensors"],
        "last_error": state["last_error"],
    })

    socketio.emit("echoRangeUpdate", {
        "range": state["distance_m"] if state["distance_m"] is not None else 1.0,
    })

    socketio.emit("explorationState", {
        "enabled": exploration["enabled"],
        "mode": exploration["mode"],
        "phase": exploration["phase"],
        "waypoint": exploration["waypoint"],
        "target": exploration["target"],
        "coverage": explored_ratio(),
        "replans": exploration["stats"]["replans"],
        "visited_frontiers": exploration["stats"]["visited_frontiers"],
        "recovery_side": exploration.get("recovery_side"),
        "blocked_attempts": exploration.get("blocked_attempts", 0),
        "backup_count": exploration.get("backup_count", 0),
        "search_best_score": exploration.get("search_best_score"),
        "search_reason": exploration.get("search_reason"),
    })


def telemetry_loop() -> None:
    print(">>> telemetry_loop démarrée")
    last_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        with robot_lock:
            try:
                update_pose(dt)

                if robot is None:
                    state["distance_m"] = None
                    state["sensor_available"] = False
                    state["line_sensor_available"] = False
                    state["line_sensors"] = {"left": False, "center": False, "right": False}
                else:
                    d = get_sensor_distance_safe()
                    state["distance_m"] = clamp(d, 0.0, MAX_SENSOR_RANGE) if d is not None else None
                    state["sensor_available"] = d is not None

                    ls = read_line_sensors_safe()
                    state["line_sensor_available"] = ls is not None
                    state["line_sensors"] = ls or {"left": False, "center": False, "right": False}

                if state["distance_m"] is not None:
                    update_occupancy_from_sensor(state["distance_m"])
                if state["line_sensor_available"]:
                    update_occupancy_from_line_sensors(state["line_sensors"])

                try:
                    exploration_step()
                    state["last_error"] = None
                except Exception as exc:
                    state["last_error"] = f"exploration_step: {exc}"
                    command_stop()
                    exploration["path"] = []
                    exploration["path_index"] = 0
                    exploration["waypoint"] = None
                    exploration["target"] = None
                    if exploration.get("enabled"):
                        exploration["no_path_count"] = int(exploration.get("no_path_count", 0)) + 1
                        if exploration.get("mode") in {"bfs", "dfs"}:
                            start_search_sweep(f"{exploration.get('mode')}_recover")
                        else:
                            start_search_sweep("random_recover")
                    else:
                        set_manual_speed()

            except Exception as exc:
                state["last_error"] = str(exc)
                state["sensor_available"] = False
                set_manual_speed()
                command_stop()
                exploration["enabled"] = False
                exploration["mode"] = "idle"
                exploration["phase"] = "idle"

            emit_full_state()

        socketio.sleep(0.05)


def ensure_background_task() -> None:
    global background_started
    if not background_started:
        print(">>> lancement background task")
        background_started = True
        socketio.start_background_task(target=telemetry_loop)


@socketio.on_error_default
def default_error_handler(e):
    import traceback
    print(">>> SocketIO error:", repr(e))
    traceback.print_exc()


@socketio.on("connect")
def on_connect():
    print(">>> client connecté")
    ensure_background_task()
    emit_full_state()


@socketio.on("initGPIO")
def init_gpio(_data=None):
    global robot
    print(">>> initGPIO reçu")

    try:
        with robot_lock:
            if robot is None:
                print(">>> création de robot_utils.Robot(...)")
                robot = robot_utils.Robot(speed=MANUAL_SPEED)

            set_manual_speed()
            state["robot_available"] = True
            state["sensor_available"] = True
            state["line_sensor_available"] = setup_line_sensors()
            state["last_error"] = None

        ensure_background_task()
        print(">>> robot initialisé OK")
        socketio.emit("robotStatus", {"ok": True, "message": "Robot initialisé"})
        emit_full_state()

    except Exception as exc:
        state["robot_available"] = False
        state["sensor_available"] = False
        state["last_error"] = str(exc)
        print(">>> initGPIO erreur:", repr(exc))
        socketio.emit("robotStatus", {"ok": False, "message": f"Init impossible: {exc}"})
        emit_full_state()


@socketio.on("resetPose")
def reset_pose(_data=None):
    with robot_lock:
        pose["x"] = WORLD_W / 2
        pose["y"] = WORLD_H / 2
        pose["angle"] = 0.0
        state["distance_m"] = None
        state["last_error"] = None
        occupancy.clear()
        exploration["enabled"] = False
        exploration["mode"] = "idle"
        exploration["phase"] = "idle"
        exploration["scan_goal"] = 0.0
        exploration["did_initial_scan"] = False
        exploration["path"] = []
        exploration["path_index"] = 0
        exploration["waypoint"] = None
        exploration["target"] = None
        exploration["move_until"] = 0.0
        exploration["pause_until"] = 0.0
        exploration["waypoints_since_scan"] = 0
        exploration["nudge_until"] = 0.0
        exploration["no_path_count"] = 0
        exploration["recovery_side"] = None
        exploration["wall_follow_until"] = 0.0
        exploration["blocked_context"] = None
        exploration["blocked_attempts"] = 0
        exploration["blocked_resume_phase"] = "move"
        exploration["backup_until"] = 0.0
        exploration["backup_count"] = 0
        exploration["search_best_angle"] = None
        exploration["search_best_score"] = -1e9
        exploration["search_target_angle"] = None
        exploration["search_move_until"] = 0.0
        exploration["search_reason"] = None
        exploration["stats"] = {"visited_frontiers": 0, "replans": 0}
        set_manual_speed()
        command_stop()

    emit_full_state()


@socketio.on("startExploration")
def start_exploration(data=None):
    data = data or {}
    mode = str(data.get("mode", "random")).lower()
    if mode not in {"random", "bfs", "dfs"}:
        mode = "random"

    with robot_lock:
        set_algo_speed()
        exploration["enabled"] = True
        exploration["mode"] = mode
        exploration["phase"] = "idle"
        exploration["scan_goal"] = 0.0
        exploration["did_initial_scan"] = False
        exploration["path"] = []
        exploration["path_index"] = 0
        exploration["waypoint"] = None
        exploration["target"] = None
        exploration["turn_dir"] = None
        exploration["turn_until"] = 0.0
        exploration["move_until"] = 0.0
        exploration["pause_until"] = 0.0
        exploration["waypoints_since_scan"] = 0
        exploration["nudge_until"] = 0.0
        exploration["no_path_count"] = 0
        exploration["blocked_context"] = None
        exploration["blocked_attempts"] = 0
        exploration["blocked_resume_phase"] = "move"
        exploration["backup_until"] = 0.0
        exploration["backup_count"] = 0
        exploration["search_best_angle"] = None
        exploration["search_best_score"] = -1e9
        exploration["search_target_angle"] = None
        exploration["search_move_until"] = 0.0
        exploration["search_reason"] = None

    socketio.emit("robotStatus", {"ok": True, "message": f"Exploration {mode.upper()} démarrée"})
    emit_full_state()


@socketio.on("stopExploration")
def stop_exploration(_data=None):
    with robot_lock:
        exploration["enabled"] = False
        exploration["mode"] = "idle"
        exploration["phase"] = "idle"
        exploration["path"] = []
        exploration["path_index"] = 0
        exploration["waypoint"] = None
        exploration["target"] = None
        exploration["move_until"] = 0.0
        exploration["pause_until"] = 0.0
        exploration["nudge_until"] = 0.0
        exploration["no_path_count"] = 0
        exploration["recovery_side"] = None
        exploration["wall_follow_until"] = 0.0
        exploration["blocked_context"] = None
        exploration["blocked_attempts"] = 0
        exploration["blocked_resume_phase"] = "move"
        exploration["backup_until"] = 0.0
        exploration["backup_count"] = 0
        exploration["search_best_angle"] = None
        exploration["search_best_score"] = -1e9
        exploration["search_target_angle"] = None
        exploration["search_move_until"] = 0.0
        exploration["search_reason"] = None
        set_manual_speed()
        command_stop()

    socketio.emit("robotStatus", {"ok": True, "message": "Exploration arrêtée"})
    emit_full_state()


@socketio.on("resetExplorationMap")
def reset_exploration_map(_data=None):
    with robot_lock:
        occupancy.clear()
    socketio.emit("robotStatus", {"ok": True, "message": "Carte serveur réinitialisée"})
    emit_full_state()


@socketio.on("getExplorationConfig")
def get_exploration_config(_data=None):
    emit_exploration_config()


@socketio.on("setExplorationConfig")
def set_exploration_config(data=None):
    with robot_lock:
        cfg = apply_exploration_config(data or {})
    socketio.emit("robotStatus", {"ok": True, "message": "Paramètres exploration mis à jour"})
    socketio.emit("explorationConfig", cfg)
    emit_full_state()


@socketio.on("resetExplorationConfig")
def reset_exploration_config(_data=None):
    with robot_lock:
        cfg = reset_exploration_config_values()
    socketio.emit("robotStatus", {"ok": True, "message": "Paramètres exploration réinitialisés"})
    socketio.emit("explorationConfig", cfg)
    emit_full_state()


@socketio.on("goUp")
def go_up(_data=None):
    with robot_lock:
        exploration["enabled"] = False
        exploration["mode"] = "idle"
        set_manual_speed()
        command_forward()


@socketio.on("goDown")
def go_down(_data=None):
    with robot_lock:
        exploration["enabled"] = False
        exploration["mode"] = "idle"
        set_manual_speed()
        command_backward()


@socketio.on("goLeft")
def go_left(_data=None):
    with robot_lock:
        exploration["enabled"] = False
        exploration["mode"] = "idle"
        set_manual_speed()
        command_left()


@socketio.on("goRight")
def go_right(_data=None):
    with robot_lock:
        exploration["enabled"] = False
        exploration["mode"] = "idle"
        set_manual_speed()
        command_right()


@socketio.on("stopGo")
def stop_go(_data=None):
    with robot_lock:
        exploration["enabled"] = False
        exploration["mode"] = "idle"
        set_manual_speed()
        command_stop()


@socketio.on("cleanupGPIO")
def cleanup_gpio(_data=None):
    global robot
    with robot_lock:
        command_stop()
        exploration["enabled"] = False
        exploration["mode"] = "idle"
        exploration["phase"] = "idle"

        if robot is not None:
            try:
                robot.cleanup()
            except Exception:
                pass

        close_line_sensors()
        robot = None
        state["robot_available"] = False
        state["sensor_available"] = False
        state["line_sensor_available"] = False
        state["line_sensors"] = {"left": False, "center": False, "right": False}
        state["distance_m"] = None

    emit_full_state()


if __name__ == "__main__":
    print(f"SocketIO mode = {socketio.async_mode}")
    print(f"Serveur en cours sur http://127.0.0.1:{PORT} et http://0.0.0.0:{PORT}")
    socketio.run(app, host=HOST, port=PORT, debug=False, allow_unsafe_werkzeug=True)
