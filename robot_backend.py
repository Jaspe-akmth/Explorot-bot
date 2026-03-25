#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import os
import threading
import time
from typing import Optional

from flask import Flask, jsonify, request, send_from_directory
from flask_cors import CORS

import robot_utils

HOST = "0.0.0.0"
PORT = 8765

WORLD_W = 6.0
WORLD_H = 6.0

# estimation simple, proche de ton ancienne logique
LINEAR_SPEED_M_S = 0.10
ANGULAR_SPEED_RAD_S = 2.20
MAX_SENSOR_RANGE_M = 1.0

app = Flask(__name__)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CORS(app)

robot_lock = threading.Lock()
robot: Optional[robot_utils.Robot] = None

state = {
    "robot_available": False,
    "sensor_available": False,
    "mode": "stop",
    "pose": {"x": WORLD_W / 2, "y": WORLD_H / 2, "angle_deg": 0.0},
    "distance_m": None,
    "last_tick": time.time(),
    "last_error": None,
}

last_motion_cmd = "stop"


def clamp(v: float, a: float, b: float) -> float:
    return max(a, min(b, v))


def normalize_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def ensure_robot() -> None:
    global robot
    if robot is None:
        robot = robot_utils.Robot(speed=90)
        state["robot_available"] = True
        state["sensor_available"] = True
        state["last_error"] = None


def update_pose(dt: float) -> None:
    mode = state["mode"]
    angle_deg = float(state["pose"]["angle_deg"])
    angle_rad = math.radians(angle_deg)

    if mode == "forward":
        state["pose"]["x"] += math.sin(angle_rad) * LINEAR_SPEED_M_S * dt
        state["pose"]["y"] += math.cos(angle_rad) * LINEAR_SPEED_M_S * dt
    elif mode == "backward":
        state["pose"]["x"] -= math.sin(angle_rad) * LINEAR_SPEED_M_S * dt
        state["pose"]["y"] -= math.cos(angle_rad) * LINEAR_SPEED_M_S * dt
    elif mode == "left":
        state["pose"]["angle_deg"] = normalize_deg(
            angle_deg - math.degrees(ANGULAR_SPEED_RAD_S * dt)
        )
    elif mode == "right":
        state["pose"]["angle_deg"] = normalize_deg(
            angle_deg + math.degrees(ANGULAR_SPEED_RAD_S * dt)
        )

    state["pose"]["x"] = clamp(float(state["pose"]["x"]), 0.0, WORLD_W)
    state["pose"]["y"] = clamp(float(state["pose"]["y"]), 0.0, WORLD_H)


def get_sensor_distance_safe() -> float:
    global robot
    if robot is None:
        return 1.0

    try:
        distance = robot.get_sensor_distance()
        if distance is None:
            return 1.0
        return float(distance)
    except Exception:
        return 1.0


def telemetry_loop() -> None:
    while True:
        time.sleep(0.05)
        now = time.time()
        dt = max(0.0, min(0.2, now - float(state["last_tick"])))
        state["last_tick"] = now

        try:
            update_pose(dt)

            if robot is not None:
                state["distance_m"] = clamp(get_sensor_distance_safe(), 0.0, MAX_SENSOR_RANGE_M)
                state["sensor_available"] = True
        except Exception as exc:
            state["last_error"] = str(exc)


def apply_robot_mode(mode: str) -> None:
    global last_motion_cmd

    state["mode"] = mode

    if robot is None:
        return

    if mode == last_motion_cmd:
        return

    if mode == "forward":
        robot.avance()
    elif mode == "backward":
        robot.recule()
    elif mode == "left":
        robot.gauche()
    elif mode == "right":
        robot.droite()
    else:
        robot.stop()

    last_motion_cmd = mode


@app.get("/health")
def health():
    return jsonify({"ok": True, "message": "robot backend actif"})


@app.get("/status")
def status():
    return jsonify(state)


@app.post("/init")
def init_robot():
    try:
        with robot_lock:
            ensure_robot()
            apply_robot_mode("stop")
        return jsonify({"ok": True, "message": "robot initialisé", "robot_available": True})
    except Exception as exc:
        state["robot_available"] = False
        state["sensor_available"] = False
        state["last_error"] = str(exc)
        return jsonify({"ok": False, "message": f"init impossible: {exc}"}), 500


@app.post("/move")
def move():
    data = request.get_json(force=True, silent=True) or {}
    cmd = str(data.get("command", "stop")).strip().lower()

    if cmd not in {"forward", "backward", "left", "right", "stop"}:
        return jsonify({"ok": False, "message": f"commande inconnue: {cmd}"}), 400

    try:
        with robot_lock:
            if robot is None:
                return jsonify({
                    "ok": False,
                    "message": "robot non initialisé, clique d'abord sur Init robot"
                }), 400
            apply_robot_mode(cmd)
        return jsonify({"ok": True, "mode": state["mode"]})
    except Exception as exc:
        state["last_error"] = str(exc)
        return jsonify({"ok": False, "message": str(exc)}), 500


@app.post("/stop")
def stop():
    try:
        with robot_lock:
            if robot is not None:
                robot.stop()
            apply_robot_mode("stop")
        return jsonify({"ok": True, "mode": "stop"})
    except Exception as exc:
        state["last_error"] = str(exc)
        return jsonify({"ok": False, "message": str(exc)}), 500


@app.post("/reset_pose")
def reset_pose():
    global last_motion_cmd

    state["pose"] = {"x": WORLD_W / 2, "y": WORLD_H / 2, "angle_deg": 0.0}
    state["mode"] = "stop"
    state["distance_m"] = None
    state["last_error"] = None
    last_motion_cmd = "stop"

    if robot is not None:
        robot.stop()

    return jsonify({"ok": True})


@app.post("/cleanup")
def cleanup():
    global robot, last_motion_cmd
    try:
        with robot_lock:
            if robot is not None:
                robot.cleanup()
            robot = None
            last_motion_cmd = "stop"
            state["robot_available"] = False
            state["sensor_available"] = False
            state["mode"] = "stop"
            state["distance_m"] = None
        return jsonify({"ok": True})
    except Exception as exc:
        state["last_error"] = str(exc)
        return jsonify({"ok": False, "message": str(exc)}), 500


@app.get("/")
def index():
    return send_from_directory(BASE_DIR, "frontend_robot.html")


if __name__ == "__main__":
    threading.Thread(target=telemetry_loop, daemon=True).start()
    app.run(host=HOST, port=PORT, debug=False, threaded=True)