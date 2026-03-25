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

LINEAR_SPEED = 0.10      # m/s
ANGULAR_SPEED = 1.22     # rad/s
MAX_SENSOR_RANGE_M = 1.0

app = Flask(__name__)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CORS(app)

robot_lock = threading.Lock()
robot: Optional[robot_utils.Robot] = None

pose = {
    "x": WORLD_W / 2,
    "y": WORLD_H / 2,
    "angle": 0.0,   # radians
}

motion_state = {
    "mode": "stop",   # forward, backward, left, right, stop
    "updated_at": time.time(),
}

state = {
    "robot_available": False,
    "sensor_available": False,
    "distance_m": None,
    "last_error": None,
}

last_motion_cmd = "stop"


def clamp(v: float, a: float, b: float) -> float:
    return max(a, min(b, v))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def ensure_robot() -> None:
    global robot
    if robot is None:
        robot = robot_utils.Robot(speed=90)
        state["robot_available"] = True
        state["sensor_available"] = True
        state["last_error"] = None


def set_motion(mode: str) -> None:
    motion_state["mode"] = mode
    motion_state["updated_at"] = time.time()


def update_pose(dt: float) -> None:
    mode = motion_state["mode"]

    if mode == "forward":
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
    last_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        with robot_lock:
            try:
                update_pose(dt)
                state["distance_m"] = clamp(get_sensor_distance_safe(), 0.0, MAX_SENSOR_RANGE_M)
                state["sensor_available"] = True
            except Exception as exc:
                state["last_error"] = str(exc)

        time.sleep(0.05)   # 20 Hz, comme l'ancienne version


def apply_robot_mode(mode: str) -> None:
    global last_motion_cmd

    set_motion(mode)

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
    return jsonify({
        "robot_available": state["robot_available"],
        "sensor_available": state["sensor_available"],
        "mode": motion_state["mode"],
        "pose": {
            "x": pose["x"],
            "y": pose["y"],
            "angle_deg": math.degrees(pose["angle"]),
        },
        "distance_m": state["distance_m"],
        "last_error": state["last_error"],
    })


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
        return jsonify({"ok": True, "mode": motion_state["mode"]})
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
    with robot_lock:
        pose["x"] = WORLD_W / 2
        pose["y"] = WORLD_H / 2
        pose["angle"] = 0.0
        set_motion("stop")
        state["distance_m"] = None
        state["last_error"] = None
        last_motion_cmd = "stop"

        if robot is not None:
            try:
                robot.stop()
            except Exception:
                pass

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