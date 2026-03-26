#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import os
import threading
import time
from typing import Optional

from flask import Flask, send_from_directory
from flask_socketio import SocketIO

import robot_utils

HOST = "0.0.0.0"
PORT = 8765

WORLD_W = 6.0
WORLD_H = 6.0

LINEAR_SPEED = 0.10      # m/s
ANGULAR_SPEED = 1.42     # rad/s
MAX_SENSOR_RANGE = 1.0

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

pose = {
    "x": WORLD_W / 2,
    "y": WORLD_H / 2,
    "angle": 0.0,   # radians
}

motion_state = {
    "mode": "stop",
    "updated_at": time.time(),
}

state = {
    "robot_available": False,
    "sensor_available": False,
    "distance_m": None,
    "last_error": None,
}


@app.route("/", methods=["GET"])
def index():
    return send_from_directory(BASE_DIR, "frontend_robot.html")


def clamp(v: float, a: float, b: float) -> float:
    return max(a, min(b, v))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def set_motion(mode: str):
    motion_state["mode"] = mode
    motion_state["updated_at"] = time.time()


def update_pose(dt: float):
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


def get_sensor_distance_safe():
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


def emit_full_state():
    socketio.emit("robotPoseUpdate", {
        "x": pose["x"],
        "y": pose["y"],
        "angle": pose["angle"],
        "angle_deg": math.degrees(pose["angle"]),
        "mode": motion_state["mode"],
        "robot_available": state["robot_available"],
        "sensor_available": state["sensor_available"],
        "last_error": state["last_error"],
    })

    socketio.emit("echoRangeUpdate", {
        "range": state["distance_m"] if state["distance_m"] is not None else 1.0
    })


def telemetry_loop():
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
                else:
                    d = get_sensor_distance_safe()
                    state["distance_m"] = clamp(d, 0.0, MAX_SENSOR_RANGE) if d is not None else None
                    state["sensor_available"] = d is not None

            except Exception as exc:
                state["last_error"] = str(exc)
                state["sensor_available"] = False

            emit_full_state()

        socketio.sleep(0.05)   # 20 Hz


def ensure_background_task():
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
                robot = robot_utils.Robot(speed=90)

            state["robot_available"] = True
            state["sensor_available"] = True
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
        set_motion("stop")
        state["distance_m"] = None
        state["last_error"] = None

        if robot is not None:
            try:
                robot.stop()
            except Exception:
                pass

    emit_full_state()


@socketio.on("goUp")
def go_up(_data=None):
    global robot
    with robot_lock:
        set_motion("forward")
        if robot is not None:
            robot.avance()


@socketio.on("goDown")
def go_down(_data=None):
    global robot
    with robot_lock:
        set_motion("backward")
        if robot is not None:
            robot.recule()


@socketio.on("goLeft")
def go_left(_data=None):
    global robot
    with robot_lock:
        set_motion("left")
        if robot is not None:
            robot.gauche()


@socketio.on("goRight")
def go_right(_data=None):
    global robot
    with robot_lock:
        set_motion("right")
        if robot is not None:
            robot.droite()


@socketio.on("stopGo")
def stop_go(_data=None):
    global robot
    with robot_lock:
        set_motion("stop")
        if robot is not None:
            robot.stop()


@socketio.on("cleanupGPIO")
def cleanup_gpio(_data=None):
    global robot
    with robot_lock:
        if robot is not None:
            try:
                robot.cleanup()
            except Exception:
                pass
        robot = None
        state["robot_available"] = False
        state["sensor_available"] = False
        state["distance_m"] = None
        set_motion("stop")

    emit_full_state()


if __name__ == "__main__":
    print(f"SocketIO mode = {socketio.async_mode}")
    print(f"Serveur en cours sur http://127.0.0.1:{PORT} et http://0.0.0.0:{PORT}")
    socketio.run(app, host=HOST, port=PORT, debug=False, allow_unsafe_werkzeug=True)