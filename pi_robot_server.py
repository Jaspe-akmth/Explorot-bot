#!/usr/bin/env python3
"""
Serveur HTTP minimal pour piloter le robot depuis le PC.

À lancer sur le Raspberry Pi :
    python3 pi_robot_server.py --host 0.0.0.0 --port 8765

Endpoints:
- GET  /health   -> état général
- GET  /status   -> distance capteur + dernière commande
- POST /command  -> {"command": "forward|backward|left|right|stop"}
"""

from __future__ import annotations

import argparse
import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any


MAX_SENSOR_RANGE = 1.0


class RobotManager:
    def __init__(self) -> None:
        self.robot = None
        self.available = False
        self.sensor_available = False
        self.last_error = "non initialisé"
        self.last_command = "stop"
        self.started_at = time.time()
        self.lock = threading.Lock()

    def init(self) -> tuple[bool, str]:
        if self.robot is not None:
            return True, "Robot déjà initialisé"

        try:
            import robot_utils  # type: ignore
        except Exception as exc:
            self.available = False
            self.sensor_available = False
            self.last_error = f"Import robot_utils impossible: {exc}"
            return False, self.last_error

        try:
            self.robot = robot_utils.Robot()
            self.available = True
            self.last_error = ""
            try:
                _ = float(self.robot.get_sensor_distance())
                self.sensor_available = True
            except Exception:
                self.sensor_available = False
            return True, "Robot initialisé"
        except Exception as exc:
            self.robot = None
            self.available = False
            self.sensor_available = False
            self.last_error = f"Init robot impossible: {exc}"
            return False, self.last_error

    def ensure_init(self) -> None:
        if self.robot is None:
            self.init()

    def get_distance(self) -> float:
        self.ensure_init()
        if self.robot is None:
            return MAX_SENSOR_RANGE
        try:
            value = float(self.robot.get_sensor_distance())
            self.sensor_available = True
            return max(0.0, min(MAX_SENSOR_RANGE, value))
        except Exception:
            self.sensor_available = False
            return MAX_SENSOR_RANGE

    def send_command(self, command: str) -> tuple[bool, str]:
        self.ensure_init()
        if self.robot is None:
            return False, self.last_error or "Robot indisponible"

        mapping = {
            "forward": self.robot.avance,
            "backward": self.robot.recule,
            "left": self.robot.gauche,
            "right": self.robot.droite,
            "stop": self.robot.stop,
        }
        func = mapping.get(command)
        if func is None:
            return False, f"Commande inconnue: {command}"

        try:
            with self.lock:
                func()
                self.last_command = command
            return True, f"Commande envoyée: {command}"
        except Exception as exc:
            self.last_error = str(exc)
            return False, f"Échec commande {command}: {exc}"

    def stop(self) -> None:
        if self.robot is None:
            return
        try:
            with self.lock:
                self.robot.stop()
                self.last_command = "stop"
        except Exception:
            pass

    def cleanup(self) -> None:
        if self.robot is None:
            return
        try:
            with self.lock:
                try:
                    self.robot.stop()
                except Exception:
                    pass
                try:
                    self.robot.cleanup()
                except Exception:
                    pass
        finally:
            self.last_command = "stop"


MANAGER = RobotManager()
SERVER: ThreadingHTTPServer | None = None


class RobotRequestHandler(BaseHTTPRequestHandler):
    server_version = "ExploroRobotHTTP/1.0"

    def log_message(self, format: str, *args: Any) -> None:
        print("[%s] %s" % (self.address_string(), format % args))

    def _send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self) -> None:
        self._send_json(200, {"ok": True})

    def do_GET(self) -> None:
        if self.path == "/" or self.path == "":
            self._send_json(
                200,
                {
                    "ok": True,
                    "message": "Serveur robot actif",
                    "endpoints": ["/health", "/status", "/command"],
                },
            )
            return

        if self.path == "/health":
            ok, msg = MANAGER.init()
            self._send_json(
                200,
                {
                    "ok": ok,
                    "message": msg,
                    "robot_available": MANAGER.available,
                    "sensor_available": MANAGER.sensor_available,
                    "last_error": MANAGER.last_error,
                    "last_command": MANAGER.last_command,
                    "uptime_s": round(time.time() - MANAGER.started_at, 2),
                },
            )
            return

        if self.path == "/status":
            distance = MANAGER.get_distance()
            self._send_json(
                200,
                {
                    "ok": True,
                    "distance": distance,
                    "robot_available": MANAGER.available,
                    "sensor_available": MANAGER.sensor_available,
                    "last_error": MANAGER.last_error,
                    "last_command": MANAGER.last_command,
                    "uptime_s": round(time.time() - MANAGER.started_at, 2),
                },
            )
            return

        self._send_json(404, {"ok": False, "error": "Not found"})

    def do_POST(self) -> None:
        if self.path != "/command":
            self._send_json(404, {"ok": False, "error": "Not found"})
            return

        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8"))
        except Exception:
            self._send_json(400, {"ok": False, "error": "JSON invalide"})
            return

        command = str(payload.get("command", "")).strip().lower()
        ok, message = MANAGER.send_command(command)
        self._send_json(
            200 if ok else 400,
            {
                "ok": ok,
                "message": message,
                "last_command": MANAGER.last_command,
                "robot_available": MANAGER.available,
                "sensor_available": MANAGER.sensor_available,
                "last_error": MANAGER.last_error,
            },
        )


def shutdown(*_: Any) -> None:
    global SERVER
    print("\nArrêt du serveur robot...")
    MANAGER.cleanup()
    if SERVER is not None:
        SERVER.shutdown()
        SERVER.server_close()


def main() -> None:
    global SERVER

    parser = argparse.ArgumentParser(description="Serveur HTTP pour robot Raspberry Pi")
    parser.add_argument("--host", default="0.0.0.0", help="Adresse d'écoute")
    parser.add_argument("--port", type=int, default=8765, help="Port d'écoute")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    ok, msg = MANAGER.init()
    print(msg)
    print(f"robot_available={MANAGER.available} sensor_available={MANAGER.sensor_available}")
    print(f"robot_utils.py attendu dans: {Path.cwd()}")

    SERVER = ThreadingHTTPServer((args.host, args.port), RobotRequestHandler)
    print(f"Serveur robot en écoute sur http://{args.host}:{args.port}")
    SERVER.serve_forever()


if __name__ == "__main__":
    main()
