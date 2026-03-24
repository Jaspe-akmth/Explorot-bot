#!/usr/bin/env python3
"""
Exploro-Bot PC Controller - interface Pygame + robot distant

Ce script affiche l'interface Pygame sur le PC et envoie les commandes au
Raspberry Pi via HTTP, tout en gardant le même fonctionnement global :
- grande carte centrale
- panneau d'état
- choix de stratégie naive / BFS / DFS
- vue pseudo-FPS
- obstacles ajoutés à la souris
- destination au clic droit
- pilotage clavier
- exploration automatique en simulation
- mode robot réel distant via pi_robot_server.py

Dépendance : pygame
    pip install pygame

Lancement :
    python pc_controller.py --host 192.168.137.92 --port 8765
"""

from __future__ import annotations

import argparse
import json
import os
import math
import random
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import deque
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame


# ============================================================
# Configuration générale
# ============================================================
WINDOW_W = 1480
WINDOW_H = 900
FPS = 60

GRID_SIZE = 300
GRID_CENTER = 150
CELL_WORLD_SIZE = 0.01  # 1 cm par case

MAX_SENSOR_RANGE = 1.0  # cohérent avec robot_utils.py (max_distance=1)
SIM_MOVE_STEP = 0.01    # comme dans le JS original
SIM_STEER_STEP = 0.01   # comme dans le JS original
SIM_MAX_STEER = 0.1     # comme dans le JS original
REAL_LINEAR_SPEED = 0.10
REAL_ANGULAR_SPEED = 2.20

COLOR_BG = (245, 245, 245)
COLOR_PANEL = (255, 255, 255)
COLOR_BORDER = (0, 0, 0)
COLOR_TEXT = (20, 20, 20)
COLOR_MUTED = (90, 90, 90)
COLOR_FREE = (0, 210, 210)
COLOR_VISITED = (0, 130, 130)
COLOR_OBSTACLE = (255, 0, 120)
COLOR_SENSOR = (180, 160, 0)
COLOR_ROBOT = (15, 15, 15)
COLOR_ACCENT = (60, 110, 255)
COLOR_BTN = (240, 240, 240)
COLOR_BTN_HOVER = (225, 235, 255)
COLOR_BTN_ACTIVE = (215, 230, 255)
COLOR_FPS = (255, 130, 130)


# ============================================================
# Helpers
# ============================================================
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


# ============================================================
# UI minimaliste
# ============================================================
@dataclass
class Button:
    rect: pygame.Rect
    label: str
    callback: Callable[[], None]
    active: bool = False

    def draw(self, screen: pygame.Surface, font: pygame.font.Font, mouse_pos: Tuple[int, int]) -> None:
        hovered = self.rect.collidepoint(mouse_pos)
        color = COLOR_BTN_ACTIVE if self.active else COLOR_BTN_HOVER if hovered else COLOR_BTN
        pygame.draw.rect(screen, color, self.rect, border_radius=8)
        pygame.draw.rect(screen, COLOR_BORDER, self.rect, 1, border_radius=8)
        text = font.render(self.label, True, COLOR_TEXT)
        screen.blit(text, text.get_rect(center=self.rect.center))

    def handle_mouse_down(self, pos: Tuple[int, int]) -> bool:
        if self.rect.collidepoint(pos):
            self.callback()
            return True
        return False


@dataclass
class Checkbox:
    rect: pygame.Rect
    label: str
    getter: Callable[[], bool]
    setter: Callable[[bool], None]

    def draw(self, screen: pygame.Surface, font: pygame.font.Font) -> None:
        box = pygame.Rect(self.rect.x, self.rect.y, 20, 20)
        pygame.draw.rect(screen, COLOR_PANEL, box)
        pygame.draw.rect(screen, COLOR_BORDER, box, 1)
        if self.getter():
            pygame.draw.line(screen, COLOR_ACCENT, (box.left + 4, box.centery), (box.centerx - 1, box.bottom - 5), 3)
            pygame.draw.line(screen, COLOR_ACCENT, (box.centerx - 1, box.bottom - 5), (box.right - 4, box.top + 4), 3)
        label = font.render(self.label, True, COLOR_TEXT)
        screen.blit(label, (box.right + 8, box.y - 1))

    def handle_mouse_down(self, pos: Tuple[int, int]) -> bool:
        if self.rect.collidepoint(pos):
            self.setter(not self.getter())
            return True
        return False


@dataclass
class RadioOption:
    label: str
    value: str


@dataclass
class RadioGroup:
    rect: pygame.Rect
    options: List[RadioOption]
    getter: Callable[[], str]
    setter: Callable[[str], None]

    def draw(self, screen: pygame.Surface, font: pygame.font.Font) -> None:
        x = self.rect.x
        y = self.rect.y
        for opt in self.options:
            circle_center = (x + 10, y + 11)
            pygame.draw.circle(screen, COLOR_BORDER, circle_center, 8, 1)
            if self.getter() == opt.value:
                pygame.draw.circle(screen, COLOR_ACCENT, circle_center, 4)
            label = font.render(opt.label, True, COLOR_TEXT)
            screen.blit(label, (x + 26, y + 2))
            y += 28

    def handle_mouse_down(self, pos: Tuple[int, int]) -> bool:
        x = self.rect.x
        y = self.rect.y
        for opt in self.options:
            row_rect = pygame.Rect(x, y, self.rect.w, 22)
            if row_rect.collidepoint(pos):
                self.setter(opt.value)
                return True
            y += 28
        return False


@dataclass
class Slider:
    rect: pygame.Rect
    min_value: float
    max_value: float
    getter: Callable[[], float]
    setter: Callable[[float], None]
    label_getter: Callable[[], str]
    dragging: bool = False

    def _handle_rect(self) -> pygame.Rect:
        ratio = (self.getter() - self.min_value) / (self.max_value - self.min_value)
        ratio = clamp(ratio, 0.0, 1.0)
        handle_x = int(self.rect.x + ratio * self.rect.w)
        return pygame.Rect(handle_x - 8, self.rect.centery - 10, 16, 20)

    def draw(self, screen: pygame.Surface, font: pygame.font.Font) -> None:
        label = font.render(self.label_getter(), True, COLOR_TEXT)
        screen.blit(label, (self.rect.x, self.rect.y - 22))
        pygame.draw.rect(screen, (220, 220, 220), self.rect, border_radius=6)
        pygame.draw.rect(screen, COLOR_BORDER, self.rect, 1, border_radius=6)
        handle = self._handle_rect()
        pygame.draw.rect(screen, COLOR_ACCENT, handle, border_radius=6)
        pygame.draw.rect(screen, COLOR_BORDER, handle, 1, border_radius=6)

    def handle_mouse_down(self, pos: Tuple[int, int]) -> bool:
        if self.rect.collidepoint(pos) or self._handle_rect().collidepoint(pos):
            self.dragging = True
            self.update_from_mouse(pos[0])
            return True
        return False

    def handle_mouse_up(self) -> None:
        self.dragging = False

    def handle_mouse_motion(self, pos: Tuple[int, int]) -> None:
        if self.dragging:
            self.update_from_mouse(pos[0])

    def update_from_mouse(self, mouse_x: int) -> None:
        ratio = clamp((mouse_x - self.rect.x) / max(1, self.rect.w), 0.0, 1.0)
        value = self.min_value + ratio * (self.max_value - self.min_value)
        self.setter(value)


# ============================================================
# Modèles métier
# ============================================================
@dataclass
class Wall:
    left: float
    top: float
    right: float
    bottom: float

    @classmethod
    def from_points(cls, p1: Tuple[float, float], p2: Tuple[float, float]) -> "Wall":
        x1, y1 = p1
        x2, y2 = p2
        return cls(left=min(x1, x2), top=max(y1, y2), right=max(x1, x2), bottom=min(y1, y2))

    def contains(self, x: float, y: float) -> bool:
        return self.left < x < self.right and self.bottom < y < self.top


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    width: float = 0.1
    height: float = 0.1
    steer_angle: float = 0.0
    turn_angle: float = 0.0
    going_left: bool = False
    going_right: bool = False
    going_up: bool = False
    going_down: bool = False
    going_somewhere: bool = False
    exploring: bool = False
    go_to: Tuple[float, float] = (0.0, 0.0)
    attempts: int = 0
    temp_range: float = -1.0
    temp_turn: float = 0.0

    def naive_going_to(self, sensor_range: float) -> None:
        goal_x, goal_y = self.go_to

        # Version robuste du comportement JS : on considère l'objectif atteint
        # lorsqu'on est proche au lieu de comparer des floats à l'identique.
        reached = abs(self.x - goal_x) <= self.width / 2 and abs(self.y - goal_y) <= self.height / 2
        self.going_somewhere = not reached
        if reached:
            return

        self.turn_angle += self.steer_angle

        goal_angle = math.atan2(goal_x - self.x, goal_y - self.y)
        diff = math.atan2(math.sin(goal_angle - self.turn_angle), math.cos(goal_angle - self.turn_angle))
        self.steer_angle = clamp(diff, -SIM_MAX_STEER, SIM_MAX_STEER)

        if sensor_range < 0.08:
            self.attempts += 1
            if self.attempts > 5:
                self.attempts = 0
                # léger recul pour débloquer la situation
                self.y -= SIM_MOVE_STEP * math.cos(self.turn_angle)
                self.x -= SIM_MOVE_STEP * math.sin(self.turn_angle)
            return

        self.attempts = 0
        if abs(diff) <= 0.05:
            self.y += SIM_MOVE_STEP * math.cos(self.turn_angle)
            self.x += SIM_MOVE_STEP * math.sin(self.turn_angle)

    def update_manual_sim(self) -> None:
        if self.going_up or self.going_down or self.going_left or self.going_right:
            self.going_somewhere = False
            self.exploring = False

            if self.going_up:
                self.y += SIM_MOVE_STEP * math.cos(self.turn_angle)
                self.x += SIM_MOVE_STEP * math.sin(self.turn_angle)
                self.turn_angle += self.steer_angle

            if self.going_down:
                self.y -= SIM_MOVE_STEP * math.cos(self.turn_angle)
                self.x -= SIM_MOVE_STEP * math.sin(self.turn_angle)
                self.turn_angle -= self.steer_angle

            if self.going_right and self.steer_angle < SIM_MAX_STEER:
                self.steer_angle += SIM_STEER_STEP
            if self.going_left and self.steer_angle > -SIM_MAX_STEER:
                self.steer_angle -= SIM_STEER_STEP

        self.turn_angle = normalize_angle(self.turn_angle)


# ============================================================
# Robot réel optionnel
# ============================================================
class RobotHardwareProxy:
    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip('/')
        self.available = False
        self.last_error = 'Serveur robot non connecté'

    def set_base_url(self, base_url: str) -> None:
        self.base_url = base_url.rstrip('/')
        self.available = False

    def _request(self, method: str, path: str, payload: Optional[dict] = None, timeout: float = 1.5) -> dict:
        url = f"{self.base_url}{path}"
        data = None
        headers = {'Accept': 'application/json'}
        if payload is not None:
            data = json.dumps(payload).encode('utf-8')
            headers['Content-Type'] = 'application/json'
        request = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                raw = response.read().decode('utf-8')
                body = json.loads(raw) if raw else {}
                self.available = True
                self.last_error = ''
                return body
        except urllib.error.URLError as exc:
            self.available = False
            self.last_error = str(exc.reason if hasattr(exc, 'reason') else exc)
            raise
        except Exception as exc:
            self.available = False
            self.last_error = str(exc)
            raise

    def init(self) -> Tuple[bool, str]:
        try:
            body = self._request('GET', '/health', timeout=1.0)
            sensor = body.get('sensor_available', False)
            msg = f"Serveur robot connecté ({'capteur OK' if sensor else 'capteur indisponible'})"
            return True, msg
        except Exception as exc:
            return False, f"Impossible de joindre le Raspberry : {exc}"

    def distance(self) -> float:
        try:
            body = self._request('GET', '/status', timeout=0.5)
            return clamp(float(body.get('distance', MAX_SENSOR_RANGE)), 0.0, MAX_SENSOR_RANGE)
        except Exception:
            return MAX_SENSOR_RANGE

    def _send_command(self, command: str) -> None:
        try:
            self._request('POST', '/command', {'command': command}, timeout=0.7)
        except Exception:
            pass

    def avance(self) -> None:
        self._send_command('forward')

    def recule(self) -> None:
        self._send_command('backward')

    def gauche(self) -> None:
        self._send_command('left')

    def droite(self) -> None:
        self._send_command('right')

    def stop(self) -> None:
        self._send_command('stop')

    def cleanup(self) -> None:
        self.stop()


# ============================================================
# Application principale
# ============================================================
class ExploroBotApp:
    def __init__(self, base_url: str) -> None:
        pygame.init()
        pygame.display.set_caption("Exploro-Bot PC Controller")
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()

        self.font = pygame.font.SysFont("Arial", 18)
        self.small_font = pygame.font.SysFont("Arial", 15)
        self.title_font = pygame.font.SysFont("Arial", 20, bold=True)

        self.robot = RobotState()
        self.hardware = RobotHardwareProxy(base_url)

        self.running = True
        self.real_robot_mode = False
        self.search_mode = "naive"
        self.room_size_cm = 200.0
        self.current_echo_range = MAX_SENSOR_RANGE
        self.current_motion_mode = "stop"
        self.last_command_sent = "stop"
        self.frame_ms = 0.0
        self.last_update_time = time.perf_counter()

        self.exploration_graph = self.create_exploration_graph()
        self.exploration_queue: deque[Tuple[float, float]] = deque([(0.0, 0.0)])
        self.exploration_stack: List[Tuple[float, float]] = [(0.0, 0.0)]
        self.fps_view = [0.1] * 11

        self.walls: List[Wall] = []
        self.pending_wall_start: Optional[Tuple[float, float]] = None

        self.log_lines: List[str] = [
            "Interface PC connectée au Raspberry par HTTP.",
            f"Serveur visé : {base_url}",
            "↑/↓ : avancer / reculer",
            "←/→ : braquage en simulation, rotation en mode robot réel",
            "E : activer/désactiver l'exploration",
            "Clic gauche x2 : créer un obstacle rectangulaire",
            "Clic droit : définir une destination",
        ]

        self.buttons: List[Button] = []
        self.checkbox: Optional[Checkbox] = None
        self.radio_group: Optional[RadioGroup] = None
        self.slider: Optional[Slider] = None

        self.rebuild_ui()

    # --------------------------------------------------------
    # Utilitaires UI / layout
    # --------------------------------------------------------
    def rebuild_ui(self) -> None:
        w, h = self.screen.get_size()
        margin = 12
        left_w = int(w * 0.25)
        center_w = int(w * 0.50)
        right_w = w - left_w - center_w
        top_h = int(h * 0.55)
        bottom_h = h - top_h

        self.rect_top_left = pygame.Rect(margin, margin, left_w - 2 * margin, top_h - 2 * margin)
        self.rect_map = pygame.Rect(left_w + margin, margin, center_w - 2 * margin, h - 2 * margin)
        self.rect_top_right = pygame.Rect(left_w + center_w + margin, margin, right_w - 2 * margin, top_h - 2 * margin)
        self.rect_bottom_right = pygame.Rect(left_w + center_w + margin, top_h + margin, right_w - 2 * margin, bottom_h - 2 * margin)
        self.rect_bottom_left = pygame.Rect(margin, top_h + margin, left_w - 2 * margin, bottom_h - 2 * margin)

        btn_y = self.rect_top_left.y + 208
        btn_w = max(90, (self.rect_top_left.w - 24) // 3)
        btn_h = 34

        def add_log(message: str) -> None:
            self.push_log(message)

        self.buttons = [
            Button(pygame.Rect(self.rect_top_left.x, btn_y, btn_w, btn_h), "Init robot", self.init_robot),
            Button(pygame.Rect(self.rect_top_left.x + btn_w + 8, btn_y, btn_w, btn_h), "Reset pose", self.reset_pose),
            Button(pygame.Rect(self.rect_top_left.x + 2 * (btn_w + 8), btn_y, btn_w, btn_h), "Clear map", self.clear_map),
        ]

        radio_rect = pygame.Rect(self.rect_top_right.x + 8, self.rect_top_right.y + 18, self.rect_top_right.w - 16, 100)
        self.radio_group = RadioGroup(
            rect=radio_rect,
            options=[
                RadioOption("naive", "naive"),
                RadioOption("BFS", "BFS"),
                RadioOption("DFS", "DFS"),
            ],
            getter=lambda: self.search_mode,
            setter=self.set_search_mode,
        )

        checkbox_rect = pygame.Rect(self.rect_top_right.x + 8, self.rect_top_right.y + 114, self.rect_top_right.w - 16, 24)
        self.checkbox = Checkbox(
            rect=checkbox_rect,
            label="Real robot distant ?",
            getter=lambda: self.real_robot_mode,
            setter=self.set_real_robot_mode,
        )

        slider_rect = pygame.Rect(self.rect_top_right.x + 8, self.rect_top_right.y + 170, self.rect_top_right.w - 32, 12)
        self.slider = Slider(
            rect=slider_rect,
            min_value=50,
            max_value=500,
            getter=lambda: self.room_size_cm,
            setter=self.set_room_size_cm,
            label_getter=lambda: f"Size in CM: {int(round(self.room_size_cm))}",
        )

    def draw_panel(self, rect: pygame.Rect) -> None:
        pygame.draw.rect(self.screen, COLOR_PANEL, rect)
        pygame.draw.rect(self.screen, COLOR_BORDER, rect, 1)

    def push_log(self, message: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.log_lines.append(f"[{timestamp}] {message}")
        self.log_lines = self.log_lines[-16:]

    # --------------------------------------------------------
    # Monde
    # --------------------------------------------------------
    def room_half_size_world(self) -> float:
        return (self.room_size_cm / 100.0) / 2.0

    def create_exploration_graph(self) -> List[List[int]]:
        return [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

    def clear_map(self) -> None:
        self.exploration_graph = self.create_exploration_graph()
        self.exploration_queue = deque([(self.robot.x, self.robot.y)])
        self.exploration_stack = [(self.robot.x, self.robot.y)]
        self.pending_wall_start = None
        self.push_log("Carte effacée")

    def reset_pose(self) -> None:
        self.robot.x = 0.0
        self.robot.y = 0.0
        self.robot.turn_angle = 0.0
        self.robot.steer_angle = 0.0
        self.robot.go_to = (0.0, 0.0)
        self.robot.going_somewhere = False
        self.robot.attempts = 0
        self.current_motion_mode = "stop"
        self.hardware.stop()
        self.push_log("Pose réinitialisée")

    def init_robot(self) -> None:
        ok, msg = self.hardware.init()
        self.push_log(msg)
        if ok:
            self.real_robot_mode = True

    def set_search_mode(self, value: str) -> None:
        self.search_mode = value
        self.push_log(f"Mode de recherche : {value}")

    def set_real_robot_mode(self, enabled: bool) -> None:
        self.real_robot_mode = enabled
        self.last_command_sent = "stop"
        self.current_motion_mode = "stop"
        if enabled:
            ok, msg = self.hardware.init()
            self.push_log(msg)
            if not ok:
                self.real_robot_mode = False
        else:
            self.hardware.stop()
            self.push_log("Mode robot réel désactivé")

    def set_room_size_cm(self, value: float) -> None:
        self.room_size_cm = round(value)

    def is_inside_room(self, x: float, y: float) -> bool:
        half = self.room_half_size_world()
        return -half <= x <= half and -half <= y <= half

    def point_inside_wall(self, x: float, y: float) -> bool:
        return any(w.contains(x, y) for w in self.walls)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int(math.floor(x * 100)) + GRID_CENTER
        gy = int(math.floor(y * 100)) + GRID_CENTER
        return gx, gy

    def in_grid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE

    def set_cell(self, gx: int, gy: int, value: int) -> None:
        if not self.in_grid(gx, gy):
            return
        current = self.exploration_graph[gx][gy]
        if current == 2 and value != 2:
            return
        if current == 3 and value == 1:
            return
        self.exploration_graph[gx][gy] = value

    def mark_robot_cell(self) -> None:
        gx, gy = self.world_to_grid(self.robot.x, self.robot.y)
        if self.in_grid(gx, gy):
            self.exploration_graph[gx][gy] = 2

    def get_range_from_echo(self, turn_angle: Optional[float] = None) -> float:
        if self.real_robot_mode and self.hardware.available:
            return self.hardware.distance()

        angle = self.robot.turn_angle if turn_angle is None else turn_angle
        d = 0.0
        while d < MAX_SENSOR_RANGE:
            d += 0.01
            temp_x = self.robot.x + math.sin(angle) * d
            temp_y = self.robot.y + math.cos(angle) * d
            if not self.is_inside_room(temp_x, temp_y):
                return d
            if self.point_inside_wall(temp_x, temp_y):
                return d
        return MAX_SENSOR_RANGE

    def mark_sensor_observation(self) -> None:
        self.mark_robot_cell()
        distance = clamp(self.current_echo_range, 0.0, MAX_SENSOR_RANGE)
        free_limit = distance if distance >= MAX_SENSOR_RANGE else max(0.0, distance - CELL_WORLD_SIZE)

        step = 0.0
        while step <= free_limit:
            x = self.robot.x + math.sin(self.robot.turn_angle) * step
            y = self.robot.y + math.cos(self.robot.turn_angle) * step
            gx, gy = self.world_to_grid(x, y)
            self.set_cell(gx, gy, 1)
            step += CELL_WORLD_SIZE

        if distance < MAX_SENSOR_RANGE:
            ox = self.robot.x + math.sin(self.robot.turn_angle) * distance
            oy = self.robot.y + math.cos(self.robot.turn_angle) * distance
            gx, gy = self.world_to_grid(ox, oy)
            self.set_cell(gx, gy, 3)

        # Les obstacles dessinés à la main sont toujours marqués dans la grille
        for wall in self.walls:
            left_gx, top_gy = self.world_to_grid(wall.left, wall.top)
            right_gx, bottom_gy = self.world_to_grid(wall.right, wall.bottom)
            for gx in range(min(left_gx, right_gx), max(left_gx, right_gx) + 1):
                for gy in range(min(bottom_gy, top_gy), max(bottom_gy, top_gy) + 1):
                    if self.in_grid(gx, gy):
                        self.exploration_graph[gx][gy] = 3

    def update_real_robot_pose(self, dt: float) -> None:
        mode = self.current_motion_mode
        if mode == "forward":
            self.robot.x += math.sin(self.robot.turn_angle) * REAL_LINEAR_SPEED * dt
            self.robot.y += math.cos(self.robot.turn_angle) * REAL_LINEAR_SPEED * dt
        elif mode == "backward":
            self.robot.x -= math.sin(self.robot.turn_angle) * REAL_LINEAR_SPEED * dt
            self.robot.y -= math.cos(self.robot.turn_angle) * REAL_LINEAR_SPEED * dt
        elif mode == "left":
            self.robot.turn_angle -= REAL_ANGULAR_SPEED * dt
        elif mode == "right":
            self.robot.turn_angle += REAL_ANGULAR_SPEED * dt
        self.robot.turn_angle = normalize_angle(self.robot.turn_angle)

    def sync_robot_command(self) -> None:
        if not self.real_robot_mode:
            return

        command = "stop"
        if self.robot.going_up:
            command = "forward"
        elif self.robot.going_down:
            command = "backward"
        elif self.robot.going_left:
            command = "left"
        elif self.robot.going_right:
            command = "right"

        if command == self.last_command_sent:
            return

        self.last_command_sent = command
        self.current_motion_mode = command

        if command == "forward":
            self.hardware.avance()
        elif command == "backward":
            self.hardware.recule()
        elif command == "left":
            self.hardware.gauche()
        elif command == "right":
            self.hardware.droite()
        else:
            self.hardware.stop()

    def handle_exploration(self) -> None:
        if not self.robot.exploring or self.real_robot_mode:
            return

        if self.search_mode == "naive":
            self.naive_exploration_step()
        elif self.search_mode == "BFS":
            self.bfs_exploration_step()
        elif self.search_mode == "DFS":
            self.dfs_exploration_step()

    def naive_exploration_step(self) -> None:
        temp_found = False
        for i in range(50, GRID_SIZE - 50):
            for j in range(50, GRID_SIZE - 50):
                if self.exploration_graph[i][j] != 0:
                    continue

                fully_surrounded = (
                    (i <= 50 or self.exploration_graph[i - 1][j] == 1)
                    and (i >= 250 or self.exploration_graph[i + 1][j] == 1)
                    and (j <= 50 or self.exploration_graph[i][j - 1] == 1)
                    and (j >= 250 or self.exploration_graph[i][j + 1] == 1)
                )

                if fully_surrounded:
                    self.exploration_graph[i][j] = 1
                    continue

                goal = ((i - 150) / 100.0, (j - 150) / 100.0)
                self.robot.go_to = goal
                self.robot.going_somewhere = True
                temp_found = True
                return

        if not temp_found:
            self.robot.exploring = False
            self.push_log("Exploration naive terminée")

    def bfs_exploration_step(self) -> None:
        if not self.exploration_queue:
            self.exploration_queue.append((self.robot.x, self.robot.y))

        gx, gy = self.world_to_grid(*self.robot.go_to)
        current_goal_known = self.in_grid(gx, gy) and self.exploration_graph[gx][gy] >= 1

        if current_goal_known:
            pt = self.exploration_queue.popleft()
            for nx, ny in self.neighbors(pt):
                ngx, ngy = self.world_to_grid(nx, ny)
                if not self.in_grid(ngx, ngy):
                    continue
                if self.exploration_graph[ngx][ngy] != 2:
                    self.exploration_queue.append((nx, ny))
            ngx, ngy = self.world_to_grid(*pt)
            if self.in_grid(ngx, ngy) and self.exploration_graph[ngx][ngy] != 2:
                self.robot.go_to = pt
        self.robot.going_somewhere = True

    def dfs_exploration_step(self) -> None:
        if not self.exploration_stack:
            self.exploration_stack.append((self.robot.x, self.robot.y))

        gx, gy = self.world_to_grid(*self.robot.go_to)
        current_goal_known = self.in_grid(gx, gy) and self.exploration_graph[gx][gy] >= 1

        if current_goal_known:
            pt = self.exploration_stack.pop()
            for nx, ny in self.neighbors(pt):
                ngx, ngy = self.world_to_grid(nx, ny)
                if not self.in_grid(ngx, ngy):
                    continue
                if self.exploration_graph[ngx][ngy] != 2:
                    self.exploration_stack.append((nx, ny))
            ngx, ngy = self.world_to_grid(*pt)
            if self.in_grid(ngx, ngy) and self.exploration_graph[ngx][ngy] != 2:
                self.robot.go_to = pt
        self.robot.going_somewhere = True

    def neighbors(self, pt: Tuple[float, float]) -> List[Tuple[float, float]]:
        x, y = pt
        delta = 0.01
        return [
            (x + delta, y),
            (x - delta, y),
            (x, y + delta),
            (x, y - delta),
        ]

    # --------------------------------------------------------
    # Coordonnées écran / monde
    # --------------------------------------------------------
    def world_to_map_px(self, x: float, y: float) -> Tuple[int, int]:
        rect = self.rect_map
        half = self.room_half_size_world()
        px = rect.x + int(((x + half) / (half * 2)) * rect.w)
        py = rect.y + int((1.0 - ((y + half) / (half * 2))) * rect.h)
        return px, py

    def map_px_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        rect = self.rect_map
        half = self.room_half_size_world()
        x = ((mx - rect.x) / rect.w) * (half * 2) - half
        y = -((((my - rect.y) / rect.h) * (half * 2)) - half)
        return x, y

    # --------------------------------------------------------
    # Rendu
    # --------------------------------------------------------
    def draw_text(self, text: str, x: int, y: int, *, small: bool = False, color: Tuple[int, int, int] = COLOR_TEXT) -> None:
        font = self.small_font if small else self.font
        self.screen.blit(font.render(text, True, color), (x, y))

    def draw_top_left(self) -> None:
        self.draw_panel(self.rect_top_left)
        x = self.rect_top_left.x + 12
        y = self.rect_top_left.y + 12

        self.draw_text(f"frametime: {self.frame_ms:.2f} ms", x, y)
        y += 26
        self.draw_text(f"xPos: {self.robot.x:.2f}", x, y)
        y += 26
        self.draw_text(f"yPos: {self.robot.y:.2f}", x, y)
        y += 26
        self.draw_text(f"turnAngle: {(self.robot.turn_angle * 180 / math.pi) % 360:.2f}°", x, y)
        y += 26
        self.draw_text(f"echoRange: {self.current_echo_range:.2f} m", x, y)
        y += 26
        self.draw_text(f"mode: {self.current_motion_mode}", x, y)
        y += 26
        self.draw_text(f"real robot: {'on' if self.real_robot_mode else 'off'}", x, y)
        y += 26
        self.draw_text(f"goingTo: {self.robot.go_to[0]:.2f} {self.robot.go_to[1]:.2f}" if self.robot.going_somewhere or self.robot.go_to != (0.0, 0.0) else "goingTo: --", x, y)
        y += 32

        self.draw_text("steerAngle:", x, y)
        track = pygame.Rect(x + 95, y + 7, self.rect_top_left.w - 130, 8)
        pygame.draw.rect(self.screen, (220, 220, 220), track, border_radius=5)
        pygame.draw.rect(self.screen, COLOR_BORDER, track, 1, border_radius=5)
        ratio = (self.robot.steer_angle + SIM_MAX_STEER) / (SIM_MAX_STEER * 2)
        handle_x = track.x + int(track.w * ratio)
        pygame.draw.circle(self.screen, COLOR_ACCENT, (handle_x, track.centery), 8)
        pygame.draw.circle(self.screen, COLOR_BORDER, (handle_x, track.centery), 8, 1)

        mouse_pos = pygame.mouse.get_pos()
        for btn in self.buttons:
            btn.draw(self.screen, self.small_font, mouse_pos)

    def draw_top_right(self) -> None:
        self.draw_panel(self.rect_top_right)
        self.draw_text("Paramètres", self.rect_top_right.x + 12, self.rect_top_right.y + 10)
        if self.radio_group:
            self.radio_group.draw(self.screen, self.font)
        if self.checkbox:
            self.checkbox.draw(self.screen, self.font)
        if self.slider:
            self.slider.draw(self.screen, self.font)

        info_y = self.rect_top_right.y + 220
        help_lines = [
            "Flèches clavier :",
            "↑ avancer",
            "↓ reculer",
            "← / → braquage en simulation",
            "← / → rotation en robot réel",
            "E : exploration on/off",
        ]
        for line in help_lines:
            self.draw_text(line, self.rect_top_right.x + 12, info_y, small=True, color=COLOR_MUTED)
            info_y += 20

    def draw_bottom_left(self) -> None:
        self.draw_panel(self.rect_bottom_left)
        self.draw_text("Journal / mode d'emploi", self.rect_bottom_left.x + 12, self.rect_bottom_left.y + 10)
        y = self.rect_bottom_left.y + 40
        for line in self.log_lines[-14:]:
            self.draw_text(line, self.rect_bottom_left.x + 12, y, small=True)
            y += 18

    def draw_bottom_right(self) -> None:
        self.draw_panel(self.rect_bottom_right)
        inner = self.rect_bottom_right.inflate(-14, -14)
        pygame.draw.rect(self.screen, (255, 255, 255), inner)
        pygame.draw.rect(self.screen, COLOR_BORDER, inner, 1)

        bars = len(self.fps_view)
        bar_w = inner.w / bars
        for i, value in enumerate(self.fps_view):
            h = value * inner.h * 0.9
            rect = pygame.Rect(
                int(inner.x + i * bar_w + 4),
                int(inner.centery - h / 2),
                max(6, int(bar_w - 8)),
                int(h),
            )
            pygame.draw.rect(self.screen, COLOR_FPS, rect)

    def draw_map(self) -> None:
        self.draw_panel(self.rect_map)
        inner = self.rect_map.inflate(-2, -2)
        pygame.draw.rect(self.screen, (255, 255, 255), inner)

        half = self.room_half_size_world()
        cell_px = max(1, int(min(inner.w, inner.h) / (half * 2 / CELL_WORLD_SIZE)))

        # cellules explorées
        for gx in range(GRID_SIZE):
            for gy in range(GRID_SIZE):
                value = self.exploration_graph[gx][gy]
                if value == 0:
                    continue
                world_x = (gx - GRID_CENTER) * CELL_WORLD_SIZE
                world_y = (gy - GRID_CENTER) * CELL_WORLD_SIZE
                px, py = self.world_to_map_px(world_x, world_y)

                if value == 1:
                    color = COLOR_FREE
                elif value == 2:
                    color = COLOR_VISITED
                else:
                    color = COLOR_OBSTACLE

                pygame.draw.rect(self.screen, color, (px, py, cell_px, cell_px))

        # limite de pièce
        room_left_top = self.world_to_map_px(-half, half)
        room_right_bottom = self.world_to_map_px(half, -half)
        room_rect = pygame.Rect(
            room_left_top[0],
            room_left_top[1],
            room_right_bottom[0] - room_left_top[0],
            room_right_bottom[1] - room_left_top[1],
        )
        pygame.draw.rect(self.screen, COLOR_BORDER, room_rect, 2)

        # obstacles manuels
        for wall in self.walls:
            tl = self.world_to_map_px(wall.left, wall.top)
            br = self.world_to_map_px(wall.right, wall.bottom)
            wall_rect = pygame.Rect(tl[0], tl[1], br[0] - tl[0], br[1] - tl[1])
            pygame.draw.rect(self.screen, COLOR_OBSTACLE, wall_rect)
            pygame.draw.rect(self.screen, COLOR_BORDER, wall_rect, 1)

        # obstacle en cours de création
        if self.pending_wall_start is not None:
            mx, my = pygame.mouse.get_pos()
            if self.rect_map.collidepoint((mx, my)):
                x2, y2 = self.map_px_to_world(mx, my)
                preview = Wall.from_points(self.pending_wall_start, (x2, y2))
                tl = self.world_to_map_px(preview.left, preview.top)
                br = self.world_to_map_px(preview.right, preview.bottom)
                preview_rect = pygame.Rect(tl[0], tl[1], br[0] - tl[0], br[1] - tl[1])
                pygame.draw.rect(self.screen, (255, 170, 210), preview_rect)
                pygame.draw.rect(self.screen, COLOR_BORDER, preview_rect, 1)

        # rayon capteur
        start = self.world_to_map_px(self.robot.x, self.robot.y)
        end = self.world_to_map_px(
            self.robot.x + math.sin(self.robot.turn_angle) * self.current_echo_range,
            self.robot.y + math.cos(self.robot.turn_angle) * self.current_echo_range,
        )
        pygame.draw.line(self.screen, COLOR_SENSOR, start, end, 3)

        # robot (rectangle tourné)
        self.draw_rotated_robot()

    def draw_rotated_robot(self) -> None:
        cx, cy = self.world_to_map_px(self.robot.x, self.robot.y)
        px_per_world_x = self.rect_map.w / (self.room_half_size_world() * 2)
        px_per_world_y = self.rect_map.h / (self.room_half_size_world() * 2)
        rw = self.robot.width * px_per_world_x
        rh = self.robot.height * px_per_world_y

        corners = [
            (-rw / 2, -rh / 2),
            (rw / 2, -rh / 2),
            (rw / 2, rh / 2),
            (-rw / 2, rh / 2),
        ]

        points = []
        for x, y in corners:
            # rotation écran : même convention que le JS (angle 0 = vers le haut)
            rx = x * math.cos(-self.robot.turn_angle) - y * math.sin(-self.robot.turn_angle)
            ry = x * math.sin(-self.robot.turn_angle) + y * math.cos(-self.robot.turn_angle)
            points.append((cx + rx, cy + ry))

        pygame.draw.polygon(self.screen, COLOR_ROBOT, points)
        pygame.draw.polygon(self.screen, COLOR_BORDER, points, 1)

        nose = self.world_to_map_px(
            self.robot.x + math.sin(self.robot.turn_angle) * 0.12,
            self.robot.y + math.cos(self.robot.turn_angle) * 0.12,
        )
        pygame.draw.line(self.screen, (255, 210, 0), (cx, cy), nose, 3)

    def update_fps_view(self) -> None:
        values = []
        for i in range(11):
            if i < 6:
                angle = self.robot.turn_angle + (i / 5.0)
            else:
                angle = self.robot.turn_angle - ((i - 5) / 5.0)
            dist = self.get_range_from_echo(angle)
            values.append(max(0.1, 1.0 - dist))
        self.fps_view = values

    # --------------------------------------------------------
    # Boucle d'update
    # --------------------------------------------------------
    def update(self, dt: float) -> None:
        if self.real_robot_mode:
            self.sync_robot_command()
            self.update_real_robot_pose(dt)
            self.current_echo_range = self.get_range_from_echo()
        else:
            self.robot.update_manual_sim()
            self.current_echo_range = self.get_range_from_echo()
            self.handle_exploration()
            if self.robot.going_somewhere:
                self.robot.naive_going_to(self.current_echo_range)

        self.robot.turn_angle = normalize_angle(self.robot.turn_angle)
        self.mark_sensor_observation()
        self.update_fps_view()

    # --------------------------------------------------------
    # Événements
    # --------------------------------------------------------
    def handle_key_down(self, key: int) -> None:
        if key == pygame.K_LEFT:
            self.robot.going_left = True
        elif key == pygame.K_RIGHT:
            self.robot.going_right = True
        elif key == pygame.K_UP:
            self.robot.going_up = True
        elif key == pygame.K_DOWN:
            self.robot.going_down = True
        elif key == pygame.K_e:
            self.robot.exploring = not self.robot.exploring
            self.push_log(f"Exploration {'activée' if self.robot.exploring else 'désactivée'}")

    def handle_key_up(self, key: int) -> None:
        if key == pygame.K_LEFT:
            self.robot.going_left = False
        elif key == pygame.K_RIGHT:
            self.robot.going_right = False
        elif key == pygame.K_UP:
            self.robot.going_up = False
        elif key == pygame.K_DOWN:
            self.robot.going_down = False

    def handle_mouse_down(self, event: pygame.event.Event) -> None:
        pos = event.pos

        # UI
        for btn in self.buttons:
            if btn.handle_mouse_down(pos):
                return
        if self.checkbox and self.checkbox.handle_mouse_down(pos):
            return
        if self.radio_group and self.radio_group.handle_mouse_down(pos):
            return
        if self.slider and self.slider.handle_mouse_down(pos):
            return

        # Carte
        if not self.rect_map.collidepoint(pos):
            return

        if event.button == 3:  # clic droit -> destination
            self.robot.go_to = self.map_px_to_world(*pos)
            self.robot.going_somewhere = True
            self.push_log(f"Nouvelle destination : {self.robot.go_to[0]:.2f}, {self.robot.go_to[1]:.2f}")
            return

        if event.button == 1:  # clic gauche -> obstacle en 2 points
            world_pt = self.map_px_to_world(*pos)
            if self.pending_wall_start is None:
                self.pending_wall_start = world_pt
                self.push_log("Premier point d'obstacle posé")
            else:
                wall = Wall.from_points(self.pending_wall_start, world_pt)
                self.walls.append(wall)
                self.pending_wall_start = None
                self.push_log("Obstacle ajouté")

    def handle_mouse_up(self, _event: pygame.event.Event) -> None:
        if self.slider:
            self.slider.handle_mouse_up()

    def handle_mouse_motion(self, event: pygame.event.Event) -> None:
        if self.slider:
            self.slider.handle_mouse_motion(event.pos)

    # --------------------------------------------------------
    # Boucle principale
    # --------------------------------------------------------
    def run(self) -> None:
        while self.running:
            frame_start = time.perf_counter()
            dt = min(0.05, frame_start - self.last_update_time)
            self.last_update_time = frame_start

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.VIDEORESIZE:
                    self.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
                    self.rebuild_ui()
                elif event.type == pygame.KEYDOWN:
                    self.handle_key_down(event.key)
                elif event.type == pygame.KEYUP:
                    self.handle_key_up(event.key)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.handle_mouse_down(event)
                elif event.type == pygame.MOUSEBUTTONUP:
                    self.handle_mouse_up(event)
                elif event.type == pygame.MOUSEMOTION:
                    self.handle_mouse_motion(event)

            self.update(dt)
            self.draw()
            pygame.display.flip()

            self.frame_ms = (time.perf_counter() - frame_start) * 1000.0
            self.clock.tick(FPS)

        self.hardware.stop()
        self.hardware.cleanup()
        pygame.quit()

    def draw(self) -> None:
        self.screen.fill(COLOR_BG)
        self.draw_top_left()
        self.draw_map()
        self.draw_top_right()
        self.draw_bottom_right()
        self.draw_bottom_left()


def main() -> None:
    parser = argparse.ArgumentParser(description="Exploro-Bot PC controller")
    parser.add_argument('--host', default='192.168.137.92', help='IP du Raspberry Pi')
    parser.add_argument('--port', type=int, default=8765, help='Port HTTP du serveur robot')
    parser.add_argument('--url', default=None, help='URL complète, ex: http://192.168.137.92:8765')
    args = parser.parse_args()

    base_url = args.url or f"http://{args.host}:{args.port}"
    app = ExploroBotApp(base_url)
    app.run()


if __name__ == "__main__":
    main()
