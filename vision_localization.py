#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover - optional on target
    cv2 = None

try:
    from picamera2 import Picamera2
except Exception:  # pragma: no cover - optional on target
    Picamera2 = None


@dataclass
class VisionUpdate:
    camera_available: bool
    reference_reset: bool = False
    delta_angle_rad: Optional[float] = None
    elapsed_s: float = 0.0
    matches: int = 0
    keypoints1: int = 0
    keypoints2: int = 0
    detector: str = "none"
    last_yaw_deg: Optional[float] = None
    error: Optional[str] = None
    visual_scores: Dict[str, float] = field(default_factory=lambda: {"left": 0.5, "center": 0.5, "right": 0.5})
    visual_blocked: Dict[str, bool] = field(default_factory=lambda: {"left": False, "center": False, "right": False})
    visual_preferred_turn: Optional[str] = None
    visual_confidence: float = 0.0


class VisionYawLocalizer:
    """Small monocular helper dedicated to yaw correction while the robot turns.

    It keeps the spirit of the user's standalone script:
    - feature detection (SIFT when available, ORB otherwise)
    - ratio test on BFMatcher matches
    - essential matrix + recoverPose
    - median of multiple random subsets for robustness

    The estimate is only used for *rotation* correction. Position still comes from odometry.
    """

    def __init__(
        self,
        sample_period_s: float = 0.28,
        base_fx: float = 700.0,
        base_fy: float = 700.0,
        base_width: float = 640.0,
        base_height: float = 480.0,
        min_good_matches: int = 18,
        subset_size: int = 20,
        num_trials: int = 40,
        max_abs_yaw_deg: float = 35.0,
    ) -> None:
        self.sample_period_s = float(sample_period_s)
        self.base_fx = float(base_fx)
        self.base_fy = float(base_fy)
        self.base_width = float(base_width)
        self.base_height = float(base_height)
        self.min_good_matches = int(min_good_matches)
        self.subset_size = int(subset_size)
        self.num_trials = int(num_trials)
        self.max_abs_yaw_deg = float(max_abs_yaw_deg)

        self.picam2 = None
        self.camera_available = False
        self.last_error: Optional[str] = None

        self.prev_frame = None
        self.prev_mode: Optional[str] = None
        self.prev_t: float = 0.0
        self.last_refresh_t: float = 0.0

        self.last_yaw_deg: Optional[float] = None
        self.last_matches: int = 0
        self.last_detector: str = "none"
        self.last_visual_scores: Dict[str, float] = {"left": 0.5, "center": 0.5, "right": 0.5}
        self.last_visual_blocked: Dict[str, bool] = {"left": False, "center": False, "right": False}
        self.last_visual_preferred_turn: Optional[str] = None
        self.last_visual_confidence: float = 0.0

    def start(self) -> bool:
        if self.picam2 is not None and self.camera_available:
            return True
        if cv2 is None:
            self.camera_available = False
            self.last_error = "opencv indisponible"
            return False
        if Picamera2 is None:
            self.camera_available = False
            self.last_error = "picamera2 indisponible"
            return False

        try:
            self.picam2 = Picamera2()
            self.picam2.start()
            time.sleep(0.25)
            self.camera_available = True
            self.last_error = None
            self.reset_reference()
            return True
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.picam2 = None
            self.camera_available = False
            self.last_error = str(exc)
            return False

    def close(self) -> None:
        if self.picam2 is not None:
            try:
                self.picam2.stop()
            except Exception:
                pass
        self.picam2 = None
        self.camera_available = False
        self.reset_reference()

    def reset_reference(self) -> None:
        self.prev_frame = None
        self.prev_mode = None
        self.prev_t = 0.0
        self.last_refresh_t = 0.0
        self.last_yaw_deg = None
        self.last_matches = 0
        self.last_visual_scores = {"left": 0.5, "center": 0.5, "right": 0.5}
        self.last_visual_blocked = {"left": False, "center": False, "right": False}
        self.last_visual_preferred_turn = None
        self.last_visual_confidence = 0.0

    def _capture_bgr(self):
        if self.picam2 is None:
            return None
        try:
            img = self.picam2.capture_array()
        except Exception as exc:  # pragma: no cover - hardware dependent
            self.last_error = str(exc)
            return None

        if img is None:
            return None
        if len(img.shape) == 2:
            return img
        if img.shape[2] == 4:
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    def _camera_matrix(self, shape: Tuple[int, int, int]) -> np.ndarray:
        h, w = shape[:2]
        fx = self.base_fx * (w / self.base_width)
        fy = self.base_fy * (h / self.base_height)
        cx = w / 2.0
        cy = h / 2.0
        return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)

    def _create_detector(self):
        if hasattr(cv2, "SIFT_create"):
            return cv2.SIFT_create(), cv2.NORM_L2, "SIFT"
        return cv2.ORB_create(nfeatures=1400, fastThreshold=8), cv2.NORM_HAMMING, "ORB"

    @staticmethod
    def _compute_yaw_from_points(pts1: np.ndarray, pts2: np.ndarray, K: np.ndarray) -> Optional[float]:
        E, _mask = cv2.findEssentialMat(
            pts1,
            pts2,
            K,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0,
        )
        if E is None:
            return None
        _, R, _t, _pose_mask = cv2.recoverPose(E, pts1, pts2, K)
        yaw = np.arctan2(R[0, 2], R[2, 2])
        return float(np.degrees(yaw))

    def estimate_yaw_deg(self, img1, img2) -> Tuple[Optional[float], Dict[str, int]]:
        meta = {"matches": 0, "keypoints1": 0, "keypoints2": 0, "detector": "none"}
        if cv2 is None or img1 is None or img2 is None:
            return None, meta

        gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

        detector, norm, detector_name = self._create_detector()
        meta["detector"] = detector_name
        self.last_detector = detector_name

        kp1, des1 = detector.detectAndCompute(gray1, None)
        kp2, des2 = detector.detectAndCompute(gray2, None)
        meta["keypoints1"] = 0 if kp1 is None else len(kp1)
        meta["keypoints2"] = 0 if kp2 is None else len(kp2)

        if des1 is None or des2 is None or meta["keypoints1"] < 24 or meta["keypoints2"] < 24:
            return None, meta

        bf = cv2.BFMatcher(norm)
        try:
            matches = bf.knnMatch(des1, des2, k=2)
        except Exception:
            return None, meta

        good = []
        for pair in matches:
            if len(pair) < 2:
                continue
            m, n = pair
            if m.distance < 0.75 * n.distance:
                good.append(m)

        meta["matches"] = len(good)
        self.last_matches = len(good)
        if len(good) < self.min_good_matches:
            return None, meta

        pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good])
        K = self._camera_matrix(img1.shape)

        yaws = []
        subset_size = min(self.subset_size, len(pts1))
        if subset_size < 8:
            return None, meta
        trials = min(self.num_trials, max(12, len(pts1)))

        for _ in range(trials):
            try:
                idx = np.random.choice(len(pts1), subset_size, replace=False)
            except Exception:
                break
            yaw = self._compute_yaw_from_points(pts1[idx], pts2[idx], K)
            if yaw is None:
                continue
            if abs(yaw) <= self.max_abs_yaw_deg:
                yaws.append(float(yaw))

        if not yaws:
            return None, meta

        median_yaw = float(np.median(np.asarray(yaws, dtype=np.float32)))
        return median_yaw, meta

    def analyze_visual_opening(self, img) -> Tuple[Dict[str, float], Dict[str, bool], Optional[str], float]:
        if cv2 is None or img is None:
            return dict(self.last_visual_scores), dict(self.last_visual_blocked), self.last_visual_preferred_turn, self.last_visual_confidence

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        y0 = int(h * 0.40)
        roi = gray[y0:, :]
        blur = cv2.GaussianBlur(roi, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 130)
        lap = cv2.Laplacian(blur, cv2.CV_32F)
        grad = np.abs(lap)

        thirds = {
            "left": (0, w // 3),
            "center": (w // 3, (2 * w) // 3),
            "right": ((2 * w) // 3, w),
        }
        scores: Dict[str, float] = {}
        blocked: Dict[str, bool] = {}
        for name, (x0, x1) in thirds.items():
            edge_crop = edges[:, x0:x1]
            grad_crop = grad[:, x0:x1]
            base_crop = blur[:, x0:x1]
            if edge_crop.size == 0:
                scores[name] = 0.5
                blocked[name] = False
                continue

            edge_density = float(np.mean(edge_crop > 0))
            texture = float(np.mean(np.clip(grad_crop / 255.0, 0.0, 1.0)))
            brightness = float(np.mean(base_crop) / 255.0)

            bottom_h = max(8, edge_crop.shape[0] // 3)
            bottom_edges = edge_crop[-bottom_h:, :]
            bottom_density = float(np.mean(bottom_edges > 0))

            open_score = 1.0 - (1.45 * edge_density + 0.95 * bottom_density + 0.55 * texture) + 0.18 * brightness
            open_score = float(np.clip(open_score, 0.0, 1.0))
            scores[name] = open_score
            blocked[name] = (open_score < 0.36) or (bottom_density > 0.17)

        # Hysteresis toward the previous estimate to reduce flicker.
        for name in ("left", "center", "right"):
            prev = self.last_visual_scores.get(name, 0.5)
            scores[name] = float(0.35 * prev + 0.65 * scores[name])
            blocked[name] = blocked[name] or (self.last_visual_blocked.get(name, False) and scores[name] < 0.44)

        preferred = None
        spread = scores["right"] - scores["left"]
        center_penalty = max(0.0, 0.58 - scores["center"])
        confidence = float(np.clip(abs(spread) * 1.55 + center_penalty * 0.8, 0.0, 1.0))
        if scores["center"] < 0.42 or abs(spread) > 0.08:
            preferred = "right" if spread >= 0 else "left"

        self.last_visual_scores = dict(scores)
        self.last_visual_blocked = dict(blocked)
        self.last_visual_preferred_turn = preferred
        self.last_visual_confidence = confidence
        return scores, blocked, preferred, confidence

    def _fill_visual(self, update: VisionUpdate) -> None:
        update.visual_scores = dict(self.last_visual_scores)
        update.visual_blocked = dict(self.last_visual_blocked)
        update.visual_preferred_turn = self.last_visual_preferred_turn
        update.visual_confidence = float(self.last_visual_confidence)

    def update(self, mode: str) -> VisionUpdate:
        now = time.time()
        turning = mode in {"left", "right"}
        update = VisionUpdate(camera_available=self.camera_available)

        if not self.camera_available or self.picam2 is None:
            update.error = self.last_error
            self._fill_visual(update)
            return update

        # When the robot is not turning, keep refreshing a reference frame from time to time.
        if not turning:
            if self.prev_frame is None or (now - self.last_refresh_t) >= 0.60:
                frame = self._capture_bgr()
                if frame is not None:
                    self.prev_frame = frame
                    self.prev_t = now
                    self.prev_mode = mode
                    self.last_refresh_t = now
                    update.reference_reset = True
                    self.analyze_visual_opening(frame)
            update.matches = self.last_matches
            update.last_yaw_deg = self.last_yaw_deg
            update.detector = self.last_detector
            update.error = self.last_error
            self._fill_visual(update)
            return update

        if self.prev_frame is None or self.prev_mode != mode:
            frame = self._capture_bgr()
            if frame is not None:
                self.prev_frame = frame
                self.prev_t = now
                self.prev_mode = mode
                self.last_refresh_t = now
                update.reference_reset = True
                self.analyze_visual_opening(frame)
            update.matches = self.last_matches
            update.last_yaw_deg = self.last_yaw_deg
            update.detector = self.last_detector
            update.error = self.last_error
            self._fill_visual(update)
            return update

        elapsed = now - self.prev_t
        if elapsed < self.sample_period_s:
            update.matches = self.last_matches
            update.last_yaw_deg = self.last_yaw_deg
            update.detector = self.last_detector
            update.error = self.last_error
            self._fill_visual(update)
            return update

        frame2 = self._capture_bgr()
        if frame2 is None:
            update.matches = self.last_matches
            update.last_yaw_deg = self.last_yaw_deg
            update.detector = self.last_detector
            update.error = self.last_error
            self._fill_visual(update)
            return update

        yaw_deg, meta = self.estimate_yaw_deg(self.prev_frame, frame2)
        self.analyze_visual_opening(frame2)
        self.prev_frame = frame2
        self.prev_t = now
        self.prev_mode = mode
        self.last_refresh_t = now

        update.matches = meta.get("matches", 0)
        update.keypoints1 = meta.get("keypoints1", 0)
        update.keypoints2 = meta.get("keypoints2", 0)
        update.detector = meta.get("detector", "none")
        update.elapsed_s = elapsed
        update.error = self.last_error
        self._fill_visual(update)

        if yaw_deg is None:
            update.last_yaw_deg = self.last_yaw_deg
            return update

        # Keep the user's logic but force the sign from the commanded turn direction.
        signed_deg = abs(float(yaw_deg)) * (-1.0 if mode == "left" else 1.0)
        self.last_yaw_deg = signed_deg
        update.last_yaw_deg = signed_deg
        update.delta_angle_rad = math.radians(signed_deg)
        return update
