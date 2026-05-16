#!/usr/bin/env python3
import math
import sys
import cv2
import numpy as np

# ── Force the real pip mediapipe from venv, not the ROS stub ──────────
_REAL_MP = '/opt/venv/lib/python3.12/site-packages'
if _REAL_MP not in sys.path:
    sys.path.insert(0, _REAL_MP)
# Remove any already-cached ROS stub
for _k in list(sys.modules.keys()):
    if _k == 'mediapipe' or _k.startswith('mediapipe.'):
        del sys.modules[_k]
import mediapipe as mp
# ──────────────────────────────────────────────────────────────────────


class HandState:
    def __init__(self):
        self.detected       = False
        self.wrist_x        = 0.5
        self.wrist_y        = 0.5
        self.fist_closed    = False
        self.gripper_closed = False
        self.rotation       = 0.0


class HandTracker:
    WRIST      = 0
    THUMB_TIP  = 4
    INDEX_TIP  = 8
    INDEX_MCP  = 5
    MIDDLE_TIP = 12
    MIDDLE_MCP = 9
    RING_TIP   = 16
    PINKY_TIP  = 20

    def __init__(self, pinch_threshold: float = 0.07):
        self.pinch_threshold = pinch_threshold
        self.mp_hands   = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_styles  = mp.solutions.drawing_styles
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6,
        )

    def process_frame(self, bgr_frame):
        left  = HandState()
        right = HandState()
        rgb = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False
        results = self.hands.process(rgb)
        annotated = bgr_frame.copy()

        if results.multi_hand_landmarks and results.multi_handedness:
            for lm, handedness in zip(
                    results.multi_hand_landmarks,
                    results.multi_handedness):
                is_left = (handedness.classification[0].label == 'Right')
                state = left if is_left else right
                self._fill_state(state, lm)
                self.mp_drawing.draw_landmarks(
                    annotated, lm,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_styles.get_default_hand_landmarks_style(),
                    self.mp_styles.get_default_hand_connections_style())

        return left, right, annotated

    def _fill_state(self, state: HandState, landmarks):
        lm = landmarks.landmark
        state.detected = True
        state.wrist_x  = lm[self.WRIST].x
        state.wrist_y  = lm[self.WRIST].y

        def dist(a, b):
            return math.hypot(lm[a].x - lm[b].x, lm[a].y - lm[b].y)

        state.fist_closed = (
            dist(self.INDEX_TIP,  self.WRIST) < dist(self.INDEX_MCP,  self.WRIST) and
            dist(self.MIDDLE_TIP, self.WRIST) < dist(self.MIDDLE_MCP, self.WRIST) and
            dist(self.RING_TIP,   self.WRIST) < dist(self.INDEX_MCP,  self.WRIST) and
            dist(self.PINKY_TIP,  self.WRIST) < dist(self.INDEX_MCP,  self.WRIST)
        )
        state.gripper_closed = (
            dist(self.THUMB_TIP, self.INDEX_TIP) < self.pinch_threshold)

        dx = lm[self.MIDDLE_MCP].x - lm[self.WRIST].x
        dy = lm[self.MIDDLE_MCP].y - lm[self.WRIST].y
        state.rotation = math.atan2(-dy, dx)

    def destroy(self):
        self.hands.close()