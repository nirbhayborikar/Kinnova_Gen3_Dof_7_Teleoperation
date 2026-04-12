"""
Hand tracking using MediaPipe Hands.
Detects both hands, extracts wrist position, rotation, pinch gesture,
and finger counting (for start/stop toggle).
"""

import cv2
import numpy as np
import mediapipe as mp
from dataclasses import dataclass


@dataclass
class HandState:
    """Processed state of one hand."""
    wrist_x: float = 0.5
    wrist_y: float = 0.5
    rotation: float = 0.0
    pinch_distance: float = 1.0
    gripper_closed: bool = False
    detected: bool = False
    fist_closed: bool = False  # True when all fingers are curled (fist)


class HandTracker:
    """Tracks left and right hands using MediaPipe."""

    WRIST = 0
    THUMB_TIP = 4
    THUMB_IP = 3
    INDEX_TIP = 8
    INDEX_PIP = 6
    MIDDLE_TIP = 12
    MIDDLE_PIP = 10
    RING_TIP = 16
    RING_PIP = 14
    PINKY_TIP = 20
    PINKY_PIP = 18
    MIDDLE_MCP = 9

    def __init__(self, pinch_threshold=0.07):
        self.pinch_threshold = pinch_threshold
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.6,
        )

    def process_frame(self, frame_bgr):
        """
        Process BGR frame → (left_hand, right_hand, annotated_frame).
        Image is flipped for selfie view.
        """
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        frame_rgb = cv2.flip(frame_rgb, 1)
        frame_rgb.flags.writeable = False
        results = self.hands.process(frame_rgb)
        frame_rgb.flags.writeable = True
        annotated = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        left = HandState()
        right = HandState()

        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_lm, handedness in zip(
                results.multi_hand_landmarks, results.multi_handedness
            ):
                self.mp_draw.draw_landmarks(
                    annotated, hand_lm, self.mp_hands.HAND_CONNECTIONS
                )
                label = handedness.classification[0].label
                state = self._extract(hand_lm)

                if label == "Left":
                    left = state
                else:
                    right = state

        return left, right, annotated

    def _extract(self, hand_lm):
        """Extract HandState from landmarks."""
        lm = hand_lm.landmark

        wrist_x = lm[self.WRIST].x
        wrist_y = lm[self.WRIST].y

        # Rotation
        dx = lm[self.MIDDLE_MCP].x - lm[self.WRIST].x
        dy = lm[self.MIDDLE_MCP].y - lm[self.WRIST].y
        rotation = np.arctan2(dx, -dy)

        # Pinch distance
        pinch = np.sqrt(
            (lm[self.THUMB_TIP].x - lm[self.INDEX_TIP].x) ** 2
            + (lm[self.THUMB_TIP].y - lm[self.INDEX_TIP].y) ** 2
        )

        # Fist detection: check if all 4 finger tips are below their PIP joints
        # (finger curled = tip is closer to wrist than PIP in Y direction)
        fingers_curled = 0
        finger_pairs = [
            (self.INDEX_TIP, self.INDEX_PIP),
            (self.MIDDLE_TIP, self.MIDDLE_PIP),
            (self.RING_TIP, self.RING_PIP),
            (self.PINKY_TIP, self.PINKY_PIP),
        ]
        for tip, pip_joint in finger_pairs:
            if lm[tip].y > lm[pip_joint].y:  # tip below PIP = finger curled
                fingers_curled += 1

        # Also check thumb (tip closer to wrist than IP joint in x-direction)
        thumb_curled = abs(lm[self.THUMB_TIP].x - lm[self.WRIST].x) < \
                       abs(lm[self.THUMB_IP].x - lm[self.WRIST].x)
        if thumb_curled:
            fingers_curled += 1

        fist_closed = fingers_curled >= 4  # At least 4 of 5 fingers curled

        return HandState(
            wrist_x=wrist_x,
            wrist_y=wrist_y,
            rotation=rotation,
            pinch_distance=pinch,
            gripper_closed=pinch < self.pinch_threshold,
            detected=True,
            fist_closed=fist_closed,
        )

    def destroy(self):
        self.hands.close()
