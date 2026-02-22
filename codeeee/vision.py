"""
vision.py — Camera pipeline + YOLO detection.

Camera runs in a ThreadPoolExecutor so cv2.read() never blocks event loop.
Detection result → SharedState.detection (off_x, off_y) in meters.
"""

import asyncio
import math
import time
import numpy as np
import cv2
from concurrent.futures import ThreadPoolExecutor
from ultralytics import YOLO
from state import SharedState
from geo import compute_mpp


class CameraPipeline:

    def __init__(self, config: dict, state: SharedState):
        self.config = config
        self.state = state
        self.frame_queue: asyncio.Queue = asyncio.Queue(maxsize=2)
        self._executor = ThreadPoolExecutor(max_workers=1)
        self._cap = None
        self._model = None
        self._running = False

    # =====================================================================
    # STARTUP
    # =====================================================================

    def _load_model(self):
        print("[VISION] Loading YOLOv8 model...")
        self._model = YOLO("best_yolov8s.pt")
        # Warm-up inference
        dummy = np.zeros((self.config["image_h"], self.config["image_w"], 3), dtype=np.uint8)
        self._model(dummy, verbose=False)
        print("[VISION] Model loaded and warmed up.")

    def _open_camera(self) -> bool:
        idx = self.config.get("camera_index", 0)
        self._cap = cv2.VideoCapture(idx)
        if not self._cap.isOpened():
            print(f"[VISION] ✗ Camera {idx} failed to open")
            return False
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config["image_w"])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config["image_h"])
        print(f"[VISION] ✓ Camera {idx} opened "
              f"({self.config['image_w']}x{self.config['image_h']})")
        return True

    def _blocking_read(self):
        """Blocking camera read — runs in executor thread."""
        if self._cap is None:
            return False, None
        ret, frame = self._cap.read()
        return ret, frame

    # =====================================================================
    # MAIN LOOP
    # =====================================================================

    async def run(self):
        loop = asyncio.get_event_loop()
        self._load_model()
        ok = self._open_camera()
        if not ok:
            print("[VISION] Camera unavailable — detection disabled")
            return

        self._running = True
        fail_count = 0
        max_fails = self.config.get("max_camera_fails", 50)

        while True:
            ret, frame = await loop.run_in_executor(self._executor, self._blocking_read)

            if not ret or frame is None:
                fail_count += 1
                self.state.camera_fail_count = fail_count
                if fail_count >= max_fails:
                    print(f"[VISION] ✗ Camera failed {fail_count} times — triggering failsafe")
                    self.state.abort_reason = "Camera failure"
                    from state import MissionPhase
                    if self.state.phase not in (MissionPhase.RTL, MissionPhase.COMPLETE,
                                                 MissionPhase.IDLE, MissionPhase.READY):
                        self.state.phase = MissionPhase.ABORT
                await asyncio.sleep(0.01)
                continue

            fail_count = 0
            self.state.camera_fail_count = 0
            self.state.latest_frame = frame

            # Drain queue if full so we always have latest frame
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            try:
                self.frame_queue.put_nowait(frame)
            except asyncio.QueueFull:
                pass

            await asyncio.sleep(0)   # yield to event loop

    # =====================================================================
    # DETECTION
    # =====================================================================

    def detect(self, frame, alt: float):
        """
        Run YOLO on frame. Returns (off_x_m, off_y_m, confidence) or None.
        off_x: positive = target is RIGHT of center
        off_y: positive = target is FORWARD (down in image) of center
        NOTE: Validate sign convention vs your camera mount before flight!
        """
        if self._model is None or frame is None:
            return None

        results = self._model(frame, verbose=False)
        if not results or len(results[0].boxes) == 0:
            return None

        box = results[0].boxes[0]
        conf = float(box.conf[0])

        if conf < self.config["min_confidence"]:
            return None

        xywh = box.xywh[0]
        cx, cy = float(xywh[0]), float(xywh[1])

        W = self.config["image_w"]
        H = self.config["image_h"]

        # Separate mpp for X and Y axes (aspect ratio fix)
        mpp_x = compute_mpp(alt, self.config["camera_hfov_deg"], W)
        mpp_y = compute_mpp(alt, self.config["camera_vfov_deg"], H)

        off_x = (cx - W / 2) * mpp_x   # meters right of center
        off_y = (cy - H / 2) * mpp_y   # meters forward of center

        return off_x, off_y, conf

    def get_latest_frame(self):
        try:
            return self.frame_queue.get_nowait()
        except asyncio.QueueEmpty:
            return self.state.latest_frame   # use last known (max ~33ms stale)

    # =====================================================================
    # LIVE TEST MODES (called from CLI)
    # =====================================================================

    async def test_raw(self):
        """Show raw camera feed until 'q' pressed."""
        loop = asyncio.get_event_loop()
        print("[VISION] Raw camera test — press Q to quit")
        while True:
            ret, frame = await loop.run_in_executor(self._executor, self._blocking_read)
            if ret:
                cv2.imshow("Camera Test — RAW", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    async def test_detect(self):
        """Show camera feed with YOLO bounding boxes until 'q' pressed."""
        loop = asyncio.get_event_loop()
        print("[VISION] Detection test — press Q to quit")
        while True:
            ret, frame = await loop.run_in_executor(self._executor, self._blocking_read)
            if ret:
                alt = self.state.alt_agl if self.state.alt_agl > 0.5 else 5.0
                result = self.detect(frame, alt)

                # Draw detections
                res = self._model(frame, verbose=False)
                for r in res:
                    if r.boxes is not None:
                        for b in r.boxes.xyxy:
                            x1, y1, x2, y2 = map(int, b)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Draw crosshair
                H, W = frame.shape[:2]
                cv2.line(frame, (W//2 - 20, H//2), (W//2 + 20, H//2), (0, 0, 255), 1)
                cv2.line(frame, (W//2, H//2 - 20), (W//2, H//2 + 20), (0, 0, 255), 1)

                if result:
                    ox, oy, conf = result
                    cv2.putText(frame, f"off_x={ox:.2f}m off_y={oy:.2f}m conf={conf:.2f}",
                                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                cv2.imshow("Detection Test — YOLOv8", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

    def release(self):
        if self._cap:
            self._cap.release()
        self._executor.shutdown(wait=False)
