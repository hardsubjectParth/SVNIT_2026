import cv2
from ultralytics import YOLO

class HDetector:
    def __init__(self, model_path='best.pt', confidence=0.5):
        # Load the YOLOv8 model (Nano version recommended for Pi 5)
        self.model = YOLO(model_path)
        self.confidence = confidence
        print(f"[H_DETECTOR] Model loaded: {model_path} with conf: {confidence}")

    def get_h_coords(self, frame):
        """
        Returns: (found_boolean, center_x, center_y)
        Coordinates are normalized relative to image center.
        """
        if frame is None:
            return False, 0, 0

        # Run inference
        results = self.model.predict(frame, conf=self.confidence, verbose=False)
        
        for r in results:
            for box in r.boxes:
                # Get coordinates
                x1, y1, x2, y2 = box.xyxy[0]
                center_x = int((x1 + x2) / 2)
                center_y = int((y1 + y2) / 2)

                # Get image dimensions
                h, w, _ = frame.shape
                
                # Calculate pixel offset from image center
                # Result is in pixels where center is (0,0)
                offset_x = center_x - (w / 2)
                offset_y = center_y - (h / 2)
                
                return True, offset_x, offset_y

        return False, 0, 0