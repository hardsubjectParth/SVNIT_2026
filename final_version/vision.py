import cv2
import numpy as np
from ultralytics import YOLO
import config_manager

class VisionSystem:
    def __init__(self):
        conf = config_manager.load_config()
        self.model = YOLO("landing_h.pt")
        self.conf_threshold = conf['min_confidence']
        self.fov_rad = np.deg2rad(conf['camera_fov'])
        self.res = (640, 480)
        
        # Video recording setup
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter('mission_log.avi', fourcc, 20.0, self.res)

    def get_target(self, frame, alt):
        results = self.model(frame, conf=self.conf_threshold, verbose=False)
        target = None
        
        for r in results:
            if len(r.boxes) > 0:
                box = r.boxes[0].xywh[0] # x_center, y_center, w, h
                px_x, px_y = float(box[0]), float(box[1])
                
                # Pixels to Meters conversion
                view_w = 2 * alt * np.tan(self.fov_rad / 2)
                m_per_px = view_w / self.res[0]
                
                off_x = (px_x - (self.res[0]/2)) * m_per_px
                off_y = (px_y - (self.res[1]/2)) * m_per_px
                target = (off_x, off_y) # Meters from center
                
                # Visual feedback for recording
                cv2.rectangle(frame, (int(px_x-10), int(px_y-10)), (int(px_x+10), int(px_y+10)), (0,255,0), 2)
        
        self.out.write(frame)
        return target

    def __del__(self):
        if hasattr(self, 'out'):
            self.out.release()