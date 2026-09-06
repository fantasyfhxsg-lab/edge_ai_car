#!/usr/bin/env python3
"""
AEB 自动紧急制动控制器
状态机: NORMAL → WARN → BRAKE → FAULT，带迟滞防止抖动
"""
from enum import Enum

class AEBState(Enum):
    NORMAL = "NORMAL"
    WARN = "WARN"
    BRAKE = "BRAKE"
    FAULT = "FAULT"

class AEBController:
    def __init__(self, safe_distance=2.0, danger_distance=1.0,
                 hysteresis=0.3, warn_speed_limit=0.5):
        self.state = AEBState.NORMAL
        self.safe_distance = safe_distance
        self.danger_distance = danger_distance
        self.hysteresis = hysteresis
        self.warn_speed_limit = warn_speed_limit
        self.brake_command = False
        self.target_speed_scale = 1.0

    def update(self, min_distance, valid=True):
        """更新状态，返回(是否制动, 速度比例, 状态)"""
        if not valid:
            self.state = AEBState.FAULT
            self.brake_command = True
            self.target_speed_scale = 0.0
            return self.brake_command, self.target_speed_scale, self.state
        if min_distance is None:
            if self.state != AEBState.FAULT:
                self.state = AEBState.NORMAL
                self.brake_command = False
                self.target_speed_scale = 1.0
            return self.brake_command, self.target_speed_scale, self.state

        if self.state == AEBState.NORMAL:
            if min_distance < self.danger_distance:
                self.state = AEBState.BRAKE; self.brake_command = True; self.target_speed_scale = 0.0
            elif min_distance < self.safe_distance:
                self.state = AEBState.WARN; self.target_speed_scale = self.warn_speed_limit
        elif self.state == AEBState.WARN:
            if min_distance < self.danger_distance:
                self.state = AEBState.BRAKE; self.brake_command = True; self.target_speed_scale = 0.0
            elif min_distance > self.safe_distance + self.hysteresis:
                self.state = AEBState.NORMAL; self.target_speed_scale = 1.0
        elif self.state == AEBState.BRAKE:
            if min_distance > self.safe_distance + self.hysteresis:
                self.state = AEBState.NORMAL; self.brake_command = False; self.target_speed_scale = 1.0
        elif self.state == AEBState.FAULT:
            self.brake_command = True; self.target_speed_scale = 0.0
        return self.brake_command, self.target_speed_scale, self.state

    def reset(self):
        self.state = AEBState.NORMAL; self.brake_command = False; self.target_speed_scale = 1.0

def estimate_distance(class_id, box_height_px, focal_length=800):
    """单目测距: distance = focal * real_height / pixel_height"""
    real_heights = {0: 1.7, 1: 1.5, 2: 0.7}
    if box_height_px <= 0: return None
    return round(focal_length * real_heights.get(class_id, 1.0) / box_height_px, 2)

if __name__ == '__main__':
    aeb = AEBController()
    for d in [5.0, 3.0, 2.5, 1.8, 1.2, 0.8, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]:
        brake, speed, state = aeb.update(d)
        print(f"距离={d:.1f}m → {state.value:6s} 制动={brake} 速度比例={speed}")
