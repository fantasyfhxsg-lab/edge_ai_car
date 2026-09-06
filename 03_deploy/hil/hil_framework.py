#!/usr/bin/env python3
"""HIL在环测试框架：模拟传感器输入注入AI板，捕获CAN输出，自动验证"""
import time, subprocess, threading, can, cv2, numpy as np
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class CANMessage:
    timestamp: float; arbitration_id: int; data: bytes

class HILFramework:
    def __init__(self, can_interface='vcan0'):
        self.can_interface=can_interface;self.bus=None
        self.captured_messages:List[CANMessage]=[];self.capture_running=False
    def setup_vcan(self):
        """设置虚拟CAN接口"""
        subprocess.run(['sudo','modprobe','vcan'],check=True)
        subprocess.run(['sudo','ip','link','add','dev',self.can_interface,'type','vcan'],check=False)
        subprocess.run(['sudo','ip','link','set','up',self.can_interface],check=True)
        print(f"[HIL] 虚拟CAN {self.can_interface} 已就绪")
    def start_capture(self):
        self.bus=can.interface.Bus(channel=self.can_interface,bustype='socketcan')
        self.captured_messages=[];self.capture_running=True
        threading.Thread(target=self._capture_loop,daemon=True).start()
    def stop_capture(self):
        self.capture_running=False
        if self.bus:self.bus.shutdown()
    def _capture_loop(self):
        while self.capture_running:
            try:
                msg=self.bus.recv(timeout=0.1)
                if msg:self.captured_messages.append(CANMessage(time.time(),msg.arbitration_id,bytes(msg.data)))
            except:break
    def inject_image(self,frame):
        """注入一帧图像（通过文件+标志文件模拟）"""
        cv2.imwrite('/tmp/hil_input.jpg',frame)
        with open('/tmp/hil_new_frame','w') as f:f.write(str(time.time()))
    def inject_can(self,can_id,data):
        self.bus.send(can.Message(arbitration_id=can_id,data=list(data),is_extended_id=False))
    def assert_brake_triggered(self,max_wait=2.0):
        start=time.time()
        while time.time()-start<max_wait:
            if any(m.arbitration_id==0x100 and len(m.data)>3 and m.data[3]==0x01 for m in self.captured_messages):
                return True
            time.sleep(0.05)
        return False
    def assert_no_brake(self,duration=2.0):
        time.sleep(duration)
        return not any(m.arbitration_id==0x100 and len(m.data)>3 and m.data[3]==0x01 for m in self.captured_messages)
