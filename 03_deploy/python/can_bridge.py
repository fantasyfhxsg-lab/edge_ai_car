#!/usr/bin/env python3
"""
CAN通信桥接模块（AI板 ↔ STM32）
功能: CAN收发 + 心跳监控 + 故障检测
依赖: pip3 install python-can
使用前: sudo ip link set can0 type can bitrate 500000 && sudo ip link set can0 up
"""
import can
import time
import threading
from collections import deque

CAN_ID_CONTROL = 0x100
CAN_ID_STATUS = 0x101
CAN_ID_HEARTBEAT_AI = 0x102
CAN_ID_HEARTBEAT_STM32 = 0x103

class CANBridge:
    def __init__(self, channel='can0', bitrate=500000, heartbeat_timeout=0.5):
        self.channel = channel
        self.bitrate = bitrate
        self.heartbeat_timeout = heartbeat_timeout
        self.bus = None
        self.captured_messages = deque(maxlen=1000)
        self.running = False
        self.last_stm32_heartbeat = time.time()
        self.ai_heartbeat_counter = 0
        self.status = {'speed': 0, 'voltage': 0, 'fault_code': 0, 'status_word': 0, 'connected': False}

    def start(self):
        try:
            self.bus = can.interface.Bus(channel=self.channel, bustype='socketcan')
        except Exception as e:
            print(f"[CAN] 打开失败: {e}")
            print(f"[CAN] 请先执行: sudo ip link set {self.channel} type can bitrate {self.bitrate} && sudo ip link set {self.channel} up")
            return False
        self.running = True
        threading.Thread(target=self._recv_loop, daemon=True).start()
        threading.Thread(target=self._heartbeat_loop, daemon=True).start()
        print(f"[CAN] 启动: {self.channel} @ {self.bitrate}bps")
        return True

    def stop(self):
        self.running = False
        if self.bus: self.bus.shutdown()

    def _recv_loop(self):
        while self.running:
            try:
                msg = self.bus.recv(timeout=0.1)
                if msg is None: continue
                self.captured_messages.append({'timestamp': time.time(), 'id': msg.arbitration_id, 'data': bytes(msg.data)})
                if msg.arbitration_id == CAN_ID_STATUS:
                    self.status['status_word'] = msg.data[0]
                    self.status['speed'] = msg.data[1]
                    self.status['voltage'] = msg.data[2] / 10.0
                    self.status['fault_code'] = msg.data[3]
                elif msg.arbitration_id == CAN_ID_HEARTBEAT_STM32:
                    self.last_stm32_heartbeat = time.time()
                    self.status['connected'] = True
            except Exception as e:
                print(f"[CAN] 接收错误: {e}"); time.sleep(0.5)

    def _heartbeat_loop(self):
        while self.running:
            try:
                data = [self.ai_heartbeat_counter & 0xFF, 0,0,0,0,0,0,0]
                self.bus.send(can.Message(arbitration_id=CAN_ID_HEARTBEAT_AI, data=data, is_extended_id=False))
                self.ai_heartbeat_counter = (self.ai_heartbeat_counter + 1) % 256
            except Exception as e:
                print(f"[CAN] 心跳发送失败: {e}")
            time.sleep(1.0)

    def send_control(self, target_speed=0, steering=0, brake=False, enable=True):
        """发送控制指令: target_speed(-100~100), steering(-100~100)"""
        data = [0x01 if enable else 0x00, target_speed & 0xFF, steering & 0xFF,
                0x01 if brake else 0x00, 0,0,0,0]
        try:
            self.bus.send(can.Message(arbitration_id=CAN_ID_CONTROL, data=data, is_extended_id=False))
            return True
        except Exception as e:
            print(f"[CAN] 发送失败: {e}"); return False

    def is_connected(self):
        if time.time() - self.last_stm32_heartbeat > self.heartbeat_timeout:
            self.status['connected'] = False; return False
        return True

    def get_status(self):
        return self.status.copy()

if __name__ == '__main__':
    bridge = CANBridge()
    if bridge.start():
        try:
            while True:
                bridge.send_control(target_speed=30)
                print(f"连接:{bridge.is_connected()} 状态:{bridge.get_status()}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            bridge.stop()
