#!/usr/bin/env python3
"""AEB系统HIL测试用例，运行: python3 -m pytest test_aeb.py -v"""
import pytest, time, cv2, numpy as np, os, sys
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
from hil_framework import HILFramework

@pytest.fixture(scope='module')
def hil():
    framework=HILFramework();framework.setup_vcan();framework.start_capture()
    yield framework;framework.stop_capture()

def test_aeb_normal_brake(hil):
    """障碍物靠近应触发制动"""
    hil.captured_messages.clear()
    for i in range(30):
        frame=np.zeros((480,640,3),dtype=np.uint8)
        size=50+i*10
        cv2.rectangle(frame,(320-size//2,240-size//2),(320+size//2,240+size//2),(0,255,0),-1)
        hil.inject_image(frame);time.sleep(0.033)
    assert hil.assert_brake_triggered(),"障碍物靠近时未触发制动"

def test_aeb_no_false_alarm(hil):
    """空旷道路不应触发制动"""
    hil.captured_messages.clear()
    for i in range(30):
        frame=np.zeros((480,640,3),dtype=np.uint8);frame[:]=(135,206,235)
        hil.inject_image(frame);time.sleep(0.033)
    assert hil.assert_no_brake(duration=1.0),"空旷道路误触发制动"

def test_heartbeat_timeout(hil):
    """STM32心跳丢失应进入故障态"""
    hil.captured_messages.clear()
    for i in range(5):hil.inject_can(0x103,bytes([i,0,0,0,0,0,0,0]));time.sleep(0.2)
    time.sleep(1.0)
    brake_msgs=[m for m in hil.captured_messages if m.arbitration_id==0x100 and len(m.data)>3 and m.data[3]==0x01]
    assert len(brake_msgs)>0,"心跳超时后未进入故障态"

def test_can_message_format(hil):
    """CAN控制帧格式正确"""
    hil.captured_messages.clear()
    frame=np.zeros((480,640,3),dtype=np.uint8)
    cv2.rectangle(frame,(200,100),(440,380),(0,255,0),-1)
    hil.inject_image(frame);time.sleep(0.5)
    control_msgs=[m for m in hil.captured_messages if m.arbitration_id==0x100]
    assert len(control_msgs)>0,"未收到控制帧"
    msg=control_msgs[-1]
    assert len(msg.data)==8,f"数据长度应为8，实际{len(msg.data)}"
    assert msg.data[0] in [0x00,0x01],f"控制字无效:{msg.data[0]}"
    assert msg.data[3] in [0x00,0x01],f"制动指令无效:{msg.data[3]}"

if __name__=='__main__':
    pytest.main([__file__,'-v'])
