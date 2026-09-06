#!/usr/bin/env python3
"""日志回放工具：按时间戳同步播放图像+CAN+决策，用法: python3 replay.py --log aeb_log.bin"""
import argparse, struct, time, cv2, numpy as np
from collections import deque

class LogReplayer:
    MAGIC=b'CARL'
    def __init__(self,log_file):self.log_file=log_file;self.entries=[];self.start_time=0
    def load(self):
        with open(self.log_file,'rb') as f:
            magic=f.read(4)
            if magic!=self.MAGIC:raise ValueError(f"无效文件: {magic}")
            version=struct.unpack('<I',f.read(4))[0]
            self.start_time=struct.unpack('<Q',f.read(8))[0]
            print(f"版本: {version}, 开始时间戳: {self.start_time}")
            while True:
                type_data=f.read(1)
                if not type_data:break
                entry_type=type_data[0]
                timestamp=struct.unpack('<Q',f.read(8))[0]
                length=struct.unpack('<I',f.read(4))[0]
                data=f.read(length)
                self.entries.append({'type':entry_type,'timestamp':timestamp,'data':data})
        print(f"加载完成: {len(self.entries)} 条记录, 时长: {(self.entries[-1]['timestamp']-self.entries[0]['timestamp'])/1e9:.1f}s")
    def play(self,speed=1.0):
        if not self.entries:return
        base=self.entries[0]['timestamp'];play_start=time.time();last_decision=None
        for entry in self.entries:
            entry_rel=(entry['timestamp']-base)/1e9
            target=play_start+entry_rel/speed
            now=time.time()
            if now<target:time.sleep(target-now)
            rel=(entry['timestamp']-base)/1e9
            if entry['type']==1:
                img=cv2.imdecode(np.frombuffer(entry['data'],dtype=np.uint8),cv2.IMREAD_COLOR)
                if img is not None:
                    cv2.putText(img,f"t={rel:.2f}s",(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)
                    if last_decision:cv2.putText(img,f"state={last_decision}",(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
                    cv2.imshow('Replay',img)
                    if cv2.waitKey(1)&0xFF==ord('q'):break
            elif entry['type']==2:
                can_id=struct.unpack('<I',entry['data'][:4])[0]
                print(f"[{rel:.3f}s] CAN 0x{can_id:03X}: {entry['data'][4:].hex()}")
            elif entry['type']==3:
                state=struct.unpack('<i',entry['data'][8:12])[0]
                states={0:'NORMAL',1:'WARN',2:'BRAKE',3:'FAULT'}
                last_decision=states.get(state,str(state))
                print(f"[{rel:.3f}s] DECISION: state={last_decision}")
        cv2.destroyAllWindows()

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--log',required=True)
    parser.add_argument('--speed',type=float,default=1.0)
    args=parser.parse_args()
    r=LogReplayer(args.log);r.load();r.play(speed=args.speed)

if __name__=='__main__':main()
