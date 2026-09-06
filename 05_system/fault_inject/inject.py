#!/usr/bin/env python3
"""故障注入工具：模拟各种故障验证系统降级，用法: python3 inject.py --fault camera_loss --duration 5"""
import argparse, subprocess, time, os, signal

class FaultInjector:
    def __init__(self):pass
    def list_faults(self):
        print("可用故障:")
        for name,desc in [('camera_loss','摄像头掉线'),('can_loss','CAN丢包'),
                          ('inference_timeout','推理超时'),('cpu_stress','CPU高负载'),
                          ('memory_pressure','内存压力')]:
            print(f"  {name:20s} - {desc}")
    def inject_camera_loss(self,duration=5,**kw):
        print(f"[故障] 摄像头掉线，持续{duration}s")
        print("  请拔掉摄像头USB，按Enter开始...")
        input();print("  故障已注入");time.sleep(duration)
        print("  故障结束，请插回摄像头");input("按Enter恢复...")
    def inject_can_loss(self,duration=5,loss_rate=0.5,interface='can0',**kw):
        print(f"[故障] CAN丢包率{loss_rate*100}%，持续{duration}s")
        try:
            subprocess.run(['sudo','tc','qdisc','add','dev',interface,'root','netem','loss',f'{int(loss_rate*100)}%'],check=True)
            print(f"  已设置丢包率{loss_rate*100}%");time.sleep(duration)
            subprocess.run(['sudo','tc','qdisc','del','dev',interface,'root'],check=True)
            print("  已恢复")
        except subprocess.CalledProcessError as e:print(f"  失败: {e}")
    def inject_inference_timeout(self,duration=5,**kw):
        print(f"[故障] 推理超时，持续{duration}s")
        result=subprocess.run(['pgrep','-f','aeb_system|main.py'],capture_output=True,text=True)
        pids=result.stdout.strip().split('\n')
        if not pids or not pids[0]:print("  未找到推理进程");return
        pid=pids[0];print(f"  暂停进程PID={pid}");os.kill(int(pid),signal.SIGSTOP)
        time.sleep(duration);print(f"  恢复进程PID={pid}");os.kill(int(pid),signal.SIGCONT)
    def inject_cpu_stress(self,duration=10,workers=4,**kw):
        print(f"[故障] CPU高负载，{workers}核，持续{duration}s")
        try:
            proc=subprocess.Popen(['stress','--cpu',str(workers),'--timeout',str(duration)])
            print(f"  stress已启动PID={proc.pid}");proc.wait();print("  已结束")
        except FileNotFoundError:
            print("  未安装stress，用Python模拟")
            import multiprocessing
            def burn():
                while True:pass
            procs=[multiprocessing.Process(target=burn) for _ in range(workers)]
            for p in procs:p.start()
            time.sleep(duration)
            for p in procs:p.terminate()
    def inject_memory_pressure(self,duration=5,size_mb=500,**kw):
        print(f"[故障] 内存压力，分配{size_mb}MB，持续{duration}s")
        big=bytearray(size_mb*1024*1024);print(f"  已分配{size_mb}MB")
        time.sleep(duration);del big;print("  已释放")
    def run(self,fault,**kw):
        funcs={'camera_loss':self.inject_camera_loss,'can_loss':self.inject_can_loss,
               'inference_timeout':self.inject_inference_timeout,'cpu_stress':self.inject_cpu_stress,
               'memory_pressure':self.inject_memory_pressure}
        if fault not in funcs:print(f"未知故障: {fault}");self.list_faults();return
        funcs[fault](**kw)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--fault',type=str)
    parser.add_argument('--duration',type=int,default=5)
    parser.add_argument('--loss-rate',type=float,default=0.5)
    parser.add_argument('--interface',type=str,default='can0')
    parser.add_argument('--list',action='store_true')
    args=parser.parse_args()
    inj=FaultInjector()
    if args.list or not args.fault:inj.list_faults();return
    inj.run(args.fault,duration=args.duration,loss_rate=args.loss_rate,interface=args.interface)

if __name__=='__main__':main()
