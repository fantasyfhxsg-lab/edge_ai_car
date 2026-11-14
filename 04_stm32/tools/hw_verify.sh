#!/usr/bin/env bash
# ============================================================================
# hw_verify.sh —— ③ 真机验收（回环自测镜像，SWD 读回，不依赖串口）
#
# 做三件事：
#   1) 打印当前 elf 里 g_* 全局**实际地址**（每次改代码都会平移，禁止照抄文档）
#   2) openocd 烧写 + verify + reset（等价 make flash）
#   3) 第二段会话：t+1.5s 读一次（开机态）→ 再等 6s 读一次（跑完一轮 AEB 自测），
#      并自动判定 g_can_test_echo_bad / g_lb_fail / g_lb_last_round_fail 等
#
# 用法（WSL 里必须 root：usbipd attach 后的 /dev/bus/usb/* 属 root:root）：
#   make clean && make CAN_LOOPBACK=1     # 先构建自测镜像（普通用户即可）
#   sudo bash tools/hw_verify.sh          # 烧写 + 读回 + 判定
#   sudo bash tools/hw_verify.sh --serial # 顺带用 screen 看串口日志（Ctrl-A 然后 K 退出）
#
# 前提（Windows 管理员 PowerShell，一次性）：
#   usbipd list                            # 记下 ST-Link / CH340 的 BUSID
#   usbipd bind   --busid <STLINK_BUSID>   # 每个设备只需 bind 一次
#   usbipd attach --wsl --busid <STLINK_BUSID>
#   usbipd attach --wsl --busid <CH340_BUSID>      # 想看串口日志才需要
#
# ⚠ 用 CAN1 时禁止插板载 USB SLAVE 口（PA11/PA12 与 OTG_FS 冲突，见 .clinerules §1）
# ============================================================================
set -u

ELF=${ELF:-build/car.elf}
OCD="openocd -f interface/stlink.cfg -f target/stm32f4x.cfg"
DUMP=${DUMP:-/tmp/hw_verify_dump.txt}   # 读回原始输出（保留不删，失败时自己看）

[ -f "$ELF" ] || { echo "找不到 $ELF：先 make clean && make CAN_LOOPBACK=1"; exit 2; }
command -v openocd >/dev/null || { echo "缺 openocd（容器里用 run_toolchain.sh 进环境）"; exit 2; }
if [ "$(id -u)" != "0" ]; then
    echo "⚠ 建议 root 运行（sudo bash $0）：/dev/bus/usb/* 属 root，否则 openocd 报 'Error: open failed'"
fi
if ! arm-none-eabi-nm "$ELF" 2>/dev/null | grep -q ' g_lb_fail$'; then
    echo "⚠ 当前 $ELF 不是回环自测镜像（无 g_lb_* 符号），判定项会缺失"
fi

echo "=== 1) 本版 elf 的全局地址（现读现用）==="
SYMS=$(arm-none-eabi-nm -n "$ELF" | grep -E ' [bBdD] (g_can_test_pass|g_can_test_rx_count|g_can_test_tx_counter|g_can_test_echo_bad|g_ticks|g_speed_valid|g_brake_level|g_brake_latched|g_lb_round|g_lb_step|g_lb_fail|g_lb_last_round_fail|g_lb_speed_echo|g_lb_speed_echo_ok)$')
echo "$SYMS"
export SYMS

echo
echo "=== 2) 烧写 + verify + reset ==="
$OCD -c 'gdb_port 0' -c "program $ELF verify reset exit" || { echo "烧写失败：检查 usbipd attach / ST-Link 连线"; exit 3; }
sleep 1      # 烧写会话退出后 ST-Link 常重新枚举，立刻再开会 open failed

echo
echo "=== 3) SWD 读回（t+1.5s 开机态 → 再等 6s 跑完一轮）==="
echo "  （注意 openocd 的 mdw 输出是 '0x20000000: ...'，带 0x 前缀）"
rm -f "$DUMP"
for try in 1 2 3; do
    $OCD -c 'gdb_port 0' -c 'init' -c 'sleep 1500' -c 'halt' -c 'reg pc' \
         -c 'mdw 0x20000000 32' -c 'mdw 0x40006400 8' \
         -c 'resume' -c 'sleep 6000' -c 'halt' -c 'reg pc' -c 'mdw 0x20000000 32' \
         -c 'reset run' -c 'shutdown' > "$DUMP" 2>&1
    grep -q '0x20000000:' "$DUMP" && break
    echo "  ⚠ 第 $try 次读回会话未取到数据，2s 后重试（多因 ST-Link 重枚举 / USB 未稳定）"
    sleep 2
done
sed -n '/ pc (/p;/^0x20000000:/p;/^0x40006400:/p' "$DUMP" | sed 's/^/  /'
grep -q '0x20000000:' "$DUMP" || { echo "  ❌ 三次都没读到内存，openocd 原始输出尾部："; tail -12 "$DUMP" | sed 's/^/    /'; }

echo
echo "=== 4) 自动判定（解析最后一次 .data/.bss dump）==="
python3 - "$DUMP" <<'PY'
import os, re, sys

syms = {}
for line in os.environ.get('SYMS', '').splitlines():
    m = re.match(r'\s*([0-9a-fA-F]+)\s+\S+\s+(\S+)', line)
    if m:
        syms[m.group(2)] = int(m.group(1), 16)

dumps, cur = [], None
for line in open(sys.argv[1], encoding='utf-8', errors='replace'):
    # openocd 的 mdw 输出形如 "0x20000000: ffffffff ..."（带 0x 前缀，早先按无前缀写会解析不到）
    m = re.match(r'^0x([0-9a-fA-F]{8}):\s+(.*)$', line.strip())
    if m is None:
        continue
    addr, rest = int(m.group(1), 16), m.group(2).split()
    if addr == 0x20000000:
        cur = {}
        dumps.append(cur)
    if cur is None:
        continue
    for i, w in enumerate(rest):
        v = int(w, 16)
        for b in range(4):
            cur[addr + i * 4 + b] = (v >> (8 * b)) & 0xFF

def rd(name, size):
    a = syms.get(name)
    if a is None or not dumps:
        return None
    d = dumps[-1]
    if not all((a + i) in d for i in range(size)):
        return None
    return int.from_bytes(bytes(d[a + i] for i in range(size)), 'little')

def show(name, size, want=None):
    v, a = rd(name, size), syms.get(name)
    addr = ('0x%08X' % a) if a else '??'
    if v is None:
        print('  ??   %-22s @%s  未取到（地址不在 dump 范围，用 nm 复核）' % (name, addr))
        return None
    tail = '' if want is None else ('   ✅' if v == want else '   ❌ 期望 %s' % want)
    print('  %-24s @%s = %-8s%s' % (name, addr, v, tail))
    return v

if not dumps:
    print('  ❌ dump 里没有 0x20000000 的 mdw 输出 —— 连接/烧写失败；原始 openocd 输出见 %s' % sys.argv[1])
    sys.exit(0)

# ⚠ 位宽必须与源码声明一致（读宽了会串到相邻字段）：
#   uint8/boolean=1  g_can_test_tx_counter/g_lb_step/g_lb_last_round_fail/g_speed_valid/
#                    g_brake_level/g_brake_latched/g_can_test_pass/g_lb_speed_echo_ok
#   uint16=2         g_lb_speed_echo
#   uint32=4         g_can_test_rx_count/g_can_test_echo_bad/g_lb_fail/g_lb_round/g_ticks
bad, warn = [], []
print('  -- 判定项（0 = 正常）--')
if show('g_can_test_echo_bad', 4, 0) not in (0, None):
    bad.append('echo_bad≠0')
if show('g_lb_fail', 4, 0) not in (0, None):
    bad.append('lb_fail≠0')
v = show('g_lb_last_round_fail', 1)
if v == 0xFF:
    warn.append('上一轮还没跑完(0xFF) —— 再等几秒重读；若一直 0xFF 看串口是否卡在某步')
elif v not in (0, None):
    bad.append('last_round_fail=%s' % v)

print('  -- 观测量 --')
tx = show('g_can_test_tx_counter', 1)
rx = show('g_can_test_rx_count', 4)
show('g_can_test_pass', 1)
show('g_lb_round', 4)
show('g_lb_step', 1)
show('g_lb_speed_echo', 2)
show('g_lb_speed_echo_ok', 1)
show('g_ticks', 4)
show('g_speed_valid', 1)
show('g_brake_level', 1)
show('g_brake_latched', 1)

if tx is not None and rx is not None:
    d = (rx - tx) % 256          # tx 是 uint8，256s（1 帧/s）回绕一次
    if rx >= 1 and d in (0, 255):
        print('  ✅ 通信闭环：rx=%d 与 tx=%d 自洽（d=%d；255 = 恰有一帧在途）' % (rx, tx, d))
    else:
        bad.append('rx/tx 不自洽(tx=%s rx=%s)' % (tx, rx))
        print('  ❌ 通信闭环异常：tx=%s rx=%s（d=%s）' % (tx, rx, d))
if warn:
    print('  ⚠ ' + '；'.join(warn))
print()
if bad:
    print('  结论：❌ 失败项 %s —— 串口对应 AEBtest=FAIL，按 README §6.3 排查顺序查' % bad)
elif warn:
    print('  结论：⚠ 未发现失败项，但有提示：%s' % warn)
else:
    print('  结论：✅ 回环自测通过（回读内容无误 / 自测步全过 / 车速链路有效）')
PY

echo "（原始 openocd 输出保留在 $DUMP，可自行核对）"

if [ "${1:-}" = "--serial" ]; then
    echo
    echo "=== 5) 串口日志（screen；退出：Ctrl-A 然后 K）==="
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  没看到串口设备：Windows 侧 usbipd attach CH340"
    DEV=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | head -1)
    if [ -n "${DEV:-}" ]; then
        screen "$DEV" 115200
    fi
fi
echo "完成。"
