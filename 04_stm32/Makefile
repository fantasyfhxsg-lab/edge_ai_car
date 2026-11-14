# edge_ai_car 04_stm32 Makefile
# 用法: make          -> 编译生成 build/car.elf  build/car.bin
#       make flash    -> openocd 烧写（ST-Link V2）
#       make flash2   -> st-flash 烧写（备用）
#       make clean

# ===== 产物名与输出目录 =====
# 注：Makefile 的变量赋值行**不要**写行尾注释 —— make 只删注释、不删注释前的空格，
#     值会带上尾随空格（如 BUILD="build   "），拼进规则后目标里出现空格 → 解析报错。
# 产物前缀：生成 build/car.elf、build/car.bin
TARGET    = car
# 所有中间产物（.o/.d/.map/.buildflags）都放这里，clean 就是删它
BUILD     = build

# ===== 交叉工具链（必须是 arm-none-eabi，不能用系统 gcc）=====
CC        = arm-none-eabi-gcc
# 汇编也用 gcc 驱动（统一处理 -mcpu 等参数）
AS        = arm-none-eabi-gcc
# elf -> bin
OBJCOPY   = arm-none-eabi-objcopy
# 打印 text/data/bss 体积
SIZE      = arm-none-eabi-size

# ===== 目标机与优化 =====
# Cortex-M4 内核 + Thumb 指令集
CPU       = -mcpu=cortex-m4 -mthumb
# 软浮点 ABI（与启动汇编 .fpu softvfp 保持一致）
FPU       = -mfloat-abi=soft
# 体积优先优化（Flash 只 1MB，代码越小越好）
OPT       = -Os
CFLAGS    = $(CPU) $(FPU) $(OPT) -Wall -Wextra -std=gnu11 -ffunction-sections -fdata-sections
# -Wall -Wextra：验收要求 0 警告；-ffunction-sections/-fdata-sections 配合链接期 --gc-sections 剔除未用代码

# ===== 镜像选择（不改源码，只换构建参数）=====
#   make                -> 默认 Normal：Can_Init()，接 TJA1050 与板端联调
#   make CAN_LOOPBACK=1 -> 回环自测镜像：-DCAN_LOOPBACK_SELFTEST=1
#   （换镜像/换协议不必手动 make clean：构建指纹 STAMP 变了会自动整树重编，见下方）
#        Can_InitLoopback()（CAN_BTR.LBKM|SILM，无需收发器/对端/120ohm）
#        上电发 0x111 {counter,'C','A','N',0,0,0,0}，Task_10ms 轮询回读，
#        每秒串口打印 PASS/FAIL + TxErr/RxErr/BusOff/Ovf/bad
ifeq ($(CAN_LOOPBACK),1)
CFLAGS   += -DCAN_LOOPBACK_SELFTEST=1
$(info [build] CAN mode = silent-loopback self-test (BTR.LBKM|SILM, 0x111 echo))
else
$(info [build] CAN mode = normal (Can_Init + TJA1050, AEB link-up))
endif

# ===== 协议版本选择（编译期二选一，.clinerules §5；必须与板端 can.protocol 一致）=====
#   make                     -> heartbeat_v1（默认：0x100/0x101/0x102）
#   make PROTOCOL=brake_v2   -> brake_v2   （0x123/0x124/0x125）
#   ⚠ 宏直接编进 .o，历史上必须 make clean 才不残留上一协议的目标文件；现由构建指纹
#     STAMP（见下方）自动检测参数变化并整树重编 —— 手动 clean 仍可用、但不再必需
ifneq ($(PROTOCOL),)
ifneq ($(PROTOCOL),heartbeat_v1)
ifneq ($(PROTOCOL),brake_v2)
$(error PROTOCOL 只支持 heartbeat_v1（默认，可省略）或 brake_v2，收到 "$(PROTOCOL)")
endif
endif
endif
ifeq ($(PROTOCOL),brake_v2)
CFLAGS   += -DCAN_PROTOCOL_HEARTBEAT_V1=0 -DCAN_PROTOCOL_BRAKE_V2=1
$(info [build] CAN protocol = brake_v2 (0x123/0x124/0x125))
else
CFLAGS   += -DCAN_PROTOCOL_HEARTBEAT_V1=1 -DCAN_PROTOCOL_BRAKE_V2=0
$(info [build] CAN protocol = heartbeat_v1 (0x100/0x101/0x102))
endif

# 汇编只要内核与 ABI 参数（.s 不做 C 预处理）
ASFLAGS   = $(CPU) $(FPU)
LDFLAGS   = $(CPU) $(FPU) -Tstartup/stm32f407zgtx_flash.ld \
            -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map \
            -specs=nano.specs -lc -lm
# -T：链接脚本（内存布局、向量表位置）
# --gc-sections：回收未引用的段（配合 -ffunction-sections 明显减体积）
# -Map：生成内存映射表，排查"某函数被链进来了吗/占多大"
# nano.specs：用 newlib-nano（小体积；注意它让 vsnprintf 极易 HardFault，故工程禁格式化）
# -lc -lm：标准 C 库与数学库（sqrtf 等）

# ===== 构建指纹：CFLAGS/ASFLAGS/LDFLAGS 一变 → 整树重编（防"换宏不 clean"残留旧 .o）=====
#   背景：.o 规则只跟踪头文件（-MMD -MP），跟踪不到命令行宏；切 PROTOCOL/CAN_LOOPBACK
#   后不 make clean 会残留上一变体的目标文件，链接出"混合"镜像（曾把板上 flash 与本版
#   elf 弄成两份字节流，干扰真机验收）。这里让所有 .o 依赖一个内容==当前构建参数的指纹
#   文件：参数不变 → 指纹 mtime 不动（不触发重编）；参数一变 → 指纹重写 → 全部 .o 重编，
#   效果等价于自动 clean，手动 `make clean` 照常可用（指纹在 build/ 里，一并删除）。
# 指纹文件内容 = 当前 CFLAGS/ASFLAGS/LDFLAGS
STAMP     = $(BUILD)/.buildflags

# ===== 源文件与目标文件清单 =====
# Src 下全部 .c：驱动层 + 应用层 + 调度层（7 个）
SRCS_C    = $(wildcard Src/*.c)
# 启动汇编（向量表 + 复位流程）
SRCS_S    = startup/startup_stm32f407xx.s
# Src/xxx.c -> build/Src/xxx.o；startup/xxx.s -> build/startup/xxx.o（续行反斜杠必须是行尾）
OBJS      = $(patsubst %.c,$(BUILD)/%.o,$(SRCS_C)) \
            $(patsubst %.s,$(BUILD)/%.o,$(SRCS_S))
# -MMD -MP 生成的头文件依赖（末尾 -include 引入）
DEPS      = $(OBJS:.o=.d)

# 头文件搜索路径：Inc（驱动/协议/配置）+ Core/Inc（寄存器定义）
INC       = -IInc -ICore/Inc

# 默认目标：链接 elf + 转 bin，然后打印体积（验收要看的 text/data/bss）
all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin
	@$(SIZE) $(BUILD)/$(TARGET).elf

# C 源文件 -> 目标文件（同时生成 .d 依赖，头文件改了会自动重编）
$(BUILD)/%.o: %.c $(STAMP)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INC) -MMD -MP -c $< -o $@

# 汇编源文件 -> 目标文件
$(BUILD)/%.o: %.s $(STAMP)
	@mkdir -p $(dir $@)
	$(AS) $(ASFLAGS) -c $< -o $@

# 指纹文件：内容 == 当前构建参数 → 保持原 mtime（.o 不重编）；一变就重写（.o 全重编）
# 注：该规则靠 FORCE 每次"先算后比"，故 `make -n`（dry-run）会**多报**所有 .o 待编译
#     （dry-run 假定"跑过的目标即更新"）；实际执行时空跑 0 重编（已实测）。
$(STAMP): FORCE
	@mkdir -p $(dir $@)
	@printf '%s\n%s\n%s\n' '$(CFLAGS)' '$(ASFLAGS)' '$(LDFLAGS)' > $@.new
	@if [ -f $@ ] && cmp -s $@ $@.new; then rm -f $@.new; \
	 else if [ -f $@ ]; then echo "[build] build flags changed -> full rebuild"; fi; mv -f $@.new $@; fi

# 空目标：永远"比依赖旧"，用来强制每次 make 都执行指纹比对
FORCE:

# 链接：所有 .o 合成 elf（链接脚本决定内存布局）
$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) $^ -o $@

# elf -> 裸二进制（可直接烧到 0x08000000）
$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# 烧写方式一：ST-Link + openocd（烧完校验并复位运行）
flash: $(BUILD)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
	        -c "program $(BUILD)/$(TARGET).elf verify reset exit"

# 烧写方式二：st-flash 直接写 bin 到 Flash 起始地址（备用方案）
flash2: $(BUILD)/$(TARGET).bin
	st-flash --reset write $(BUILD)/$(TARGET).bin 0x08000000

# 清理：删掉整个 build（含指纹文件，下次 make 必全量重编）
clean:
	rm -rf $(BUILD)

# 引入编译器自动生成的头文件依赖（文件不存在时 -include 静默跳过）
-include $(DEPS)

.PHONY: all flash flash2 clean FORCE
