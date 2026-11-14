/* STM32F407ZGT6: FLASH 1MB @ 0x08000000, RAM 192KB @ 0x20000000 */
/* 注：F407ZG 共有 128KB 主 SRAM（SRAM1，0x20000000）+ 64KB CCM（0x10000000）。
 * 本工程只用主 SRAM 的 128KB（CCM 不放栈/堆），所以下面 LENGTH 写的是 128K。 */
ENTRY(Reset_Handler)      /* 入口符号：由启动汇编提供，复位后 PC 就指向它 */

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K   /* 1MB Flash：代码 + 常量 + .data 初值 */
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K    /* 128KB 主 SRAM：.data/.bss/堆/栈 */
}

/* 初始栈顶 = RAM 起始 + RAM 长度（栈从高地址向下生长），由启动汇编第 0 个向量使用 */
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    /* 中断向量表放最前面：复位后硬件固定从 0x08000000 取初始 SP 和入口地址 */
    .isr_vector :
    {
        . = ALIGN(4);            /* 4 字节对齐 */
        KEEP(*(.isr_vector))     /* KEEP：即使 --gc-sections 也不能把它回收掉 */
        . = ALIGN(4);
    } >FLASH

    /* 代码段 */
    .text :
    {
        . = ALIGN(4);
        *(.text)                 /* 普通函数 */
        *(.text*)                /* 加 -ffunction-sections 后每个函数独立段，都收进来 */
        *(.glue_7)               /* ARM/Thumb 互调用胶合代码（arm-none-eabi 标准写法） */
        *(.glue_7t)
        *(.eh_frame)             /* 异常帧信息（C++/某些库会生成） */
        . = ALIGN(4);
        _etext = .;              /* 代码段结束地址（调试/体积统计用） */
    } >FLASH

    /* 只读数据：字符串常量、"..." 字面量、const 表（g_rdk_cfg 就在这里） */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /* .data 的加载地址（在 Flash 里），启动汇编靠它把初值搬到 RAM */
    _sidata = LOADADDR(.data);

    /* 已初始化全局/静态变量：运行时在 RAM，初值镜像存在 Flash（AT> FLASH） */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;              /* 启动汇编的拷贝目标起点 */
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;              /* 拷贝目标终点 */
    } >RAM AT> FLASH

    /* 未初始化（或初值 0）的全局/静态变量：启动时整片清零，不占 Flash 空间 */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;               /* 清零起点 */
        __bss_start__ = .;       /* newlib 约定的别名符号 */
        *(.bss)
        *(.bss*)
        *(COMMON)                /* 未初始化的 tentative 定义（旧式 C 写法） */
        . = ALIGN(4);
        _ebss = .;               /* 清零终点 */
        __bss_end__ = .;
    } >RAM

    /* 堆起点：platform.c 的 _sbrk 用 end 符号作为堆的起始地址（本工程不用 malloc） */
    . = ALIGN(4);
    end = .;
    __end__ = .;
}
