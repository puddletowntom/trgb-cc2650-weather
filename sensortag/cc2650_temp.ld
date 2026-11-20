ENTRY(ResetISR)

MEMORY
{
    FLASH (RX)  : ORIGIN = 0x00000000, LENGTH = 0x0001FFA8   /* 128K minus CCFG */
    SRAM  (RWX) : ORIGIN = 0x20000000, LENGTH = 0x00005000   /* 20K */
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
    .text :
    {
        KEEP(*(.vectors))
        *(.text*)
        *(.rodata*)
        _etext = .;
    } > FLASH = 0

    .data :
    {
        _data = .;
        *(vtable)
        *(.data*)
        _edata = .;
        _ldata = LOADADDR(.data);
    } > SRAM AT > FLASH

    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > SRAM

    /* fixed-address publish area for JTAG polling (below factory stack) */
    .scratch 0x20004000 :
    {
        KEEP(*(.scratch))
    } > SRAM

    _stack = ORIGIN(SRAM) + LENGTH(SRAM);
}
