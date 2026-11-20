#!/bin/bash
set -e
GCC=/c/msys64/mingw64/bin/arm-none-eabi-gcc
OBJCOPY=/c/msys64/mingw64/bin/arm-none-eabi-objcopy
SIZE=/c/msys64/mingw64/bin/arm-none-eabi-size
INC="-I cc26xxware -I cc26xxware/inc -I cc26xxware/driverlib"
CFLAGS="-mcpu=cortex-m3 -mthumb -Os -g3 -ffunction-sections -Wall"

$GCC $CFLAGS $INC -c main.c -o main.o
$GCC $CFLAGS $INC -c startup_gcc.c -o startup_gcc.o

# Compile the WHOLE driverlib: setup.c pulls in setup_rom.c which pulls in
# ddi/osc/etc.  A hand-picked subset always ends in undefined NOROM_* symbols.
OBJS="main.o startup_gcc.o"
for f in cc26xxware/driverlib/*.c; do
  o="$(basename "${f%.c}").o"
  $GCC $CFLAGS $INC -c "$f" -o "$o"
  OBJS="$OBJS $o"
done

$GCC -mcpu=cortex-m3 -mthumb -nostartfiles -T cc2650_temp.ld \
    -Wl,--gc-sections -o temp_hum.elf \
    $OBJS

# .scratch lives in SRAM — strip it or objcopy pads the image across the gap
$OBJCOPY -O binary --remove-section=.scratch temp_hum.elf temp_hum.bin
$SIZE temp_hum.elf
ls -la temp_hum.bin
