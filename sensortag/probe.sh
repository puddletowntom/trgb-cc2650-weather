#!/bin/bash
set -e
OO="C:/Users/ronan/.espressif/tools/openocd-esp32/v0.12.0-esp32-20241016/openocd-esp32"
export PATH="$OO/bin:$PATH"
export OPENOCD_SCRIPTS="$OO/share/openocd/scripts"

openocd -f interface/ftdi/esp32_devkitj_v1.cfg -f target/ti_cc26x0.cfg \
  -c "adapter speed 1000" \
  -c "init" \
  -c "cortex_m reset_config sysresetreq" \
  -c "reset halt" \
  -c "reg pc" \
  -c "reg msp" \
  -c "resume" \
  -c "sleep 3000" \
  -c "halt" \
  -c "reg pc" \
  -c "mdw 0x20004000 7" \
  -c "shutdown" 2>&1
