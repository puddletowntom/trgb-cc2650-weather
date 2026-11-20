# LilyGO T-RGB + CC2650 SensorTag — ambient temperature & humidity display

Two boards, one job: the CC2650 SensorTag owns the sensors, the LilyGO T-RGB
(ESP32-S3) owns the pixels. The link is **wireless BLE** — no wires between the
boards.

```
  CC2650 SensorTag (stock TI BLE firmware)      LilyGO T-RGB 2.1" (ESP32-S3)
  +------------------------------+              +---------------------------+
  | HDC1000 humidity             |  BLE 2.4GHz  | BLE central               |
  | TMP007 ambient temperature   | ~~~~~~~~~~~~ | scan -> connect -> notify |
  | Humidity svc F000AA20...     |              |                           |
  | IR temp svc  F000AA00...     |              |                           |
  | advertises "SensorTag"       |              | 480x480 round RGB panel   |
  +------------------------------+              +---------------------------+
```

Why BLE: the CC2650 is a BLE radio with no WiFi, and its factory firmware already
exposes a Humidity service. The ESP32-S3 has BLE built in, so the link needs no
extra hardware and no custom firmware on the tag at all.

A wired I2C variant was prototyped first (SensorTag as I2C master, display as
slave) but the DevPack connector would not take the available wires, so the
design moved to BLE. The bare-metal SensorTag firmware in `sensortag/` is that
wired variant, kept as an alternative/fallback.

---

## Repository layout

| Path | What it is |
|---|---|
| `display/trgb_temp_humidity/trgb_temp_humidity.ino` | Arduino sketch for the T-RGB: BLE central + graphics |
| `display/preview.png` | Render of the display design |
| `sensortag/main.c` | Bare-metal CC2650 firmware: HDC1000 over I2C, results to RAM |
| `sensortag/build.sh` | arm-none-eabi build for the SensorTag image |
| `sensortag/cc2650_temp.ld` | Linker script for the SensorTag image |
| `sensortag/probe.sh` | OpenOCD/JTAG helper to read the published results |

---

## 1. SensorTag side — nothing to flash for the wireless build

The tag's stock TI firmware already advertises the Humidity service. If your tag
has never been reflashed, just fit a CR2032 and press the side button.

If it has been reflashed with the custom image, restore TI's official image over
JTAG (XDS110 on the 10-pin P1 header), then pull and re-insert the coin cell so
the tag cold-boots. The official image is `CC2650SensorTag_BLE_v1.50.hex` from
TI's SensorTag wiki.

### Custom bare-metal firmware (`sensortag/`)

This is the **wired/I2C-era** variant. It reads the on-board HDC1000 directly
(addr 0x43, SDA = DIO_5, SCL = DIO_6) and publishes results to a fixed RAM
location (`0x20004000`) for inspection over JTAG. It does **not** implement BLE.

Build it (needs `arm-none-eabi-gcc` and TI's `cc26xxware` driverlib in the same
directory):

```
cd sensortag
./build.sh
```

Then flash `temp_hum.bin` over JTAG with OpenOCD. `probe.sh` reads the published
counters/values.

---

## 2. What the tag advertises

| Item | Value |
|---|---|
| Device name | SensorTag |
| Humidity service | `F000AA20-0451-4000-B000-000000000000` |
| Data (notify) | `F000AA21-...` 4 bytes: temp LSB, temp MSB, RH LSB, RH MSB |
| Config (write) | `F000AA22-...` write `0x01` to start measuring |
| Period | `F000AA23-...` default 1000 ms |

### IR temperature service (TMP007)

| Item | Value |
|---|---|
| Service | `F000AA00-0451-4000-B000-000000000000` |
| Data (notify) | `F000AA01-...` 4 bytes: obj LSB, obj MSB, amb LSB, amb MSB |
| Config (write) | `F000AA02-...` write `0x01` to start measuring |
| Period | `F000AA03-...` default 1000 ms |

Conversions applied in the sketch:

```
HDC1000  T  = raw_t / 65536 * 165 - 40        degC  (raw_t signed 16-bit)
HDC1000  RH = (raw_rh & 0xFFFC) / 65536 * 100  %     (low 2 bits are status)
TMP007   T_amb = raw_amb / 128.0              degC  (bytes 2..3, signed 16-bit)
```

**Which temperature is on screen:** the TMP007's ambient (die) channel. Humidity
comes from the HDC1000. The HDC1000's own temperature is decoded and kept as a
fallback in case AA01 never arrives.

Measured on the bench, both chips agree to within ~0.3 °C because they sit on the
same board at the same temperature — TMP007 ambient 14.53 °C, HDC1000 14.85 °C in
the same second. Switching source therefore changes the display by a fraction of
a degree, not by several degrees. The TMP007's die runs a little warm from its own
self-heating, so neither figure is a precision room thermometer.

The phone app's "temperature" figure was observed to read ~3.5 °C lower than the
TMP007 ambient register in the same conditions. That is not the ambient register
reporting two values at once — it is a different channel or a compensated value.
The display deliberately shows the raw ambient register and does not apply a
fudge offset to match the app.

---

## 3. T-RGB firmware — `display/trgb_temp_humidity.ino`

Arduino sketch using only "GFX Library for Arduino" (Arduino_GFX) — no LVGL, no
touch, no LilyGo helper libraries. BLE uses the ESP32 core's bundled stack.

### Library pin

With ESP32 core **2.0.11** you must use **Arduino_GFX 1.4.9** (1.5.x targets
core 3.x and fails to compile against IDF 4.4). Verified compiling with core
2.0.11 + GFX 1.4.9, 2.1" panel variant:

```
Sketch uses 953573 bytes (72%) of program storage space.
Global variables use 43732 bytes (13%) of dynamic memory.
```

The BLE stack is the bulk of that flash — roughly 615 KB more than the wired
version. Still fits in the 1.3 MB app partition.

### Build gotcha

The Arduino builder injects generated function prototypes right after the last
`#include`, which is above the full `struct Reading` definition. The sketch
therefore forward-declares `struct Reading;` at the top. Remove that line and the
build fails with `variable or field 'drawScreen' declared void`.

### Board settings (Arduino IDE → ESP32S3 Dev Module)

| Setting | Value |
|---|---|
| PSRAM | OPI PSRAM |
| Flash Size | 16MB |
| USB CDC On Boot | Enabled |
| Partition | 16M Flash (3MB APP) or larger |

`PANEL_2_8_INCH` at the top of the sketch: **0 = 2.1"** (this board),
**1 = 2.8"** full circle. Wrong value = blank screen with backlight on.

### Command-line build / flash

```
arduino-cli compile -b "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=cdc" \
  display/trgb_temp_humidity
arduino-cli upload  -b "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,CDCOnBoot=cdc" \
  -p COM5 display/trgb_temp_humidity
```

### Serial diagnostics

`DEBUG_BLE` at the top of the sketch: `1` prints a heartbeat, the raw counts from
both sensors and every BLE advertisement seen; `0` leaves the serial port quiet.
Set it to `0` for a production build.

### What it draws

* 480x480 radial gradient; outer 260° arc = temperature (−10..40 °C) with a
  blue → teal → amber → red gradient and a glowing tip at the value.
* Inner thin arc = humidity 0..100 %RH.
* Big 7-segment temperature (TMP007 ambient) with °C, droplet + humidity, dew
  point, min/max since boot.
* Status badge: green LIVE, amber STALE (>5 s without a packet), red NO DATA.

The image is composed into a PSRAM framebuffer and blitted in one go. BLE runs on
its own task; the render loop only reads a mutex-protected copy of the latest
reading.

---

## 4. Serial output to expect

```
[T-RGB] ambient panel (BLE)
[T-RGB] BLE central started
[BLE] found SensorTag (xx:xx:xx:xx:xx:xx)
[BLE] connected
[BLE] humidity notifications on
[BLE] IR temp notifications on
```

If it prints "found" but never "connected", the tag is already paired to a phone —
disconnect it there. If it never prints "found", the tag is out of range, has a
flat coin cell, or is not advertising.

---

## 5. Gotchas

* I2C is boot-only. The ST7701S needs its I2C init through the XL9535 expander
  on GPIO8/48, so `Wire` is started once for `gfx->begin()`. Nothing else
  touches that bus afterwards.
* No spare GPIOs on the T-RGB — the Grove port is shared with the touch
  controller and the expander.
* BLE and WiFi share the radio. This sketch never enables WiFi, so BLE gets the
  full airtime. Adding WiFi will slow notifications.
* The tag only advertises while no central is connected. The sketch rescans
  every 5 s after a disconnect, so it reconnects on its own when the tag returns.
* A 5 s gap in packets flips the badge to STALE; the last values stay on screen
  rather than blanking.
