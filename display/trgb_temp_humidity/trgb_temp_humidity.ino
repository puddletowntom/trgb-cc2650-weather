/*
 * T-RGB ambient panel — LilyGO T-RGB (ESP32-S3, ST7701S 480x480 round)
 * -------------------------------------------------------------------------
 * Display side of the SensorTag link, now WIRELESS.  The CC2650 SensorTag runs
 * TI's stock firmware and advertises the Humidity service; this board is the
 * BLE central and draws the reading.  No wires between the boards.
 *
 * Humidity service F000AA20-0451-4000-B000-000000000000
 *   AA21  data   (notify) 4 bytes: temp LSB, temp MSB, hum LSB, hum MSB
 *   AA22  config (write)  0x01 = start measuring
 *   Conversion: T = raw/65536*165 - 40,  RH = (raw & 0xFFFC)/65536*100
 *
 * IR temperature service F000AA00-0451-4000-B000-000000000000
 *   AA01  data   (notify) 4 bytes: obj LSB, obj MSB, amb LSB, amb MSB
 *   AA02  config (write)  0x01 = start measuring
 *   Ambient (die) temperature = raw/128.0 degC  -> this is what we display
 *
 * The temperature on screen is the TMP007's ambient (die) channel, matching
 * what the phone app reports.  Humidity comes from the HDC1000.  The HDC1000's
 * own temperature is kept as a fallback if AA01 never arrives.
 *
 * NOTE ON THE BUS: the Grove I2C pins are shared with the XL9535 expander that
 * initialises the ST7701S.  I2C is used once, for panel init, then left alone.
 * BLE does not touch those pins.
 *
 * Libraries: "GFX Library for Arduino" (moononournation) — Arduino_GFX.
 * Board:     ESP32S3 Dev Module, PSRAM = OPI, Flash 16MB, USB CDC on boot.
 * No touch, no LVGL, no extra screens.
 */

#define PANEL_2_8_INCH  0   // 1 = 2.8" full circle, 0 = 2.1" (this board is 2.1")
#define DEBUG_BLE       1   // 1 = print raw sensor bytes + heartbeat to serial

// Forward declaration FIRST: the Arduino builder injects generated function
// prototypes immediately after the last #include, which is above the full
// definition of Reading further down.  Without this, the generated prototype
// for drawScreen() cannot name the type and the build fails.
struct Reading;

#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <BLEAdvertisedDevice.h>
#include <math.h>

// ---------------------------------------------------------------- pin map ---
#define TFT_BL          46
#define BOARD_I2C_SDA   8
#define BOARD_I2C_SCL   48

#define W 480
#define H 480
#define CX 240
#define CY 240

// --------------------------------------------------------------- palette ----
#define RGB565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

static const uint16_t C_TXT  = RGB565(146, 168, 194);
static const uint16_t C_DIM  = RGB565(86, 102, 126);
static const uint16_t C_HI   = RGB565(238, 245, 255);
static const uint16_t C_RULE = RGB565(30, 38, 52);
static const uint16_t C_TRK  = RGB565(25, 33, 47);
static const uint16_t C_OK   = RGB565(70, 235, 140);
static const uint16_t C_WARN = RGB565(255, 190, 60);
static const uint16_t C_BAD  = RGB565(255, 90, 90);

static const uint16_t TEMP_STOPS[] = { RGB565(58, 140, 255), RGB565(60, 214, 170),
                                       RGB565(255, 209, 102), RGB565(255, 93, 93) };
static const uint16_t HUM_STOPS[]  = { RGB565(40, 120, 255), RGB565(60, 200, 255),
                                       RGB565(120, 255, 230) };

// ------------------------------------------------------------------ panel ---
Arduino_DataBus *bus = new Arduino_XL9535SWSPI(BOARD_I2C_SDA, BOARD_I2C_SCL,
                                              2 /* XL PWD */, 3 /* XL CS */,
                                              5 /* XL SCK */, 4 /* XL MOSI */);
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
    45 /* DE */, 41 /* VSYNC */, 47 /* HSYNC */, 42 /* PCLK */,
    21, 18, 17, 16, 15,              // R0..R4
    14, 13, 12, 11, 10, 9,           // G0..G5
    7, 6, 5, 3, 2,                   // B0..B4
    1, 50, 1, 30,                    // hsync polarity/front/pulse/back
    1, 20, 1, 30,                    // vsync polarity/front/pulse/back
    1 /* pclk active neg */);

#if PANEL_2_8_INCH
// ST7701S init for the 2.8" full-circle panel (from LilyGO's T-RGB example)
static const uint8_t INIT_OPS[] = {
    BEGIN_WRITE,
    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x13,
    WRITE_C8_D8, 0xEF, 0x08,
    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x10,
    WRITE_COMMAND_8, 0xC0, WRITE_BYTES, 2, 0x3B, 0x00,
    WRITE_COMMAND_8, 0xC1, WRITE_BYTES, 2, 0x10, 0x0C,
    WRITE_COMMAND_8, 0xC2, WRITE_BYTES, 2, 0x07, 0x0A,
    WRITE_C8_D8, 0xC7, 0x00,
    WRITE_C8_D8, 0xCC, 0x10,
    WRITE_C8_D8, 0xCD, 0x08,
    WRITE_COMMAND_8, 0xB0,
    WRITE_BYTES, 16, 0x05, 0x12, 0x98, 0x0e, 0x0F, 0x07, 0x07, 0x09,
    0x09, 0x23, 0x05, 0x52, 0x0F, 0x67, 0x2C, 0x11,
    WRITE_COMMAND_8, 0xB1,
    WRITE_BYTES, 16, 0x0B, 0x11, 0x97, 0x0C, 0x12, 0x06, 0x06, 0x08,
    0x08, 0x22, 0x03, 0x51, 0x11, 0x66, 0x2B, 0x0F,
    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x11,
    WRITE_C8_D8, 0xB0, 0x5D,
    WRITE_C8_D8, 0xB1, 0x2D,
    WRITE_C8_D8, 0xB2, 0x81,
    WRITE_C8_D8, 0xB3, 0x80,
    WRITE_C8_D8, 0xB5, 0x4E,
    WRITE_C8_D8, 0xB7, 0x85,
    WRITE_C8_D8, 0xB8, 0x20,
    WRITE_C8_D8, 0xC1, 0x78,
    WRITE_C8_D8, 0xC2, 0x78,
    WRITE_C8_D8, 0xD0, 0x88,
    WRITE_COMMAND_8, 0xE0,
    WRITE_BYTES, 3, 0x00, 0x00, 0x02,
    WRITE_COMMAND_8, 0xE1,
    WRITE_BYTES, 11, 0x06, 0x30, 0x08, 0x30, 0x05, 0x30, 0x07, 0x30,
    0x00, 0x33, 0x33,
    WRITE_COMMAND_8, 0xE2,
    WRITE_BYTES, 12, 0x11, 0x11, 0x33, 0x33, 0xf4, 0x00, 0x00, 0x00,
    0xf4, 0x00, 0x00, 0x00,
    WRITE_COMMAND_8, 0xE3,
    WRITE_BYTES, 4, 0x00, 0x00, 0x11, 0x11,
    WRITE_COMMAND_8, 0xE4,
    WRITE_BYTES, 2, 0x44, 0x44,
    WRITE_COMMAND_8, 0xE5,
    WRITE_BYTES, 16, 0x0d, 0xf5, 0x30, 0xf0, 0x0f, 0xf7, 0x30, 0xf0,
    0x09, 0xf1, 0x30, 0xf0, 0x0b, 0xf3, 0x30, 0xf0,
    WRITE_COMMAND_8, 0xE6,
    WRITE_BYTES, 4, 0x00, 0x00, 0x11, 0x11,
    WRITE_COMMAND_8, 0xE7,
    WRITE_BYTES, 2, 0x44, 0x44,
    WRITE_COMMAND_8, 0xE8,
    WRITE_BYTES, 16, 0x0c, 0xf4, 0x30, 0xf0, 0x0e, 0xf6, 0x30, 0xf0,
    0x08, 0xf0, 0x30, 0xf0, 0x0a, 0xf2, 0x30, 0xf0,
    WRITE_C8_D8, 0xE9, 0x36,
    WRITE_COMMAND_8, 0xEB,
    WRITE_BYTES, 7, 0x00, 0x01, 0xe4, 0xe4, 0x44, 0x88, 0x40,
    WRITE_COMMAND_8, 0xED,
    WRITE_BYTES, 16, 0xff, 0x10, 0xaf, 0x76, 0x54, 0x2b, 0xcf, 0xff,
    0xff, 0xfc, 0xb2, 0x45, 0x67, 0xfa, 0x01, 0xff,
    WRITE_COMMAND_8, 0xEF,
    WRITE_BYTES, 6, 0x08, 0x08, 0x08, 0x45, 0x3f, 0x54,
    WRITE_COMMAND_8, 0xFF,
    WRITE_BYTES, 5, 0x77, 0x01, 0x00, 0x00, 0x00,
    WRITE_COMMAND_8, 0x11,          // sleep out
    END_WRITE,
    DELAY, 150,
    BEGIN_WRITE,
    WRITE_C8_D8, 0x3A, 0x66,        // RGB565
    WRITE_C8_D8, 0x36, 0x08,
    WRITE_C8_D8, 0x35, 0x00,
    WRITE_COMMAND_8, 0x29,          // display on
    END_WRITE,
    DELAY, 20
};
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    480, 480, rgbpanel, 0, true, bus, GFX_NOT_DEFINED, INIT_OPS, sizeof(INIT_OPS));
#else
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(
    480, 480, rgbpanel, 0, true, bus, GFX_NOT_DEFINED,
    st7701_type4_init_operations, sizeof(st7701_type4_init_operations));
#endif

// ----------------------------------------------------------- draw buffers ---
static uint16_t *fb  = nullptr;   // frame being composed
static uint16_t *bg  = nullptr;   // static layer (gradient + tracks + rules)
static size_t    fbBytes = (size_t)W * H * 2;

// Declared before the first function on purpose: the Arduino preprocessor
// injects generated prototypes there, and they must be able to name this type.
struct Reading {
    float t = 0;        // temperature drawn on screen (TMP007 ambient)
    float tHdc = 0;     // HDC1000 temperature (fallback source)
    float h = 0, dew = 0;
    float tmin = 99, tmax = -99;
    bool  valid = false;
    bool  irOk = false; // true once AA01 has delivered a TMP007 reading
    uint32_t ms = 0;
    uint8_t seq = 0;
    bool  sensorOk = false;
};

// =============================================================== primitives ==
static inline void px(int x, int y, uint16_t c)
{
    if ((unsigned)x < (unsigned)W && (unsigned)y < (unsigned)H) fb[y * W + x] = c;
}

static inline void hline(int x, int y, int w, uint16_t c)
{
    if ((unsigned)y >= (unsigned)H) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > W) w = W - x;
    if (w <= 0) return;
    uint16_t *p = &fb[y * W + x];
    while (w--) *p++ = c;
}

static inline void vline(int x, int y, int h, uint16_t c)
{
    if ((unsigned)x >= (unsigned)W) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > H) h = H - y;
    uint16_t *p = &fb[y * W + x];
    while (h--) { *p = c; p += W; }
}

static void fillRect(int x, int y, int w, int h, uint16_t c)
{
    for (int i = 0; i < h; i++) hline(x, y + i, w, c);
}

static void fillCircle(int cx, int cy, int r, uint16_t c)
{
    if (r <= 0) return;
    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)(sqrtf((float)(r * r - dy * dy)) + 0.5f);
        hline(cx - dx, cy + dy, 2 * dx + 1, c);
    }
}

// hollow ring of stroke width `thick` centred on radius r
static void ringOutline(int cx, int cy, int r, int thick, uint16_t c)
{
    int ro = r + (thick + 1) / 2;
    int ri = r - thick / 2;
    if (ri < 0) ri = 0;
    for (int dy = -ro; dy <= ro; dy++) {
        for (int dx = -ro; dx <= ro; dx++) {
            int d2 = dx * dx + dy * dy;
            if (d2 <= ro * ro && d2 >= ri * ri) px(cx + dx, cy + dy, c);
        }
    }
}

static void fillRoundRect(int x, int y, int w, int h, int r, uint16_t c)
{
    if (w <= 0 || h <= 0) return;
    if (r * 2 > w) r = w / 2;
    if (r * 2 > h) r = h / 2;
    if (r <= 0) { fillRect(x, y, w, h, c); return; }
    fillRect(x + r, y, w - 2 * r, h, c);
    fillRect(x, y + r, r, h - 2 * r, c);
    fillRect(x + w - r, y + r, r, h - 2 * r, c);
    fillCircle(x + r, y + r, r, c);
    fillCircle(x + w - r - 1, y + r, r, c);
    fillCircle(x + r, y + h - r - 1, r, c);
    fillCircle(x + w - r - 1, y + h - r - 1, r, c);
}

// colour ramp over an RGB565 stop list
static uint16_t ramp(const uint16_t *s, int n, float t)
{
    if (n <= 1) return s[0];
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;
    float x = t * (n - 1);
    int i = (int)x;
    if (i > n - 2) i = n - 2;
    if (i < 0) i = 0;
    float f = x - i;
    int r0 = (s[i] >> 11) & 0x1F, g0 = (s[i] >> 5) & 0x3F, b0 = s[i] & 0x1F;
    int r1 = (s[i + 1] >> 11) & 0x1F, g1 = (s[i + 1] >> 5) & 0x3F, b1 = s[i + 1] & 0x1F;
    return (uint16_t)(((int)(r0 + (r1 - r0) * f) << 11) |
                      ((int)(g0 + (g1 - g0) * f) << 5) |
                      (int)(b0 + (b1 - b0) * f));
}

// Polar-rasterised annulus segment. 0 deg = 12 o'clock, clockwise.
// cscale scales the gradient position (value relative to the gauge range).
static void arcBand(int rin, int rout, float a0, float a1,
                    const uint16_t *stops, int nstops, float cscale)
{
    float span = a1 - a0;
    if (span < 0.05f) return;
    int steps = (int)(span * rout * 0.0174533f * 1.4f);
    if (steps < 2) steps = 2;
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / steps;
        uint16_t c = ramp(stops, nstops, t * cscale);
        float a = (a0 + span * t) * 0.0174533f;
        float sa = sinf(a), ca = cosf(a);
        for (int r = rin; r <= rout; r++)
            px((int)(CX + r * sa + 0.5f), (int)(CY - r * ca + 0.5f), c);
    }
}

// ================================================================== 5x7 text =
static const char FONT_CHARS[] = " 0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ.:-%?";
static const uint8_t FONT5X7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, {0x3E, 0x51, 0x49, 0x45, 0x3E},
    {0x00, 0x42, 0x7F, 0x40, 0x00}, {0x42, 0x61, 0x51, 0x49, 0x46},
    {0x21, 0x41, 0x45, 0x4B, 0x31}, {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3C, 0x4A, 0x49, 0x49, 0x30},
    {0x01, 0x71, 0x09, 0x05, 0x03}, {0x36, 0x49, 0x49, 0x49, 0x36},
    {0x06, 0x49, 0x49, 0x29, 0x1E}, {0x7E, 0x09, 0x09, 0x09, 0x7E},
    {0x7F, 0x49, 0x49, 0x49, 0x36}, {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, {0x7F, 0x49, 0x49, 0x49, 0x41},
    {0x7F, 0x09, 0x09, 0x09, 0x01}, {0x3E, 0x41, 0x41, 0x49, 0x7A},
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01}, {0x7F, 0x08, 0x14, 0x22, 0x41},
    {0x7F, 0x40, 0x40, 0x40, 0x40}, {0x7F, 0x02, 0x0C, 0x02, 0x7F},
    {0x7F, 0x02, 0x04, 0x08, 0x7F}, {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06}, {0x3E, 0x41, 0x51, 0x21, 0x5E},
    {0x7F, 0x09, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31},
    {0x01, 0x01, 0x7F, 0x01, 0x01}, {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x7F, 0x20, 0x18, 0x20, 0x7F},
    {0x63, 0x14, 0x08, 0x14, 0x63}, {0x03, 0x04, 0x78, 0x04, 0x03},
    {0x61, 0x51, 0x49, 0x45, 0x43}, {0x00, 0x60, 0x60, 0x00, 0x00},
    {0x00, 0x36, 0x36, 0x00, 0x00}, {0x00, 0x08, 0x08, 0x08, 0x00},
    {0x63, 0x13, 0x08, 0x64, 0x63}, {0x02, 0x01, 0x51, 0x09, 0x06},
};
#define FONT_COUNT (int)(sizeof(FONT5X7) / sizeof(FONT5X7[0]))

static int textWidth(const char *s, int scale) { return (int)strlen(s) * 6 * scale - scale; }

static void text(const char *s, int x, int y, int scale, uint16_t c, bool center)
{
    if (center) x -= textWidth(s, scale) / 2;
    for (const char *p = s; *p; p++) {
        char ch = *p;
        const char *hit = strchr(FONT_CHARS, ch);
        if (!hit && ch >= 'a' && ch <= 'z') hit = strchr(FONT_CHARS, ch - 32);
        int idx = hit ? (int)(hit - FONT_CHARS) : FONT_COUNT - 1;
        if (idx >= FONT_COUNT) idx = FONT_COUNT - 1;
        const uint8_t *g = FONT5X7[idx];
        for (int col = 0; col < 5; col++) {
            uint8_t bits = g[col];
            for (int row = 0; row < 7; row++) {
                if (bits & (1 << row))
                    fillRect(x + col * scale, y + row * scale, scale, scale, c);
            }
        }
        x += 6 * scale;
    }
}

// ============================================================ 7-segment digits
static const uint8_t SEG_MASK[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
}; // bit0=a bit1=b ... bit6=g

static float numWidth(const char *s, int w, int gap)
{
    float tot = 0;
    for (const char *p = s; *p; p++)
        tot += (*p == '1') ? 0.50f * w
             : (*p == '.') ? 0.34f * w
             : (*p == '-') ? 0.55f * w
                           : (float)w;
    return tot + (float)(gap * ((int)strlen(s) - 1));
}

static void segDigit(int x, int y, int w, int h, int t, char ch, uint16_t c, bool dot)
{
    if (dot) {
        int r = (int)(t * 0.72f);
        fillCircle(x + w / 2, y + h - r, r, c);
        return;
    }
    if (ch == '-') {                       // minus sign (sub-zero morning)
        fillRoundRect(x + t / 2, y + h / 2 - t / 2, w - t, t, t / 2, c);
        return;
    }
    if (ch < '0' || ch > '9') return;
    uint8_t m = SEG_MASK[ch - '0'];
    int x0 = x + t / 2, x1 = x + w - t / 2;
    int y0 = y + t / 2, y1 = y + h / 2, y2 = y + h - t / 2;
    int r  = t / 2;
    if (m & 0x01) fillRoundRect(x0, y0, x1 - x0, t, r, c);                 // a
    if (m & 0x02) fillRoundRect(x1 - t / 2, y0, t, y1 - y0, r, c);         // b
    if (m & 0x04) fillRoundRect(x1 - t / 2, y1, t, y2 - y1, r, c);         // c
    if (m & 0x08) fillRoundRect(x0, y2 - t / 2, x1 - x0, t, r, c);         // d
    if (m & 0x10) fillRoundRect(x0 - t / 2, y1, t, y2 - y1, r, c);         // e
    if (m & 0x20) fillRoundRect(x0 - t / 2, y0, t, y1 - y0, r, c);         // f
    if (m & 0x40) fillRoundRect(x0, y1 - t / 2, x1 - x0, t, r, c);         // g
}

// returns the drawn width
static float num(const char *s, int x, int y, int w, int h, int t,
                 uint16_t c, bool center, int gap)
{
    float tot = numWidth(s, w, gap);
    float ox = center ? x - tot / 2.0f : (float)x;
    int startX = (int)ox;
    for (const char *p = s; *p; p++) {
        float cw = (*p == '1') ? 0.50f * w
                 : (*p == '.') ? 0.34f * w
                 : (*p == '-') ? 0.55f * w
                               : (float)w;
        segDigit((int)ox, y, (int)cw, h, t, *p, c, *p == '.');
        ox += cw + gap;
    }
    (void)startX;
    return tot;
}

// ==================================================================== frames ==
static void drawBackground()
{
    // static layer: radial gradient + gauge tracks + hairline rule
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float dx = x - CX, dy = y - CY;
            float r = sqrtf(dx * dx + dy * dy) / 240.0f;
            if (r > 1.0f) r = 1.0f;
            float g = 1.0f - sqrtf(dx * dx + dy * dy * 1.32f) / 155.0f;
            if (g < 0) g = 0;
            g *= g;
            // base #0F1420 -> #04060B, glow towards #192132
            int R = (int)(15 + (4 - 15) * r * 0.95f);
            int G = (int)(20 + (6 - 20) * r * 0.95f);
            int B = (int)(32 + (11 - 32) * r * 0.95f);
            R = (int)(R + (25 - R) * g * 0.8f);
            G = (int)(G + (33 - G) * g * 0.8f);
            B = (int)(B + (50 - B) * g * 0.8f);
            fb[y * W + x] = RGB565(R, G, B);
        }
    }
    // gauge tracks
    float a0 = -130.0f, a1 = 130.0f;
    arcBand(208, 226, a0, a1, &C_TRK, 1, 1.0f);
    arcBand(188, 194, a0, a1, &C_TRK, 1, 1.0f);
    // start caps
    for (int i = 0; i < 2; i++) {
        float a = -130.0f * 0.0174533f;
        int rr = i ? 190 : 217;
        fillCircle((int)(CX + rr * sinf(a)), (int)(CY - rr * cosf(a)), 3, RGB565(64, 78, 98));
    }
    // hairline under the big number
    fillRect(184, 276, 112, 2, C_RULE);
}

static void drawDroplet(int x, int y, uint16_t c)
{
    // rounded teardrop ~22 x 26 at (x, y)
    for (int i = 0; i <= 12; i++) {
        int hw = (int)(11.0f * i / 12.0f) + 1;
        hline(x + 13 - hw, y + i, 2 * hw, c);
    }
    fillCircle(x + 13, y + 16, 11, c);
}

static void dewp(float t, float rh, float *out)
{
    // Magnus-Tetens
    const float a = 17.62f, b = 243.12f;
    float g = logf(rh / 100.0f) + (a * t) / (b + t);
    *out = (b * g) / (a - g);
}

static void drawScreen(Reading &r, bool fresh, bool ever)
{
    memcpy(fb, bg, fbBytes);

    const float A0 = -130.0f, SWEEP = 260.0f;
    const float TMIN = -10.0f, TMAX = 40.0f;

    // ---- temperature gauge (outer) ----
    float ft = 0;
    if (ever) {
        ft = (r.t - TMIN) / (TMAX - TMIN);
        if (ft < 0) ft = 0;
        if (ft > 1) ft = 1;
    }
    if (ft > 0.004f) {
        arcBand(209, 225, A0, A0 + SWEEP * ft, TEMP_STOPS, 4, ft);
        // bright travelling tip
        float a = (A0 + SWEEP * ft) * 0.0174533f;
        int tx = (int)(CX + 217 * sinf(a)), ty = (int)(CY - 217 * cosf(a));
        uint16_t tip = ramp(TEMP_STOPS, 4, ft);
        fillCircle(tx, ty, 11, RGB565(28, 36, 52));   // dark halo
        ringOutline(tx, ty, 9, 2, tip);               // coloured ring
        fillCircle(tx, ty, 6, C_HI);                  // white core
    }

    // ---- humidity gauge (inner) ----
    float fh = 0;
    if (ever) {
        fh = r.h / 100.0f;
        if (fh < 0) fh = 0;
        if (fh > 1) fh = 1;
    }
    if (fh > 0.004f) arcBand(188, 194, A0, A0 + SWEEP * fh, HUM_STOPS, 3, fh);

    // ---- status ----
    uint16_t sc = !ever ? C_BAD : (fresh ? C_OK : C_WARN);
    fillCircle(CX - 48, 110, 4, sc);
    text(fresh ? "LIVE" : (ever ? "STALE" : "NO DATA"), CX - 36, 104, 2, sc, false);

    text("AMBIENT", CX, 136, 3, C_TXT, true);

    // ---- big temperature ----
    char buf[24];
    float tshow = ever ? r.t : 0.0f;
    snprintf(buf, sizeof(buf), "%.1f", tshow);
    const int GAP = 15, DW = 48, DH = 86, DT = 12;
    float tw = numWidth(buf, DW, GAP);
    int groupX = CX - (int)(tw / 2.0f) + 6;
    num(buf, groupX, 168, DW, DH, DT, ever ? C_HI : C_DIM, false, GAP);

    int dx = groupX + (int)tw + 10;
    ringOutline(dx + 14, 171, 7, 4, ever ? C_HI : C_DIM);
    text("C", dx + 5, 186, 3, ever ? C_HI : C_DIM, false);

    // ---- humidity row ----
    uint16_t hcol = ramp(HUM_STOPS, 3, fh);
    snprintf(buf, sizeof(buf), "%.1f", ever ? r.h : 0.0f);
    const int HW = 31, HH = 44, HT = 8, HGAP = 9;
    float hw = numWidth(buf, HW, HGAP);
    int grp = 26 + 10 + (int)hw + 6 + 16;
    int gx = CX - grp / 2, gy = 300;
    drawDroplet(gx, gy - 1, ever ? hcol : C_DIM);
    num(buf, gx + 36, gy - 4, HW, HH, HT, ever ? RGB565(205, 245, 255) : C_DIM, false, HGAP);
    text("%", gx + 36 + (int)hw + 9, gy + 20, 2, ever ? RGB565(205, 245, 255) : C_DIM, false);

    // ---- footer ----
    snprintf(buf, sizeof(buf), "DEW %.1f", ever ? r.dew : 0.0f);
    text(buf, CX, 356, 2, C_DIM, true);
    if (ever) {
        snprintf(buf, sizeof(buf), "MIN %.1f   MAX %.1f", r.tmin, r.tmax);
    } else {
        snprintf(buf, sizeof(buf), "MIN --.-   MAX --.-");
    }
    text(buf, CX, 382, 2, C_DIM, true);
}

// =================================================================== BLE link ==
// The SensorTag runs TI's stock firmware and advertises the Humidity service
// (F000AA20-0451-4000-B000-000000000000).  This board is the BLE central:
// scan -> connect -> enable AA21 notifications -> read 4-byte payloads.
static BLEUUID SVC_HUM ("f000aa20-0451-4000-b000-000000000000");
static BLEUUID CH_DATA ("f000aa21-0451-4000-b000-000000000000");
static BLEUUID CH_CONF ("f000aa22-0451-4000-b000-000000000000");
static BLEUUID SVC_IR  ("f000aa00-0451-4000-b000-000000000000");
static BLEUUID CH_IR_DATA ("f000aa01-0451-4000-b000-000000000000");
static BLEUUID CH_IR_CONF ("f000aa02-0451-4000-b000-000000000000");
static BLEUUID CCCD    ("00002902-0000-1000-8000-00805f9b34fb");

static Reading reading;
static portMUX_TYPE rMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool     bleUp = false;
static volatile uint32_t lastPacketMs = 0;
static volatile uint32_t pktCount = 0;

// ---- notification callbacks (run in the BT task) ---------------------------
// Caller must hold rMux.
static void applyReading(Reading &r)
{
    if (!r.valid) { r.tmin = r.t; r.tmax = r.t; }
    else {
        if (r.t < r.tmin) r.tmin = r.t;
        if (r.t > r.tmax) r.tmax = r.t;
    }
    dewp(r.t, r.h < 1.0f ? 1.0f : r.h, &r.dew);
    r.valid = true;
    r.sensorOk = true;
    r.ms = millis();
    r.seq = (uint8_t)pktCount;
}

// HDC1000 — humidity (and its own temperature, kept as a fallback)
static void onHumidity(BLERemoteCharacteristic *ch, uint8_t *data, size_t len, bool isNotify)
{
    if (len < 4) return;
    int16_t  traw = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    uint16_t hraw = (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
    float tHdc = ((float)traw / 65536.0f) * 165.0f - 40.0f;
    float h    = ((float)(hraw & 0xFFFC) / 65536.0f) * 100.0f;
    if (h > 100.0f) h = 100.0f;
    if (h < 0.0f)   h = 0.0f;

    portENTER_CRITICAL(&rMux);
    reading.h = h;
    reading.tHdc = tHdc;
    if (!reading.irOk) reading.t = tHdc;      // fallback until TMP007 reports
    applyReading(reading);
    lastPacketMs = millis();
    pktCount++;
    portEXIT_CRITICAL(&rMux);

#if DEBUG_BLE
    static uint32_t lastHumLog = 0;
    if (millis() - lastHumLog > 3000) {
        lastHumLog = millis();
        Serial.printf("[HUM] rawT=%d rawH=%d  tHdc=%.2f  rh=%.2f\n",
                      (int)traw, (int)hraw, tHdc, h);
    }
#endif
}

// TMP007 — ambient (die) temperature, bytes 2..3 of the AA01 payload
static void onIRTemp(BLERemoteCharacteristic *ch, uint8_t *data, size_t len, bool isNotify)
{
    if (len < 4) return;
    int16_t rawObj = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    int16_t rawAmb = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
    float tAmb = rawAmb / 128.0f;             // TMP007 ambient, degC

    portENTER_CRITICAL(&rMux);
    reading.t = tAmb;
    reading.irOk = true;
    applyReading(reading);
    lastPacketMs = millis();
    pktCount++;
    portEXIT_CRITICAL(&rMux);

#if DEBUG_BLE
    static uint32_t lastIrLog = 0;
    if (millis() - lastIrLog > 3000) {
        lastIrLog = millis();
        Serial.printf("[IR ] raw0=%d raw1=%d  v0=%.2f v1=%.2f\n",
                      (int)rawObj, (int)rawAmb, rawObj / 128.0f, rawAmb / 128.0f);
    }
#endif
}

// ---- client callbacks -------------------------------------------------------
class ClientCB : public BLEClientCallbacks {
    void onConnect(BLEClient *c) override {
        bleUp = true;
        Serial.println("[BLE] connected");
    }
    void onDisconnect(BLEClient *c) override {
        bleUp = false;
        Serial.println("[BLE] disconnected");
    }
};

// ---- scan callback ----------------------------------------------------------
static BLEAdvertisedDevice *foundDev = nullptr;

class ScanCB : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) override {
        if (foundDev) return;
#if DEBUG_BLE
        static uint32_t lastScanLog = 0;
        if (millis() - lastScanLog > 1200) {
            lastScanLog = millis();
            Serial.printf("[SCAN] %s rssi=%d name=%s svc=%d\n",
                          dev.getAddress().toString().c_str(), dev.getRSSI(),
                          dev.haveName() ? dev.getName().c_str() : "-",
                          dev.haveServiceUUID() ? 1 : 0);
        }
#endif
        bool nameHit = dev.haveName() && dev.getName().find("SensorTag") != std::string::npos;
        bool svcHit  = dev.haveServiceUUID() && dev.getServiceUUID().equals(SVC_HUM);
        if (nameHit || svcHit) {
            foundDev = new BLEAdvertisedDevice(dev);
            Serial.printf("[BLE] found %s (%s)\n",
                          dev.haveName() ? dev.getName().c_str() : "?",
                          dev.getAddress().toString().c_str());
        }
    }
};

// ---- link task: scan, connect, subscribe, repeat ----------------------------
static void bleTask(void *)
{
    BLEScan *scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCB());
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);

    for (;;) {
        if (!bleUp) {
            foundDev = nullptr;
            scan->start(5, false);          // blocking 5 s scan
            scan->clearResults();

            if (foundDev) {
                BLEClient *c = BLEDevice::createClient();
                c->setClientCallbacks(new ClientCB());
                if (c->connect(foundDev)) {
                    BLERemoteService *svc = c->getService(SVC_HUM);
                    if (svc) {
                        BLERemoteCharacteristic *conf = svc->getCharacteristic(CH_CONF);
                        BLERemoteCharacteristic *data = svc->getCharacteristic(CH_DATA);
                        if (conf) conf->writeValue((uint8_t)0x01);   // start measuring
                        if (data && data->canNotify()) {
                            data->registerForNotify(onHumidity);
                            BLERemoteDescriptor *d = data->getDescriptor(CCCD);
                            if (d) { uint8_t en[2] = { 0x01, 0x00 }; d->writeValue(en, 2, true); }
                            Serial.println("[BLE] humidity notifications on");
                        }
                    } else {
                        Serial.println("[BLE] humidity service missing");
                    }

                    // TMP007 IR service — ambient (die) channel drives the display
                    BLERemoteService *ir = c->getService(SVC_IR);
                    if (ir) {
                        BLERemoteCharacteristic *iconf = ir->getCharacteristic(CH_IR_CONF);
                        BLERemoteCharacteristic *idata = ir->getCharacteristic(CH_IR_DATA);
                        if (iconf) iconf->writeValue((uint8_t)0x01);   // start measuring
                        if (idata && idata->canNotify()) {
                            idata->registerForNotify(onIRTemp);
                            BLERemoteDescriptor *d2 = idata->getDescriptor(CCCD);
                            if (d2) { uint8_t en[2] = { 0x01, 0x00 }; d2->writeValue(en, 2, true); }
                            Serial.println("[BLE] IR temp notifications on");
                        }
                    } else {
                        Serial.println("[BLE] IR service missing");
                    }
                    Serial.printf("[BLE] services: hum=%d ir=%d\n", svc ? 1 : 0, ir ? 1 : 0);
                } else {
                    Serial.println("[BLE] connect failed");
                }
                delete foundDev;
                foundDev = nullptr;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ===================================================================== setup =
void setup()
{
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[T-RGB] ambient panel (BLE)");

    if (!psramFound()) {
        Serial.println("FATAL: PSRAM not enabled (Board: PSRAM = OPI)");
        while (1) delay(1000);
    }

    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL, 400000);   // panel init only
    if (!gfx->begin()) {
        Serial.println("FATAL: panel init failed (check PANEL_2_8_INCH)");
        while (1) delay(1000);
    }
    gfx->fillScreen(RGB565(4, 6, 11));
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    fb = (uint16_t *)ps_malloc(fbBytes);
    bg = (uint16_t *)ps_malloc(fbBytes);
    if (!fb || !bg) {
        Serial.println("FATAL: framebuffer alloc failed");
        while (1) delay(1000);
    }

    drawBackground();
    memcpy(bg, fb, fbBytes);
    drawScreen(reading, false, false);
    gfx->draw16bitRGBBitmap(0, 0, fb, W, H);

    BLEDevice::init("T-RGB-Display");
    xTaskCreatePinnedToCore(bleTask, "ble", 8192, nullptr, 1, nullptr, 0);
    Serial.println("[T-RGB] BLE central started");
}

void loop()
{
    static uint32_t lastDraw = 0;
    Reading snap;
    uint32_t last;
    portENTER_CRITICAL(&rMux);
    snap = reading;
    last = lastPacketMs;
    portEXIT_CRITICAL(&rMux);

    bool fresh = snap.valid && (millis() - last < 5000);
#if DEBUG_BLE
    static uint32_t lastBeat = 0;
    if (millis() - lastBeat > 3000) {
        lastBeat = millis();
        Serial.printf("[BEAT] t=%.2f tHdc=%.2f rh=%.2f irOk=%d up=%d\n",
                      snap.t, snap.tHdc, snap.h, snap.irOk ? 1 : 0, bleUp ? 1 : 0);
    }
#endif
    if (millis() - lastDraw > 500) {
        lastDraw = millis();
        drawScreen(snap, fresh, snap.valid);
        gfx->draw16bitRGBBitmap(0, 0, fb, W, H);
    }
    delay(5);
}
