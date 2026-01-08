// =============================================================================
// TFT_Display_CMWS.cpp — CockpitOS CMWS Threat Ring Display (LOW-MEM, DIRECT-DRAW)
// =============================================================================
// AH-64D Apache Countermeasures Warning System Display
//
// Hardware Support:
//   - ESP32 Classic + IdeasPark ST7789 170×320 TFT (4-wire SPI)
//   - ESP32-S3 + LilyGo T-Display S3 ST7789 170×320 TFT (8-bit Parallel)
//
// DESIGN GOALS (ESP32 Classic: no PSRAM, tight SRAM):
//   1) No full-frame sprites.
//   2) Precompute geometry once (ticks + arrows + AABBs).
//   3) Incremental redraw that is CORRECT: no "erase with black" unless
//      we also restore whatever was under (ticks + other arrows).
//   4) Deterministic: snapshot state under a critical section.
//   5) Moderate complexity: rectangular region restore + redraw layers.
//
// LAYERS:
//   Base layer: black background + tick marks (excluding angles occupied by small arrows).
//   Dynamic layer: 8 small arrows + 4 large arrows + D/R letters + inventory/BIT text.
//
// Incremental update strategy:
//   For each changed element, redraw its AABB region:
//     - clear region to black
//     - redraw ticks that intersect region
//     - redraw ANY arrows (small/large) whose AABB intersects region
//     - redraw D/R if region intersects their fixed rects
//   Inventory/BIT text uses dedicated rect clears.
//
// DEVICE ON/OFF RULES:
//   - Device is OFF only when ALL 4 large arrows have BOTH BRT=0 AND DIM=0
//   - When OFF: LAMP=0%, small arrows=OFF, ticks=OFF, nothing drawn
//   - When ON: small arrows and ticks are always DIM
//
// PAGE MODE RULES:
//   - Only accept exact full strings: "MAIN" or "TEST" (ignore partials)
//   - MAIN: show FLARE_LETTER/FLARE_COUNT and CHAFF_LETTER/CHAFF_COUNT
//   - TEST: show BIT_LINE_1 and BIT_LINE_2
//   - Field updates only accepted when valid for current page mode
//
// =============================================================================

#include "../Globals.h"

#if defined(HAS_CMWS_DISPLAY) && defined(ENABLE_TFT_GAUGES) && (ENABLE_TFT_GAUGES == 1)

#include "../HIDManager.h"
#include "../DCSBIOSBridge.h"
#include "includes/TFT_Display_CMWS.h"

// =============================================================================
// PANEL REGISTRATION
// =============================================================================
#if defined(HAS_CMWS_DISPLAY)
    REGISTER_PANEL(TFTCmws, nullptr, nullptr, CMWSDisplay_init, CMWSDisplay_loop, nullptr, 100);
#endif

#if !__has_include(<LovyanGFX.hpp>)
#error "❌ Missing LovyanGFX.hpp — Please install LovyanGFX library"
#endif

#include <LovyanGFX.hpp>
#include <cstring>
#include <cmath>

// =============================================================================
// MISC HELPERS (NO HEAP, FIXED-WIDTH FIELDS)
// =============================================================================
#include <cstdlib>   // atoi

// Count is ALWAYS stored/displayed as exactly 2 digits: "00".."99"
static inline void formatCount2(char out[3], const char* value) {
    int vi = 0;
    if (value) vi = atoi(value);     // skips leading spaces
    if (vi < 0)  vi = 0;
    if (vi > 99) vi = 99;

    const unsigned v = static_cast<unsigned>(vi);
    out[0] = static_cast<char>('0' + (v / 10U));
    out[1] = static_cast<char>('0' + (v % 10U));
    out[2] = '\0';
}

// BIT line is ALWAYS stored/displayed as exactly 4 chars (space padded), plus '\0'.
static inline void formatField4(char out[5], const char* in) {
    out[0] = ' '; out[1] = ' '; out[2] = ' '; out[3] = ' ';
    out[4] = '\0';

    if (!in) return;

    for (int i = 0; i < 4; ++i) {
        const char c = in[i];
        if (c == '\0') break;
        out[i] = c;
    }
}

// Inventory line is ALWAYS exactly 4 chars: "F 60", "C 01"
static inline void buildInvLine4(char out[5], const char letter[2], const char count[3]) {
    out[0] = (letter && letter[0]) ? letter[0] : ' ';
    out[1] = ' ';
    out[2] = (count) ? count[0] : '0';
    out[3] = (count) ? count[1] : '0';
    out[4] = '\0';
}

// =============================================================================
// FONTS - Converted from TTF to .h using https://rop.nl/truetype2gfx/
// =============================================================================

// Doto Font 26pt
#include "Assets/Fonts/Doto_Rounded_Black26pt7b.h"
static const GFXfont* const FONT_DOTO = &Doto_Rounded_Black26pt7b;

// Milspec Font 10pt
#include "Assets/Fonts/MilSpec3355810pt7b.h"
static const GFXfont* const FONT_MILSPEC = &MilSpec3355810pt7b;

// =============================================================================
// CONFIG
// =============================================================================
static constexpr uint32_t CMWS_REFRESH_INTERVAL_MS = 33;   // ~30 FPS max

static constexpr bool     RUN_AS_TASK     = false;
static constexpr uint16_t TASK_STACK_SIZE = 4096;
static constexpr uint8_t  TASK_PRIORITY   = 2;
static constexpr uint8_t  CPU_CORE        = 0;

#define RUN_BIT_TEST_ON_INIT 1

// =============================================================================
// DISPLAY INTERFACE SELECTION
// =============================================================================
// Uncomment ONE of the following to select your hardware configuration:
//
// #define CMWS_USE_SPI_INTERFACE        // ESP32 Classic with 4-wire SPI display
// #define CMWS_USE_PARALLEL_INTERFACE   // ESP32-S3 with 8-bit parallel display (e.g., LilyGo T-Display S3)
//
// If neither is defined, auto-detect based on chip family:

#if !defined(CMWS_USE_SPI_INTERFACE) && !defined(CMWS_USE_PARALLEL_INTERFACE)
    #if defined(CONFIG_IDF_TARGET_ESP32S3)
        #define CMWS_USE_PARALLEL_INTERFACE
    #elif defined(ESP_FAMILY_CLASSIC) || defined(CONFIG_IDF_TARGET_ESP32)
        #define CMWS_USE_SPI_INTERFACE
    #else
        // Default to SPI for unknown platforms
        #define CMWS_USE_SPI_INTERFACE
    #endif
#endif

// =============================================================================
// PIN DEFINITIONS - SPI INTERFACE (ESP32 Classic + IdeasPark ST7789)
// =============================================================================
#if defined(CMWS_USE_SPI_INTERFACE)

    #if defined(HAS_CMWS_DISPLAY)
        static constexpr int8_t PIN_MOSI = PIN(23);
        static constexpr int8_t PIN_SCLK = PIN(18);
        static constexpr int8_t PIN_CS   = PIN(15);
        static constexpr int8_t PIN_DC   = PIN(2);
        static constexpr int8_t PIN_RST  = PIN(4);   // MUST BE GPIO4 for this module
        static constexpr int8_t PIN_BLK  = PIN(32);
    #else
        static constexpr int8_t PIN_MOSI = -1;
        static constexpr int8_t PIN_SCLK = -1;
        static constexpr int8_t PIN_CS   = -1;
        static constexpr int8_t PIN_DC   = -1;
        static constexpr int8_t PIN_RST  = -1;
        static constexpr int8_t PIN_BLK  = -1;
    #endif

    #if defined(ESP_FAMILY_CLASSIC)
        static constexpr spi_host_device_t CMWS_SPI_HOST = VSPI_HOST;
    #else
        static constexpr spi_host_device_t CMWS_SPI_HOST = SPI2_HOST;
    #endif

#endif // CMWS_USE_SPI_INTERFACE

// =============================================================================
// PIN DEFINITIONS - 8-BIT PARALLEL INTERFACE (LilyGo T-Display S3)
// =============================================================================
#if defined(CMWS_USE_PARALLEL_INTERFACE)

    // LilyGo T-Display S3 Pinout (ST7789 170x320)
    static constexpr int8_t PIN_D0  = 39;
    static constexpr int8_t PIN_D1  = 40;
    static constexpr int8_t PIN_D2  = 41;
    static constexpr int8_t PIN_D3  = 42;
    static constexpr int8_t PIN_D4  = 45;
    static constexpr int8_t PIN_D5  = 46;
    static constexpr int8_t PIN_D6  = 47;
    static constexpr int8_t PIN_D7  = 48;

    static constexpr int8_t PIN_WR  = 8;    // Write strobe
    static constexpr int8_t PIN_RD  = 9;    // Read strobe
    static constexpr int8_t PIN_DC  = 7;    // Data/Command (RS)
    static constexpr int8_t PIN_CS  = 6;    // Chip Select
    static constexpr int8_t PIN_RST = 5;    // Reset
    static constexpr int8_t PIN_BLK = 38;   // Backlight

    static constexpr int8_t PIN_POWER = 15; // LCD Power Enable (T-Display S3 specific)

#endif // CMWS_USE_PARALLEL_INTERFACE

// =============================================================================
// GEOMETRY
// =============================================================================
static constexpr int16_t SCREEN_W = 320;
static constexpr int16_t SCREEN_H = 170;

static constexpr int16_t RING_CX = 235;
static constexpr int16_t RING_CY = 85;

static constexpr int16_t TICK_INNER_R = 66;
static constexpr int16_t TICK_OUTER_R = 76;

static constexpr int     TICK_COUNT = 24;    // 15° increments

// Y positions (baseline positions)
static constexpr int16_t TEXT_LINE1 = 28;      // Line 1 baseline
static constexpr int16_t TEXT_LINE2 = 100;     // Line 2 baseline

// Clear rectangle dimensions
static constexpr int16_t TEXT_CLEAR_H = 42;    // Glyph height 33 + 9px margin (safe)
static constexpr int16_t TEXT_CLEAR_W = 135;   // 4 chars × 31px = 124 + 11px margin (safe)

// D/R letter offsets
static constexpr int16_t DR_OFFSET = 40;
static constexpr int16_t DR_X_OFFSET = 3;  // Shift D/R right by this many pixels

// Text X position
static constexpr int16_t TEXT_X = 10;

// =============================================================================
// ARROW SHAPE
// =============================================================================
static constexpr float LARGE_TIP_Y       = 30.0f;
static constexpr float LARGE_TIP_BASE_Y  = 11.0f;
static constexpr float LARGE_BODY_BASE_Y = 0.0f;
static constexpr float LARGE_TIP_HALF_W  = 16.0f;
static constexpr float LARGE_BODY_HALF_W = 8.5f;
static constexpr float SMALL_ARROW_SCALE = 0.5f;

// =============================================================================
// ARROW POSITIONS
// =============================================================================
static constexpr int LARGE_ARROW_COUNT = 4;
static constexpr int LARGE_ARROW_ANGLES[LARGE_ARROW_COUNT] = { 45, 135, 225, 315 };

// Small arrows: 8 positions (every 45°) INSIDE tick band
static constexpr int SMALL_ARROW_COUNT = 8;
static constexpr int SMALL_ARROW_ANGLES[SMALL_ARROW_COUNT] = { 0, 45, 90, 135, 180, 225, 270, 315 };

// Small arrows sit inside the same boundary band as ticks
static constexpr int16_t SMALL_ARROW_RADIUS = TICK_OUTER_R - 10;

// Large radius computed at init
static int16_t g_largeArrowRadius = 0;

// =============================================================================
// COLORS (pixel-matched to reference display)
// =============================================================================
#define RGB565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))

static constexpr uint16_t COL_BLACK = 0x0000;
static constexpr uint16_t COL_GREEN = RGB565(115, 190, 100);  // punchy green
static constexpr uint16_t COL_AMBER_BRT = RGB565(255, 200, 0);  // more golden
static constexpr uint16_t COL_AMBER_DIM = RGB565(8, 4, 0);  // even darker

// =============================================================================
// STATE
// =============================================================================
enum class ElemState : uint8_t { OFF = 0, DIM = 1, BRT = 2 };

// Page mode enum for clarity
enum class PageMode : uint8_t { MAIN = 0, TEST = 1 };

static inline ElemState computeStateFromBits(bool brt, bool dim) {
    if (brt) return ElemState::BRT;
    if (dim) return ElemState::DIM;
    return ElemState::OFF;
}

static inline uint16_t colorFor(ElemState s) {
    return (s == ElemState::BRT) ? COL_AMBER_BRT : COL_AMBER_DIM;
}

// =============================================================================
// PRECOMPUTED CACHES
// =============================================================================
struct ArrowCache {
    TFT_Point tip;
    TFT_Point tipBaseL;
    TFT_Point tipBaseR;
    TFT_Point bodyTopL;
    TFT_Point bodyTopR;
    TFT_Point bodyBotL;
    TFT_Point bodyBotR;
};
struct TickCache {
    TFT_Point inner;
    TFT_Point outer;
};
struct RectI16 {
    int16_t x, y, w, h;
};

static ArrowCache g_largeArrows[LARGE_ARROW_COUNT];
static ArrowCache g_smallArrows[SMALL_ARROW_COUNT];
static TickCache  g_ticks[TICK_COUNT];

static RectI16 g_largeAabb[LARGE_ARROW_COUNT];
static RectI16 g_smallAabb[SMALL_ARROW_COUNT];

static RectI16 g_dRect; // bounding rect for "D"
static RectI16 g_rRect; // bounding rect for "R"

// =============================================================================
// LOVYANGFX DEVICE CLASS - SPI INTERFACE
// =============================================================================
#if defined(CMWS_USE_SPI_INTERFACE)

class LGFX_CMWS final : public lgfx::LGFX_Device {
    lgfx::Bus_SPI       _bus;
    lgfx::Panel_ST7789  _panel;
    lgfx::Light_PWM     _light;

public:
    LGFX_CMWS() {
        {
            auto cfg = _bus.config();
            cfg.spi_host    = CMWS_SPI_HOST;
            cfg.spi_mode    = 0;
            cfg.freq_write  = 80000000;     // if corruption: drop to 60000000 or 40000000
            cfg.freq_read   = 16000000;
            cfg.spi_3wire   = false;
            cfg.use_lock    = false;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_mosi    = PIN_MOSI;
            cfg.pin_miso    = -1;
            cfg.pin_sclk    = PIN_SCLK;
            cfg.pin_dc      = PIN_DC;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            auto cfg = _panel.config();
            cfg.pin_cs          = PIN_CS;
            cfg.pin_rst         = PIN_RST;
            cfg.pin_busy        = -1;

            cfg.memory_width    = 240;
            cfg.memory_height   = 320;
            cfg.panel_width     = 170;
            cfg.panel_height    = 320;
            cfg.offset_x        = 35;
            cfg.offset_y        = 0;
            cfg.offset_rotation = 0;

            cfg.readable        = false;
            cfg.bus_shared      = false;
            cfg.invert          = true;
            cfg.rgb_order       = false;
            cfg.dlen_16bit      = false;
            _panel.config(cfg);
        }
        {
            auto cfg = _light.config();
            cfg.pin_bl      = PIN_BLK;
            cfg.invert      = false;
            cfg.freq        = 12000;
            cfg.pwm_channel = 7;
            _light.config(cfg);
            _panel.setLight(&_light);
        }
        setPanel(&_panel);
    }
};

#endif // CMWS_USE_SPI_INTERFACE

// =============================================================================
// LOVYANGFX DEVICE CLASS - 8-BIT PARALLEL INTERFACE (LilyGo T-Display S3)
// =============================================================================
#if defined(CMWS_USE_PARALLEL_INTERFACE)

class LGFX_CMWS final : public lgfx::LGFX_Device {
    lgfx::Bus_Parallel8 _bus;
    lgfx::Panel_ST7789  _panel;
    lgfx::Light_PWM     _light;

public:
    LGFX_CMWS() {
        {
            auto cfg = _bus.config();
            cfg.freq_write  = 20000000;    // Write clock (max 80MHz for S3, but 20MHz is safer)
            cfg.freq_read   = 8000000;     // Read clock

            cfg.pin_wr      = PIN_WR;      // Write strobe pin
            cfg.pin_rd      = PIN_RD;      // Read strobe pin
            cfg.pin_rs      = PIN_DC;      // D/C (Data/Command) pin

            // 8-bit data bus
            cfg.pin_d0      = PIN_D0;
            cfg.pin_d1      = PIN_D1;
            cfg.pin_d2      = PIN_D2;
            cfg.pin_d3      = PIN_D3;
            cfg.pin_d4      = PIN_D4;
            cfg.pin_d5      = PIN_D5;
            cfg.pin_d6      = PIN_D6;
            cfg.pin_d7      = PIN_D7;

            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            auto cfg = _panel.config();
            cfg.pin_cs          = PIN_CS;
            cfg.pin_rst         = PIN_RST;
            cfg.pin_busy        = -1;

            cfg.memory_width    = 240;
            cfg.memory_height   = 320;
            cfg.panel_width     = 170;
            cfg.panel_height    = 320;
            cfg.offset_x        = 35;
            cfg.offset_y        = 0;
            cfg.offset_rotation = 0;

            cfg.readable        = true;    // Parallel interface supports read
            cfg.bus_shared      = false;
            cfg.invert          = true;
            cfg.rgb_order       = false;
            cfg.dlen_16bit      = false;
            _panel.config(cfg);
        }
        {
            auto cfg = _light.config();
            cfg.pin_bl      = PIN_BLK;
            cfg.invert      = false;
            cfg.freq        = 12000;
            cfg.pwm_channel = 7;
            _light.config(cfg);
            _panel.setLight(&_light);
        }
        setPanel(&_panel);
    }
};

#endif // CMWS_USE_PARALLEL_INTERFACE

static LGFX_CMWS tft;
static TaskHandle_t taskHandle = nullptr;

// =============================================================================
// PLATFORM-SPECIFIC INITIALIZATION
// =============================================================================
static void platformInit() {
#if defined(CMWS_USE_PARALLEL_INTERFACE)
    // LilyGo T-Display S3: Enable LCD power
    pinMode(PIN_POWER, OUTPUT);
    digitalWrite(PIN_POWER, HIGH);
    delay(10);  // Allow power to stabilize
#endif
}

// =============================================================================
// CONCURRENCY: snapshot state
// =============================================================================
static portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;

struct CmwsState {
    PageMode pageMode;        // MAIN or TEST
    bool     deviceOn;        // true when at least one large arrow has DIM or BRT set

    ElemState large[LARGE_ARROW_COUNT];
    ElemState small[SMALL_ARROW_COUNT];
    ElemState dispense;
    ElemState ready;

    uint8_t  backlight;       // user-requested backlight (only applied when deviceOn)

    char flareLetter[4];
    char chaffLetter[4];
    char flareCount[8];
    char chaffCount[8];
    char bitLine1[8];
    char bitLine2[8];
};

static CmwsState g_pending;     // written by callbacks under mux
static CmwsState g_lastDrawn;   // last frame snapshot (no mux needed inside draw task)

static volatile bool g_dirty = true;
static volatile bool g_forceFull = true;
static uint32_t g_lastDrawMs = 0;

// Cached brt/dim bits for large arrows + D/R (no getMetadataValue() readback)
static volatile bool g_largeBrt[LARGE_ARROW_COUNT] = { false, false, false, false };
static volatile bool g_largeDim[LARGE_ARROW_COUNT] = { false, false, false, false };

static volatile bool g_dispBrt = false;
static volatile bool g_dispDim = false;
static volatile bool g_readyBrt = false;
static volatile bool g_readyDim = false;

// =============================================================================
// DEVICE ON/OFF LOGIC
// =============================================================================
// Device is OFF only when ALL 4 large arrows have BOTH BRT=0 AND DIM=0
static inline bool computeDeviceOn() {
    for (int i = 0; i < LARGE_ARROW_COUNT; ++i) {
        if (g_largeBrt[i] || g_largeDim[i]) {
            return true;  // At least one arrow has signal → device is ON
        }
    }
    return false;  // All arrows have no signal → device is OFF
}

// =============================================================================
// SMALL HELPERS
// =============================================================================
static inline int16_t clampI16(int32_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return static_cast<int16_t>(v);
}

static inline RectI16 rectFromPoints(const TFT_Point* pts, int count, int16_t pad = 1) {
    int16_t minX = 32767, minY = 32767, maxX = -32768, maxY = -32768;
    for (int i = 0; i < count; ++i) {
        if (pts[i].x < minX) minX = pts[i].x;
        if (pts[i].y < minY) minY = pts[i].y;
        if (pts[i].x > maxX) maxX = pts[i].x;
        if (pts[i].y > maxY) maxY = pts[i].y;
    }
    minX = clampI16(minX - pad, 0, SCREEN_W - 1);
    minY = clampI16(minY - pad, 0, SCREEN_H - 1);
    maxX = clampI16(maxX + pad, 0, SCREEN_W - 1);
    maxY = clampI16(maxY + pad, 0, SCREEN_H - 1);
    return RectI16{ minX, minY, static_cast<int16_t>(maxX - minX + 1), static_cast<int16_t>(maxY - minY + 1) };
}

static inline bool rectIntersects(const RectI16& a, const RectI16& b) {
    return !(a.x + a.w <= b.x || b.x + b.w <= a.x || a.y + a.h <= b.y || b.y + b.h <= a.y);
}

// =============================================================================
// TRIG / VECTORS (uses your header LUT for 15° multiples)
// =============================================================================
static void computeForward(int angleDeg, float& fx, float& fy) {
    angleDeg = TFT_Trig::normalizeAngle(angleDeg);
    if ((angleDeg % 15) == 0) {
        fx = TFT_Trig::fastSin15(angleDeg);
        fy = -TFT_Trig::fastCos15(angleDeg);
    } else {
        const float rad = static_cast<float>(angleDeg) * 0.017453292f;
        fx = sinf(rad);
        fy = -cosf(rad);
    }
}

static float maxCenterRadiusForTipOnScreen(int angleDeg, float scale) {
    float fx, fy;
    computeForward(angleDeg, fx, fy);
    const float tipOffset = LARGE_TIP_Y * scale;

    float tMaxX = 1e9f;
    if (fx > 0.0001f) {
        tMaxX = (static_cast<float>(SCREEN_W - 1 - RING_CX) / fx) - tipOffset;
    } else if (fx < -0.0001f) {
        tMaxX = (static_cast<float>(0 - RING_CX) / fx) - tipOffset;
    }

    float tMaxY = 1e9f;
    if (fy > 0.0001f) {
        tMaxY = (static_cast<float>(SCREEN_H - 1 - RING_CY) / fy) - tipOffset;
    } else if (fy < -0.0001f) {
        tMaxY = (static_cast<float>(0 - RING_CY) / fy) - tipOffset;
    }

    const float tMax = (tMaxX < tMaxY) ? tMaxX : tMaxY;
    return (tMax > 0.0f) ? tMax : 0.0f;
}

static int computeLargeArrowRadiusSymmetric() {
    float r = 1e9f;
    for (int i = 0; i < LARGE_ARROW_COUNT; ++i) {
        const float ri = maxCenterRadiusForTipOnScreen(LARGE_ARROW_ANGLES[i], 1.0f);
        if (ri < r) r = ri;
    }
    return (r > 0) ? static_cast<int>(r) : 0;
}

static void computeArrowVertices(ArrowCache& cache, int angleDeg, int16_t radius, float scale) {
    float fx, fy;
    computeForward(angleDeg, fx, fy);

    // right vector
    const float rx = fy;
    const float ry = -fx;

    // center
    const float cx = static_cast<float>(RING_CX) + fx * static_cast<float>(radius);
    const float cy = static_cast<float>(RING_CY) + fy * static_cast<float>(radius);

    const float tipY      = LARGE_TIP_Y * scale;
    const float tipBaseY  = LARGE_TIP_BASE_Y * scale;
    const float bodyBaseY = LARGE_BODY_BASE_Y * scale;
    const float tipHalfW  = LARGE_TIP_HALF_W * scale;
    const float bodyHalfW = LARGE_BODY_HALF_W * scale;

    auto toWorld = [&](float lx, float ly, int16_t& wx, int16_t& wy) {
        const float worldX = cx + (lx * rx) + (ly * fx);
        const float worldY = cy + (lx * ry) + (ly * fy);
        wx = static_cast<int16_t>(worldX >= 0 ? (worldX + 0.5f) : (worldX - 0.5f));
        wy = static_cast<int16_t>(worldY >= 0 ? (worldY + 0.5f) : (worldY - 0.5f));
    };

    toWorld(0.0f,        tipY,      cache.tip.x,      cache.tip.y);
    toWorld(-tipHalfW,   tipBaseY,  cache.tipBaseL.x, cache.tipBaseL.y);
    toWorld(+tipHalfW,   tipBaseY,  cache.tipBaseR.x, cache.tipBaseR.y);

    toWorld(-bodyHalfW,  tipBaseY,  cache.bodyTopL.x, cache.bodyTopL.y);
    toWorld(+bodyHalfW,  tipBaseY,  cache.bodyTopR.x, cache.bodyTopR.y);
    toWorld(-bodyHalfW,  bodyBaseY, cache.bodyBotL.x, cache.bodyBotL.y);
    toWorld(+bodyHalfW,  bodyBaseY, cache.bodyBotR.x, cache.bodyBotR.y);
}

static void computeTickVertices(TickCache& cache, int angleDeg) {
    float fx, fy;
    computeForward(angleDeg, fx, fy);

    cache.inner.x = static_cast<int16_t>(RING_CX + fx * TICK_INNER_R + 0.5f);
    cache.inner.y = static_cast<int16_t>(RING_CY + fy * TICK_INNER_R + 0.5f);
    cache.outer.x = static_cast<int16_t>(RING_CX + fx * TICK_OUTER_R + 0.5f);
    cache.outer.y = static_cast<int16_t>(RING_CY + fy * TICK_OUTER_R + 0.5f);
}

// =============================================================================
// DRAW PRIMITIVES (DIRECT)
// =============================================================================
static void drawArrowDirect(const ArrowCache& a, uint16_t color) {
    // Tip triangle
    tft.fillTriangle(a.tip.x, a.tip.y, a.tipBaseL.x, a.tipBaseL.y, a.tipBaseR.x, a.tipBaseR.y, color);
    // Body (2 triangles)
    tft.fillTriangle(a.bodyTopL.x, a.bodyTopL.y, a.bodyTopR.x, a.bodyTopR.y, a.bodyBotR.x, a.bodyBotR.y, color);
    tft.fillTriangle(a.bodyTopL.x, a.bodyTopL.y, a.bodyBotR.x, a.bodyBotR.y, a.bodyBotL.x, a.bodyBotL.y, color);
}

static void drawTickDirect(const TickCache& t, uint16_t color) {
    tft.drawLine(t.inner.x, t.inner.y, t.outer.x, t.outer.y, color);
}

// =============================================================================
// BASE LAYER HELPERS
// =============================================================================
static bool angleIsSmallArrow(int angleDeg) {
    for (int i = 0; i < SMALL_ARROW_COUNT; ++i) {
        if (angleDeg == SMALL_ARROW_ANGLES[i]) return true;
    }
    return false;
}

// Draw ticks intersecting rect - only when device is ON
static void drawTicksIntersectingRect(const RectI16& r, bool deviceOn) {
    if (!deviceOn) return;  // Device OFF: no ticks drawn

    // ticks are at 15° increments
    for (int i = 0; i < TICK_COUNT; ++i) {
        const int angle = (i * 360) / TICK_COUNT;

        // ticks replaced by small arrows (including diagonals)
        if (angleIsSmallArrow(angle)) continue;

        // AABB test for the tick line (cheap)
        const TickCache& t = g_ticks[i];
        int16_t minX = (t.inner.x < t.outer.x) ? t.inner.x : t.outer.x;
        int16_t maxX = (t.inner.x > t.outer.x) ? t.inner.x : t.outer.x;
        int16_t minY = (t.inner.y < t.outer.y) ? t.inner.y : t.outer.y;
        int16_t maxY = (t.inner.y > t.outer.y) ? t.inner.y : t.outer.y;

        RectI16 tr { minX, minY, static_cast<int16_t>(maxX - minX + 1), static_cast<int16_t>(maxY - minY + 1) };
        if (rectIntersects(r, tr)) {
            drawTickDirect(t, COL_AMBER_DIM);
        }
    }
}

// Redraw dynamic arrows that intersect the region
static void drawArrowsIntersectingRect(const RectI16& r, const CmwsState& s) {
    // Small arrows: only draw when device is ON (always DIM when ON)
    if (s.deviceOn) {
        for (int i = 0; i < SMALL_ARROW_COUNT; ++i) {
            if (!rectIntersects(r, g_smallAabb[i])) continue;
            // Small arrows are always DIM when device is ON
            drawArrowDirect(g_smallArrows[i], COL_AMBER_DIM);
        }
    }

    // Large arrows: draw based on their individual state
    for (int i = 0; i < LARGE_ARROW_COUNT; ++i) {
        if (!rectIntersects(r, g_largeAabb[i])) continue;
        const ElemState st = s.large[i];
        if (st == ElemState::OFF) continue;
        drawArrowDirect(g_largeArrows[i], colorFor(st));
    }
}

static void drawDRIntersectingRect(const RectI16& r, const CmwsState& s) {
    if (!s.deviceOn) return;  // Device OFF: no D/R drawn

    tft.setFont(FONT_MILSPEC);
    tft.setTextDatum(textdatum_t::middle_center);

    if (rectIntersects(r, g_dRect)) {
        if (s.dispense != ElemState::OFF) {
            tft.setTextColor(colorFor(s.dispense));
            tft.drawString("D", RING_CX + DR_X_OFFSET, RING_CY - DR_OFFSET);
        }
    }
    if (rectIntersects(r, g_rRect)) {
        if (s.ready != ElemState::OFF) {
            tft.setTextColor(colorFor(s.ready));
            tft.drawString("R", RING_CX + DR_X_OFFSET, RING_CY + DR_OFFSET);
        }
    }
}

// One "correct" region restore + recompose for ring areas.
static void redrawRegion(const RectI16& r, const CmwsState& s) {
    // clamp
    RectI16 rr = r;
    if (rr.x < 0) { rr.w += rr.x; rr.x = 0; }
    if (rr.y < 0) { rr.h += rr.y; rr.y = 0; }
    if (rr.x + rr.w > SCREEN_W) rr.w = SCREEN_W - rr.x;
    if (rr.y + rr.h > SCREEN_H) rr.h = SCREEN_H - rr.y;
    if (rr.w <= 0 || rr.h <= 0) return;

    // 1) clear
    tft.fillRect(rr.x, rr.y, rr.w, rr.h, COL_BLACK);

    // 2) base layer (ticks - only if device ON)
    drawTicksIntersectingRect(rr, s.deviceOn);

    // 3) dynamic layers that overlap
    drawArrowsIntersectingRect(rr, s);
    drawDRIntersectingRect(rr, s);
}

// =============================================================================
// STATE SNAPSHOT
// =============================================================================
static inline void snapshotState(CmwsState& out) {
    portENTER_CRITICAL(&g_stateMux);
    out = g_pending;
    portEXIT_CRITICAL(&g_stateMux);
}

// =============================================================================
// FULL REDRAW
// =============================================================================
static void fullRedraw(const CmwsState& s) {
    tft.fillScreen(COL_BLACK);

    // If device is OFF, just black screen (LAMP already set to 0)
    if (!s.deviceOn) {
        return;
    }

    // Base layer: all ticks not replaced by small arrows
    RectI16 full { 0, 0, SCREEN_W, SCREEN_H };
    drawTicksIntersectingRect(full, s.deviceOn);

    // Dynamic: small arrows + large arrows
    drawArrowsIntersectingRect(full, s);

    // D/R
    drawDRIntersectingRect(full, s);

    // Text
    tft.setFont(FONT_DOTO);
    tft.setTextColor(COL_GREEN);
    tft.setTextDatum(textdatum_t::top_left);

    if (s.pageMode == PageMode::MAIN) {
        tft.setCursor(TEXT_X, TEXT_LINE1);
        tft.printf("%s%s", s.flareLetter, s.flareCount);

        tft.setCursor(TEXT_X, TEXT_LINE2);
        tft.printf("%s%s", s.chaffLetter, s.chaffCount);
    } else {
        // TEST mode
        tft.setCursor(TEXT_X, TEXT_LINE1);
        tft.print(s.bitLine1);

        tft.setCursor(TEXT_X, TEXT_LINE2);
        tft.print(s.bitLine2);
    }
}

// =============================================================================
// INCREMENTAL TEXT REDRAW
// =============================================================================
static void redrawTextLine(int16_t x, int16_t y, const char* text) {
    tft.fillRect(x, y, TEXT_CLEAR_W, TEXT_CLEAR_H, COL_BLACK);
    tft.setCursor(x, y);
    tft.print(text);
}

// =============================================================================
// MAIN DRAW
// =============================================================================
static void CMWSDisplay_draw(bool force = false) {

    if (!force && !isMissionRunning()) return;

    // quick check to avoid snapshot work
    if (!force && !g_dirty && !g_forceFull) return;

    const uint32_t now = millis();
    if (!force && !g_forceFull && (now - g_lastDrawMs < CMWS_REFRESH_INTERVAL_MS)) return;
    g_lastDrawMs = now;

    CmwsState s;
    snapshotState(s);

    // Backlight: 0% when device OFF, user-requested when ON
    const uint8_t effectiveBacklight = s.deviceOn ? s.backlight : 0;
    tft.setBrightness(effectiveBacklight);

    // Device OFF => blank screen
    if (!s.deviceOn) {
        if (g_forceFull || g_lastDrawn.deviceOn) {
            tft.startWrite();
            tft.fillScreen(COL_BLACK);
            tft.endWrite();
        }
        g_forceFull = false;
        g_dirty = false;
        g_lastDrawn = s;
        return;
    }

    tft.startWrite();

    // Full redraw if forced or device just came ON
    if (g_forceFull || !g_lastDrawn.deviceOn || (s.pageMode != g_lastDrawn.pageMode)) {
        fullRedraw(s);
        g_forceFull = false;
        g_dirty = false;
        g_lastDrawn = s;
        tft.endWrite();
        return;
    }

    // Incremental:
    // 1) Large arrows that changed => redraw their AABB region
    for (int i = 0; i < LARGE_ARROW_COUNT; ++i) {
        if (s.large[i] != g_lastDrawn.large[i]) {
            redrawRegion(g_largeAabb[i], s);
            g_lastDrawn.large[i] = s.large[i];
        }
    }

    // 2) D/R changed => redraw their rect
    if (s.dispense != g_lastDrawn.dispense) {
        redrawRegion(g_dRect, s);
        g_lastDrawn.dispense = s.dispense;
    }
    if (s.ready != g_lastDrawn.ready) {
        redrawRegion(g_rRect, s);
        g_lastDrawn.ready = s.ready;
    }

    // 3) text content changed => redraw lines
    tft.setFont(FONT_DOTO);
    tft.setTextColor(COL_GREEN);
    tft.setTextDatum(textdatum_t::top_left);

    char line1[24];
    char line2[24];

    if (s.pageMode == PageMode::MAIN) {
        snprintf(line1, sizeof(line1), "%s%s", s.flareLetter, s.flareCount);
        snprintf(line2, sizeof(line2), "%s%s", s.chaffLetter, s.chaffCount);
    } else {
        strncpy(line1, s.bitLine1, sizeof(line1) - 1);
        line1[sizeof(line1) - 1] = '\0';
        strncpy(line2, s.bitLine2, sizeof(line2) - 1);
        line2[sizeof(line2) - 1] = '\0';
    }

    char prev1[24];
    char prev2[24];
    if (g_lastDrawn.pageMode == PageMode::MAIN) {
        snprintf(prev1, sizeof(prev1), "%s%s", g_lastDrawn.flareLetter, g_lastDrawn.flareCount);
        snprintf(prev2, sizeof(prev2), "%s%s", g_lastDrawn.chaffLetter, g_lastDrawn.chaffCount);
    } else {
        strncpy(prev1, g_lastDrawn.bitLine1, sizeof(prev1) - 1);
        prev1[sizeof(prev1) - 1] = '\0';
        strncpy(prev2, g_lastDrawn.bitLine2, sizeof(prev2) - 1);
        prev2[sizeof(prev2) - 1] = '\0';
    }

    if (strcmp(line1, prev1) != 0) {
        redrawTextLine(TEXT_X, TEXT_LINE1, line1);
    }
    if (strcmp(line2, prev2) != 0) {
        redrawTextLine(TEXT_X, TEXT_LINE2, line2);
    }

    // Update last-drawn text fields
    g_lastDrawn.pageMode = s.pageMode;
    strncpy(g_lastDrawn.flareLetter, s.flareLetter, sizeof(g_lastDrawn.flareLetter));
    strncpy(g_lastDrawn.chaffLetter, s.chaffLetter, sizeof(g_lastDrawn.chaffLetter));
    strncpy(g_lastDrawn.flareCount,  s.flareCount,  sizeof(g_lastDrawn.flareCount));
    strncpy(g_lastDrawn.chaffCount,  s.chaffCount,  sizeof(g_lastDrawn.chaffCount));
    strncpy(g_lastDrawn.bitLine1,    s.bitLine1,    sizeof(g_lastDrawn.bitLine1));
    strncpy(g_lastDrawn.bitLine2,    s.bitLine2,    sizeof(g_lastDrawn.bitLine2));

    g_dirty = false;

    tft.endWrite();
}

// =============================================================================
// GEOMETRY PRECOMPUTE
// =============================================================================
static void precomputeGeometry() {
    g_largeArrowRadius = static_cast<int16_t>(computeLargeArrowRadiusSymmetric());

    // Large arrows
    for (int i = 0; i < LARGE_ARROW_COUNT; ++i) {
        computeArrowVertices(g_largeArrows[i], LARGE_ARROW_ANGLES[i], g_largeArrowRadius, 1.0f);

        TFT_Point pts[7] = {
            g_largeArrows[i].tip, g_largeArrows[i].tipBaseL, g_largeArrows[i].tipBaseR,
            g_largeArrows[i].bodyTopL, g_largeArrows[i].bodyTopR, g_largeArrows[i].bodyBotL, g_largeArrows[i].bodyBotR
        };
        g_largeAabb[i] = rectFromPoints(pts, 7, 2);
    }

    // Small arrows
    for (int i = 0; i < SMALL_ARROW_COUNT; ++i) {
        computeArrowVertices(g_smallArrows[i], SMALL_ARROW_ANGLES[i], SMALL_ARROW_RADIUS, SMALL_ARROW_SCALE);

        TFT_Point pts[7] = {
            g_smallArrows[i].tip, g_smallArrows[i].tipBaseL, g_smallArrows[i].tipBaseR,
            g_smallArrows[i].bodyTopL, g_smallArrows[i].bodyTopR, g_smallArrows[i].bodyBotL, g_smallArrows[i].bodyBotR
        };
        g_smallAabb[i] = rectFromPoints(pts, 7, 2);
    }

    // Ticks
    for (int i = 0; i < TICK_COUNT; ++i) {
        const int angle = (i * 360) / TICK_COUNT;
        computeTickVertices(g_ticks[i], angle);
    }

    // D/R rects (conservative 30x30 like your previous)
    g_dRect = RectI16{ static_cast<int16_t>(RING_CX + DR_X_OFFSET - 15), static_cast<int16_t>(RING_CY - DR_OFFSET - 15), 30, 30 };
    g_rRect = RectI16{ static_cast<int16_t>(RING_CX + DR_X_OFFSET - 15), static_cast<int16_t>(RING_CY + DR_OFFSET - 15), 30, 30 };
}

// =============================================================================
// DCS-BIOS CALLBACKS (ALL WRITE UNDER MUX)
// =============================================================================
static void markDirty() { g_dirty = true; }

// Recompute device ON state and update all dependent states
static void updateDeviceState() {
    const bool wasOn = g_pending.deviceOn;
    const bool nowOn = computeDeviceOn();

    portENTER_CRITICAL(&g_stateMux);

    g_pending.deviceOn = nowOn;

    // Update all 4 large arrow states
    for (int i = 0; i < LARGE_ARROW_COUNT; ++i) {
        g_pending.large[i] = computeStateFromBits(g_largeBrt[i], g_largeDim[i]);
    }

    // Update D/R states
    g_pending.dispense = computeStateFromBits(g_dispBrt, g_dispDim);
    g_pending.ready = computeStateFromBits(g_readyBrt, g_readyDim);

    portEXIT_CRITICAL(&g_stateMux);

    // Force full redraw on ON/OFF transitions
    if (wasOn != nowOn) {
        g_forceFull = true;
    }

    markDirty();
}

// Large arrow callbacks
static void onFwdRightBrt(const char*, uint16_t v) { g_largeBrt[0] = (v != 0); updateDeviceState(); }
static void onFwdRightDim(const char*, uint16_t v) { g_largeDim[0] = (v != 0); updateDeviceState(); }

static void onAftRightBrt(const char*, uint16_t v) { g_largeBrt[1] = (v != 0); updateDeviceState(); }
static void onAftRightDim(const char*, uint16_t v) { g_largeDim[1] = (v != 0); updateDeviceState(); }

static void onAftLeftBrt(const char*, uint16_t v)  { g_largeBrt[2] = (v != 0); updateDeviceState(); }
static void onAftLeftDim(const char*, uint16_t v)  { g_largeDim[2] = (v != 0); updateDeviceState(); }

static void onFwdLeftBrt(const char*, uint16_t v)  { g_largeBrt[3] = (v != 0); updateDeviceState(); }
static void onFwdLeftDim(const char*, uint16_t v)  { g_largeDim[3] = (v != 0); updateDeviceState(); }

// D/R callbacks
static void onDispenseBrt(const char*, uint16_t v) {
    g_dispBrt = (v != 0);
    updateDeviceState();
}
static void onDispenseDim(const char*, uint16_t v) {
    g_dispDim = (v != 0);
    updateDeviceState();
}
static void onReadyBrt(const char*, uint16_t v) {
    g_readyBrt = (v != 0);
    updateDeviceState();
}
static void onReadyDim(const char*, uint16_t v) {
    g_readyDim = (v != 0);
    updateDeviceState();
}

// Inventory / BIT strings - simple validation, no page mode gating
// If DCS-BIOS sends valid data, we store it. Period.
static void onFlareCount(const char*, const char* value) {
    if (!value) return;
    if (strlen(value) < 2) return;  // Need 2+ chars for count

    portENTER_CRITICAL(&g_stateMux);
    if (strncmp(g_pending.flareCount, value, 3) != 0) {
        strncpy(g_pending.flareCount, value, 3);
        g_pending.flareCount[3] = '\0';
        portEXIT_CRITICAL(&g_stateMux);
        markDirty();
        return;
    }
    portEXIT_CRITICAL(&g_stateMux);
}

static void onChaffCount(const char*, const char* value) {
    if (!value) return;
    if (strlen(value) < 2) return;  // Need 2+ chars for count

    portENTER_CRITICAL(&g_stateMux);
    if (strncmp(g_pending.chaffCount, value, 3) != 0) {
        strncpy(g_pending.chaffCount, value, 3);
        g_pending.chaffCount[3] = '\0';
        portEXIT_CRITICAL(&g_stateMux);
        markDirty();
        return;
    }
    portEXIT_CRITICAL(&g_stateMux);
}

static void onFlareLetter(const char*, const char* value) {
    if (!value || value[0] == '\0') return;  // Need 1+ char for letter

    portENTER_CRITICAL(&g_stateMux);
    if (g_pending.flareLetter[0] != value[0]) {
        g_pending.flareLetter[0] = value[0];
        g_pending.flareLetter[1] = '\0';
        portEXIT_CRITICAL(&g_stateMux);
        markDirty();
        return;
    }
    portEXIT_CRITICAL(&g_stateMux);
}

static void onChaffLetter(const char*, const char* value) {
    if (!value || value[0] == '\0') return;  // Need 1+ char for letter

    portENTER_CRITICAL(&g_stateMux);
    if (g_pending.chaffLetter[0] != value[0]) {
        g_pending.chaffLetter[0] = value[0];
        g_pending.chaffLetter[1] = '\0';
        portEXIT_CRITICAL(&g_stateMux);
        markDirty();
        return;
    }
    portEXIT_CRITICAL(&g_stateMux);
}

static void onBitLine1(const char*, const char* value) {
    if (!value) return;
    if (strlen(value) < 3) return;  // Need 3+ chars to be considered non-empty

    char norm[5];
    formatField4(norm, value);

    portENTER_CRITICAL(&g_stateMux);
    if (memcmp(g_pending.bitLine1, norm, 4) != 0) {
        memcpy(g_pending.bitLine1, norm, 5);
        portEXIT_CRITICAL(&g_stateMux);
        markDirty();
        return;
    }
    portEXIT_CRITICAL(&g_stateMux);
}

static void onBitLine2(const char*, const char* value) {
    if (!value) return;
    if (strlen(value) < 3) return;  // Need 3+ chars to be considered non-empty

    char norm[5];
    formatField4(norm, value);

    portENTER_CRITICAL(&g_stateMux);
    if (memcmp(g_pending.bitLine2, norm, 4) != 0) {
        memcpy(g_pending.bitLine2, norm, 5);
        portEXIT_CRITICAL(&g_stateMux);
        markDirty();
        return;
    }
    portEXIT_CRITICAL(&g_stateMux);
}

static void onPage(const char*, const char* value) {
    // Only accept exact full strings: "MAIN" or "TEST"
    // Ignore partials like "MA", "TE", etc.
    if (!value) return;

    PageMode newMode = g_pending.pageMode;  // Keep current if no match

    if (strcmp(value, "MAIN") == 0) {
        newMode = PageMode::MAIN;
    } else if (strcmp(value, "TEST") == 0) {
        newMode = PageMode::TEST;
    } else {
        // Ignore partial strings or unknown values
        return;
    }

    portENTER_CRITICAL(&g_stateMux);
    if (g_pending.pageMode != newMode) {
        g_pending.pageMode = newMode;
        portEXIT_CRITICAL(&g_stateMux);

        g_forceFull = true;
        g_dirty = true;
        debugPrintf("[CMWS] Page changed to: %s\n", value);
        return;
    }
    portEXIT_CRITICAL(&g_stateMux);
}

static void onLampChange(const char*, uint16_t value, uint16_t maxValue) {
    if (maxValue == 0) return;
    const uint8_t newLevel = static_cast<uint8_t>((static_cast<uint32_t>(value) * 255UL) / maxValue);

    portENTER_CRITICAL(&g_stateMux);
    if (g_pending.backlight != newLevel) {
        g_pending.backlight = newLevel;
        portEXIT_CRITICAL(&g_stateMux);
        markDirty();
        return;
    }
    portEXIT_CRITICAL(&g_stateMux);
}


// =============================================================================
// TASK
// =============================================================================
#if RUN_AS_TASK
static void CMWSDisplay_task(void*) {
    for (;;) {
        CMWSDisplay_draw(false);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
#endif

// =============================================================================
// API
// =============================================================================
void CMWSDisplay_init() {
    // Platform-specific initialization (e.g., enable LCD power on T-Display S3)
    platformInit();

    // Precompute geometry before first draw
    precomputeGeometry();

    // Init display
    tft.init();
    tft.setRotation(3);
    tft.setColorDepth(16);
    tft.setSwapBytes(true);
    tft.setBrightness(255);
    tft.fillScreen(COL_BLACK);

    // Initialize pending state deterministically
    CmwsState init{};
    init.pageMode = PageMode::MAIN;
    init.deviceOn = false;  // Start OFF until DCS-BIOS tells us otherwise

    for (int i = 0; i < LARGE_ARROW_COUNT; ++i) init.large[i] = ElemState::OFF;
    for (int i = 0; i < SMALL_ARROW_COUNT; ++i) init.small[i] = ElemState::DIM;
    init.dispense = ElemState::OFF;
    init.ready    = ElemState::OFF;

    init.backlight = 255;

    strncpy(init.flareLetter, "F", sizeof(init.flareLetter));
    strncpy(init.chaffLetter, "C", sizeof(init.chaffLetter));
    strncpy(init.flareCount,  " 00", sizeof(init.flareCount));
    strncpy(init.chaffCount,  " 00", sizeof(init.chaffCount));
    init.bitLine1[0] = '\0';
    init.bitLine2[0] = '\0';

    portENTER_CRITICAL(&g_stateMux);
    g_pending = init;
    portEXIT_CRITICAL(&g_stateMux);

    g_lastDrawn = init;
    g_forceFull = true;
    g_dirty = true;

    // Subscriptions
    subscribeToMetadataChange("PLT_CMWS_FWD_RIGHT_BRT_L", onFwdRightBrt);
    subscribeToMetadataChange("PLT_CMWS_FWD_RIGHT_DIM_L", onFwdRightDim);
    subscribeToMetadataChange("PLT_CMWS_AFT_RIGHT_BRT_L", onAftRightBrt);
    subscribeToMetadataChange("PLT_CMWS_AFT_RIGHT_DIM_L", onAftRightDim);
    subscribeToMetadataChange("PLT_CMWS_AFT_LEFT_BRT_L", onAftLeftBrt);
    subscribeToMetadataChange("PLT_CMWS_AFT_LEFT_DIM_L", onAftLeftDim);
    subscribeToMetadataChange("PLT_CMWS_FWD_LEFT_BRT_L", onFwdLeftBrt);
    subscribeToMetadataChange("PLT_CMWS_FWD_LEFT_DIM_L", onFwdLeftDim);

    subscribeToMetadataChange("PLT_CMWS_D_BRT_L", onDispenseBrt);
    subscribeToMetadataChange("PLT_CMWS_D_DIM_L", onDispenseDim);
    subscribeToMetadataChange("PLT_CMWS_R_BRT_L", onReadyBrt);
    subscribeToMetadataChange("PLT_CMWS_R_DIM_L", onReadyDim);

    subscribeToDisplayChange("PLT_CMWS_FLARE_COUNT", onFlareCount);
    subscribeToDisplayChange("PLT_CMWS_CHAFF_COUNT", onChaffCount);
    subscribeToDisplayChange("PLT_CMWS_FLARE_LETTER", onFlareLetter);
    subscribeToDisplayChange("PLT_CMWS_CHAFF_LETTER", onChaffLetter);
    subscribeToDisplayChange("PLT_CMWS_BIT_LINE_1", onBitLine1);
    subscribeToDisplayChange("PLT_CMWS_BIT_LINE_2", onBitLine2);
    subscribeToDisplayChange("PLT_CMWS_PAGE", onPage);

    subscribeToLedChange("PLT_CMWS_LAMP", onLampChange);

    // Initial draw
    CMWSDisplay_draw(true);

#if RUN_BIT_TEST_ON_INIT
    CMWSDisplay_bitTest();
#endif

#if RUN_AS_TASK
    xTaskCreatePinnedToCore(CMWSDisplay_task, "CMWSTask", TASK_STACK_SIZE, nullptr, TASK_PRIORITY, &taskHandle, CPU_CORE);
#endif

    // Log initialization details based on interface type
#if defined(CMWS_USE_SPI_INTERFACE)
    debugPrintf("✅ CMWS Display initialized (SPI): MOSI=%d SCLK=%d CS=%d DC=%d RST=%d BLK=%d\n",
        PIN_MOSI, PIN_SCLK, PIN_CS, PIN_DC, PIN_RST, PIN_BLK);
#elif defined(CMWS_USE_PARALLEL_INTERFACE)
    debugPrintf("✅ CMWS Display initialized (8-bit Parallel): D0-D7=%d,%d,%d,%d,%d,%d,%d,%d WR=%d RD=%d DC=%d CS=%d RST=%d BLK=%d PWR=%d\n",
        PIN_D0, PIN_D1, PIN_D2, PIN_D3, PIN_D4, PIN_D5, PIN_D6, PIN_D7,
        PIN_WR, PIN_RD, PIN_DC, PIN_CS, PIN_RST, PIN_BLK, PIN_POWER);
#endif
    debugPrintf("   Large arrow radius=%d, small arrows=%d (always DIM when ON), ticks=%d\n",
        g_largeArrowRadius, SMALL_ARROW_COUNT, TICK_COUNT);
}

void CMWSDisplay_loop() {
#if !RUN_AS_TASK
    CMWSDisplay_draw(false);
#endif
}

void CMWSDisplay_notifyMissionStart() {
    g_forceFull = true;
    g_dirty = true;
}

void CMWSDisplay_deinit() {
#if RUN_AS_TASK
    if (taskHandle) {
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }
#endif
    tft.fillScreen(COL_BLACK);
    tft.setBrightness(0);

#if defined(CMWS_USE_PARALLEL_INTERFACE)
    // Turn off LCD power on T-Display S3
    digitalWrite(PIN_POWER, LOW);
#endif
}

// =============================================================================
// BIT TEST (blocking) — preserves and restores full state
// =============================================================================
void CMWSDisplay_bitTest() {
    CmwsState saved;
    snapshotState(saved);

    // Force on + bright for test
    CmwsState tmp = saved;
    tmp.pageMode = PageMode::MAIN;
    tmp.deviceOn = true;
    tmp.backlight = 255;
    strncpy(tmp.flareCount, " 88", sizeof(tmp.flareCount));
    strncpy(tmp.chaffCount, " 88", sizeof(tmp.chaffCount));

    // Phase 1: all bright
    for (int i = 0; i < LARGE_ARROW_COUNT; ++i) tmp.large[i] = ElemState::BRT;
    for (int i = 0; i < SMALL_ARROW_COUNT; ++i) tmp.small[i] = ElemState::DIM;  // Small always DIM
    tmp.dispense = ElemState::BRT;
    tmp.ready    = ElemState::BRT;

    portENTER_CRITICAL(&g_stateMux);
    g_pending = tmp;
    portEXIT_CRITICAL(&g_stateMux);
    g_forceFull = true; g_dirty = true;
    CMWSDisplay_draw(true);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Phase 2: all dim
    for (int i = 0; i < LARGE_ARROW_COUNT; ++i) tmp.large[i] = ElemState::DIM;
    tmp.dispense = ElemState::DIM;
    tmp.ready    = ElemState::DIM;

    portENTER_CRITICAL(&g_stateMux);
    g_pending = tmp;
    portEXIT_CRITICAL(&g_stateMux);
    g_forceFull = true; g_dirty = true;
    CMWSDisplay_draw(true);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Phase 3: device OFF (all arrows OFF, lamp 0%)
    tmp.deviceOn = false;
    for (int i = 0; i < LARGE_ARROW_COUNT; ++i) tmp.large[i] = ElemState::OFF;
    tmp.dispense = ElemState::OFF;
    tmp.ready    = ElemState::OFF;

    portENTER_CRITICAL(&g_stateMux);
    g_pending = tmp;
    portEXIT_CRITICAL(&g_stateMux);
    g_forceFull = true; g_dirty = true;
    CMWSDisplay_draw(true);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Phase 4: device back ON, rotate large arrows bright
    tmp.deviceOn = true;
    for (int a = 0; a < LARGE_ARROW_COUNT; ++a) {
        for (int i = 0; i < LARGE_ARROW_COUNT; ++i) tmp.large[i] = ElemState::DIM;
        tmp.large[a] = ElemState::BRT;
        tmp.dispense = ElemState::DIM;
        tmp.ready    = ElemState::DIM;

        portENTER_CRITICAL(&g_stateMux);
        g_pending = tmp;
        portEXIT_CRITICAL(&g_stateMux);
        g_forceFull = true; g_dirty = true;
        CMWSDisplay_draw(true);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    // Restore full state exactly
    portENTER_CRITICAL(&g_stateMux);
    g_pending = saved;
    portEXIT_CRITICAL(&g_stateMux);

    g_forceFull = true;
    g_dirty = true;
    CMWSDisplay_draw(true);
}

#else

void CMWSDisplay_init() {}
void CMWSDisplay_loop() {}
void CMWSDisplay_deinit() {}
void CMWSDisplay_notifyMissionStart() {}
void CMWSDisplay_bitTest() {}

#endif
