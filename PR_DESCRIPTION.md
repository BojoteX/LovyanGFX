## Summary

This PR fixes a compilation error affecting ESP32-S3 users who use Arduino-ESP32 v3.x (based on ESP-IDF v5.x). The RGB parallel bus driver (`Bus_RGB.cpp`) fails to compile due to a removed GPIO HAL function.

**Fixes #734**

## The Problem

When compiling for ESP32-S3 with Arduino-ESP32 v3.x, users encounter:

```
error: 'gpio_hal_iomux_func_sel' was not declared in this scope
   98 |     gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
      |     ^~~~~~~~~~~~~~~~~~~~~~~
```

This affects all ESP32-S3 boards using RGB parallel displays (e.g., Sunton ESP32-8048S043/S050/S070, Elecrow displays, Makerfabs panels).

## Root Cause Analysis

In ESP-IDF v5.0, Espressif refactored the GPIO Hardware Abstraction Layer (HAL):

| ESP-IDF Version | Function | Signature | Status |
|-----------------|----------|-----------|--------|
| v4.x | `gpio_hal_iomux_func_sel` | `(mux_reg, func)` | ✅ Available |
| v5.x | `gpio_hal_iomux_func_sel` | — | ❌ **Removed** |
| v5.x | `gpio_ll_func_sel` | `(&GPIO, pin, func)` | ✅ Replacement |

The HAL layer function was removed, but the low-level (LL) function `gpio_ll_func_sel` provides equivalent functionality with a different calling convention.

## The Solution

Add compile-time version detection to use the appropriate API:

```cpp
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    gpio_ll_func_sel(&GPIO, pin, PIN_FUNC_GPIO);
#else
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
#endif
```

### What Both Functions Do

Both functions configure the IO MUX to set a pin to GPIO function mode. This is required before connecting the pin to LCD peripheral signals via `esp_rom_gpio_connect_out_signal()`.

| Aspect | Old API (v4.x) | New API (v5.x) |
|--------|----------------|----------------|
| Function | `gpio_hal_iomux_func_sel` | `gpio_ll_func_sel` |
| Pin reference | MUX register address | GPIO device + pin number |
| Layer | HAL (Hardware Abstraction) | LL (Low-Level) |
| Effect | Sets IO_MUX to GPIO function | Sets IO_MUX to GPIO function |

## Changes

**File:** `src/lgfx/v1/platforms/esp32s3/Bus_RGB.cpp`

1. Added `#include <esp_idf_version.h>` for version detection (line 47)
2. Added preprocessor conditional for API selection (lines 100-104)

```diff
+#include <esp_idf_version.h>
+
 namespace lgfx
 {
  inline namespace v1
@@ -95,7 +97,11 @@ namespace lgfx

   static void _gpio_pin_sig(uint32_t pin, uint32_t sig)
   {
+#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
+    gpio_ll_func_sel(&GPIO, pin, PIN_FUNC_GPIO);
+#else
     gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
+#endif
     gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
     esp_rom_gpio_connect_out_signal(pin, sig, false, false);
   }
```

## Risk Assessment

| Risk Factor | Assessment |
|-------------|------------|
| Breaking existing builds | ✅ **None** - ESP-IDF v4.x path unchanged |
| Runtime behavior change | ✅ **None** - Same hardware configuration |
| New dependencies | ✅ **None** - Uses existing includes |
| Affected platforms | ESP32-S3 only (Bus_RGB is S3-exclusive) |

## Testing

- [x] Compilation verified on ESP32-S3 with Arduino-ESP32 v3.x (ESP-IDF v5.x)
- [x] Original ESP-IDF v4.x codepath preserved via preprocessor
- [x] Follows existing patterns in codebase (see `Bus_Parallel8.cpp`, `common.cpp`)

## Compatibility Matrix

| Arduino-ESP32 | ESP-IDF | Result |
|---------------|---------|--------|
| v2.x | v4.4.x | ✅ Uses `gpio_hal_iomux_func_sel` |
| v3.x | v5.x | ✅ Uses `gpio_ll_func_sel` |

## Related

- Issue: #734
- ESP-IDF GPIO HAL changes: [espressif/esp-idf@5.0 release notes](https://github.com/espressif/esp-idf/releases/tag/v5.0)
