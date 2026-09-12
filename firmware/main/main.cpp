#include <atomic>
#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "nvs_flash.h"

#include "app_controller.hpp"
#include "attitude_estimator.hpp"
#include "battery_monitor.hpp"
#include "calibration_mode.hpp"
#include "debug_config.hpp"
#include "sleep_mode.hpp"
#include "gui_manager.hpp"
#include "i2c_scan.hpp"
#include "lgfx_config.hpp"
#include "nvs_calibration.hpp"
#include "qmi8658.hpp"

namespace {

constexpr int kLvglTickPeriodMs = 5;
// Two draw buffers enable LVGL's double-buffered partial mode: while
// chunk N is being DMA'd out over SPI, LVGL renders chunk N+1 into the
// other buffer instead of waiting.
constexpr int kDrawBufRows = 60;
// Sampling faster than what's filtered/displayed leaves headroom for the
// low-pass filters above the display's ~60fps cap. QMI8658's own ODR is
// 1000Hz and the I2C read itself is well under a millisecond, so 120Hz
// polling fits comfortably in the 5ms main-loop cadence.
constexpr int64_t kSensorUpdatePeriodUs = 1000000 / 120;  // ~120Hz

// A voltage reading has no reason to be as fresh as attitude — voltage
// drifts on a minutes scale, and BatteryMonitor::ReadVoltage() already
// blocks for kNumSamples (32) raw ADC reads per call.
constexpr int64_t kBatteryReadPeriodUs = 10 * 1000000;  // 0.1Hz

// Stage 2 of the low-battery safety net — see app_controller.hpp design
// note 11 for stage 1 (the warning screen, down to kLowBatteryEnterV).
// Below this second, lower threshold, refuse to keep running at all:
// forced deep sleep instead of the normal light-sleep+WoM loop — see the
// idle-sleep block below. Enter threshold is
// AppController::kCriticalBatteryEnterV (3.0V), read from there rather
// than duplicated here so the two constants can't drift apart.
//
// Exit threshold is set far above entry (3.8V, not e.g. 3.2V) because a
// LiPo's terminal voltage jumps up a real step the instant charging
// current starts flowing, before any meaningful capacity has gone back
// in (internal-resistance IR drop, not state of charge — and a degraded
// cell's higher internal resistance makes this jump bigger, not
// smaller). A narrow gap risks reading that transient alone as
// "recovered" and resuming full active operation while the battery's
// real stored energy has barely moved.
constexpr float kCriticalBatteryExitV = 3.8f;
// Deep sleep current is tiny (~8uA) next to a brief active-boot check,
// so this can afford to be fairly frequent.
constexpr uint64_t kCriticalBatteryCheckPeriodUs = 30ULL * 1000 * 1000;

// Hides the debug overlay (angle/taps/is_moving label) without deleting
// the code.
constexpr bool kDebugOverlayEnabled = kDebugEnabled;

// One live "ATT," CSV line per sensor tick with AttitudeEstimator's
// gyro-only angle, accel-only angle, fused angle, and gz.
constexpr bool kAttitudeDebugLogEnabled = false;

// Hold BOOT (GPIO0) this long, while the app is already running, to enter
// calibration mode. NOT checked at power-on/reset — see calibration_mode.hpp.
constexpr int64_t kCalibrationHoldUs = 3 * 1000 * 1000;

// Tap Engine config — see qmi8658.hpp's ConfigureTap() for what each
// parameter means. The idle-sleep path (Qmi8658::SetLowPowerAccelOnly())
// deliberately leaves the accel ODR these were tuned against unchanged,
// so these same values keep applying whether or not gyro is enabled.
constexpr uint8_t kTapPriority = 0;
constexpr uint8_t kTapPeakWindow = 40;
constexpr uint16_t kTapTapWindow = 100;
constexpr uint16_t kTapDTapWindow = 500;
constexpr float kTapAlpha = 0.0625f;
constexpr float kTapGamma = 0.25f;
constexpr float kTapPeakMagThr = 0.8f;
constexpr float kTapUdmThr = 0.4f;

// Wake-on-Motion config for idle sleep — see qmi8658.hpp's
// EnterWakeOnMotion(). kWomThresholdMg is maxed out at the register's
// 255 ceiling — even there, WoM wakes on any sufficiently large
// accelerometer slope, not specifically a deliberate tap (see that
// method's trade-off note), which is why the two-stage confirm below
// exists as a software-side filter on top. kWomBlankingSamples is also
// maxed (6-bit field, max 63, ~63ms at 1000Hz accel ODR) to absorb the
// transient STATUS1 latch that toggling CTRL7 on mode entry can cause.
constexpr uint8_t kWomThresholdMg = 255;
constexpr uint8_t kWomBlankingSamples = 63;

// Two-stage WoM wake confirmation — see the idle-sleep block's own
// comment. A deliberate double-tap-to-wake gesture, not just a debounce:
// a single physical tap only ever registers on whichever detector is
// active at that instant (WoM, since the tap engine isn't running during
// light sleep), so this window waits for a genuinely separate second
// tap. 1500ms gives a natural two-tap rhythm enough room, confirmed
// reliable both bare-board and inside the finished enclosure.
constexpr int kWomConfirmWindowMs = 1500;
constexpr int kWomConfirmPollMs = 20;

// Measures the real elapsed time from RunIdleSleep() returning to the
// confirm-window poll loop starting — the cost of
// ExitWakeOnMotion()+ConfigureTap()+the spurious-latch discard.
constexpr bool kWomConfirmLatencyLogEnabled = kDebugEnabled;

static LGFX lcd;
static uint8_t lvgl_draw_buf1[240 * kDrawBufRows * 2];  // RGB565, 2 bytes/px
static uint8_t lvgl_draw_buf2[240 * kDrawBufRows * 2];

int64_t g_lvgl_flush_accum_us = 0;
uint32_t g_lvgl_flush_call_count = 0;

void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map)
{
    const int32_t w = area->x2 - area->x1 + 1;
    const int32_t h = area->y2 - area->y1 + 1;
    const int64_t flush_start_us = esp_timer_get_time();
    // Display is configured LV_COLOR_FORMAT_RGB565_SWAPPED to match the
    // byte order pushImage() expects.
    //
    // pushImageDMA() + waitDMA(): with two draw buffers, LVGL alternates
    // between them each flush, so while this chunk's bytes are still
    // going out over SPI, LVGL can render the next chunk into the other
    // buffer instead of blocking. waitDMA() at the top, not the bottom,
    // blocks only on the *previous* transfer if it hasn't finished yet —
    // this call's own transfer is left running and collected by the next
    // call's waitDMA() instead.
    lcd.waitDMA();
    lcd.pushImageDMA(area->x1, area->y1, w, h, reinterpret_cast<uint16_t*>(px_map));
    g_lvgl_flush_accum_us += esp_timer_get_time() - flush_start_us;
    ++g_lvgl_flush_call_count;
    lv_display_flush_ready(disp);
}

void lvgl_tick_cb(void*)
{
    lv_tick_inc(kLvglTickPeriodMs);
}

// LVGL's default builtin sprintf doesn't implement %f (it silently drops
// the conversion and leaves the literal 'f' character), so format floats
// as fixed-point integers ourselves rather than depend on a float-capable
// sprintf/libc combination.
int RoundToFixed(float value, int scale)
{
    return static_cast<int>(value * scale + (value >= 0 ? 0.5f : -0.5f));
}

// Splits a fixed-point value into sign + whole + fractional digits for
// display. Needed because plain `fixed / scale` truncates toward zero:
// for fixed=-85, scale=100, that gives whole=0 with no sign information
// left in it, so "%+d" on the whole part alone prints "+0" — silently
// hiding the sign on every reading with magnitude under 1.0. Splitting
// off the sign before dividing the (now always non-negative) magnitude
// avoids that.
struct FixedParts { char sign; int whole; int frac; };

FixedParts SplitFixed(int fixed, int scale)
{
    const int mag = fixed < 0 ? -fixed : fixed;
    return FixedParts{fixed < 0 ? '-' : '+', mag / scale, mag % scale};
}

// AttitudeEstimator deliberately doesn't include qmi8658.hpp (no
// hardware/I2C knowledge, so it stays host-testable), so its Sample type
// is distinct from Qmi8658::Sample even though the fields line up —
// convert explicitly at the call site instead.
AttitudeEstimator::Sample ToAttitudeSample(const Qmi8658::Sample& s)
{
    AttitudeEstimator::Sample out;
    out.accel_g[0] = s.accel_g[0];
    out.accel_g[1] = s.accel_g[1];
    out.accel_g[2] = s.accel_g[2];
    out.gyro_dps[0] = s.gyro_dps[0];
    out.gyro_dps[1] = s.gyro_dps[1];
    out.gyro_dps[2] = s.gyro_dps[2];
    return out;
}

// Diagnostic, off by default: switches the IMU into WoM mode for
// kWomEdgeTestDurationMs, watches both IMU_INT1/IMU_INT2 with plain
// edge-triggered ISRs, and dumps every transition's pin/level/timestamp —
// useful if WoM ever needs re-diagnosing (see RunIdleSleep() for the
// actual sleep/wake path in normal operation).
constexpr bool kRunWomEdgeTestOnBoot = false;
constexpr gpio_num_t kWomEdgeTestInt1Gpio = GPIO_NUM_47;
constexpr gpio_num_t kWomEdgeTestInt2Gpio = GPIO_NUM_48;
constexpr uint32_t kWomEdgeTestDurationMs = 15000;
constexpr size_t kWomEdgeTestMaxEvents = 256;

struct WomEdgeEvent {
    int64_t t_us;
    gpio_num_t gpio;
    int level;
};
WomEdgeEvent g_wom_edge_events[kWomEdgeTestMaxEvents];
std::atomic<size_t> g_wom_edge_write_idx{0};

void IRAM_ATTR WomEdgeIsr(void* arg)
{
    const gpio_num_t gpio = static_cast<gpio_num_t>(reinterpret_cast<intptr_t>(arg));
    const size_t idx = g_wom_edge_write_idx.fetch_add(1, std::memory_order_relaxed);
    if (idx < kWomEdgeTestMaxEvents) {
        g_wom_edge_events[idx].t_us = esp_timer_get_time();
        g_wom_edge_events[idx].gpio = gpio;
        g_wom_edge_events[idx].level = gpio_get_level(gpio);
    }
}

void RunWomEdgeTest(Qmi8658& imu)
{
    for (gpio_num_t gpio : {kWomEdgeTestInt1Gpio, kWomEdgeTestInt2Gpio}) {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << gpio;
        cfg.mode = GPIO_MODE_INPUT;
        cfg.intr_type = GPIO_INTR_ANYEDGE;
        gpio_config(&cfg);
    }
    gpio_install_isr_service(0);
    gpio_isr_handler_add(kWomEdgeTestInt1Gpio, WomEdgeIsr, reinterpret_cast<void*>(kWomEdgeTestInt1Gpio));
    gpio_isr_handler_add(kWomEdgeTestInt2Gpio, WomEdgeIsr, reinterpret_cast<void*>(kWomEdgeTestInt2Gpio));

    imu.EnterWakeOnMotion(kWomThresholdMg, kWomBlankingSamples);
    printf("WOM_EDGE_TEST: started, %lus window, tap/move the device now — not entering sleep, watching both INT1(47) and INT2(48)\n",
           static_cast<unsigned long>(kWomEdgeTestDurationMs / 1000));

    // Deliberately just waits — no I2C traffic to this device for the
    // whole window, see the class comment for why.
    vTaskDelay(pdMS_TO_TICKS(kWomEdgeTestDurationMs));

    gpio_isr_handler_remove(kWomEdgeTestInt1Gpio);
    gpio_isr_handler_remove(kWomEdgeTestInt2Gpio);
    imu.ExitWakeOnMotion();
    imu.ConfigureTap(kTapPriority, kTapPeakWindow, kTapTapWindow, kTapDTapWindow, kTapAlpha, kTapGamma,
                      kTapPeakMagThr, kTapUdmThr);
    (void)imu.PollTapEvent();  // discard the same CTRL7/CTRL8-toggle spurious latch ConfigureTap() always produces

    size_t count = g_wom_edge_write_idx.load(std::memory_order_relaxed);
    const bool overflowed = count > kWomEdgeTestMaxEvents;
    if (overflowed) count = kWomEdgeTestMaxEvents;
    printf("WOM_EDGE_TEST: done, %zu edge(s) captured%s\n", count, overflowed ? " (buffer overflowed, some dropped)" : "");
    for (size_t i = 0; i < count; ++i) {
        printf("WOM_EDGE,i=%zu,t_us=%lld,gpio=%d,level=%d\n", i, static_cast<long long>(g_wom_edge_events[i].t_us),
               static_cast<int>(g_wom_edge_events[i].gpio), g_wom_edge_events[i].level);
    }
}

}  // namespace

extern "C" void app_main(void)
{
    // GPIO0 (BOOT) as a normal input once past the ROM bootloader's
    // strapping check — see calibration_mode.hpp for why this is only
    // polled here, not checked at reset. Configured before the
    // critical-battery escape hatch below so it can share this call.
    gpio_config_t boot_btn_cfg = {};
    boot_btn_cfg.pin_bit_mask = 1ULL << GPIO_NUM_0;
    boot_btn_cfg.mode = GPIO_MODE_INPUT;
    boot_btn_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&boot_btn_cfg);

    // Stage 2's periodic recheck. Deep sleep is a full reboot, not a
    // resume — app_main() runs again from scratch, and RTC memory (this
    // flag's storage class) is the only thing that survives it, which is
    // how a fresh boot tells "woken to re-check critical battery" apart
    // from a normal boot. Checked before any of the heavier LCD/LVGL/IMU
    // init below, and kept minimal when it IS a recheck, so a cycle that
    // goes back to sleep costs as little active time as possible.
    static RTC_DATA_ATTR bool in_critical_battery_sleep = false;
    if (in_critical_battery_sleep && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        // Escape hatch: hold BOOT through a wake to force a real boot
        // regardless of voltage. Exists because a bug in this still-new
        // mechanism could otherwise strand the device in an
        // indefinite sleep-reboot-sleep loop with no way back in short of
        // reflashing (which itself needs a real boot to reach the
        // download-mode entry point) — same "don't want a firmware bug to
        // look like a bricked device" reasoning as design note 9's
        // double-tap wake, one level more serious here since deep sleep
        // exits this whole function rather than just a loop inside it.
        const bool boot_held = gpio_get_level(GPIO_NUM_0) == 0;
        if (!boot_held) {
            BatteryMonitor critical_battery_monitor;
            const float v = critical_battery_monitor.ReadVoltage();
            // Always-on, not gated behind kDebugEnabled (debug_config.hpp)
            // — this is a new, higher-stakes mechanism (a real reboot
            // cycle) worth being able to see regardless of the general
            // debug flag state, same reasoning qmi8658.hpp gives for
            // always printing real errors. Plain %.2f, not the LVGL
            // fixed-point workaround main.cpp's on-screen label needs
            // (LVGL's own lightweight sprintf doesn't support %f) — this
            // goes to the serial console's real printf, which already
            // uses %f elsewhere in this file (see the ATT log above).
            printf("CRITICAL_BATTERY_RECHECK,v=%.2fV\n", v);
            if (v < kCriticalBatteryExitV) {
                esp_sleep_enable_timer_wakeup(kCriticalBatteryCheckPeriodUs);
                esp_deep_sleep_start();  // never returns
            }
            printf("CRITICAL_BATTERY_RECOVERED — resuming normal boot\n");
            // else: voltage recovered — fall through to a real boot below.
        } else {
            printf("CRITICAL_BATTERY_ESCAPE_HATCH — BOOT held, forcing a real boot\n");
        }
        in_critical_battery_sleep = false;
    }

    printf("Kairos gravity timer — hello from C++\n");

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_err = nvs_flash_init();
    }
    if (nvs_err != ESP_OK) {
        printf("nvs_flash_init failed: %d\n", nvs_err);
    }

    CalibrationData calibration;
    if (!LoadCalibration(calibration)) {
        printf("No saved calibration — using uncalibrated defaults until "
               "RunCalibrationMode() is run once (hold BOOT ~3s)\n");
    }

    lcd.init();
    lcd.setBrightness(255);  // full bright at boot; AppController takes over from here
    printf("LCD initialized, backlight on (GPIO40 via PWM)\n");

    RunI2cScan();

    lv_init();

    lv_display_t* disp = lv_display_create(240, 240);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);
    // LovyanGFX's pushImage() expects SPI-panel byte order, which is
    // swapped relative to LVGL's plain RGB565. Without this, only pure
    // black/white survive untouched; every anti-aliased/blended pixel
    // (i.e. most of any glyph's edges) comes out with a scrambled hue.
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565_SWAPPED);
    lv_display_set_buffers(disp, lvgl_draw_buf1, lvgl_draw_buf2, sizeof(lvgl_draw_buf1),
                            LV_DISPLAY_RENDER_MODE_PARTIAL);

    const esp_timer_create_args_t tick_timer_args = {
        .callback = &lvgl_tick_cb,
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = false,
    };
    esp_timer_handle_t tick_timer;
    esp_timer_create(&tick_timer_args, &tick_timer);
    esp_timer_start_periodic(tick_timer, kLvglTickPeriodMs * 1000);

    // min_freq_mhz == max_freq_mhz deliberately disables DFS (dynamic CPU
    // frequency scaling) — even a brief automatic drop below 160MHz
    // measurably hurt the software screen-rotation redraw's throughput.
    esp_pm_config_t pm_config = {};
    pm_config.max_freq_mhz = 160;
    pm_config.min_freq_mhz = 160;
    pm_config.light_sleep_enable = true;
    esp_pm_configure(&pm_config);

    // Black background — the default LVGL theme is light, which clashes
    // once AppController starts dimming/warming the primary label's color.
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, 0);

    printf("LVGL running\n");
    if (kAttitudeDebugLogEnabled) {
        printf("ATT,t_ms,gz_dps,gyro_angle,accel_angle,fused_angle,is_moving,in_valid_plane\n");
    }

    // Deliberately not rotating with the primary label (see
    // gui_manager.hpp's screen counter-rotation note) — a plain fixed
    // child of lv_screen_active(). Created before GuiManager below so
    // root_'s children end up later in lv_screen_active()'s child list
    // and draw on top of this overlay, not under it.
    lv_obj_t* label = nullptr;
    if (kDebugOverlayEnabled) {
        label = lv_label_create(lv_screen_active());
        lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_label_set_text(label, "waiting for IMU...");
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);
    }

    // AppController owns attitude-driven face switching (via
    // AttitudeEstimator::Output, computed below) plus tap routing and
    // TimerFace lifecycle. See app_controller.hpp for the design.
    static GuiManager gui_manager(lcd);
    static AppController app_controller(gui_manager);

    static Qmi8658 imu(GPIO_NUM_6, GPIO_NUM_7);

    // Battery-check gesture — see battery_monitor.hpp.
    static BatteryMonitor battery_monitor;

    // Starting point, not a finished tune — adjust these while watching
    // the tap count below and re-flashing. Windows are ported from
    // SensorLib's deprecated tap example (peak_window=20, tap_window=50,
    // d_tap_window=250, "@500Hz ODR"); doubled here since our accel ODR is
    // 1000Hz, to keep roughly the same real-time windows. alpha/gamma and
    // the g^2 thresholds are that example's values, unchanged (ODR-independent).
    imu.ConfigureTap(kTapPriority, kTapPeakWindow, kTapTapWindow, kTapDTapWindow,
                      kTapAlpha, kTapGamma, kTapPeakMagThr, kTapUdmThr);
    // Toggling Ctrl7/Ctrl8 while configuring the tap engine latches a
    // spurious STATUS1 tap flag (observed as "Taps 1" right at boot, with
    // no physical tap). Discard it here so the count starts clean.
    (void)imu.PollTapEvent();

    if (kRunWomEdgeTestOnBoot) {
        RunWomEdgeTest(imu);  // blocks ~15s — see its own comment
    }

    static AttitudeEstimator attitude_estimator(calibration.face_a_offset_deg, calibration.accel_bias_g[0],
                                                 calibration.accel_bias_g[1]);
    {
        // Gyro bias comes from the saved calibration (see
        // RunCalibrationMode), not a fresh live read here: this device
        // has no real power-cycle in normal use (battery + deep sleep),
        // and any wake is tap-triggered, so "assume stationary right
        // now" doesn't hold reliably enough for a live measurement.
        AttitudeEstimator::Sample bias_sample{};
        bias_sample.gyro_dps[0] = calibration.gyro_bias_dps[0];
        bias_sample.gyro_dps[1] = calibration.gyro_bias_dps[1];
        bias_sample.gyro_dps[2] = calibration.gyro_bias_dps[2];
        attitude_estimator.CalibrateGyroZeroOffset(bias_sample);
    }
    {
        // Seed angle_deg_ from a real reading instead of leaving it at
        // 0.0f — without this, booting anywhere other than face A's
        // reference orientation (e.g. plugged in via USB-C while resting
        // on face D) starts a full-size error that only closes at the
        // complementary filter's normal per-tick rate, visible as
        // "takes a moment to reach the right angle" right after flashing.
        Qmi8658::Sample seed_sample;
        if (imu.Read(seed_sample)) {
            attitude_estimator.SeedInitialAngle(ToAttitudeSample(seed_sample));
        }
    }

    int64_t next_sensor_update_us = 0;
    int64_t last_sensor_update_us = esp_timer_get_time();
    int64_t next_battery_read_us = 0;
    float last_battery_voltage_v = 0.0f;  // shown in the debug overlay below — 0 until the first real read
    int64_t boot_button_press_start_us = 0;
    int tap_count = 0;
    // Antialiasing off while actively rotating (not worth its per-pixel
    // cost during a fast flip), on once settled — dirty-checked so it's
    // only set on an actual is_moving transition, not every tick.
    bool antialiasing_enabled = true;

    int64_t last_fps_calc_us = esp_timer_get_time();
    uint32_t fps_display = 0;
    uint32_t last_update_count = 0;

    // Per-second breakdown of where main loop time actually goes.
    // loop_count answers "how many times did while(true) run in the last
    // second"; the four *_us accumulators say which section it went into.
    //
    // This, kAttitudeDebugLogEnabled, and kRunWomEdgeTestOnBoot stay
    // independent of the shared kDebugEnabled group (debug_config.hpp) —
    // all of these are serial printf streams competing for the same
    // UART, so turning several on together interleaves their output and
    // defeats whichever one you meant to read. Flip only the one
    // relevant to what's being debugged.
    constexpr bool kLoopTimingLogEnabled = false;
    uint32_t loop_count = 0;
    int64_t tap_poll_accum_us = 0;
    int64_t sensor_block_accum_us = 0;
    int64_t lvgl_accum_us = 0;
    int64_t delay_accum_us = 0;

    while (true) {
        ++loop_count;
        const int64_t now_fps_us = esp_timer_get_time();
        if (now_fps_us - last_fps_calc_us >= 1000000) {
            // GuiManager's own real-update count, not a physical flush
            // count — root_ (200x60) needs ~3 flush calls per logical
            // redraw in LVGL's partial render mode (draw buffer only
            // fits ~24 rows at that width), so counting flushes directly
            // would overstate the real update rate by ~3x.
            const uint32_t update_count = gui_manager.GetUpdateCount();
            fps_display = update_count - last_update_count;
            last_update_count = update_count;
            last_fps_calc_us = now_fps_us;

            if (kLoopTimingLogEnabled) {
                printf("LOOP,hz=%lu,tap_i2c_us=%lld,sensor_us=%lld,lvgl_us=%lld,delay_us=%lld,"
                       "flush_calls=%lu,flush_us=%lld\n",
                       static_cast<unsigned long>(loop_count), tap_poll_accum_us, sensor_block_accum_us,
                       lvgl_accum_us, delay_accum_us, static_cast<unsigned long>(g_lvgl_flush_call_count),
                       g_lvgl_flush_accum_us);
                loop_count = 0;
                tap_poll_accum_us = 0;
                sensor_block_accum_us = 0;
                lvgl_accum_us = 0;
                delay_accum_us = 0;
                g_lvgl_flush_call_count = 0;
                g_lvgl_flush_accum_us = 0;
            }
        }

        if (gpio_get_level(GPIO_NUM_0) == 0) {
            const int64_t now_us_btn = esp_timer_get_time();
            if (boot_button_press_start_us == 0) {
                boot_button_press_start_us = now_us_btn;
            } else if (now_us_btn - boot_button_press_start_us >= kCalibrationHoldUs) {
                RunCalibrationMode(imu, gui_manager);  // never returns — ends in esp_restart()
            }
        } else {
            boot_button_press_start_us = 0;
        }

        const int64_t tap_poll_start_us = esp_timer_get_time();
        if (imu.PollTapEvent() != Qmi8658::TapEvent::kNone) {
            ++tap_count;
            app_controller.OnTap();
        }
        tap_poll_accum_us += esp_timer_get_time() - tap_poll_start_us;

        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_sensor_update_us) {
            // Measure real elapsed time rather than assuming exactly
            // kSensorUpdatePeriodUs — this block doesn't fire at a
            // perfectly fixed cadence (loop jitter from the blocking I2C
            // tap poll, LVGL rendering, etc.), and both the gyro
            // integration in AttitudeEstimator and AppController's
            // onTick() need the real value or they drift.
            //
            // Round to the nearest ms (+500 before truncating), not
            // truncate — plain integer division here would systematically
            // discard the sub-millisecond remainder every tick, and
            // TimerFace::onTick(dt_ms) accumulates this same dt_ms
            // directly, so the loss would compound into real timer drift.
            const uint32_t sensor_dt_ms =
                static_cast<uint32_t>((now_us - last_sensor_update_us + 500) / 1000);
            last_sensor_update_us = now_us;

            Qmi8658::Sample sample;
            if (imu.Read(sample)) {
                const AttitudeEstimator::Output attitude =
                    attitude_estimator.Update(ToAttitudeSample(sample), sensor_dt_ms);

                // Raw screen_angle_deg for now, no filtering — revisit
                // with a simple low-pass if it looks jittery on hardware.
                gui_manager.SetRotationDeg(attitude.screen_angle_deg);

                if (antialiasing_enabled == attitude.is_moving) {
                    antialiasing_enabled = !attitude.is_moving;
                    lv_display_set_antialiasing(disp, antialiasing_enabled);
                }

                if (kAttitudeDebugLogEnabled) {
                    printf("ATT,%lu,%.1f,%.2f,%.2f,%.2f,%d,%d\n", static_cast<unsigned long>(now_us / 1000),
                           attitude.debug_gz_dps, attitude.debug_gyro_only_angle_deg,
                           attitude.debug_accel_only_angle_deg, attitude.screen_angle_deg,
                           attitude.is_moving ? 1 : 0, attitude.in_valid_plane ? 1 : 0);
                }

                app_controller.Update(attitude, sensor_dt_ms);

                if (app_controller.ShouldEnterIdleSleep()) {
                    // Stage 2 of the low-battery safety net, checked first
                    // ahead of the normal light-sleep+WoM path below: once
                    // voltage is this low, skip WoM/tap entirely and go
                    // straight to deep sleep, not coming back for real
                    // until a periodic voltage recheck sees it recovered.
                    if (last_battery_voltage_v < AppController::kCriticalBatteryEnterV) {
                        printf("CRITICAL_BATTERY_ENTER,v=%.2fV — deep sleep starting\n", last_battery_voltage_v);
                        in_critical_battery_sleep = true;
                        esp_sleep_enable_timer_wakeup(kCriticalBatteryCheckPeriodUs);
                        esp_deep_sleep_start();  // never returns
                    }

                    // tick_timer's 5ms period floors every idle gap
                    // FreeRTOS sees, below CONFIG_FREERTOS_IDLE_TIME_BEFORE_SLEEP
                    // (8 ticks), so automatic light sleep would never
                    // engage while it keeps running. Stop it for the
                    // duration of RunIdleSleep() and restart right after.
                    esp_timer_stop(tick_timer);

                    // Wake-on-Motion for the duration of the nap loop — a
                    // tap's INT2 pulse is too brief for light sleep's
                    // level-wakeup detector (see qmi8658.hpp's
                    // EnterWakeOnMotion() comment).
                    //
                    // Double-tap-to-wake: WoM alone is a pre-wake, not a
                    // real one — even at the threshold register's ceiling,
                    // an incidental hand bump can trigger it. Rather than
                    // lighting the screen on every WoM event, this loop
                    // keeps the screen off and demands a real tap within a
                    // bounded window (kWomConfirmWindowMs) right after
                    // each WoM pre-wake before treating it as a genuine
                    // wake — the tap engine isn't running yet at the
                    // instant of the physical tap (still in WoM mode until
                    // RunIdleSleep() returns), so it always needs a
                    // genuinely separate second tap, not the same one's
                    // tail end.
                    bool wom_confirmed_by_tap = false;
                    while (!wom_confirmed_by_tap) {
                        imu.EnterWakeOnMotion(kWomThresholdMg, kWomBlankingSamples);
                        // Discard window for the spurious STATUS1 latch a
                        // CTRL7 toggle can cause.
                        for (int i = 0; i < 30; ++i) {
                            (void)imu.PollWomEvent();
                            vTaskDelay(pdMS_TO_TICKS(20));
                        }
                        const IdleSleepWakeReason wake_reason =
                            RunIdleSleep(imu, battery_monitor, AppController::kCriticalBatteryEnterV);
                        if (wake_reason == IdleSleepWakeReason::kCriticalBattery) {
                            // Voltage crossed critical while already
                            // asleep with nobody around to wake it — skip
                            // the WoM-confirm dance and go straight to
                            // deep sleep. No need to ExitWakeOnMotion()/
                            // ConfigureTap() first: deep sleep reboots on
                            // the way back regardless, so whatever state
                            // the IMU is left in gets reinitialized from
                            // scratch next real boot anyway.
                            printf("CRITICAL_BATTERY_ENTER (from light sleep) — deep sleep starting\n");
                            in_critical_battery_sleep = true;
                            esp_sleep_enable_timer_wakeup(kCriticalBatteryCheckPeriodUs);
                            esp_deep_sleep_start();  // never returns
                        }
                        const int64_t wom_confirmed_us = esp_timer_get_time();
                        imu.ExitWakeOnMotion();
                        // ExitWakeOnMotion() deliberately leaves CTRL7
                        // disabled (see its own comment) — ConfigureTap()
                        // both restores normal 6DOF operation (its own final
                        // CTRL7=0x03 write) and re-establishes the tap
                        // engine's thresholds, which aren't guaranteed to
                        // have survived a WoM configuration cycle reusing the
                        // same CAL1-4 scratch registers.
                        imu.ConfigureTap(kTapPriority, kTapPeakWindow, kTapTapWindow, kTapDTapWindow,
                                          kTapAlpha, kTapGamma, kTapPeakMagThr, kTapUdmThr);
                        // Same spurious-latch quirk as the boot-time
                        // ConfigureTap() call — discard it here too so the
                        // confirmation poll below isn't immediately misread
                        // as a tap that just happened.
                        (void)imu.PollTapEvent();
                        if (kWomConfirmLatencyLogEnabled) {
                            printf("WOM_CONFIRM_LATENCY,us=%lld\n",
                                   static_cast<long long>(esp_timer_get_time() - wom_confirmed_us));
                        }

                        // Confirmation window — screen still off, real
                        // sensor polling (needs the tap engine actively
                        // running, unlike light sleep's GPIO wakeup).
                        for (int elapsed_ms = 0; elapsed_ms < kWomConfirmWindowMs;
                             elapsed_ms += kWomConfirmPollMs) {
                            if (imu.PollTapEvent() != Qmi8658::TapEvent::kNone) {
                                wom_confirmed_by_tap = true;
                                break;
                            }
                            vTaskDelay(pdMS_TO_TICKS(kWomConfirmPollMs));
                        }
                        // Falls through to re-enter WoM at the top of the
                        // loop if the window closed with no tap — same
                        // ConfigureTap()->EnterWakeOnMotion() register
                        // reuse the normal exit path already does, no
                        // special-case teardown needed either way.
                    }
                    // Gyro Turn On Time is 150ms + 3/ODR (accel is
                    // near-instant) — wait past it before feeding gyro
                    // samples back into AttitudeEstimator::Update(),
                    // which otherwise briefly reads as the screen not
                    // rotating right after waking.
                    vTaskDelay(pdMS_TO_TICKS(160));

                    esp_timer_start_periodic(tick_timer, kLvglTickPeriodMs * 1000);

                    // Called before NotifyWokeFromIdleSleep(): a full
                    // panel re-init could disturb backlight state, so
                    // brightness needs to be reapplied after this.
                    gui_manager.ForceRedraw();
                    app_controller.NotifyWokeFromIdleSleep();

                    // last_sensor_update_us is now far in the past, so
                    // reset it to avoid feeding a huge dt_ms into gyro
                    // integration. angle_deg_ from before the nap is
                    // likely stale too, so seed it fresh rather than
                    // letting the complementary filter walk to the right
                    // value at its normal per-tick rate.
                    last_sensor_update_us = esp_timer_get_time();
                    next_sensor_update_us = last_sensor_update_us;
                    Qmi8658::Sample wake_sample;
                    if (imu.Read(wake_sample)) {
                        attitude_estimator.SeedInitialAngle(ToAttitudeSample(wake_sample));
                    }
                }

                if (label) {
                    const FixedParts angle = SplitFixed(RoundToFixed(attitude.screen_angle_deg, 10), 10);
                    // 2 decimal places, not 1 like angle above — 0.1V
                    // differences matter a lot on a LiPo's curve.
                    const FixedParts batt = SplitFixed(RoundToFixed(last_battery_voltage_v, 100), 100);
                    // Split to 3 short lines — the panel is round, and
                    // this label sits near the narrow top, so each line
                    // needs to stay short. R%d%c: last averaged raw ADC
                    // count plus a C/U flag for whether BatteryMonitor's
                    // calibration scheme is active.
                    lv_label_set_text_fmt(label, "Ang %c%d.%01d Taps%d\nMv%d FPS%u\nBat%d.%02dV R%d%c",
                        angle.sign, angle.whole, angle.frac, tap_count,
                        attitude.is_moving ? 1 : 0, static_cast<unsigned int>(fps_display),
                        batt.whole, batt.frac,
                        battery_monitor.LastRawAverage(), battery_monitor.IsCalibrated() ? 'C' : 'U');
                }
            } else if (label) {
                lv_label_set_text(label, "IMU read failed");
            }
            next_sensor_update_us = now_us + kSensorUpdatePeriodUs;
        }
        sensor_block_accum_us += esp_timer_get_time() - now_us;

        if (now_us >= next_battery_read_us) {
            last_battery_voltage_v = battery_monitor.ReadVoltage();
            app_controller.SetBatteryVoltage(last_battery_voltage_v);
            next_battery_read_us = now_us + kBatteryReadPeriodUs;
        }

        const int64_t lvgl_start_us = esp_timer_get_time();
        lv_timer_handler();
        lvgl_accum_us += esp_timer_get_time() - lvgl_start_us;

        const int64_t delay_start_us = esp_timer_get_time();
        vTaskDelay(pdMS_TO_TICKS(kLvglTickPeriodMs));
        delay_accum_us += esp_timer_get_time() - delay_start_us;
    }
}
