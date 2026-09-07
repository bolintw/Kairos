#include <cstdio>

#include "driver/gpio.h"
#include "esp_pm.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "nvs_flash.h"

#include "app_controller.hpp"
#include "attitude_estimator.hpp"
#include "calibration_mode.hpp"
#include "sleep_mode.hpp"
#include "gui_manager.hpp"
#include "i2c_scan.hpp"
#include "lgfx_config.hpp"
#include "nvs_calibration.hpp"
#include "qmi8658.hpp"

namespace {

constexpr int kLvglTickPeriodMs = 5;
// 20 -> 60 rows (2026-09-06), and split into two buffers instead of one —
// enables LVGL's double-buffered partial mode: while chunk N is being
// DMA'd out over SPI (lvgl_flush_cb, pushImageDMA), LVGL renders chunk
// N+1 into the other buffer instead of waiting. Room freed by
// CONFIG_LV_MEM_SIZE_KILOBYTES 256->128 (see that config's sdkconfig
// comment) — two 60-row RGB565 buffers are 240*60*2*2 = 57.6KB, well
// under the ~51KB single worst-case rotation buffer this pool already had
// to fit (gui_manager.hpp's kRootWidthPx*kRootHeightPx*4 note) at half
// the total pool size.
constexpr int kDrawBufRows = 60;
// 150ms -> 30ms -> 120Hz (2026-08-25): raised again so a future low-pass
// filter on the raw gyro/accel samples has real headroom above both the
// display's ~60fps redraw cap and its own filter bandwidth — sampling
// faster than what you filter/display is the sane order, not the other
// way round. QMI8658's own ODR is 1000Hz (see qmi8658.hpp), and the I2C
// read itself (~13 bytes @ 400kHz) is well under a millisecond, so 120Hz
// polling has plenty of room in the 5ms main-loop cadence. dt_ms-based
// timing elsewhere (AttitudeEstimator, AppController) is unaffected by
// the rate itself, just gets finer-grained inputs — EXCEPT the
// complementary filter's alpha: BlendTowardAngle() absorbs a fixed
// *fraction* of the gyro/accel gap per call, not per unit time, so more
// calls/sec at the same alpha=0.9 means faster real-time convergence
// toward the accel reading than what 0.9 was tuned to feel like at the
// old 30ms rate. Worth re-checking the settle "feel" on hardware after
// this change — may want to nudge alpha up to compensate.
constexpr int64_t kSensorUpdatePeriodUs = 1000000 / 120;  // ~120Hz

// Flip to false to hide the debug overlay entirely (angle/taps/is_moving
// label at the top) without deleting the code — flip back on when
// debugging attitude/tap behavior again.
//
// Briefly tested false 2026-09-06 to check whether this label (the one
// piece of on-screen text that bypasses GuiManager's dirty-check —
// lv_label_set_text_fmt() called unconditionally every sensor tick, right
// below) was behind flush_calls=143/s at rest seen in a main-loop timing
// breakdown. It wasn't — flush_calls barely dropped (143->110) with the
// label off, same ~11-chunks-per-tick ratio either way. Real cause turned
// out to be root_'s rotation redraw (see GuiManager::SetRotationDeg's
// comment) — fixed there instead, so this stays on.
constexpr bool kDebugOverlayEnabled = true;

// Temporary diagnostic: one live "ATT," CSV line per sensor tick with
// AttitudeEstimator's gyro-only angle, accel-only angle, fused angle, and
// gz (see attitude_estimator.hpp's Output::debug_* fields) — plain
// printf, no buffering.
//
// History (2026-09-06): briefly went through a RAM-ring-buffer-then-dump
// version instead, on the theory that live per-tick printf's blocking
// console UART I/O was itself throttling the sensor loop down to ~10Hz
// (real hardware showed consecutive ATT rows 80-140ms apart instead of
// the intended ~8.3ms/120Hz). Root-caused with kLoopTimingLogEnabled
// instead: the throttling turned out to be root_'s rotation redraw (see
// GuiManager::SetRotationDeg's comment), not this logging — the buffered
// version showed the exact same ~10Hz rate, which is what proved that.
// Once printf itself was cleared, the extra ring-buffer/dump-on-settle
// complexity (and its own side effect, a multi-second stutter every time
// it dumps) stopped earning its cost. Back to the simple version; revisit
// the buffered approach only if something *else* turns out to need
// hiding I/O from the timing-critical window again.
constexpr bool kAttitudeDebugLogEnabled = false;

// Hold BOOT (GPIO0) this long, while the app is already running, to enter
// calibration mode. NOT checked at power-on/reset — see calibration_mode.hpp.
constexpr int64_t kCalibrationHoldUs = 3 * 1000 * 1000;

// Tap Engine config (M4's starting point, see qmi8658.hpp's
// ConfigureTap() for what each parameter means). Hoisted to named
// constants 2026-09-06 (previously inline literals only in the boot-time
// call below) mainly for self-documentation now — M9's idle-sleep path
// (Qmi8658::SetLowPowerAccelOnly()) deliberately leaves the accel ODR
// these were tuned against unchanged, so these same values keep applying
// whether or not gyro is currently enabled; no separate sleep-mode
// tap config needed (see that method's comment for why an earlier
// attempt at one broke tap detection outright).
constexpr uint8_t kTapPriority = 0;
constexpr uint8_t kTapPeakWindow = 40;
constexpr uint16_t kTapTapWindow = 100;
constexpr uint16_t kTapDTapWindow = 500;
constexpr float kTapAlpha = 0.0625f;
constexpr float kTapGamma = 0.25f;
constexpr float kTapPeakMagThr = 0.8f;
constexpr float kTapUdmThr = 0.4f;

static LGFX lcd;
// Two buffers now (2026-09-06, was one) — see lvgl_flush_cb()'s
// pushImageDMA()/waitDMA() comment for why.
static uint8_t lvgl_draw_buf1[240 * kDrawBufRows * 2];  // RGB565, 2 bytes/px
static uint8_t lvgl_draw_buf2[240 * kDrawBufRows * 2];

// Temporary diagnostic (2026-09-06) — see kLoopTimingLogEnabled's comment
// in app_main(): the LOOP,... breakdown showed lv_timer_handler() eating
// ~96% of every second, *even while GuiManager's own dirty-checks were
// reporting ~0 real content changes*, which points away from the SPI
// flush itself (nothing to flush if nothing's dirty) and toward something
// else inside LVGL's per-call processing. Counting/timing pushImage()
// calls specifically (file-scope so the flush callback and app_main()'s
// print loop can both reach them) is the next cut: if flush_call_count
// stays near 0 while lvgl_us stays near 74ms/call anyway, that rules out
// the SPI write path entirely.
int64_t g_lvgl_flush_accum_us = 0;
uint32_t g_lvgl_flush_call_count = 0;

void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map)
{
    const int32_t w = area->x2 - area->x1 + 1;
    const int32_t h = area->y2 - area->y1 + 1;
    const int64_t flush_start_us = esp_timer_get_time();
    // Display is configured LV_COLOR_FORMAT_RGB565_SWAPPED to match the
    // byte order pushImage() expects — see the color format comment
    // where the display is created.
    //
    // pushImageDMA() + waitDMA() (2026-09-06, was a single blocking
    // pushImage() call) — with two draw buffers now passed to
    // lv_display_set_buffers(), LVGL alternates between them each flush,
    // so while this chunk's bytes are still going out over SPI in the
    // background, LVGL can render the *next* chunk into the other buffer
    // instead of blocking on this one. waitDMA() at the top (not the
    // bottom) is what makes this safe on a single SPI bus: it blocks only
    // if the *previous* async transfer hasn't finished yet (normally
    // already done, since a render pass takes real time too), never on
    // this call's own transfer — that's queued and left running via
    // pushImageDMA(), collected by the next call's waitDMA() instead of
    // this one's. flush_us below now measures queueing time, not full
    // transfer time — expect it to look much smaller than before, that's
    // the point of overlapping the two.
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

}  // namespace

extern "C" void app_main(void)
{
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

    // GPIO0 (BOOT) as a normal input once past the ROM bootloader's
    // strapping check — see calibration_mode.hpp for why this is only
    // polled here, not checked at reset.
    gpio_config_t boot_btn_cfg = {};
    boot_btn_cfg.pin_bit_mask = 1ULL << GPIO_NUM_0;
    boot_btn_cfg.mode = GPIO_MODE_INPUT;
    boot_btn_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&boot_btn_cfg);

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
    //
    // Tried plain LV_COLOR_FORMAT_RGB565 + lcd.setSwapBytes(true) instead
    // (2026-09-06), on the theory that the matrix-path crash below (see
    // gui_manager.hpp's history) was specific to this non-default color
    // format. It wasn't — same Guru Meditation, same
    // lv_draw_sw_blend_color_to_rgb565 (the *non*-swapped variant this
    // time) via draw_letter_cb/refr_obj_matrix, confirming the bug is in
    // LVGL 9.5.0's matrix-transformed text-glyph rendering itself,
    // independent of color format. Reverted back to this — no reason to
    // carry the untested setSwapBytes() path once it didn't avoid the
    // crash it was meant to avoid.
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

    // M9: lets RunIdleSleep()'s vTaskDelay() calls (sleep_mode.cpp) drop
    // into real light sleep automatically via FreeRTOS tickless idle,
    // instead of calling esp_light_sleep_start() by hand — see
    // sleep_mode.hpp's comment for why the hand-rolled version was
    // dropped (left the LCD permanently blank after the first sleep,
    // with no fix found short of switching to this framework path).
    // min_freq_mhz == max_freq_mhz deliberately disables DFS (dynamic
    // CPU frequency scaling): see gravity_timer_project_plan.md's M9
    // notes on the CPU-downclock experiment — even an automatic,
    // brief drop below 160MHz measurably hurt the software
    // screen-rotation redraw's throughput on hardware.
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

    // Deliberately NOT rotating with the primary label (see
    // gui_manager.hpp's screen counter-rotation note) — stays a plain
    // fixed child of lv_screen_active() for now, after the fully-rotating
    // version crashed twice on hardware. Created *before* GuiManager below
    // (2026-09-01, was after) so root_'s children end up later in
    // lv_screen_active()'s child list and therefore draw on top of this
    // overlay, not under it — with an unchanged order, calibration mode's
    // two-line "Rotate\nand hold" grew tall enough to reach up into the
    // overlay's on-screen area and the overlay (drawn later = on top)
    // visibly cut into the top of "Rotate" (caught on hardware). Pure
    // z-order fix, no positions/sizes changed.
    lv_obj_t* label = nullptr;
    if (kDebugOverlayEnabled) {
        label = lv_label_create(lv_screen_active());
        lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_label_set_text(label, "waiting for IMU...");
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);
    }

    // M6: AppController owns attitude-driven face switching (via
    // AttitudeEstimator::Output, computed below) plus tap routing and
    // TimerFace lifecycle. See app_controller.hpp for the design.
    static GuiManager gui_manager(lcd);
    static AppController app_controller(gui_manager);

    // M3/M4 debug overlay: raw accel/gyro readout plus tap count.
    static Qmi8658 imu(GPIO_NUM_6, GPIO_NUM_7);

    // M4 starting point, not a finished tune — adjust these while watching
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
    int64_t boot_button_press_start_us = 0;
    int tap_count = 0;
    // Rendering knob 1/4 (2026-09-06): antialiasing off while actively
    // rotating, on once settled — see lv_refr.c's layer_draw_dsc.antialias
    // (fed straight from lv_display_get_antialiasing()), which root_'s
    // rotated redraw pays for on every blended pixel. Not worth it during
    // a fast flip (blurriness not very noticeable while things are moving
    // anyway); still applied once resting, so settled text looks the same
    // as before. Dirty-checked so it's only ever set on an actual
    // is_moving transition, not every tick.
    bool antialiasing_enabled = true;

    int64_t last_fps_calc_us = esp_timer_get_time();
    uint32_t fps_display = 0;
    uint32_t last_update_count = 0;

    // Temporary diagnostic (2026-09-06): per-second breakdown of where
    // main loop time actually goes. Added after the ATT capture above
    // showed the sensor-tick block firing at only ~10Hz instead of the
    // designed ~120Hz, *even at rest* and *even with the debug log itself
    // reduced to near-zero-cost RAM writes* — ruling out that logging as
    // the cause. GuiManager's on-screen FPS counter can't answer this
    // either: it counts real *content changes* after GuiManager's own
    // dirty-check (see its GetUpdateCount() comment below), so reading 0-1
    // FPS at rest is that check correctly skipping redundant redraws, not
    // evidence about loop speed — a wrong turn taken (and corrected) before
    // adding this. loop_count directly answers "how many times did the
    // outer while(true) actually run in the last second"; the four *_us
    // accumulators say which section it went into.
    constexpr bool kLoopTimingLogEnabled = true;
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
            // onTick() need the real value or they drift, same class of
            // bug as the earlier stopwatch timing fix.
            //
            // Round to the nearest ms (+500 before truncating), not
            // truncate — plain integer division here systematically
            // discards the sub-millisecond remainder every tick (up to
            // 999us, ~500us on average), which is a real, measured drift
            // source: TimerFace::onTick(dt_ms) accumulates this same
            // dt_ms directly, so the loss compounds with tick rate — a
            // stopwatch measured ~2% slow at the old 150ms/~6.7Hz sensor
            // rate would lose roughly 6% at the current 120Hz if left
            // truncating (2026-08-25).
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
                    // tick_timer fires every kLvglTickPeriodMs (5ms) with
                    // skip_unhandled_events=false, which floors every
                    // idle gap FreeRTOS sees at ~5ms — below
                    // CONFIG_FREERTOS_IDLE_TIME_BEFORE_SLEEP (8 ticks =
                    // 8ms), so automatic light sleep (esp_pm_configure()
                    // above) would never see a long enough gap to engage
                    // while it keeps running. Nothing needs LVGL ticking
                    // while nothing is being rendered anyway, so stop it
                    // for the duration of RunIdleSleep() and restart it
                    // right after.
                    esp_timer_stop(tick_timer);

                    // Drop the IMU to accel-only for the duration of the
                    // nap loop — cuts its own current draw from ~1mA
                    // (normal 6DOF) to roughly 182uA, see
                    // qmi8658.hpp's SetLowPowerAccelOnly() comment. Accel
                    // ODR/tap config is deliberately left untouched by
                    // that call, so nothing needs reconfiguring here
                    // around it.
                    imu.SetLowPowerAccelOnly(true);
                    // Toggling CTRL7 reliably produces one spurious tap —
                    // caught on hardware with diagnostic logging (raw
                    // STATUS1/TAP_STATUS prints in PollTapEvent()): a
                    // clean, well-formed Single-Tap event (TAP_STATUS low
                    // 2 bits = 01, no other STATUS1 bits set) appeared
                    // every single time, always right after a 200ms
                    // discard window had already elapsed, never during
                    // it. That timing matches the tap engine's own
                    // confirmation latency, not a register-read race:
                    // peak_window (40 samples) + tap_window (100 samples)
                    // = 140 samples, ~156ms at this device's real ~896.8Hz
                    // accel ODR (see qmi8658.hpp's SetLowPowerAccelOnly()
                    // comment on why it's 896.8Hz, not 1000Hz) — the
                    // minimum time the tap engine needs from detecting a
                    // peak to confirming/reporting it as a tap. 200ms sat
                    // right at that threshold; likely the CTRL7 toggle
                    // itself causes one genuine transient in the analog
                    // front-end that the tap engine picks up as a peak,
                    // takes its normal ~156ms to confirm, every time.
                    // Widened the discard window well past that instead
                    // of the threshold itself.
                    for (int i = 0; i < 30; ++i) {
                        (void)imu.PollTapEvent();
                        vTaskDelay(pdMS_TO_TICKS(20));
                    }
                    RunIdleSleep(imu);
                    imu.SetLowPowerAccelOnly(false);
                    // Gyro Turn On Time is 150ms + 3/ODR per the
                    // QMI8658C datasheet (Tables 7/8) — the gyroscope's
                    // MEMS resonator needs real physical spin-up time
                    // after being re-enabled, unlike accel (3ms + 3/ODR,
                    // near-instant). Feeding gyro samples into
                    // AttitudeEstimator::Update() before that elapses
                    // reads as the screen briefly not rotating after
                    // waking (seen once on hardware, not reliably
                    // reproducible — consistent with a ~150ms window
                    // that's usually too short to notice). Wait past it
                    // before resuming normal operation below.
                    vTaskDelay(pdMS_TO_TICKS(160));

                    esp_timer_start_periodic(tick_timer, kLvglTickPeriodMs * 1000);

                    // Re-inits the panel and forces a full redraw — see
                    // gui_manager.hpp's ForceRedraw() comment. Called
                    // before NotifyWokeFromIdleSleep() deliberately: a
                    // full panel re-init could disturb backlight state
                    // along the way, so brightness needs to be
                    // (re-)applied after this, not before.
                    gui_manager.ForceRedraw();
                    app_controller.NotifyWokeFromIdleSleep();

                    // Two things RunIdleSleep()'s pause invalidates,
                    // both already solved once for the cold-boot case
                    // above and reused here as-is:
                    //   - last_sensor_update_us is now far in the past
                    //     (however long the nap loop ran), so the next
                    //     dt_ms computed from it would be huge — feeding
                    //     that into AttitudeEstimator::Update()'s gyro
                    //     integration would turn ordinary gyro noise into
                    //     a large bogus angle swing. Reset both timers to
                    //     now so the next tick sees a normal small dt_ms.
                    //   - RunIdleSleep() only returns because it detected
                    //     real movement, so angle_deg_ from before the
                    //     nap is likely stale by the time we get here —
                    //     SeedInitialAngle() again rather than letting
                    //     the complementary filter slowly walk to the
                    //     right value at its normal per-tick rate.
                    last_sensor_update_us = esp_timer_get_time();
                    next_sensor_update_us = last_sensor_update_us;
                    Qmi8658::Sample wake_sample;
                    if (imu.Read(wake_sample)) {
                        attitude_estimator.SeedInitialAngle(ToAttitudeSample(wake_sample));
                    }
                }

                if (label) {
                    const FixedParts angle = SplitFixed(RoundToFixed(attitude.screen_angle_deg, 10), 10);
                    lv_label_set_text_fmt(label, "Ang %c%d.%01d  Taps %d\nMv%d  FPS%u",
                        angle.sign, angle.whole, angle.frac,
                        tap_count, attitude.is_moving ? 1 : 0,
                        static_cast<unsigned int>(fps_display));
                }
            } else if (label) {
                lv_label_set_text(label, "IMU read failed");
            }
            next_sensor_update_us = now_us + kSensorUpdatePeriodUs;
        }
        sensor_block_accum_us += esp_timer_get_time() - now_us;

        const int64_t lvgl_start_us = esp_timer_get_time();
        lv_timer_handler();
        lvgl_accum_us += esp_timer_get_time() - lvgl_start_us;

        const int64_t delay_start_us = esp_timer_get_time();
        vTaskDelay(pdMS_TO_TICKS(kLvglTickPeriodMs));
        delay_accum_us += esp_timer_get_time() - delay_start_us;
    }
}
