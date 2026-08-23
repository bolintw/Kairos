#include <cstdio>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "app_controller.hpp"
#include "attitude_estimator.hpp"
#include "gui_manager.hpp"
#include "i2c_scan.hpp"
#include "lgfx_config.hpp"
#include "qmi8658.hpp"

namespace {

constexpr int kLvglTickPeriodMs = 5;
constexpr int kDrawBufRows = 20;  // partial buffer: 20 rows of the 240-wide panel
constexpr int64_t kSensorUpdatePeriodUs = 150 * 1000;  // readable, not maxed out

// Flip to false to hide the debug overlay entirely (angle/taps/is_moving
// label at the top) without deleting the code — flip back on when
// debugging attitude/tap behavior again.
constexpr bool kDebugOverlayEnabled = true;

static LGFX lcd;
static uint8_t lvgl_draw_buf[240 * kDrawBufRows * 2];  // RGB565, 2 bytes/px

void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map)
{
    const int32_t w = area->x2 - area->x1 + 1;
    const int32_t h = area->y2 - area->y1 + 1;
    // Display is configured LV_COLOR_FORMAT_RGB565_SWAPPED to match the
    // byte order pushImage() expects — see the color format comment
    // where the display is created.
    lcd.pushImage(area->x1, area->y1, w, h, reinterpret_cast<uint16_t*>(px_map));
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
    lv_display_set_buffers(disp, lvgl_draw_buf, nullptr, sizeof(lvgl_draw_buf),
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

    // Black background — the default LVGL theme is light, which clashes
    // once AppController starts dimming/warming the primary label's color.
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, 0);

    lv_obj_t* label = nullptr;
    if (kDebugOverlayEnabled) {
        label = lv_label_create(lv_screen_active());
        lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_label_set_text(label, "waiting for IMU...");
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);
    }

    printf("LVGL running\n");

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
    imu.ConfigureTap(/*priority=*/0, /*peak_window=*/40, /*tap_window=*/100,
                      /*d_tap_window=*/500, /*alpha=*/0.0625f, /*gamma=*/0.25f,
                      /*peak_mag_thr_g2=*/0.8f, /*udm_thr_g2=*/0.4f);
    // Toggling Ctrl7/Ctrl8 while configuring the tap engine latches a
    // spurious STATUS1 tap flag (observed as "Taps 1" right at boot, with
    // no physical tap). Discard it here so the count starts clean.
    (void)imu.PollTapEvent();

    static AttitudeEstimator attitude_estimator;
    {
        // Assumes the device is stationary at boot, per
        // CalibrateGyroZeroOffset()'s documented contract.
        Qmi8658::Sample boot_sample;
        if (imu.Read(boot_sample)) {
            attitude_estimator.CalibrateGyroZeroOffset(ToAttitudeSample(boot_sample));
        }
    }

    int64_t next_sensor_update_us = 0;
    int64_t last_sensor_update_us = esp_timer_get_time();
    int tap_count = 0;

    while (true) {
        if (imu.PollTapEvent() != Qmi8658::TapEvent::kNone) {
            ++tap_count;
            app_controller.OnTap();
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_sensor_update_us) {
            // Measure real elapsed time rather than assuming exactly
            // kSensorUpdatePeriodUs — this block doesn't fire at a
            // perfectly fixed cadence (loop jitter from the blocking I2C
            // tap poll, LVGL rendering, etc.), and both the gyro
            // integration in AttitudeEstimator and AppController's
            // onTick() need the real value or they drift, same class of
            // bug as the earlier stopwatch timing fix.
            const uint32_t sensor_dt_ms =
                static_cast<uint32_t>((now_us - last_sensor_update_us) / 1000);
            last_sensor_update_us = now_us;

            Qmi8658::Sample sample;
            if (imu.Read(sample)) {
                const AttitudeEstimator::Output attitude =
                    attitude_estimator.Update(ToAttitudeSample(sample), sensor_dt_ms);

                app_controller.Update(attitude, sensor_dt_ms);

                if (label) {
                    const FixedParts angle = SplitFixed(RoundToFixed(attitude.screen_angle_deg, 10), 10);
                    lv_label_set_text_fmt(label, "Ang %c%d.%01d  Taps %d  Mv%d",
                        angle.sign, angle.whole, angle.frac,
                        tap_count, attitude.is_moving ? 1 : 0);
                }
            } else if (label) {
                lv_label_set_text(label, "IMU read failed");
            }
            next_sensor_update_us = now_us + kSensorUpdatePeriodUs;
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(kLvglTickPeriodMs));
    }
}
