#include <cstdio>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "app_controller.hpp"
#include "attitude_estimator.hpp"
#include "gpio_pin.hpp"
#include "gui_manager.hpp"
#include "i2c_scan.hpp"
#include "lgfx_config.hpp"
#include "qmi8658.hpp"

namespace {

constexpr int kLvglTickPeriodMs = 5;
constexpr int kDrawBufRows = 20;  // partial buffer: 20 rows of the 240-wide panel
constexpr int64_t kSensorUpdatePeriodUs = 150 * 1000;  // readable, not maxed out
constexpr int kGyroChartPoints = 60;      // 60 * 150ms = 9s of history
constexpr int kGyroChartRangeDps = 250;   // +-range; adjust if rotations clip

// static: app_main's task exits after returning, and GpioPin's destructor
// would call gpio_reset_pin() and turn the backlight back off. A static
// local outlives the task, so the pin stays configured and held high.
static GpioPin backlight(GPIO_NUM_40, GpioPin::Direction::Output);
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

    backlight.set(true);
    printf("Backlight on (GPIO40)\n");

    lcd.init();
    printf("LCD initialized\n");

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

    lv_obj_t* label = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(label, "Kairos\nwaiting for IMU...");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);

    // Gyro X/Y/Z scrolling chart — raw numbers refresh too fast to read a
    // trend out of, especially with how noisy gyro is at rest. Red=X,
    // Green=Y, Blue=Z.
    lv_obj_t* gyro_chart = lv_chart_create(lv_screen_active());
    lv_obj_set_size(gyro_chart, 170, 100);
    lv_obj_align(gyro_chart, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_chart_set_type(gyro_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(gyro_chart, kGyroChartPoints);
    lv_chart_set_range(gyro_chart, LV_CHART_AXIS_PRIMARY_Y, -kGyroChartRangeDps, kGyroChartRangeDps);
    lv_chart_set_update_mode(gyro_chart, LV_CHART_UPDATE_MODE_SHIFT);
    lv_chart_series_t* gx_series =
        lv_chart_add_series(gyro_chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t* gy_series =
        lv_chart_add_series(gyro_chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);
    lv_chart_series_t* gz_series =
        lv_chart_add_series(gyro_chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_PRIMARY_Y);

    printf("LVGL running\n");

    // M6: AppController owns attitude-driven face switching (via
    // AttitudeEstimator::Output, computed below) plus tap routing and
    // TimerFace lifecycle. See app_controller.hpp for the design.
    static GuiManager gui_manager;
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
                // Accel to hundredths of g, gyro to tenths of dps.
                const FixedParts ax = SplitFixed(RoundToFixed(sample.accel_g[0], 100), 100);
                const FixedParts ay = SplitFixed(RoundToFixed(sample.accel_g[1], 100), 100);
                const FixedParts az = SplitFixed(RoundToFixed(sample.accel_g[2], 100), 100);

                const AttitudeEstimator::Output attitude =
                    attitude_estimator.Update(ToAttitudeSample(sample), sensor_dt_ms);
                const FixedParts angle = SplitFixed(RoundToFixed(attitude.screen_angle_deg, 10), 10);

                app_controller.Update(attitude, sensor_dt_ms);

                lv_label_set_text_fmt(label,
                    "AX %c%d.%02dg AY %c%d.%02dg AZ %c%d.%02dg\nTaps %d  Ang %c%d.%01d Mv%d Vp%d",
                    ax.sign, ax.whole, ax.frac, ay.sign, ay.whole, ay.frac, az.sign, az.whole, az.frac,
                    tap_count, angle.sign, angle.whole, angle.frac,
                    attitude.is_moving ? 1 : 0, attitude.in_valid_plane ? 1 : 0);

                // Gyro goes to the chart instead of text — whole-dps
                // resolution is plenty for spotting a rotation's shape.
                auto to_chart_value = [](float dps) {
                    int whole = RoundToFixed(dps, 1);
                    if (whole > kGyroChartRangeDps) whole = kGyroChartRangeDps;
                    if (whole < -kGyroChartRangeDps) whole = -kGyroChartRangeDps;
                    return static_cast<lv_coord_t>(whole);
                };
                lv_chart_set_next_value(gyro_chart, gx_series, to_chart_value(sample.gyro_dps[0]));
                lv_chart_set_next_value(gyro_chart, gy_series, to_chart_value(sample.gyro_dps[1]));
                lv_chart_set_next_value(gyro_chart, gz_series, to_chart_value(sample.gyro_dps[2]));
            } else {
                lv_label_set_text(label, "IMU read failed");
            }
            next_sensor_update_us = now_us + kSensorUpdatePeriodUs;
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(kLvglTickPeriodMs));
    }
}
