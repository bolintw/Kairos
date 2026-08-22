#include <cstdio>
#include <cstdlib>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "gpio_pin.hpp"
#include "i2c_scan.hpp"
#include "lgfx_config.hpp"
#include "qmi8658.hpp"

namespace {

constexpr int kLvglTickPeriodMs = 5;
constexpr int kDrawBufRows = 20;  // partial buffer: 20 rows of the 240-wide panel
constexpr int64_t kSensorUpdatePeriodUs = 150 * 1000;  // readable, not maxed out

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
    lv_obj_center(label);

    printf("LVGL running\n");

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

    int64_t next_sensor_update_us = 0;
    int tap_count = 0;

    while (true) {
        if (imu.PollTapEvent() != Qmi8658::TapEvent::kNone) {
            ++tap_count;
        }

        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_sensor_update_us) {
            Qmi8658::Sample sample;
            if (imu.Read(sample)) {
                // Accel to hundredths of g, gyro to tenths of dps.
                const int ax = RoundToFixed(sample.accel_g[0], 100);
                const int ay = RoundToFixed(sample.accel_g[1], 100);
                const int az = RoundToFixed(sample.accel_g[2], 100);
                const int gx = RoundToFixed(sample.gyro_dps[0], 10);
                const int gy = RoundToFixed(sample.gyro_dps[1], 10);
                const int gz = RoundToFixed(sample.gyro_dps[2], 10);
                lv_label_set_text_fmt(label,
                    "AX %+d.%02dg AY %+d.%02dg\nAZ %+d.%02dg\n"
                    "GX %+d.%d GY %+d.%d\nGZ %+d.%d dps\nTaps %d",
                    ax / 100, abs(ax % 100), ay / 100, abs(ay % 100), az / 100, abs(az % 100),
                    gx / 10, abs(gx % 10), gy / 10, abs(gy % 10), gz / 10, abs(gz % 10),
                    tap_count);
            } else {
                lv_label_set_text(label, "IMU read failed");
            }
            next_sensor_update_us = now_us + kSensorUpdatePeriodUs;
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(kLvglTickPeriodMs));
    }
}
