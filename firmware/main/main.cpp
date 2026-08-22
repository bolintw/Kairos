#include <cstdio>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "gpio_pin.hpp"
#include "lgfx_config.hpp"

namespace {

constexpr int kLvglTickPeriodMs = 5;
constexpr int kDrawBufRows = 20;  // partial buffer: 20 rows of the 240-wide panel

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

}  // namespace

extern "C" void app_main(void)
{
    printf("Kairos gravity timer — hello from C++\n");

    backlight.set(true);
    printf("Backlight on (GPIO40)\n");

    lcd.init();
    printf("LCD initialized\n");

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
    lv_label_set_text(label, "Kairos");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_32, 0);
    lv_obj_center(label);

    printf("LVGL running\n");

    while (true) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(kLvglTickPeriodMs));
    }
}
