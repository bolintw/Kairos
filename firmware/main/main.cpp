#include <cstdio>

#include "gpio_pin.hpp"
#include "lgfx_config.hpp"

namespace {
// static: app_main's task exits after returning, and GpioPin's destructor
// would call gpio_reset_pin() and turn the backlight back off. A static
// local outlives the task, so the pin stays configured and held high.
static GpioPin backlight(GPIO_NUM_40, GpioPin::Direction::Output);
static LGFX lcd;
}  // namespace

extern "C" void app_main(void)
{
    printf("Kairos gravity timer — hello from C++\n");

    backlight.set(true);
    printf("Backlight on (GPIO40)\n");

    lcd.init();
    lcd.fillScreen(TFT_RED);
    printf("Screen filled red\n");
}
