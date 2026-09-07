#include "sleep_mode.hpp"

#include <cstdio>

#include "driver/gpio.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr gpio_num_t kImuInt2Gpio = GPIO_NUM_48;

// Backstop period for the timer wakeup source below — same role as the
// old plain-poll version's vTaskDelay() period, see its own history for
// why 1000ms specifically. Not a poll interval anymore in the literal
// sense (this loop doesn't vTaskDelay() at all now), just the ceiling on
// how long a single esp_light_sleep_start() call is allowed to sleep
// before waking on its own regardless of GPIO activity.
constexpr uint32_t kSleepBackstopMs = 1000;

}  // namespace

// Manual esp_light_sleep_start() loop with a real GPIO wakeup source
// (2026-09-06) — third attempt at instant tap-wake, after two prior ones
// on the *automatic* PM/tickless-idle light sleep path both failed (see
// git history / gravity_timer_project_plan.md's M9 section for the full
// writeup):
//   1. An edge-triggered runtime ISR (gpio_isr_handler_add) coexisting
//      with gpio_wakeup_enable() on the same pin crashed twice
//      (watchdog panic, ISR livelock) — gpio_wakeup_enable() silently
//      overwrites the pin's one hardware intr_type register from edge to
//      level, and if the pin was already high, the level ISR re-fires
//      forever. Confirmed against ESP-IDF's own gpio.c
//      (gpio_hal_set_intr_type() called unconditionally) and a matching
//      upstream report (esp-idf#13444).
//   2. Dropping the runtime ISR in favor of an
//      esp_pm_light_sleep_register_cbs() exit callback didn't crash, but
//      esp_sleep_get_wakeup_cause() read UNDEFINED on every single poll.
//      Confirmed against sleep_modes.c: esp_sleep_get_wakeup_cause()
//      only returns a real cause when s_light_sleep_wakeup is true, which
//      is only set when esp_light_sleep_start() returned ESP_OK — so
//      UNDEFINED forever means every attempt returned something else,
//      not "never tried." ESP32-S3's RTC_SLEEP_REJECT_MASK (soc/rtc.h)
//      includes RTC_GPIO_TRIG_EN — if IMU_INT2/GPIO48 is already at the
//      armed wakeup level (HIGH) the instant sleep is attempted, the
//      hardware rejects the whole attempt outright
//      (ESP_ERR_SLEEP_REJECT) rather than entering and immediately
//      exiting. Both failures above are explained by the same single
//      fact — GPIO48 being high when it's expected to be low — without
//      needing two separate root causes.
//
// This version sidesteps both: no runtime ISR is ever registered on this
// pin (removes failure 1's whole mechanism), and esp_light_sleep_start()
// is called directly, synchronously, in this task, so err and
// esp_sleep_get_wakeup_cause() are read right where they're produced —
// no opaque idle-task-context callback in between (removes failure 2's
// diagnostic blind spot). If GPIO48 does turn out to be high at the wrong
// times, this degrades gracefully instead of crashing or hanging: a
// rejected sleep just falls through to the existing PollTapEvent() I2C
// poll below (STATUS1/TAP_STATUS are latched until read, so a real tap's
// flag is still there even if the pulse itself already passed) and a
// short vTaskDelay() backoff, functionally similar to the old plain-poll
// version rather than worse.
void RunIdleSleep(Qmi8658& imu)
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << kImuInt2Gpio;
    cfg.mode = GPIO_MODE_INPUT;
    // INT2 is push-pull (datasheet Section 6, actively driven both ways
    // by the IMU) so this shouldn't matter in steady state, but pulling
    // to the level opposite the wakeup trigger is what the upstream
    // esp-idf#13444 discussion recommends as a defensive measure for any
    // brief window where the pin isn't being actively driven.
    cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&cfg);

    gpio_wakeup_enable(kImuInt2Gpio, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_sleep_enable_timer_wakeup(kSleepBackstopMs * 1000);

    while (true) {
        const esp_err_t err = esp_light_sleep_start();
        // Temporary diagnostic, left in deliberately for now (2026-09-06)
        // — cheap (once per real sleep attempt, not per tick), and this
        // is exactly the data needed to tell "rejected" / "slept but
        // GPIO didn't trigger" / other apart on real hardware instead of
        // guessing again.
        printf("SLEEP,err=%d,cause=%d,int2=%d\n", static_cast<int>(err),
               static_cast<int>(esp_sleep_get_wakeup_cause()), gpio_get_level(kImuInt2Gpio));

        // Tap-only wake (2026-09-06) — gyro is disabled for the duration
        // of this call (see main.cpp's SetLowPowerAccelOnly(true) call
        // just before this), so there's no gyro-magnitude check to make
        // here anymore. See qmi8658.hpp's SetLowPowerAccelOnly() comment
        // for why: gyro's own current draw barely depends on ODR, so
        // disabling it entirely is the only real lever for cutting idle
        // current, and losing rotation-based wake while asleep was an
        // accepted trade-off for that.
        if (imu.PollTapEvent() != Qmi8658::TapEvent::kNone) {
            break;
        }

        // err != ESP_OK (most likely ESP_ERR_SLEEP_REJECT, see the class
        // comment above) means esp_light_sleep_start() returned near-
        // instantly without actually sleeping — looping straight back
        // into it would busy-spin at full CPU/power until whatever's
        // holding GPIO48 high clears. This bounds that to a plain poll,
        // same shape as the old fallback version, instead of a silent
        // spin.
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    gpio_wakeup_disable(kImuInt2Gpio);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
}
