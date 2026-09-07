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

// Gates the SLEEP,err=...,cause=...,int2=... line below — see main.cpp's
// kLoopTimingLogEnabled flag-layout note for why this is its own flag
// rather than shared with anything else.
constexpr bool kSleepDebugLogEnabled = true;

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
// diagnostic blind spot). Confirmed on real hardware afterward: err=0,
// cause=4 (ESP_SLEEP_WAKEUP_TIMER) every single cycle, current a stable
// ~0.82mA — light sleep itself is genuinely healthy now. What it also
// showed: cause was *never* 7 (ESP_SLEEP_WAKEUP_GPIO), across several
// real taps — every wake fell through to the 1s timer backstop instead.
// That's what motivated switching from the tap engine to Wake-on-Motion
// (2026-09-06, wom-wake-mode branch) as the wakeup signal — see
// qmi8658.hpp's EnterWakeOnMotion()/PollWomEvent() and main.cpp's
// idle-sleep block for the IMU-side half of this change; a tap's INT2
// pulse is brief, a WoM event's is a held level, and only the latter is
// the shape light sleep's GPIO wakeup can reliably catch.
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
        // Diagnostic, left in deliberately (2026-09-06; gated behind
        // kSleepDebugLogEnabled 2026-09-07) — cheap (once per real sleep
        // attempt, not per tick), and this is exactly the data needed to
        // tell "rejected" / "slept but GPIO didn't trigger" / other apart
        // on real hardware instead of guessing again.
        if (kSleepDebugLogEnabled) {
            printf("SLEEP,err=%d,cause=%d,int2=%d\n", static_cast<int>(err),
                   static_cast<int>(esp_sleep_get_wakeup_cause()), gpio_get_level(kImuInt2Gpio));
        }

        // Wake-on-Motion (2026-09-06, wom-wake-mode branch — was
        // PollTapEvent()) — gyro is disabled for the duration of this
        // call (see main.cpp's EnterWakeOnMotion() call just before
        // this), so there's no gyro-magnitude check to make here anymore,
        // same as the tap-based version this replaced.
        if (imu.PollWomEvent()) {
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
