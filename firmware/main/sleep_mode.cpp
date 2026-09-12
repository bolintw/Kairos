#include "sleep_mode.hpp"

#include <cstdio>

#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "debug_config.hpp"

namespace {

constexpr gpio_num_t kImuInt2Gpio = GPIO_NUM_48;

// Ceiling on how long a single esp_light_sleep_start() call may sleep
// before waking on its own regardless of GPIO activity.
constexpr uint32_t kSleepBackstopMs = 1000;

// How often this loop checks battery voltage while otherwise waiting for
// WoM, piggybacked on the same backstop timer.
constexpr int64_t kBatteryCheckIntervalUs = 30LL * 1000 * 1000;

constexpr bool kSleepDebugLogEnabled = kDebugEnabled;

}  // namespace

// Manual esp_light_sleep_start() loop with a real GPIO wakeup source on
// IMU_INT2. No runtime ISR is registered on this pin — gpio_wakeup_enable()
// silently overwrites the pin's hardware intr_type from edge to level,
// and combining it with a separate edge-triggered ISR on the same pin
// causes a livelock if the pin is already high when armed. Calling
// esp_light_sleep_start() directly and synchronously here (rather than
// through ESP-IDF's automatic PM/tickless-idle path) also means err and
// esp_sleep_get_wakeup_cause() are read right where they're produced.
//
// The wakeup signal is Wake-on-Motion, not the tap engine: a tap's INT2
// pulse is too brief for light sleep's GPIO wakeup to catch reliably,
// while a WoM event holds the line until read — see
// qmi8658.hpp's EnterWakeOnMotion()/PollWomEvent().
IdleSleepWakeReason RunIdleSleep(Qmi8658& imu, BatteryMonitor& battery_monitor, float critical_battery_v)
{
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << kImuInt2Gpio;
    cfg.mode = GPIO_MODE_INPUT;
    // Defensive pull-down opposite the wakeup trigger, for any brief
    // window where the (normally push-pull) pin isn't actively driven.
    cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&cfg);

    gpio_wakeup_enable(kImuInt2Gpio, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_sleep_enable_timer_wakeup(kSleepBackstopMs * 1000);

    IdleSleepWakeReason wake_reason = IdleSleepWakeReason::kMotion;
    int64_t next_battery_check_us = esp_timer_get_time() + kBatteryCheckIntervalUs;

    while (true) {
        const esp_err_t err = esp_light_sleep_start();
        if (kSleepDebugLogEnabled) {
            printf("SLEEP,err=%d,cause=%d,int2=%d\n", static_cast<int>(err),
                   static_cast<int>(esp_sleep_get_wakeup_cause()), gpio_get_level(kImuInt2Gpio));
        }

        if (imu.PollWomEvent()) {
            wake_reason = IdleSleepWakeReason::kMotion;
            break;
        }

        // Piggyback the battery check on this same backstop wake, only
        // actually reading the ADC every kBatteryCheckIntervalUs.
        const int64_t now_us = esp_timer_get_time();
        if (now_us >= next_battery_check_us) {
            next_battery_check_us = now_us + kBatteryCheckIntervalUs;
            if (battery_monitor.ReadVoltage() < critical_battery_v) {
                wake_reason = IdleSleepWakeReason::kCriticalBattery;
                break;
            }
        }

        // err != ESP_OK means esp_light_sleep_start() returned without
        // actually sleeping; bound the retry to a plain poll instead of
        // busy-spinning.
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    gpio_wakeup_disable(kImuInt2Gpio);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    return wake_reason;
}
