#include "calibration_mode.hpp"

#include <cmath>
#include <cstdio>

#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "nvs_calibration.hpp"

namespace {
constexpr int kCountdownSeconds = 5;
constexpr int64_t kSamplingDurationUs = 2 * 1000 * 1000;
constexpr int kSamplingPeriodMs = 10;  // ~100Hz during calibration — plenty for averaging
constexpr int kUiTickMs = 5;           // matches main.cpp's LVGL tick period

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;

// This function blocks the main loop entirely, so nothing else drives
// lv_timer_handler() (the actual render+flush) while it runs — without
// this, SetPrimaryText() updates the LVGL label object in memory but
// never gets pushed to the physical panel; the screen just sits frozen
// on whatever was showing before calibration started, right up until
// esp_restart(). Chop delays into small chunks so the display keeps
// updating throughout.
void DelayWithDisplay(uint32_t total_ms)
{
    uint32_t elapsed = 0;
    while (elapsed < total_ms) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(kUiTickMs));
        elapsed += kUiTickMs;
    }
}
}  // namespace

void RunCalibrationMode(Qmi8658& imu, GuiManager& gui)
{
    // Countdown doubles as a wait-out period for any vibration from
    // releasing BOOT, and gives the user a moment to make sure the
    // device is resting in the face-A reference orientation.
    for (int s = kCountdownSeconds; s > 0; --s) {
        char buf[24];
        std::snprintf(buf, sizeof(buf), "Calib in %d", s);
        gui.SetPrimaryText(buf);
        DelayWithDisplay(1000);
    }

    gui.SetPrimaryText("Hold still...");
    lv_timer_handler();

    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    int sample_count = 0;

    const int64_t sampling_end_us = esp_timer_get_time() + kSamplingDurationUs;
    while (esp_timer_get_time() < sampling_end_us) {
        Qmi8658::Sample sample;
        if (imu.Read(sample)) {
            for (int i = 0; i < 3; ++i) {
                gyro_sum[i] += sample.gyro_dps[i];
                accel_sum[i] += sample.accel_g[i];
            }
            ++sample_count;
        }
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(kSamplingPeriodMs));
    }

    CalibrationData data;
    if (sample_count > 0) {
        for (int i = 0; i < 3; ++i) {
            data.gyro_bias_dps[i] = gyro_sum[i] / sample_count;
        }
        const float avg_ax = accel_sum[0] / sample_count;
        const float avg_ay = accel_sum[1] / sample_count;
        // Same formula AttitudeEstimator uses for its raw accel-derived
        // angle — recording it directly as the offset makes the
        // corrected angle read 0 when the device is next placed in this
        // same (face-A) orientation.
        data.face_a_offset_deg = std::atan2(-avg_ax, -avg_ay) * kRadToDeg;
    } else {
        printf("RunCalibrationMode: no samples collected, saving uncalibrated defaults\n");
    }

    SaveCalibration(data);

    gui.SetPrimaryText("Calibrated\nRebooting...");
    DelayWithDisplay(1000);
    esp_restart();
}
