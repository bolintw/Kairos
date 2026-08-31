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
constexpr int kCountdownSeconds = 10;
constexpr int64_t kSamplingDurationUs = 2 * 1000 * 1000;
constexpr int kSamplingPeriodMs = 10;  // ~100Hz during calibration — plenty for averaging
constexpr int kUiTickMs = 5;           // matches main.cpp's LVGL tick period

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;

// Accel bias phase (2026-08-31) — see nvs_calibration.hpp's accel_bias_g
// comment for what this corrects and why. kNumBiasPoints positions,
// spread far enough apart (kMinAngularSeparationDeg) that they're
// meaningfully different resting orientations, not the same spot
// re-measured — but with no notion of "which face" any of them is; the
// bias math (see the averaging below) only needs 4 points roughly 90
// degrees apart, not specific labels. kGyroStillThresholdDps/
// kStillnessWindowUs gate each capture on genuine stillness first — an
// earlier design considered just having the user slowly spin through one
// continuous turn, rejected before implementing: real dynamic
// acceleration during motion (hand tremor, uneven turning speed) would
// have contaminated the samples, the same reason the original single-
// point calibration below insists on "hold still" rather than reading
// mid-motion.
constexpr int kNumBiasPoints = 4;
constexpr float kGyroStillThresholdDps = 20.0f;    // matches attitude_estimator's kGyroMovingThresholdDps
constexpr int64_t kStillnessWindowUs = 400 * 1000;  // sustained stillness required before a capture starts
constexpr float kMinAngularSeparationDeg = 45.0f;   // reject a capture too close to one already accepted

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

float RawAccelAngleDeg(float ax, float ay)
{
    return std::atan2(-ax, -ay) * kRadToDeg;
}

// Shortest angular distance between two angles, always >= 0 — same
// formula as app_controller.cpp's AngularDistanceDeg, kept as its own
// small copy here rather than shared: both are pure, self-contained, and
// not worth cross-module coupling over.
float AngularDistanceDeg(float a_deg, float b_deg)
{
    float diff = a_deg - b_deg;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return std::fabs(diff);
}

// Blocks until it has captured kNumBiasPoints stationary, sufficiently
// separated accel readings and returns their average (ax, ay) — see the
// constants' comment above. Writes into out_bias_x/out_bias_y directly
// rather than returning a struct; the caller has no other state to bundle
// this with.
void CollectAccelBias(Qmi8658& imu, GuiManager& gui, float& out_bias_x, float& out_bias_y)
{
    gui.SetPrimaryText("Rotate & hold");
    lv_timer_handler();

    float captured_deg[kNumBiasPoints];
    float sum_ax = 0.0f;
    float sum_ay = 0.0f;
    int captured = 0;
    int64_t still_since_us = -1;  // < 0 means "not currently settled"

    while (captured < kNumBiasPoints) {
        Qmi8658::Sample sample;
        const bool got_sample = imu.Read(sample);
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(kSamplingPeriodMs));
        if (!got_sample) continue;

        const float gyro_mag_dps = std::sqrt(sample.gyro_dps[0] * sample.gyro_dps[0] +
                                              sample.gyro_dps[1] * sample.gyro_dps[1] +
                                              sample.gyro_dps[2] * sample.gyro_dps[2]);
        const int64_t now_us = esp_timer_get_time();

        if (gyro_mag_dps > kGyroStillThresholdDps) {
            still_since_us = -1;  // moving — any settle timer so far doesn't count
            continue;
        }
        if (still_since_us < 0) {
            still_since_us = now_us;  // just went still
        }
        if (now_us - still_since_us < kStillnessWindowUs) {
            continue;  // still, but not for long enough yet
        }

        // Settled. Reject if this is basically the same position as one
        // already captured (user hasn't actually moved to a new spot).
        const float candidate_deg = RawAccelAngleDeg(sample.accel_g[0], sample.accel_g[1]);
        bool too_close = false;
        for (int i = 0; i < captured; ++i) {
            if (AngularDistanceDeg(candidate_deg, captured_deg[i]) < kMinAngularSeparationDeg) {
                too_close = true;
                break;
            }
        }
        if (too_close) continue;

        // New position, genuinely settled — average over the same
        // duration/rate as the single-point calibration below.
        gui.SetPrimaryText("Hold still...");
        lv_timer_handler();
        float point_sum_ax = 0.0f;
        float point_sum_ay = 0.0f;
        int point_samples = 0;
        const int64_t sampling_end_us = esp_timer_get_time() + kSamplingDurationUs;
        while (esp_timer_get_time() < sampling_end_us) {
            Qmi8658::Sample point_sample;
            if (imu.Read(point_sample)) {
                point_sum_ax += point_sample.accel_g[0];
                point_sum_ay += point_sample.accel_g[1];
                ++point_samples;
            }
            lv_timer_handler();
            vTaskDelay(pdMS_TO_TICKS(kSamplingPeriodMs));
        }

        if (point_samples > 0) {
            const float avg_ax = point_sum_ax / point_samples;
            const float avg_ay = point_sum_ay / point_samples;
            captured_deg[captured] = RawAccelAngleDeg(avg_ax, avg_ay);
            sum_ax += avg_ax;
            sum_ay += avg_ay;
            ++captured;

            // Sized well past the real single-digit range in use — GCC's
            // -Wformat-truncation reasons about %d's worst case for an
            // int parameter, not its actual runtime values here.
            char buf[24];
            std::snprintf(buf, sizeof(buf), "Got %d/%d", captured, kNumBiasPoints);
            gui.SetPrimaryText(buf);
            DelayWithDisplay(500);
        }
        still_since_us = -1;  // require a fresh settle before the next point
        gui.SetPrimaryText("Rotate & hold");
    }

    // See nvs_calibration.hpp's accel_bias_g comment for the math: the
    // average of kNumBiasPoints readings spread around the rotation
    // cancels the true (rotating) signal and leaves the fixed offset.
    out_bias_x = sum_ax / kNumBiasPoints;
    out_bias_y = sum_ay / kNumBiasPoints;
}
}  // namespace

void RunCalibrationMode(Qmi8658& imu, GuiManager& gui)
{
    float accel_bias_x = 0.0f;
    float accel_bias_y = 0.0f;
    CollectAccelBias(imu, gui, accel_bias_x, accel_bias_y);

    // Countdown doubles as a wait-out period for any vibration from
    // releasing BOOT (or, now, from the last accel-bias capture), and
    // gives the user a moment to make sure the device is resting in the
    // face-A reference orientation.
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
    data.accel_bias_g[0] = accel_bias_x;
    data.accel_bias_g[1] = accel_bias_y;
    if (sample_count > 0) {
        for (int i = 0; i < 3; ++i) {
            data.gyro_bias_dps[i] = gyro_sum[i] / sample_count;
        }
        // Bias-corrected before computing the rotational offset — same
        // formula AttitudeEstimator uses for its raw accel-derived angle
        // (see AngleFromAccel), recording it directly as the offset makes
        // the corrected angle read 0 when the device is next placed in
        // this same (face-A) orientation.
        const float avg_ax = accel_sum[0] / sample_count - accel_bias_x;
        const float avg_ay = accel_sum[1] / sample_count - accel_bias_y;
        data.face_a_offset_deg = RawAccelAngleDeg(avg_ax, avg_ay);
    } else {
        printf("RunCalibrationMode: no samples collected, saving uncalibrated defaults\n");
    }

    SaveCalibration(data);

    gui.SetPrimaryText("Calibrated\nRebooting...");
    DelayWithDisplay(1000);
    esp_restart();
}
