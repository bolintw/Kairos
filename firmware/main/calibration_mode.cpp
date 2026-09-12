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
// Distinct from every TimerFace accent color so entering calibration
// reads as a real mode switch rather than inheriting whatever color was
// showing right before BOOT was held.
const lv_color_t kColorCalibration = lv_color_make(80, 220, 220);

constexpr int kCountdownSeconds = 10;
constexpr int64_t kSamplingDurationUs = 2 * 1000 * 1000;
constexpr int kSamplingPeriodMs = 10;  // ~100Hz during calibration — plenty for averaging
constexpr int kUiTickMs = 5;           // matches main.cpp's LVGL tick period

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;

// Accel bias phase — see nvs_calibration.hpp's accel_bias_g comment for
// what this corrects. kNumBiasPoints positions, spread far enough apart
// (kMinAngularSeparationDeg) to be meaningfully different resting
// orientations. kGyroStillThresholdDps/kStillnessWindowUs gate each
// capture on genuine stillness first — real dynamic acceleration during
// motion (hand tremor, uneven turning speed) would otherwise contaminate
// the samples.
//
// The first accepted point doubles as the face_a_offset_deg reference —
// it's captured right where the countdown in RunCalibrationMode left the
// device resting, so "face A" is whatever orientation the user settled
// into before calibration started, not wherever the flip sequence ends.
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

// Deliberately the *old* atan2(-ax,-ay) convention — see
// attitude_estimator.cpp's AngleFromAccel comment. This function's raw
// output is stored directly as face_a_offset_deg, and AttitudeEstimator's
// formula was specifically derived to consume an offset computed this
// old way correctly. Every *live rotation preview* caller needs the
// *new* convention instead (so text stays upright with GuiManager's
// kRotationSign), and negates this function's result at its own call
// site rather than this function changing — see
// DelayWithDisplayAndRotation and CollectAccelBias below. All such call
// sites must apply the negation — a missed one only shows up as wrong
// spin direction at 90/270 degrees, not 0/180, since negation is a no-op there.
float RawAccelAngleDeg(float ax, float ay)
{
    return std::atan2(-ax, -ay) * kRadToDeg;
}

// Same as DelayWithDisplay, but also keeps root_ counter-rotated to the
// device's live raw angle — used during the countdown since the user may
// still be handling/mounting the device through it, not just waiting.
//
// Uses esp_timer_get_time() for the actual elapsed time rather than
// DelayWithDisplay's "add a nominal kUiTickMs per loop" shortcut — that
// only works when every loop body costs about the same, and here
// imu.Read() (I2C) plus the rotation-triggered LVGL redraw make each
// iteration's real cost variable.
void DelayWithDisplayAndRotation(uint32_t total_ms, Qmi8658& imu, GuiManager& gui)
{
    const int64_t start_us = esp_timer_get_time();
    while (esp_timer_get_time() - start_us < static_cast<int64_t>(total_ms) * 1000) {
        Qmi8658::Sample sample;
        if (imu.Read(sample)) {
            gui.SetRotationDeg(-RawAccelAngleDeg(sample.accel_g[0], sample.accel_g[1]));
        }
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(kUiTickMs));
    }
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

struct BiasCollectionResult {
    float bias_x;
    float bias_y;
    // Raw (uncorrected) accel at the first accepted point — doubles as
    // the face_a_offset_deg reference (see kNumBiasPoints above).
    // RunCalibrationMode bias-corrects it before computing the offset,
    // same as any other point.
    float ref_ax;
    float ref_ay;
    // Averaged across every stationary sampling window in the whole
    // process, not just one — gyro bias is intrinsic to the sensor
    // electronics, not orientation-dependent, so every point's stillness
    // window is equally valid data for it.
    float gyro_bias_dps[3];
};

// Blocks until it has captured kNumBiasPoints stationary, sufficiently
// separated accel readings and returns their average plus the other
// derived values. Keeps root_ counter-rotated to the device's live raw
// angle throughout so the on-screen prompts stay upright no matter which
// face is currently up — uses the raw (un-offset) angle since
// face_a_offset_deg isn't known yet at this point, fine since this is
// purely for on-screen legibility, not a calibrated reading.
BiasCollectionResult CollectAccelBias(Qmi8658& imu, GuiManager& gui)
{
    // The device should already be resting where RunCalibrationMode's
    // countdown just left it, so there's nothing to rotate into yet.
    gui.SetPrimaryText("Hold still...");
    lv_timer_handler();

    float captured_deg[kNumBiasPoints];
    float sum_ax = 0.0f;
    float sum_ay = 0.0f;
    float ref_ax = 0.0f;
    float ref_ay = 0.0f;
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    int gyro_sample_count = 0;
    int captured = 0;
    int64_t still_since_us = -1;  // < 0 means "not currently settled"

    while (captured < kNumBiasPoints) {
        Qmi8658::Sample sample;
        const bool got_sample = imu.Read(sample);
        if (got_sample) {
            gui.SetRotationDeg(-RawAccelAngleDeg(sample.accel_g[0], sample.accel_g[1]));
        }
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
        // duration/rate as the original single-point calibration used.
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
                gyro_sum[0] += point_sample.gyro_dps[0];
                gyro_sum[1] += point_sample.gyro_dps[1];
                gyro_sum[2] += point_sample.gyro_dps[2];
                ++point_samples;
                ++gyro_sample_count;
                gui.SetRotationDeg(-RawAccelAngleDeg(point_sample.accel_g[0], point_sample.accel_g[1]));
            }
            lv_timer_handler();
            vTaskDelay(pdMS_TO_TICKS(kSamplingPeriodMs));
        }

        if (point_samples > 0) {
            const float avg_ax = point_sum_ax / point_samples;
            const float avg_ay = point_sum_ay / point_samples;
            if (captured == 0) {
                ref_ax = avg_ax;
                ref_ay = avg_ay;
            }
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
        if (captured < kNumBiasPoints) {
            // Two lines — one line ("Rotate & hold") runs past root_'s
            // kRootWidthPx at kPrimaryFont's size and clips off-screen.
            gui.SetPrimaryText("Rotate\nand hold");
        }
    }

    // See nvs_calibration.hpp's accel_bias_g comment for the math: the
    // average of kNumBiasPoints readings spread around the rotation
    // cancels the true (rotating) signal and leaves the fixed offset.
    BiasCollectionResult result;
    result.bias_x = sum_ax / kNumBiasPoints;
    result.bias_y = sum_ay / kNumBiasPoints;
    result.ref_ax = ref_ax;
    result.ref_ay = ref_ay;
    for (int i = 0; i < 3; ++i) {
        result.gyro_bias_dps[i] = gyro_sample_count > 0 ? gyro_sum[i] / gyro_sample_count : 0.0f;
    }
    return result;
}
}  // namespace

void RunCalibrationMode(Qmi8658& imu, GuiManager& gui)
{
    gui.SetAccentColor(kColorCalibration);

    // Countdown runs first — the window for the user to mount the device
    // back into the enclosure and settle it in whatever orientation
    // should become the rotational reference ("face A"), and a wait-out
    // period for any vibration from releasing BOOT. Running it after the
    // accel-bias flip sequence instead would let the reference lock onto
    // wherever the flips happen to end, not what the user meant as face A.
    for (int s = kCountdownSeconds; s > 0; --s) {
        char buf[24];
        std::snprintf(buf, sizeof(buf), "Calib in %d", s);
        gui.SetPrimaryText(buf);
        DelayWithDisplayAndRotation(1000, imu, gui);
    }

    const BiasCollectionResult bias = CollectAccelBias(imu, gui);

    CalibrationData data;
    data.accel_bias_g[0] = bias.bias_x;
    data.accel_bias_g[1] = bias.bias_y;
    for (int i = 0; i < 3; ++i) {
        data.gyro_bias_dps[i] = bias.gyro_bias_dps[i];
    }
    // Bias-corrected before computing the rotational offset. Uses
    // RawAccelAngleDeg's old-convention formula, not AttitudeEstimator's
    // AngleFromAccel — recording this old-convention raw angle directly
    // as the offset is what makes the corrected angle read 0 back at
    // this same reference orientation, given AngleFromAccel's offset-ADDED formula.
    data.face_a_offset_deg = RawAccelAngleDeg(bias.ref_ax - bias.bias_x, bias.ref_ay - bias.bias_y);

    SaveCalibration(data);

    gui.SetPrimaryText("Calibrated\nRebooting...");
    DelayWithDisplay(1000);
    esp_restart();
}
