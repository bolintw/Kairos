#include "attitude_estimator.hpp"

#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;

// Tuning starting points, not yet validated against real usage — see the
// header's OPEN items this resolves. Adjust while watching the debug
// overlay and re-flashing.
constexpr float kGyroMovingThresholdDps = 20.0f;
constexpr float kAzInvalidThresholdG = 0.3f;
// History: 0.98 -> 0.5 -> 0.8 -> 0.95 -> 0.9 (2026-08-24, final for now).
// 0.98 took ~17s to converge (matched the "十幾秒" hardware report). 0.5
// converged fast (~0.5s) but felt too accel-dominant/vibration-sensitive.
// 0.8 was a middle ground (~1.5s). Once the gyro full-scale-range bug was
// fixed (CTRL3 scale now correctly matches +-256dps, see qmi8658.hpp),
// the overshoot that motivated trusting accel more went away, and 0.95
// felt close to the user's prior flight-controller tuning experience —
// but tuned down slightly to 0.9 on further hardware testing, still
// gyro-dominant (90% gyro / 10% accel per tick) but with a touch more
// accel correction than 0.95.
constexpr float kComplementaryAlpha = 0.9f;  // weight on gyro-integrated angle

// Single-pole low-pass on the raw samples — see the header's field
// comment. 0.8 -> 0.2 -> 0.4 (2026-08-25, all same day). 0.8 was
// backwards from the intended smoothing strength (~5ms tau, barely any
// effect). 0.2 (~37ms tau) visibly steadied the resting jitter but made
// active rotation noticeably laggier. 0.4 (~16ms tau) trades back some
// smoothing for responsiveness — adjust further in either direction on
// hardware if this balance isn't right.
constexpr float kLowPassAlpha = 0.4f;

float LowPass(float new_x, float old_x, float alpha)
{
    return alpha * new_x + (1.0f - alpha) * old_x;
}

float WrapDeg180(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// Nudges `base` toward `target` by `target_weight` of the shortest angular
// distance between them. A plain linear blend of two raw angles breaks
// near the +-180 wraparound (e.g. averaging +179 and -179 gives ~0,
// instead of ~180); going through the wrapped difference avoids that.
float BlendTowardAngle(float base, float target, float target_weight)
{
    const float diff = WrapDeg180(target - base);
    return WrapDeg180(base + target_weight * diff);
}

float GyroMagnitudeDps(float gx, float gy, float gz)
{
    return std::sqrt(gx * gx + gy * gy + gz * gz);
}

// Shared by Update() and SeedInitialAngle() — same formula, see the
// header's sign-convention comment. Takes ax/ay directly (not a Sample)
// so Update() can pass its low-pass-filtered values.
float AngleFromAccel(float ax, float ay, float face_a_offset_deg)
{
    const float raw_accel_angle = std::atan2(-ax, -ay) * kRadToDeg;
    return WrapDeg180(raw_accel_angle - face_a_offset_deg);
}

}  // namespace

AttitudeEstimator::AttitudeEstimator(float face_a_offset_deg)
    : face_a_offset_deg_(face_a_offset_deg)
{
}

void AttitudeEstimator::CalibrateGyroZeroOffset(const Sample& stationary_sample)
{
    gyro_bias_dps_[0] = stationary_sample.gyro_dps[0];
    gyro_bias_dps_[1] = stationary_sample.gyro_dps[1];
    gyro_bias_dps_[2] = stationary_sample.gyro_dps[2];
}

void AttitudeEstimator::SeedInitialAngle(const Sample& sample)
{
    const bool in_valid_plane = std::fabs(sample.accel_g[2]) < kAzInvalidThresholdG;
    if (!in_valid_plane) {
        return;  // leave angle_deg_ at its default; Update() converges normally
    }
    angle_deg_ = AngleFromAccel(sample.accel_g[0], sample.accel_g[1], face_a_offset_deg_);
}

AttitudeEstimator::Output AttitudeEstimator::Update(const Sample& sample, uint32_t dt_ms)
{
    // Low-pass the raw sample first — everything below reads the
    // filtered values, never the raw sample directly. See the header's
    // field comment for why (and for kComplementaryAlpha vs
    // kLowPassAlpha being two different things).
    if (has_filtered_sample_) {
        for (int i = 0; i < 3; ++i) {
            filtered_accel_g_[i] = LowPass(sample.accel_g[i], filtered_accel_g_[i], kLowPassAlpha);
            filtered_gyro_dps_[i] = LowPass(sample.gyro_dps[i], filtered_gyro_dps_[i], kLowPassAlpha);
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            filtered_accel_g_[i] = sample.accel_g[i];
            filtered_gyro_dps_[i] = sample.gyro_dps[i];
        }
        has_filtered_sample_ = true;
    }

    const float gx = filtered_gyro_dps_[0] - gyro_bias_dps_[0];
    const float gy = filtered_gyro_dps_[1] - gyro_bias_dps_[1];
    const float gz = filtered_gyro_dps_[2] - gyro_bias_dps_[2];

    // Sign flipped 2026-08-23 (was CCW=positive): CCW rotation (as seen
    // by the user) now *decreases* screen_angle_deg, CW increases it.
    // The underlying hardware fact is unchanged — CCW still reads as
    // negative GZ — only which direction we call "positive" changed, so
    // both the gyro term and the accel formula's sign flip together.
    const float angle_from_gyro =
        WrapDeg180(angle_deg_ + gz * (static_cast<float>(dt_ms) / 1000.0f));

    const bool in_valid_plane = std::fabs(filtered_accel_g_[2]) < kAzInvalidThresholdG;

    if (in_valid_plane) {
        const float angle_from_accel = AngleFromAccel(filtered_accel_g_[0], filtered_accel_g_[1], face_a_offset_deg_);
        angle_deg_ = BlendTowardAngle(angle_from_gyro, angle_from_accel, 1.0f - kComplementaryAlpha);
    } else {
        // Accel isn't trustworthy while the screen isn't facing the
        // user — fall back to pure gyro integration for this tick.
        angle_deg_ = angle_from_gyro;
    }

    Output out;
    out.screen_angle_deg = angle_deg_;
    out.is_moving = GyroMagnitudeDps(gx, gy, gz) > kGyroMovingThresholdDps;
    out.in_valid_plane = in_valid_plane;
    return out;
}
