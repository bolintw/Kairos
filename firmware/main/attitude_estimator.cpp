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

AttitudeEstimator::Output AttitudeEstimator::Update(const Sample& sample, uint32_t dt_ms)
{
    const float gx = sample.gyro_dps[0] - gyro_bias_dps_[0];
    const float gy = sample.gyro_dps[1] - gyro_bias_dps_[1];
    const float gz = sample.gyro_dps[2] - gyro_bias_dps_[2];

    // Sign flipped 2026-08-23 (was CCW=positive): CCW rotation (as seen
    // by the user) now *decreases* screen_angle_deg, CW increases it.
    // The underlying hardware fact is unchanged — CCW still reads as
    // negative GZ — only which direction we call "positive" changed, so
    // both the gyro term and the accel formula's sign flip together.
    const float angle_from_gyro =
        WrapDeg180(angle_deg_ + gz * (static_cast<float>(dt_ms) / 1000.0f));

    const bool in_valid_plane = std::fabs(sample.accel_g[2]) < kAzInvalidThresholdG;

    if (in_valid_plane) {
        const float raw_accel_angle =
            std::atan2(-sample.accel_g[0], -sample.accel_g[1]) * kRadToDeg;
        const float angle_from_accel = WrapDeg180(raw_accel_angle - face_a_offset_deg_);
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
