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
constexpr float kComplementaryAlpha = 0.98f;  // weight on gyro-integrated angle

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

    // Sign per the header's confirmed convention: CCW rotation (as seen
    // by the user) increases screen_angle_deg but reads as *negative* GZ.
    const float angle_from_gyro =
        WrapDeg180(angle_deg_ + (-gz) * (static_cast<float>(dt_ms) / 1000.0f));

    const bool in_valid_plane = std::fabs(sample.accel_g[2]) < kAzInvalidThresholdG;

    if (in_valid_plane) {
        const float raw_accel_angle =
            std::atan2(sample.accel_g[0], -sample.accel_g[1]) * kRadToDeg;
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
