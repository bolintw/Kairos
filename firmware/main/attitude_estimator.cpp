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
// Lowered from 0.98, then back up from 0.5 (2026-08-23). At 0.98,
// correcting a settle-time error back to accel-derived truth takes
// ln(0.1)/ln(0.98) =~ 114 ticks (~17s at the 150ms sensor period) to
// close 90% of the gap — matched the "十幾秒" convergence observed on
// hardware exactly, and was actively causing misclassified flips since
// AppController reads the angle right at the settle moment. 0.5 fixed
// that (~3 ticks, ~0.5s) but felt too accel-dominant on hardware
// (sensitive to vibration/jitter). 0.8 is a middle ground: ~10 ticks
// (~1.5s) to close 90% of a settle-time error, still far faster than
// 0.98's 17s, while trusting gyro more tick-to-tick than 0.5 did.
// 0.8 means 80% gyro / 20% accel per tick — still gyro-dominant, not
// accel-dominant (worth spelling out: easy to misread "0.8" as "close to
// accel" if recalling a convention where the number labels the other
// sensor). Gyro overshoot/bias will still show through strongly at this
// weighting; that's expected until gyro is properly calibrated (M8).
constexpr float kComplementaryAlpha = 0.8f;  // weight on gyro-integrated angle

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
