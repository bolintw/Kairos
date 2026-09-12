#include "attitude_estimator.hpp"

#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;

constexpr float kGyroMovingThresholdDps = 20.0f;

// Hysteresis band for the "picked up / tilted out of the resting plane"
// signal — also drives the battery-check gesture in AppController. Split
// into enter/return thresholds so a single bounce at the boundary doesn't
// reset AppController's multi-second hold timer.
constexpr float kAzInvalidEnterThresholdG = 0.9f;
constexpr float kAzValidReturnThresholdG = 0.5f;

// Adaptive complementary filter: accel's correction weight ramps from
// full trust near-stationary down to ~0 while actively rotating, since
// filtered_accel_g_'s heavy smoothing makes it lag further behind the
// true angle the faster the device turns — blending toward a stale value
// during a fast flip otherwise reads as undershoot + a damped ease-in
// once it stops.
constexpr float kGyroSpeedFullTrustDps = 5.0f;   // at/below: full accel trust (near-stationary)
constexpr float kGyroSpeedZeroTrustDps = 60.0f;  // at/above: trust gyro alone

// Single-pole low-pass (EMA) time constants — tau (ms), not a flat
// per-tick alpha, so filtering stays time-correct regardless of the
// actual sensor tick rate; alpha at a given dt comes from ExpAlpha()
// below. Gyro is filtered lightly (drives felt rotation responsiveness);
// accel is filtered heavily (only ever contributes a fraction of a
// correction, so it can afford to reject more vibration/shock noise).
constexpr float kGyroLowPassTauMs = 6.0f;
constexpr float kAccelLowPassTauMs = 37.35f;
// Complementary blend's at-rest accel weight — same tau as
// kAccelLowPassTauMs today, kept as its own constant since it governs a
// conceptually different thing and may end up tuned independently.
constexpr float kComplementaryTauMs = 37.35f;

float ExpAlpha(float dt_ms, float tau_ms)
{
    return 1.0f - std::exp(-dt_ms / tau_ms);
}

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
// distance between them, avoiding the +-180 wraparound issue a plain
// linear blend would have.
float BlendTowardAngle(float base, float target, float target_weight)
{
    const float diff = WrapDeg180(target - base);
    return WrapDeg180(base + target_weight * diff);
}

float GyroMagnitudeDps(float gx, float gy, float gz)
{
    return std::sqrt(gx * gx + gy * gy + gz * gz);
}

// Linear ramp from full trust at/below kGyroSpeedFullTrustDps down to
// zero at/above kGyroSpeedZeroTrustDps. gz_abs_dps is the in-plane
// rotation axis specifically, not the 3-axis magnitude used for
// is_moving (which also picks up off-axis GX/GY from being picked up).
float AccelTrustWeight(float gz_abs_dps, float dt_ms)
{
    float trust = 1.0f - (gz_abs_dps - kGyroSpeedFullTrustDps) /
                              (kGyroSpeedZeroTrustDps - kGyroSpeedFullTrustDps);
    if (trust < 0.0f) trust = 0.0f;
    if (trust > 1.0f) trust = 1.0f;
    return ExpAlpha(dt_ms, kComplementaryTauMs) * trust;
}

// Shared by Update() and SeedInitialAngle() — see the header's
// sign-convention comment. Takes ax/ay directly so Update() can pass its
// low-pass-filtered values.
float AngleFromAccel(float ax, float ay, float face_a_offset_deg)
{
    const float raw_accel_angle = std::atan2(ax, -ay) * kRadToDeg;
    return WrapDeg180(raw_accel_angle + face_a_offset_deg);
}

}  // namespace

AttitudeEstimator::AttitudeEstimator(float face_a_offset_deg, float accel_bias_x_g, float accel_bias_y_g)
    : face_a_offset_deg_(face_a_offset_deg),
      accel_bias_x_g_(accel_bias_x_g),
      accel_bias_y_g_(accel_bias_y_g)
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
    const bool in_valid_plane = std::fabs(sample.accel_g[2]) < kAzValidReturnThresholdG;
    in_valid_plane_ = in_valid_plane;
    if (!in_valid_plane) {
        return;  // leave angle_deg_ at its default; Update() converges normally
    }
    angle_deg_ = AngleFromAccel(sample.accel_g[0] - accel_bias_x_g_, sample.accel_g[1] - accel_bias_y_g_,
                                 face_a_offset_deg_);
}

AttitudeEstimator::Output AttitudeEstimator::Update(const Sample& sample, uint32_t dt_ms)
{
    const float dt_ms_f = static_cast<float>(dt_ms);

    // Low-pass the raw sample first — everything below reads the
    // filtered values, never the raw sample directly.
    if (has_filtered_sample_) {
        const float accel_alpha = ExpAlpha(dt_ms_f, kAccelLowPassTauMs);
        const float gyro_alpha = ExpAlpha(dt_ms_f, kGyroLowPassTauMs);
        for (int i = 0; i < 3; ++i) {
            filtered_accel_g_[i] = LowPass(sample.accel_g[i], filtered_accel_g_[i], accel_alpha);
            filtered_gyro_dps_[i] = LowPass(sample.gyro_dps[i], filtered_gyro_dps_[i], gyro_alpha);
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

    // CCW rotation (user's view) increases screen_angle_deg; CCW reads as
    // negative GZ, hence the subtraction (see header's sign convention).
    const float angle_from_gyro =
        WrapDeg180(angle_deg_ - gz * (static_cast<float>(dt_ms) / 1000.0f));

    // Schmitt-trigger hysteresis on |AZ|.
    if (in_valid_plane_) {
        if (std::fabs(filtered_accel_g_[2]) >= kAzInvalidEnterThresholdG) {
            in_valid_plane_ = false;
        }
    } else {
        if (std::fabs(filtered_accel_g_[2]) < kAzValidReturnThresholdG) {
            in_valid_plane_ = true;
        }
    }
    const bool in_valid_plane = in_valid_plane_;

    // Computed unconditionally (even if !in_valid_plane) so the debug log
    // can show what accel was reading through a flip.
    const float angle_from_accel = AngleFromAccel(filtered_accel_g_[0] - accel_bias_x_g_,
                                                    filtered_accel_g_[1] - accel_bias_y_g_, face_a_offset_deg_);

    if (in_valid_plane) {
        angle_deg_ = BlendTowardAngle(angle_from_gyro, angle_from_accel, AccelTrustWeight(std::fabs(gz), dt_ms_f));
    } else {
        // Accel isn't trustworthy while the screen isn't facing the
        // user — fall back to pure gyro integration for this tick.
        angle_deg_ = angle_from_gyro;
    }

    Output out;
    out.screen_angle_deg = angle_deg_;
    out.is_moving = GyroMagnitudeDps(gx, gy, gz) > kGyroMovingThresholdDps;
    out.in_valid_plane = in_valid_plane;
    out.debug_gyro_only_angle_deg = angle_from_gyro;
    out.debug_accel_only_angle_deg = angle_from_accel;
    out.debug_gz_dps = gz;
    return out;
}
