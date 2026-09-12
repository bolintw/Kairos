#pragma once

#include <cstdint>

// Pure sensor fusion: tracks the device's continuous in-plane rotation
// angle and whether it's currently moving. Knows nothing about "faces" —
// AppController decides what an angle means; this class only reports it.
//
// Device geometry: the screen always faces the user, and rotating the
// device means spinning it about the axis pointing at the user
// (perpendicular to the screen), not tipping a face up against gravity.
// So gravity stays confined to the screen's own plane (AX/AY) in normal
// use, and the rotation axis being tracked is GZ. AZ should read ~0g
// whenever the screen genuinely faces the user; a nonzero AZ means the
// device is tilted out of that plane (picked up, mid-flip, held wrong).
//
// Collapses what would otherwise be a full 3D orientation problem into a
// single scalar — an in-plane angle from atan2(accel_g[0], -accel_g[1]),
// tracked via a complementary filter (GZ integration corrected by this
// accel-derived angle). Not a general 3D attitude estimator: full 3D
// orientation only matters transiently during the flip itself.
//
// Pure computation, no hardware/I2C knowledge — can be unit tested on
// host with literal Sample arrays.
class AttitudeEstimator {
public:
    struct Sample {
        float accel_g[3];
        float gyro_dps[3];
    };

    struct Output {
        float screen_angle_deg;  // continuous in-plane rotation, updates
                                  // even while is_moving; AppController
                                  // quantizes this into a Face
        bool is_moving;          // true when full 3-axis gyro magnitude
                                  // exceeds a "settled" threshold
        bool in_valid_plane;     // true while |AZ| stays within a
                                  // hysteresis band, i.e. the screen is
                                  // genuinely facing the user; low
                                  // confidence in screen_angle_deg while
                                  // false. AppController also uses a
                                  // sustained false here as the
                                  // "pick the device up" battery-check gesture.

        // Diagnostic-only, for the serial debug log.
        float debug_gyro_only_angle_deg;   // pure gyro integration, no accel correction
        float debug_accel_only_angle_deg;  // this tick's accel-derived angle
        float debug_gz_dps;                // bias-corrected, low-passed gyro Z
    };

    // Sign convention (CCW-positive), confirmed on hardware:
    //   - screen upright, facing user: accel ~= (0, -1, 0)g, angle = 0
    //   - rotated 90 deg counter-clockwise (user's view): accel ~=
    //     (+1, 0, 0)g, angle = +90, and reads as negative GZ
    //   - so: screen_angle_deg = atan2(accel_g[0], -accel_g[1]), and
    //     d(screen_angle_deg)/dt = -gyro_dps[2]
    //   - AZ's magnitude drives Output::in_valid_plane; once |AZ| crosses
    //     the threshold, Update() falls back to pure gyro integration
    //     since accel isn't trustworthy in that state.
    //
    // face_a_offset_deg: per-device mounting calibration, in case the
    // IMU's raw angle-zero isn't exactly where the enclosure's "face A"
    // reference is.
    //
    // accel_bias_g: X/Y zero-offset of the accelerometer itself — a
    // separate, fixed translation error in sensor-frame, distinct from
    // face_a_offset_deg (which only corrects rotation). Subtracted from
    // filtered_accel_g_ before every AngleFromAccel() call.
    explicit AttitudeEstimator(float face_a_offset_deg = 0.0f, float accel_bias_x_g = 0.0f, float accel_bias_y_g = 0.0f);

    // Call once per sensor tick. dt_ms is elapsed time since the previous
    // call, used for both the gyro integration and the complementary
    // filter's time constant.
    Output Update(const Sample& sample, uint32_t dt_ms);

    // Call while the device is known stationary (e.g. once at boot) to
    // measure gyro bias; Update() subtracts it from all future samples.
    void CalibrateGyroZeroOffset(const Sample& stationary_sample);

    // Call once at boot, before the first Update(), with a fresh sample to
    // seed angle_deg_ directly instead of leaving it at 0 (which would
    // otherwise take a moment to converge if booting off face A). No-op
    // if the sample isn't in the valid plane.
    void SeedInitialAngle(const Sample& sample);

private:
    float face_a_offset_deg_;
    float accel_bias_x_g_;
    float accel_bias_y_g_;
    float gyro_bias_dps_[3] = {0.0f, 0.0f, 0.0f};
    float angle_deg_ = 0.0f;  // last output angle, already offset-corrected

    // Single-pole low-pass (EMA) on raw accel/gyro, applied before
    // anything else in Update(). Seeded from the first real sample
    // (has_filtered_sample_) rather than 0 to avoid a startup transient.
    // Accel and gyro use separate time constants (kAccelLowPassTauMs/
    // kGyroLowPassTauMs) — distinct from kComplementaryTauMs, which
    // blends gyro-integration against accel for the output angle.
    float filtered_accel_g_[3] = {0.0f, 0.0f, 0.0f};
    float filtered_gyro_dps_[3] = {0.0f, 0.0f, 0.0f};
    bool has_filtered_sample_ = false;

    // Schmitt-trigger state for Output::in_valid_plane. Defaults true;
    // SeedInitialAngle() re-seeds it if called before the first Update().
    bool in_valid_plane_ = true;
};
