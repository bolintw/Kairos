#pragma once

#include <cstdint>

// DRAFT — for discussion, not implemented yet (see notes below).
//
// Pure sensor fusion: tracks the device's continuous in-plane rotation
// angle and whether it's currently moving. Deliberately knows nothing
// about "faces" — which angle range means which TimerFace, and the
// hysteresis/debounce needed before switching, are AppController's job
// (it already owns "統一管理姿態判斷（hysteresis）" per the plan). This
// class just answers "what angle, how confidently, is it settled" —
// AppController is the one that decides what that angle *means*.
//
// Device geometry (resolved 2026-08-22): the screen always faces the
// user. Rotating the device means spinning it about the axis pointing at
// the user (perpendicular to the screen), not tipping a face up against
// gravity. So in normal use, gravity stays confined to the screen's own
// plane (AX/AY), and the rotation axis being tracked is GZ (body frame).
// AZ should read ~0g whenever the screen genuinely faces the user; a
// non-zero AZ signals the device is tilted out of that plane (picked up,
// mid-flip, held wrong) — see is_moving below.
//
// This collapses what would otherwise be a full 3D orientation problem
// into a single scalar: an in-plane rotation angle computed as
// atan2(accel_g[0], -accel_g[1]), continuously tracked via a complementary
// filter (GZ integration corrected by this accel-derived angle).
// Deliberately NOT a general 3D attitude estimator (no quaternion/DCM):
// full 3D orientation only matters transiently during the flip itself,
// and mid-flip we don't need smooth angle tracking — we're just waiting
// for motion to settle before AppController re-classifies the face.
//
// Pure computation, no hardware/I2C knowledge — Sample values are passed
// in by whatever owns the sensor (AppController), so this can be unit
// tested on host with literal arrays, no fake/mock driver needed.
class AttitudeEstimator {
public:
    struct Sample {
        float accel_g[3];
        float gyro_dps[3];
    };

    struct Output {
        float screen_angle_deg;  // continuous in-plane rotation, updates
                                  // even while is_moving (screen stays lit
                                  // through a flip, only dims once settled
                                  // and counting — freezing this mid-flip
                                  // would look broken). AppController
                                  // quantizes this into a Face with its
                                  // own hysteresis; this class doesn't
                                  // know what a "face" is.
        bool is_moving;          // true when the full 3-axis gyro
                                  // magnitude exceeds a "settled"
                                  // threshold — mixed rather than GZ-only,
                                  // since nonzero GX/GY usually means the
                                  // device is being picked up rather than
                                  // spun in-plane. If this proves too
                                  // sensitive in practice (e.g. waking
                                  // mid-work from an unrelated bump),
                                  // revisit narrowing back to GZ-only.
        bool in_valid_plane;     // true while |AZ| stays under a
                                  // threshold, i.e. the screen is actually
                                  // facing the user. AppController treats
                                  // false here as "put the screen to
                                  // sleep" — low confidence in
                                  // screen_angle_deg, and not a state
                                  // expected to be held during normal use,
                                  // so there's nothing meaningful left to
                                  // display. This class only reports the
                                  // fact; the sleep *decision* stays in
                                  // AppController, same as Face.
    };

    // Sign convention, confirmed on hardware with the debug overlay's
    // gyro chart:
    //   - screen upright, facing user: accel ~= (0, -1, 0)g,
    //     screen_angle_deg = 0
    //   - rotated 90 deg counter-clockwise (as seen by the user looking
    //     at the screen): accel ~= (+1, 0, 0)g, screen_angle_deg = +90,
    //     and this rotation reads as *negative* GZ (confirmed via the
    //     gyro chart; an earlier guess from raw numbers alone said GX —
    //     the chart corrected that)
    //   - so: screen_angle_deg = atan2(accel_g[0], -accel_g[1]) matches
    //     both points above, and d(screen_angle_deg)/dt = -gyro_dps[2]
    //   - AZ is expected ~0g by construction (see class comment). Its
    //     magnitude drives Output::in_valid_plane (see above): once |AZ|
    //     crosses a threshold, Update() also stops applying the
    //     accel-derived correction that tick and falls back to pure gyro
    //     integration, since accel isn't trustworthy in that state.
    //
    // face_a_offset_deg: per-device mounting calibration (in case the
    // IMU's raw angle-zero isn't exactly where the enclosure's "face A"
    // reference is) — a sensor-calibration concern, not a face-semantics
    // one, so it stays here rather than moving to AppController. Defaults
    // to 0; revisit once real hardware calibration is needed.
    explicit AttitudeEstimator(float face_a_offset_deg = 0.0f);

    // Call once per sensor tick. dt_ms is elapsed time since the previous
    // call, used for both the gyro integration and the complementary
    // filter's time constant. See Output::is_moving and
    // Output::in_valid_plane above for how the 3-axis gyro magnitude and
    // |AZ| thresholds factor in.
    Output Update(const Sample& sample, uint32_t dt_ms);

    // Call while the device is known stationary (e.g. once at boot) to
    // measure gyro bias; Update() subtracts it from all future samples.
    void CalibrateGyroZeroOffset(const Sample& stationary_sample);

private:
    float face_a_offset_deg_;
    float gyro_bias_dps_[3] = {0.0f, 0.0f, 0.0f};
    float angle_deg_ = 0.0f;  // last output angle, already offset-corrected
};
