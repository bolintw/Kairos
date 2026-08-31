#pragma once

// Calibration data persisted across reboots via NVS. All fields default
// to "uncalibrated" (zero) so a fresh device before assembly-time
// calibration behaves the same as it always has — no bias correction,
// face_a_offset_deg=0 — rather than crashing or refusing to run.
struct CalibrationData {
    float face_a_offset_deg = 0.0f;
    float gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f};
    // Accelerometer X/Y zero-offset (2026-08-31) — a separate error from
    // gyro_bias_dps above: a fixed offset in the accel's own electronics,
    // constant in sensor-frame regardless of orientation, distinct from
    // face_a_offset_deg (which only corrects *rotation*, i.e. where angle
    // 0 points). Left uncorrected, it shows up as a face-dependent angle
    // error rather than a uniform one — see RunCalibrationMode for how
    // it's measured (a multi-orientation circle-center fit, not a single
    // reading) and attitude_estimator.cpp for where it's applied (before
    // face_a_offset_deg, before atan2). Only X/Y: AngleFromAccel never
    // reads Z, so a Z bias wouldn't affect screen_angle_deg at all.
    float accel_bias_g[2] = {0.0f, 0.0f};
};

// Returns false if nothing has been saved yet (expected before the first
// assembly-time calibration run) — caller should fall back to a
// default-constructed CalibrationData in that case.
bool LoadCalibration(CalibrationData& out);

void SaveCalibration(const CalibrationData& data);
