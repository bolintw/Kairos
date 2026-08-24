#pragma once

// Calibration data persisted across reboots via NVS. Both fields default
// to "uncalibrated" (zero) so a fresh device before assembly-time
// calibration behaves the same as it always has — no bias correction,
// face_a_offset_deg=0 — rather than crashing or refusing to run.
struct CalibrationData {
    float face_a_offset_deg = 0.0f;
    float gyro_bias_dps[3] = {0.0f, 0.0f, 0.0f};
};

// Returns false if nothing has been saved yet (expected before the first
// assembly-time calibration run) — caller should fall back to a
// default-constructed CalibrationData in that case.
bool LoadCalibration(CalibrationData& out);

void SaveCalibration(const CalibrationData& data);
