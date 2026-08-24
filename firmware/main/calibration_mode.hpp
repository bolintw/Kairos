#pragma once

#include "gui_manager.hpp"
#include "qmi8658.hpp"

// Blocking assembly-time calibration routine, entered by holding BOOT for
// a few seconds while the app is already running (NOT at power-on/reset —
// GPIO0 is a strapping pin, so holding it at reset puts the chip into the
// ROM UART download bootloader instead of running the app at all; see
// main.cpp's long-press detection, which only starts counting once
// app_main() is already looping).
//
// Flow: countdown (also absorbs any vibration from releasing the
// button) -> sample gyro+accel for a few seconds while the device sits
// still -> average into a gyro zero-offset and a face-A angle offset ->
// save to NVS -> esp_restart(). Never returns — AttitudeEstimator is
// rebuilt from the freshly-saved values on the next boot rather than
// this function updating a live instance in place, keeping
// AttitudeEstimator's construct-then-immutable design unchanged.
void RunCalibrationMode(Qmi8658& imu, GuiManager& gui);
