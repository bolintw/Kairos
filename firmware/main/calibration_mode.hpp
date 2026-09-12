#pragma once

#include "gui_manager.hpp"
#include "qmi8658.hpp"

// Blocking assembly-time calibration routine, entered by holding BOOT for
// a few seconds while the app is already running (not at power-on/reset —
// GPIO0 is a strapping pin, so holding it at reset boots into the ROM
// UART downloader instead of the app).
//
// Flow: countdown -> sample gyro+accel while still -> average into a
// gyro zero-offset and a face-A angle offset -> save to NVS ->
// esp_restart(). Never returns; AttitudeEstimator picks up the new
// values on the next boot.
void RunCalibrationMode(Qmi8658& imu, GuiManager& gui);
