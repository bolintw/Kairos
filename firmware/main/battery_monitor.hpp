#pragma once

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"

// Reads real battery voltage via GPIO1 (ADC1_CH0), through a resistive
// divider between VBAT and the ADC pin (see hardware_pinout.md's
// "Battery ADC" row).
//
// Pure hardware driver, same split as Qmi8658 (raw sensor access) vs
// AttitudeEstimator (pure computation): this class only reports a
// voltage. AppController decides what that voltage means (which gauge
// tier to show).
class BatteryMonitor {
public:
    BatteryMonitor();
    ~BatteryMonitor();

    // Owns live ADC unit/calibration handles — not copyable/movable.
    BatteryMonitor(const BatteryMonitor&) = delete;
    BatteryMonitor& operator=(const BatteryMonitor&) = delete;

    // Blocking; averages several raw ADC reads before converting. Returns
    // VBAT in volts, already corrected for the board's divider ratio.
    float ReadVoltage();

    // Diagnostics from the most recent ReadVoltage() call. -1 if not
    // called yet, or every sample in the last call failed.
    int LastRawAverage() const { return last_raw_avg_; }
    bool IsCalibrated() const { return calibrated_; }

private:
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t cali_handle_ = nullptr;
    // Falls back to an uncalibrated linear formula if the eFuse-based
    // calibration scheme can't be created.
    bool calibrated_ = false;
    int last_raw_avg_ = -1;
};
