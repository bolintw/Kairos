#pragma once

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_oneshot.h"

// Real battery voltage reading for the battery-check gesture (see
// app_controller.hpp's design note 10) — replaces AppController's
// kStubBatteryFilledBlocks stand-in. Hardware: hardware_pinout.md's
// "Battery ADC" row — GPIO1 (ADC1_CH0), a resistive divider between VBAT
// and the ADC pin. hardware_pinout.md currently documents this as 1:1
// (VBAT/2), but real hardware measurement (2026-09-11 — see
// kDividerRatio's comment in the .cpp) found the actual ratio is ~3:1,
// not 2:1 — that doc still needs its own correction; this class's own
// kDividerRatio already reflects the measured value.
//
// Pure hardware driver, same split as Qmi8658 (raw sensor access) vs
// AttitudeEstimator (pure computation): this class only reports a
// voltage. AppController owns what that voltage *means* (which of the 5
// gauge blocks to light, matching BatteryTierColor's tiers in
// gui_manager.cpp) — same reasoning as why AttitudeEstimator reports a
// bare screen_angle_deg and doesn't know what a "face" is.
class BatteryMonitor {
public:
    BatteryMonitor();
    ~BatteryMonitor();

    // Not copyable/movable — owns live ADC unit/calibration handles, same
    // as Qmi8658's implicit non-copyability via its i2c_master_dev_handle_t.
    BatteryMonitor(const BatteryMonitor&) = delete;
    BatteryMonitor& operator=(const BatteryMonitor&) = delete;

    // Blocking; averages several raw ADC reads (see .cpp —
    // hardware_pinout.md notes the battery sags under load and reads
    // benefit from averaging) before converting. Returns the actual
    // battery voltage in volts (already corrected for the board's
    // divider ratio, i.e. this is VBAT, not the ADC pin's own voltage).
    float ReadVoltage();

    // Diagnostics from the most recent ReadVoltage() call — added
    // 2026-09-11 to track down a real hardware mismatch (see
    // ReadVoltage()'s comment). -1 if ReadVoltage() hasn't been called
    // yet, or every sample in its last call failed.
    int LastRawAverage() const { return last_raw_avg_; }
    bool IsCalibrated() const { return calibrated_; }

private:
    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t cali_handle_ = nullptr;
    bool calibrated_ = false;  // see .cpp constructor comment — falls back
                                // to an uncalibrated linear formula if the
                                // eFuse-based calibration scheme can't be
                                // created
    int last_raw_avg_ = -1;
};
