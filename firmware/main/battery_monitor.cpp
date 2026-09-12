#include "battery_monitor.hpp"

#include <cstdio>

#include "esp_adc/adc_cali_scheme.h"

namespace {
// GPIO1 = ADC1_CH0 on ESP32-S3 (hardware_pinout.md's "Battery ADC" row).
constexpr adc_unit_t kAdcUnit = ADC_UNIT_1;
constexpr adc_channel_t kAdcChannel = ADC_CHANNEL_0;
// ~0-3.3V full-scale comfortably covers the divided battery signal
// (~1.0-1.4V at the pin for a 3.0-4.2V LiPo through kDividerRatio below).
constexpr adc_atten_t kAdcAtten = ADC_ATTEN_DB_12;
constexpr adc_bitwidth_t kAdcBitwidth = ADC_BITWIDTH_DEFAULT;

// Battery voltage sags under load, so average multiple samples rather
// than trust a single read. Averaged as raw counts, not post-conversion
// voltages, so only one adc_cali_raw_to_voltage() call is needed.
constexpr int kNumSamples = 32;

// Measured end-to-end against a multimeter (firmware output vs real
// VBAT), not derived from the divider's nominal resistor values.
constexpr float kDividerRatio = 3.114f;

// Fallback linear formula (ideal 3.3V Vref, 12-bit/4095 count) for when
// the calibration scheme can't be created.
constexpr float kFallbackVrefMv = 3300.0f;
constexpr float kFallbackMaxCount = 4095.0f;
}  // namespace

BatteryMonitor::BatteryMonitor()
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {};
    unit_cfg.unit_id = kAdcUnit;
    adc_oneshot_new_unit(&unit_cfg, &adc_handle_);

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten = kAdcAtten;
    chan_cfg.bitwidth = kAdcBitwidth;
    adc_oneshot_config_channel(adc_handle_, kAdcChannel, &chan_cfg);

    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id = kAdcUnit;
    cali_cfg.chan = kAdcChannel;
    cali_cfg.atten = kAdcAtten;
    cali_cfg.bitwidth = kAdcBitwidth;
    calibrated_ = adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle_) == ESP_OK;
    if (!calibrated_) {
        printf("BatteryMonitor: ADC calibration scheme unavailable, using uncalibrated linear formula\n");
    }
}

BatteryMonitor::~BatteryMonitor()
{
    if (calibrated_) {
        adc_cali_delete_scheme_curve_fitting(cali_handle_);
    }
    if (adc_handle_ != nullptr) {
        adc_oneshot_del_unit(adc_handle_);
    }
}

float BatteryMonitor::ReadVoltage()
{
    int64_t raw_sum = 0;
    int good_samples = 0;
    for (int i = 0; i < kNumSamples; ++i) {
        int raw = 0;
        // Divide by the actual successful count below, not kNumSamples —
        // a failed adc_oneshot_read() must not silently pull the average down.
        if (adc_oneshot_read(adc_handle_, kAdcChannel, &raw) == ESP_OK) {
            raw_sum += raw;
            ++good_samples;
        }
    }
    if (good_samples == 0) {
        last_raw_avg_ = -1;  // -1: no valid sample this call, not "read 0"
        return 0.0f;
    }
    const float raw_avg = static_cast<float>(raw_sum) / static_cast<float>(good_samples);
    last_raw_avg_ = static_cast<int>(raw_avg + 0.5f);

    float pin_mv;
    if (calibrated_) {
        int cali_mv = 0;
        adc_cali_raw_to_voltage(cali_handle_, last_raw_avg_, &cali_mv);
        pin_mv = static_cast<float>(cali_mv);
    } else {
        pin_mv = raw_avg * kFallbackVrefMv / kFallbackMaxCount;
    }

    return pin_mv * kDividerRatio / 1000.0f;  // pin mV -> real battery V
}
