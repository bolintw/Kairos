#include "battery_monitor.hpp"

#include <cstdio>

#include "esp_adc/adc_cali_scheme.h"

namespace {
// GPIO1 = ADC1_CH0 on ESP32-S3 (hardware_pinout.md's "Battery ADC" row).
constexpr adc_unit_t kAdcUnit = ADC_UNIT_1;
constexpr adc_channel_t kAdcChannel = ADC_CHANNEL_0;
// ~0-3.3V full-scale — the divided battery signal (a 3.0-4.2V LiPo range
// through the real ~3:1 divider, see kDividerRatio below, is ~1.0-1.4V at
// the pin) sits well inside it, no need for a narrower/more sensitive
// attenuation.
constexpr adc_atten_t kAdcAtten = ADC_ATTEN_DB_12;
constexpr adc_bitwidth_t kAdcBitwidth = ADC_BITWIDTH_DEFAULT;

// hardware_pinout.md: "ADC measurement should average multiple samples —
// battery voltage dips briefly under load, so measurement timing affects
// the reading". Averaged as raw counts, not as post-conversion
// voltages — equivalent for a near-linear calibration curve and only
// needs one adc_cali_raw_to_voltage() call instead of kNumSamples of them.
constexpr int kNumSamples = 32;

// 2.0f -> 3.0f -> 3.114f (2026-09-11, same day): hardware_pinout.md
// documented R4=R7=100K (1:1, /2 divider) as of its 2026-08-25
// correction, but real hardware measurement contradicted it — multimeter
// read VBAT=3.945V and the ADC pin=1.3062V simultaneously, ratio
// 3.945/1.3062 = 3.02, not 2, so this was first set to a round 3.0f.
// Refined once more the same day from a second, *end-to-end* data point:
// with 3.0f in place, this class's own output read 3.8V against a
// simultaneous multimeter VBAT reading of 3.945V — scaling 3.0f by
// 3.945/3.8 gives 3.114f. This end-to-end scale factor (real VBAT over
// this class's own output) is deliberately what's used here, not a purer
// "just the resistor ratio" number the way the first correction above
// was reasoned — it also absorbs whatever small residual bias sits in
// the ADC calibration curve itself (already known to be small, ~2%, from
// the first correction's cross-check), which is exactly what's wanted
// when the goal is this class's output matching real VBAT as closely as
// possible, not deriving R4/R7's true nominal values. Not measured
// directly (see hardware_pinout.md's note — in-circuit resistance
// measurement was fighting C13's parallel charging transient); this
// voltage-ratio approach sidesteps that entirely.
constexpr float kDividerRatio = 3.114f;

// Fallback for when the calibration scheme can't be created (e.g. the
// calibration eFuse bits aren't burned on this particular chip) —
// hardware_pinout.md's own naive formula (assumes an ideal 3.3V Vref and
// a full-scale 12-bit/4095 count), less accurate than the calibrated
// curve-fitting path but a reasonable baseline; that doc's own note
// ("use the adc_cali calibration API in place of the linear formula if
// needed") frames the linear formula as the starting point this upgrades
// from, not the other way round.
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
        // Bug fixed 2026-09-11: this used to divide by kNumSamples
        // unconditionally below, silently under-averaging (and therefore
        // under-reporting voltage) by up to the full failed-sample
        // fraction whenever adc_oneshot_read() returned non-OK for some
        // samples — caught from a real hardware mismatch (multimeter said
        // 3.983V, this reported 2.56V, a ~36% low reading consistent with
        // roughly a third of the 32 reads failing and still being counted
        // in the denominator).
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
