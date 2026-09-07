#pragma once

#include <cstdint>
#include <cstdio>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Minimal QMI8658 accel+gyro+tap reader for the M3/M4 debug overlay.
// Accel/gyro register addresses and init sequence are ported from
// Waveshare's official Arduino demo for this exact board
// (ESP32-S3-LCD-1.28-Demo.zip, QMI8658.cpp), which doesn't cover tap
// detection. The Tap Engine protocol (CTRL9 command handshake, CAL1-4
// register layout, TAP_STATUS bit layout) is ported instead from
// lewisxhe/SensorLib's SensorQMI8658.hpp — a widely used community
// driver for this exact chip — rather than re-derived from the
// datasheet PDF, which isn't reliably text-extractable here.
//
// Self-contained: owns its own I2C bus + device, separate from the
// one-shot bus RunI2cScan() creates and tears down in i2c_scan.hpp.
// Whether AttitudeEstimator (M5) shares a bus with this or replaces it
// entirely is an open design question for that milestone, not decided
// here — this class exists only to drive the M3/M4 debug overlay.
class Qmi8658 {
public:
    struct Sample {
        float accel_g[3];    // X, Y, Z in units of g, +-8g range
        float gyro_dps[3];   // X, Y, Z in degrees/sec, +-256dps range
    };

    enum class TapEvent { kNone, kSingle, kDouble };

    Qmi8658(gpio_num_t sda, gpio_num_t scl, uint16_t addr = 0x6B)
    {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = I2C_NUM_0;
        bus_cfg.sda_io_num = sda;
        bus_cfg.scl_io_num = scl;
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = true;
        if (i2c_new_master_bus(&bus_cfg, &bus_) != ESP_OK) {
            printf("Qmi8658: failed to create I2C bus\n");
            return;
        }

        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = addr;
        dev_cfg.scl_speed_hz = 400000;
        if (i2c_master_bus_add_device(bus_, &dev_cfg, &dev_) != ESP_OK) {
            printf("Qmi8658: failed to add device at 0x%02X\n", addr);
            return;
        }

        // Sequence and register values match QMI8658_init() /
        // QMI8658_config_acc() / QMI8658_config_gyro() in the vendor demo:
        // Ctrl1=0x60, Ctrl2 = accel +-8g @ 1000Hz, Ctrl5=0x00 (LPF/HPF off
        // — the vendor code computes LPF bits but then unconditionally
        // overwrites them with 0 before the write), Ctrl7 = enable
        // accel+gyro.
        WriteReg(kRegCtrl1, 0x60);
        WriteReg(kRegCtrl2, 0x23);  // +-8g range, 1000Hz ODR
        WriteReg(kRegCtrl5, 0x00);
        // CTRL3 gFS<2:0> is bits[6:4] (QMI8658C datasheet Rev 0.6, Table
        // 24, p.27): 000=16, 001=32, 010=64, 011=128, 100=256, 101=512,
        // 110=1024, 111=2048 dps. 0x43 = 0100_0011 -> bits[6:4]=100 ->
        // *256dps*. This was originally a bug (code assumed 512dps, see
        // git history 2026-08-24) causing every dps reading to be 2x true
        // value — but 256dps turns out to be the range we actually want:
        // narrower range means more LSB/dps (128 here vs 64 at 512dps),
        // so the same ADC noise floor converts to less dps noise, and
        // typical desk-flip angular rates shouldn't approach 256dps
        // anyway. Kept at 0x43 deliberately now, with kGyroLsbPerDps
        // matching it below. Risk: a fast/hard flip that does exceed
        // 256dps will clip instead of overshooting — the opposite
        // failure mode (silent under-read instead of obvious overshoot,
        // easy to miss) — revisit at M10 assembly-time tuning if that
        // turns out to matter in practice; bits[3:0]=0011 (1000Hz ODR).
        WriteReg(kRegCtrl3, 0x43);  // +-256dps range, 1000Hz ODR
        WriteReg(kRegCtrl5, 0x00);
        WriteReg(kRegCtrl7, 0x03);  // accel + gyro enable
    }

    ~Qmi8658()
    {
        if (dev_) i2c_master_bus_rm_device(dev_);
        if (bus_) i2c_del_master_bus(bus_);
    }

    Qmi8658(const Qmi8658&) = delete;
    Qmi8658& operator=(const Qmi8658&) = delete;

    bool Read(Sample& out)
    {
        if (!dev_) return false;

        uint8_t buf[12];
        if (!ReadRegs(kRegAxL, buf, sizeof(buf))) return false;

        int16_t raw[6];
        for (int i = 0; i < 6; ++i) {
            raw[i] = static_cast<int16_t>(
                (static_cast<uint16_t>(buf[i * 2 + 1]) << 8) | buf[i * 2]);
        }

        for (int i = 0; i < 3; ++i) {
            out.accel_g[i] = raw[i] / kAccelLsbPerG;
            out.gyro_dps[i] = raw[3 + i] / kGyroLsbPerDps;
        }
        return true;
    }

    // Configures the hardware Tap Engine. Values here are M4's starting
    // point for calibration, not a finished tune — see main.cpp for the
    // actual numbers in use and where they came from.
    //
    // priority: which axis wins when peaks land simultaneously (0 = X>Y>Z,
    //   per SensorLib's TapDetectionPriority enum).
    // peak_window/tap_window/d_tap_window: durations in *samples* at the
    //   configured accel ODR (1000Hz here).
    // alpha/gamma: smoothing ratios for the engine's internal running
    //   averages, unitless, not ODR-dependent.
    // peak_mag_thr_g2/udm_thr_g2: peak and quiet thresholds in g^2.
    void ConfigureTap(uint8_t priority, uint8_t peak_window, uint16_t tap_window,
                       uint16_t d_tap_window, float alpha, float gamma,
                       float peak_mag_thr_g2, float udm_thr_g2)
    {
        if (!dev_) return;

        // The CAL1-4 registers double as tap-config scratch space, so
        // accel/gyro are paused while writing them (matches SensorLib's
        // configTap, which does the same before restoring CTRL7).
        WriteReg(kRegCtrl7, 0x00);

        WriteReg(kRegCal1L, peak_window);
        WriteReg(kRegCal1H, priority);
        WriteReg(kRegCal2L, tap_window & 0xFF);
        WriteReg(kRegCal2H, (tap_window >> 8) & 0xFF);
        WriteReg(kRegCal3L, d_tap_window & 0xFF);
        WriteReg(kRegCal3H, (d_tap_window >> 8) & 0xFF);
        WriteReg(kRegCal4H, 0x01);
        WriteCommandAndWait(kCmdConfigureTap);

        WriteReg(kRegCal1L, static_cast<uint8_t>(alpha * 128));
        WriteReg(kRegCal1H, static_cast<uint8_t>(gamma * 128));

        // Resolution is documented as 0.001 g^2/LSB, so value = thr * 1000.
        const uint16_t peak_val = static_cast<uint16_t>(peak_mag_thr_g2 * 1000.0f + 0.5f);
        WriteReg(kRegCal2L, peak_val & 0xFF);
        WriteReg(kRegCal2H, (peak_val >> 8) & 0xFF);

        const uint16_t udm_val = static_cast<uint16_t>(udm_thr_g2 * 1000.0f + 0.5f);
        WriteReg(kRegCal3L, udm_val & 0xFF);
        WriteReg(kRegCal3H, (udm_val >> 8) & 0xFF);
        WriteReg(kRegCal4H, 0x02);
        WriteCommandAndWait(kCmdConfigureTap);

        WriteReg(kRegCtrl7, 0x03);  // re-enable accel + gyro
        WriteReg(kRegCtrl8, 0x01);  // enable tap detection (bit 0)
    }

    // Switches between the constructor's normal 6DOF setup (accel+gyro
    // both enabled) and accel-only — added 2026-09-06 for M9 idle sleep.
    // Gyro draws roughly the same current regardless of ODR (QMI8658C.pdf
    // Table 16 — cost of driving the MEMS resonator itself, not
    // sample-rate-dependent), so disabling it is the only real lever for
    // gyro's share; accel-only High-Resolution at 1000Hz (Table 15) is
    // ~182uA versus the ~1mA this project measured with both enabled —
    // most of the win, from gyro alone. Accepted trade-off: no gyro
    // samples means no rotation-based wake while in this mode, tap
    // detection only (still works — tap detection only ever needed
    // accel, see ConfigureTap()).
    //
    // Deliberately does NOT also drop the accel ODR the way two earlier
    // versions of this function did (Low Power mode at 128Hz, then
    // High-Resolution at 125Hz) — both broke tap detection outright on
    // hardware, not just made it less sensitive. In hindsight the ODR
    // reduction was barely worth it anyway (134-182uA either way per
    // Table 15, accel's own current barely varies across its
    // High-Resolution ODR range) next to the risk: the tap engine's
    // alpha/gamma coefficients (see ConfigureTap()'s comment) are the
    // chip's own per-sample EMA weights, so they carry a real-time time
    // constant that depends on ODR the same way this project's own
    // software low-pass filters do (attitude_estimator.cpp) — dropping
    // ODR ~7x means their effective time constants stretch ~7x too, and
    // at 125Hz the math for the *original* gamma's implied time constant
    // (~4.46ms at the real 896.8Hz 6DOF rate) works out to needing
    // gamma > 1, not a representable coefficient at all. Keeping the
    // accel ODR unchanged (CTRL2 stays 0x23 in both branches below, only
    // CTRL7's enable bits move) sidesteps this entirely: the tap engine
    // keeps running with the exact same tuning that's already been
    // validated on hardware, whether or not gyro is also enabled.
    //
    // Toggling CTRL7 (this function's only job) latches a spurious
    // STATUS1 tap flag, same quirk main.cpp already discards one
    // PollTapEvent() for right after boot's ConfigureTap() call (which
    // also toggles CTRL7). Callers of SetLowPowerAccelOnly(true) need to
    // do the same discard before trusting the next real PollTapEvent() —
    // skipped without it, RunIdleSleep() woke itself up immediately every
    // time, reading that latched flag as a real tap (caught on hardware).
    // enable=true also sets CTRL7.bit5 (DRDY_DIS, 2026-09-06) — datasheet
    // Section 6.3: with DRDY_DIS=0 (the default, and what enable=false
    // leaves it at), the accelerometer's Data-Ready signal is *also*
    // routed to INT2/GPIO48 (same pin CTRL8.bit6=0 already sends tap
    // events to, see ConfigureTap()'s CTRL8 write), pulsing at the accel
    // ODR the whole time this mode is active. A GPIO-interrupt-driven
    // wake (sleep_mode.cpp's diagnostic, replacing RunIdleSleep()'s fixed
    // 1s poll — see plan doc's M9 section) needs INT2 to only pulse on a
    // real tap; left at the default, the ESP32 would wake on every accel
    // sample instead, defeating the whole point. Setting DRDY_DIS=1 here
    // blocks DRDY from INT2 without touching accel sampling itself. Not
    // set when enable=false since normal 6DOF operation never reads
    // INT2 at all currently — harmless either way there, left at its
    // default for now.
    void SetLowPowerAccelOnly(bool enable)
    {
        if (!dev_) return;
        WriteReg(kRegCtrl7, enable ? 0x21 : 0x03);  // accel-only+DRDY_DIS vs accel+gyro; CTRL2/tap CAL registers untouched
    }

    // Polls for a new tap event. TAP_STATUS (0x59) holds the *type* of the
    // most recent tap, but its value doesn't change between two same-type
    // taps in a row, so diffing it directly misses repeats — that was this
    // function's first version, and it's why only one tap ever registered.
    // STATUS1 bit1 is the actual "a new tap happened since last check"
    // signal (inferred from SensorLib's update(), which polls STATUS1
    // this way in a loop and correctly counts repeated taps — not
    // confirmed against the datasheet directly, but the alternative
    // matches observed behavior exactly). Only read TAP_STATUS for the
    // single/double detail once STATUS1 says a new event is there.
    TapEvent PollTapEvent()
    {
        if (!dev_) return TapEvent::kNone;

        uint8_t status1 = 0;
        if (!ReadRegs(kRegStatus1, &status1, 1)) return TapEvent::kNone;
        // Temporary diagnostic (2026-09-06, M9 idle-sleep debugging) —
        // print the raw byte whenever anything in it is set, not just the
        // TAP bit we act on, so a run that spuriously wakes RunIdleSleep()
        // shows exactly what STATUS1 actually looked like at that moment
        // instead of us continuing to guess. Cheap to leave in: status1
        // reads all-zero the overwhelming majority of the time in normal
        // use, so this doesn't spam the log outside of taps/whatever this
        // turns out to be.
        if (status1 != 0) {
            printf("Qmi8658::PollTapEvent: STATUS1=0x%02X\n", status1);
        }
        if ((status1 & 0x02) == 0) return TapEvent::kNone;

        uint8_t tap_status = 0;
        if (!ReadRegs(kRegTapStatus, &tap_status, 1)) return TapEvent::kNone;
        printf("Qmi8658::PollTapEvent: TAP bit set, TAP_STATUS=0x%02X\n", tap_status);

        switch (tap_status & 0x03) {
            case 1: return TapEvent::kSingle;
            case 2: return TapEvent::kDouble;
            default: return TapEvent::kNone;
        }
    }

private:
    static constexpr uint8_t kRegCtrl1 = 0x02;
    static constexpr uint8_t kRegCtrl2 = 0x03;
    static constexpr uint8_t kRegCtrl3 = 0x04;
    static constexpr uint8_t kRegCtrl5 = 0x06;
    static constexpr uint8_t kRegCtrl7 = 0x08;
    static constexpr uint8_t kRegCtrl8 = 0x09;
    static constexpr uint8_t kRegCtrl9 = 0x0A;
    static constexpr uint8_t kRegCal1L = 0x0B;
    static constexpr uint8_t kRegCal1H = 0x0C;
    static constexpr uint8_t kRegCal2L = 0x0D;
    static constexpr uint8_t kRegCal2H = 0x0E;
    static constexpr uint8_t kRegCal3L = 0x0F;
    static constexpr uint8_t kRegCal3H = 0x10;
    static constexpr uint8_t kRegCal4H = 0x12;
    static constexpr uint8_t kRegStatusInt = 0x2D;
    static constexpr uint8_t kRegStatus1 = 0x2F;
    static constexpr uint8_t kRegTapStatus = 0x59;
    static constexpr uint8_t kRegAxL = 0x35;

    static constexpr uint8_t kCmdAck = 0x00;
    static constexpr uint8_t kCmdConfigureTap = 0x0C;

    static constexpr float kAccelLsbPerG = 4096.0f;   // +-8g range
    static constexpr float kGyroLsbPerDps = 128.0f;   // +-256dps range (32768/256)

    void WriteReg(uint8_t reg, uint8_t value)
    {
        const uint8_t payload[2] = {reg, value};
        i2c_master_transmit(dev_, payload, sizeof(payload), 50);
    }

    bool ReadRegs(uint8_t reg, uint8_t* buf, size_t len)
    {
        return i2c_master_transmit_receive(dev_, &reg, 1, buf, len, 50) == ESP_OK;
    }

    // CTRL9 host command handshake: write the command, wait for the
    // "done" bit (STATUS_INT bit7), ack it, wait for the bit to clear.
    bool WriteCommandAndWait(uint8_t cmd)
    {
        WriteReg(kRegCtrl9, cmd);
        if (!WaitForStatusIntBit(true)) return false;
        WriteReg(kRegCtrl9, kCmdAck);
        return WaitForStatusIntBit(false);
    }

    bool WaitForStatusIntBit(bool want_set)
    {
        for (int i = 0; i < 200; ++i) {  // ~200ms timeout at 1ms/iteration
            uint8_t val = 0;
            if (!ReadRegs(kRegStatusInt, &val, 1)) return false;
            if (((val & 0x80) != 0) == want_set) return true;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        return false;
    }

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_ = nullptr;
};
