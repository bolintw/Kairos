#pragma once

#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "debug_config.hpp"

// QMI8658 accel+gyro+tap reader. Accel/gyro register addresses and init
// sequence are ported from Waveshare's official Arduino demo for this
// board. The Tap Engine protocol (CTRL9 command handshake, CAL1-4
// register layout, TAP_STATUS bit layout) is ported from
// lewisxhe/SensorLib's SensorQMI8658.hpp, a widely used community driver
// for this chip.
//
// Self-contained: owns its own I2C bus + device, separate from the
// one-shot bus RunI2cScan() creates and tears down in i2c_scan.hpp.
//
// Gates PollWomEvent()'s/PollTapEvent()'s STATUS1 (and TAP_STATUS)
// prints; real errors always print regardless. A file-scope constexpr,
// not a class member, so it stays a compile-time constant per
// translation unit without threading it through the constructor.
constexpr bool kQmi8658DebugLogEnabled = kDebugEnabled;

class Qmi8658 {
public:
    struct Sample {
        float accel_g[3];    // X, Y, Z in units of g, +-8g range
        float gyro_dps[3];   // X, Y, Z in degrees/sec, +-512dps range
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

        // Soft reset — this chip is a separate I2C device that a re-flash
        // or ESP32 reset does not reset on its own, so it can't be
        // assumed to be in a known state at boot. Write 0xB0 to RESET
        // (0x60); confirmed via RSTResult (0x4D) reading 0x80, up to 15ms
        // per the datasheet, polled here with margin.
        WriteReg(kRegReset, 0xB0);
        bool reset_ok = false;
        for (int i = 0; i < 20; ++i) {  // ~200ms at 10ms/iteration, well past the documented 15ms max
            vTaskDelay(pdMS_TO_TICKS(10));
            uint8_t result = 0;
            if (ReadRegs(kRegResetResult, &result, 1) && result == 0x80) {
                reset_ok = true;
                break;
            }
        }
        if (!reset_ok) {
            printf("Qmi8658: soft reset did not confirm (RSTResult != 0x80) — proceeding anyway\n");
        }

        // Sequence and register values match QMI8658_init() /
        // QMI8658_config_acc() / QMI8658_config_gyro() in the vendor
        // demo, plus CTRL1 bit3/bit4 (INT1_EN/INT2_EN — the QMI8658C
        // datasheet marks these "Reserved", but they behave as documented
        // on the sister QMI8658A part: 0 = pin left high-Z, 1 = pin
        // output enabled). Both must be set or INT1/INT2 never drive
        // their physical pins at all, regardless of any other
        // WoM/tap-engine configuration.
        WriteReg(kRegCtrl1, 0x78);
        WriteReg(kRegCtrl2, 0x23);  // +-8g range, 1000Hz ODR
        WriteReg(kRegCtrl5, 0x00);
        // CTRL3 gFS<2:0> is bits[6:4] (datasheet Table 24): ...
        // 100=256, 101=512 dps. 512dps range trades some noise-floor
        // margin for headroom against silent clipping — ordinary desk
        // flips have been measured peaking near 250dps, too close to a
        // 256dps ceiling for comfort. kGyroLsbPerDps below matches this.
        WriteReg(kRegCtrl3, 0x53);  // +-512dps range, 1000Hz ODR
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

    // Switches between the constructor's normal 6DOF setup and
    // accel-only, for idle sleep. Gyro draws roughly the same current
    // regardless of ODR (driving the MEMS resonator, not sample-rate
    // dependent), so disabling it is the real power lever — accel-only
    // is ~182uA versus ~1mA with both enabled. Trade-off: no gyro means
    // no rotation-based wake in this mode, tap detection only.
    //
    // Deliberately keeps the accel ODR unchanged (CTRL2 untouched, only
    // CTRL7's enable bits move) — the tap engine's alpha/gamma
    // coefficients are per-sample EMA weights with a real-time constant
    // that depends on ODR, and a lower ODR broke tap detection outright
    // in testing.
    //
    // Toggling CTRL7 latches a spurious STATUS1 tap flag; callers of
    // SetLowPowerAccelOnly(true) need to discard the next PollTapEvent()
    // before trusting it. enable=true also sets CTRL7.bit5 (DRDY_DIS) so
    // the accelerometer's Data-Ready signal doesn't also pulse INT2 at
    // the full accel ODR, which would otherwise wake a GPIO-interrupt-driven
    // sleep on every sample instead of only on a real tap.
    void SetLowPowerAccelOnly(bool enable)
    {
        if (!dev_) return;
        WriteReg(kRegCtrl7, enable ? 0x21 : 0x03);  // accel-only+DRDY_DIS vs accel+gyro; CTRL2/tap CAL registers untouched
    }

    // Wake-on-Motion mode (datasheet Section 12). Unlike a tap's brief
    // INT2 pulse (too short for light sleep's level-wakeup GPIO detector
    // to reliably catch), a WoM event drives INT2 to 1 and holds it
    // there until the host reads STATUS1 (PollWomEvent()) — the
    // level-shaped signal GPIO wakeup actually needs.
    //
    // Trade-off: WoM wakes on any sufficiently large accelerometer
    // slope, not specifically a tap — being picked up or the desk being
    // knocked also wake the device. Matches the "any wake comes back
    // paused, needs a real second tap to resume" model, so a spurious
    // wake just costs one extra paused screen.
    //
    // EnterWakeOnMotion()/ExitWakeOnMotion() follow the datasheet's two
    // configuration sequences (12.5 enter, 12.6 exit) rather than a
    // single toggle — they aren't symmetric (exit leaves CTRL7 disabled,
    // see its own comment). threshold_mg/blanking_samples are starting
    // points, not a finished tune. CAL1_L/CAL1_H are the same scratch
    // registers ConfigureTap() uses, carrying a different meaning
    // depending on which CTRL9 command follows — callers should re-run
    // ConfigureTap() after ExitWakeOnMotion() rather than assume the tap
    // engine's own settings survived.
    void EnterWakeOnMotion(uint8_t threshold_mg, uint8_t blanking_samples)
    {
        if (!dev_) return;
        WriteReg(kRegCtrl7, 0x00);  // disable all sensors — required before configuring WoM (datasheet 12.5)
        // Disable the tap engine so it doesn't compete with WoM for the
        // interrupt pin's routing — not in the datasheet's own
        // configuration sequence, but CTRL8 is otherwise left however
        // ConfigureTap() last set it.
        WriteReg(kRegCtrl8, 0x00);
        WriteReg(kRegCal1L, threshold_mg);
        // bits[7:6] select interrupt pin + initial value (Table 39: "01"
        // = INT2/init-0, "11" = INT2/init-1, "00" = INT1/init-0, "10" =
        // INT1/init-1); bits[5:0] is the blanking time in accelerometer
        // samples (max 63, ~63ms at 1000Hz) — screens out startup
        // transients right after enabling.
        WriteReg(kRegCal1H, static_cast<uint8_t>(0x40 | (blanking_samples & 0x3F)));
        if (!WriteCommandAndWait(kCmdWriteWomSetting)) {
            printf("Qmi8658::EnterWakeOnMotion: CTRL9 handshake failed\n");
        }
        // Accel-only enable — WoM still needs the accelerometer running
        // internally to evaluate motion (datasheet 12.3), even though "no
        // sensor data is generated" in the normal DRDY sense while in
        // this mode (Section 6). Same accel-only rationale as
        // SetLowPowerAccelOnly(true), not repeated here.
        WriteReg(kRegCtrl7, 0x01);
    }

    void ExitWakeOnMotion()
    {
        if (!dev_) return;
        WriteReg(kRegCtrl7, 0x00);  // disable all sensors (datasheet 12.6)
        WriteReg(kRegCal1L, 0x00);  // 0x00 disables WoM, returns INT pins to normal function (Table 39)
        if (!WriteCommandAndWait(kCmdWriteWomSetting)) {
            printf("Qmi8658::ExitWakeOnMotion: CTRL9 handshake failed\n");
        }
        // Deliberately leaves CTRL7 at 0x00 (sensors disabled) rather
        // than restoring 6DOF here — the caller is expected to
        // immediately call ConfigureTap() afterward (see this method's
        // class-level comment), which starts with its own CTRL7=0x00 and
        // ends by restoring CTRL7=0x03 anyway; toggling CTRL7 off then
        // back on here just to have ConfigureTap() toggle it off again
        // moments later would be pure waste.
    }

    // Polls for a Wake-on-Motion event. STATUS1.bit2 (WoM) — reading
    // STATUS1 clears the bit and resets INT2 back to its configured
    // initial value (datasheet 12.4), the same read-clears shape as
    // PollTapEvent()'s STATUS1.bit1 below.
    bool PollWomEvent()
    {
        if (!dev_) return false;
        uint8_t status1 = 0;
        if (!ReadRegs(kRegStatus1, &status1, 1)) return false;
        if (kQmi8658DebugLogEnabled && status1 != 0) {
            printf("Qmi8658::PollWomEvent: STATUS1=0x%02X\n", status1);
        }
        return (status1 & 0x04) != 0;
    }

    // Polls for a new tap event. TAP_STATUS (0x59) holds the *type* of
    // the most recent tap, but its value doesn't change between two
    // same-type taps in a row, so diffing it directly misses repeats.
    // STATUS1 bit1 is the "a new tap happened since last check" signal;
    // only read TAP_STATUS for the single/double detail once it's set.
    TapEvent PollTapEvent()
    {
        if (!dev_) return TapEvent::kNone;

        uint8_t status1 = 0;
        if (!ReadRegs(kRegStatus1, &status1, 1)) return TapEvent::kNone;
        if (kQmi8658DebugLogEnabled && status1 != 0) {
            printf("Qmi8658::PollTapEvent: STATUS1=0x%02X\n", status1);
        }
        if ((status1 & 0x02) == 0) return TapEvent::kNone;

        uint8_t tap_status = 0;
        if (!ReadRegs(kRegTapStatus, &tap_status, 1)) return TapEvent::kNone;
        if (kQmi8658DebugLogEnabled) {
            printf("Qmi8658::PollTapEvent: TAP bit set, TAP_STATUS=0x%02X\n", tap_status);
        }

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
    static constexpr uint8_t kRegReset = 0x60;
    static constexpr uint8_t kRegResetResult = 0x4D;

    static constexpr uint8_t kCmdAck = 0x00;
    static constexpr uint8_t kCmdConfigureTap = 0x0C;
    static constexpr uint8_t kCmdWriteWomSetting = 0x08;

    static constexpr float kAccelLsbPerG = 4096.0f;   // +-8g range
    static constexpr float kGyroLsbPerDps = 64.0f;    // +-512dps range (32768/512), see CTRL3=0x53 above

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
