#pragma once

#include <cstdint>
#include <cstdio>

#include "driver/i2c_master.h"

// Minimal QMI8658 accel+gyro reader for the M3 debug overlay. Register
// addresses, init sequence, and scale factors are ported from Waveshare's
// official Arduino demo for this exact board (ESP32-S3-LCD-1.28-Demo.zip,
// QMI8658.cpp) rather than re-derived from the datasheet PDF, since that
// demo is confirmed working on this hardware.
//
// Self-contained: owns its own I2C bus + device, separate from the
// one-shot bus RunI2cScan() creates and tears down in i2c_scan.hpp.
// Whether AttitudeEstimator (M5) shares a bus with this or replaces it
// entirely is an open design question for that milestone, not decided
// here — this class exists only to drive the M3/M4 debug overlay.
//
// No tap-detection support yet: that's the QMI8658 hardware Tap Engine,
// configured via the CTRL9 command protocol per datasheet Section 10,
// which is M4's job. Waveshare's demo doesn't implement it either.
class Qmi8658 {
public:
    struct Sample {
        float accel_g[3];    // X, Y, Z in units of g, +-8g range
        float gyro_dps[3];   // X, Y, Z in degrees/sec, +-512dps range
    };

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
        // Ctrl1=0x60, Ctrl2 = accel +-8g @ 1000Hz, Ctrl3 = gyro +-512dps @
        // 1000Hz, Ctrl5=0x00 (LPF/HPF off — the vendor code computes LPF
        // bits but then unconditionally overwrites them with 0 before the
        // write), Ctrl7 = enable accel+gyro.
        WriteReg(kRegCtrl1, 0x60);
        WriteReg(kRegCtrl2, 0x23);  // +-8g range, 1000Hz ODR
        WriteReg(kRegCtrl5, 0x00);
        WriteReg(kRegCtrl3, 0x43);  // +-512dps range, 1000Hz ODR
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

private:
    static constexpr uint8_t kRegCtrl1 = 0x02;
    static constexpr uint8_t kRegCtrl2 = 0x03;
    static constexpr uint8_t kRegCtrl3 = 0x04;
    static constexpr uint8_t kRegCtrl5 = 0x06;
    static constexpr uint8_t kRegCtrl7 = 0x08;
    static constexpr uint8_t kRegAxL = 0x35;

    static constexpr float kAccelLsbPerG = 4096.0f;   // +-8g range
    static constexpr float kGyroLsbPerDps = 64.0f;    // +-512dps range

    void WriteReg(uint8_t reg, uint8_t value)
    {
        const uint8_t payload[2] = {reg, value};
        i2c_master_transmit(dev_, payload, sizeof(payload), 50);
    }

    bool ReadRegs(uint8_t reg, uint8_t* buf, size_t len)
    {
        return i2c_master_transmit_receive(dev_, &reg, 1, buf, len, 50) == ESP_OK;
    }

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_ = nullptr;
};
