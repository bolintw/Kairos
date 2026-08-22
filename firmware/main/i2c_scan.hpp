#pragma once

#include <cstdio>

#include "driver/i2c_master.h"

// M0 hardware check: confirm the I2C bus is wired correctly and only the
// QMI8658 IMU is on it (hardware_pinout.md: SDA=6, SCL=7, no touch IC on
// this non-touch board variant). One-shot diagnostic — creates its own
// bus and tears it down when done, since M3+ will set up a persistent
// bus for actual sensor reads.
inline void RunI2cScan()
{
    constexpr gpio_num_t kSda = GPIO_NUM_6;
    constexpr gpio_num_t kScl = GPIO_NUM_7;

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = kSda;
    bus_cfg.scl_io_num = kScl;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus;
    if (i2c_new_master_bus(&bus_cfg, &bus) != ESP_OK) {
        printf("I2C scan: failed to create bus on SDA=%d SCL=%d\n", kSda, kScl);
        return;
    }

    printf("I2C scan (SDA=%d, SCL=%d):\n", kSda, kScl);
    int found = 0;
    uint16_t qmi8658_addr = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
        if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
            printf("  found device at 0x%02X\n", addr);
            ++found;
            // QMI8658 is 0x6B by default, 0x6A if SA0 is strapped low.
            if (addr == 0x6A || addr == 0x6B) {
                qmi8658_addr = addr;
            }
        }
    }
    printf("I2C scan done: %d device(s) found (expect exactly 1: QMI8658)\n", found);

    if (qmi8658_addr != 0) {
        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = qmi8658_addr;
        dev_cfg.scl_speed_hz = 400000;

        i2c_master_dev_handle_t dev;
        if (i2c_master_bus_add_device(bus, &dev_cfg, &dev) == ESP_OK) {
            uint8_t reg = 0x00;  // WHO_AM_I
            uint8_t who_am_i = 0;
            if (i2c_master_transmit_receive(dev, &reg, 1, &who_am_i, 1, 50) == ESP_OK) {
                printf("QMI8658 WHO_AM_I @ 0x%02X = 0x%02X (datasheet expects 0x05)\n",
                       qmi8658_addr, who_am_i);
            } else {
                printf("QMI8658 found at 0x%02X but WHO_AM_I read failed\n", qmi8658_addr);
            }
            i2c_master_bus_rm_device(dev);
        }
    }

    i2c_del_master_bus(bus);
}
