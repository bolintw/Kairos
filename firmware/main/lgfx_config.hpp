#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// Waveshare ESP32-S3-LCD-1.28 (Non-Touch): GC9A01A driving a 240x240
// round panel over SPI. Pin values from hardware_pinout.md.
//
// Backlight (GPIO40) is driven via LovyanGFX's built-in Light_PWM
// controller for real analog dimming (fade/breathing-pulse brightness
// notifications). Channel 7 avoids the low channels LovyanGFX's own
// SPI/DMA setup might touch.
class LGFX : public lgfx::LGFX_Device {
public:
    lgfx::Panel_GC9A01 _panel_instance;
    lgfx::Bus_SPI _bus_instance;
    lgfx::Light_PWM _light_instance;

    LGFX(void) {
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host = SPI2_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 80000000;  // panel supports 80MHz per hardware_pinout.md
            cfg.freq_read = 16000000;
            cfg.spi_3wire = false;
            cfg.use_lock = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = 10;
            cfg.pin_mosi = 11;
            cfg.pin_miso = -1;  // not wired on this board
            cfg.pin_dc = 8;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs = 9;
            cfg.pin_rst = 12;
            cfg.pin_busy = -1;
            cfg.panel_width = 240;
            cfg.panel_height = 240;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.readable = false;  // no MISO wired
            cfg.invert = true;  // GC9A01 panels commonly need color inversion
            cfg.rgb_order = false;
            cfg.dlen_16bit = false;
            cfg.bus_shared = false;
            _panel_instance.config(cfg);
        }
        {
            auto cfg = _light_instance.config();
            cfg.pin_bl = 40;
            cfg.invert = false;
            cfg.freq = 44100;
            cfg.pwm_channel = 7;
            _light_instance.config(cfg);
            _panel_instance.setLight(&_light_instance);
        }
        setPanel(&_panel_instance);
    }
};
