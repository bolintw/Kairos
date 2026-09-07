#pragma once

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// Waveshare ESP32-S3-LCD-1.28 (Non-Touch): GC9A01A driving a 240x240
// round panel over SPI. Pin values from hardware_pinout.md.
//
// Backlight (GPIO40) is driven via LovyanGFX's built-in Light_PWM
// controller (LEDC PWM under the hood), not a plain on/off GpioPin — M1
// used GpioPin since only on/off was needed then, but M7's brightness
// notifications (fade, breathing pulse) need real analog dimming.
// Channel 7 avoids the low channels LovyanGFX's own SPI/DMA setup might
// touch.
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
            // 40 -> 80MHz (2026-09-06) — hardware_pinout.md already notes
            // this panel supports 80MHz (theoretical full-screen flush
            // ~11.5ms), never tried until the main-loop timing
            // investigation showed SPI transfer (flush_us) as ~35% of
            // lv_timer_handler()'s cost. Short traces, no signal-integrity
            // testing done — watch for garbled pixels after flashing.
            cfg.freq_write = 80000000;
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
            // GC9A01 panels commonly need color inversion — unverified on
            // this specific board, check the first fillScreen() and flip
            // if colors look wrong.
            cfg.invert = true;
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
