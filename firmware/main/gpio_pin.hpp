#pragma once

#include "driver/gpio.h"

class GpioPin {
public:
    enum class Direction { Input, Output };

    GpioPin(gpio_num_t pin, Direction direction) : pin_(pin) {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << pin_;
        cfg.mode = (direction == Direction::Output) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&cfg);
    }

    ~GpioPin() {
        gpio_reset_pin(pin_);
    }

    GpioPin(const GpioPin&) = delete;
    GpioPin& operator=(const GpioPin&) = delete;

    void set(bool level) const {
        gpio_set_level(pin_, level ? 1 : 0);
    }

    bool get() const {
        return gpio_get_level(pin_) != 0;
    }

private:
    gpio_num_t pin_;
};
