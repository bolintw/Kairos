#pragma once

#include "lgfx_config.hpp"
#include "lvgl.h"

// Font for the primary timer display, pulled out so it's a one-line
// change to try a different size/family while tuning layout. Note: LVGL
// only compiles in font sizes enabled via menuconfig
// (CONFIG_LV_FONT_MONTSERRAT_*, in sdkconfig) — swapping this to a size
// that isn't enabled there needs that turned on too.
constexpr const lv_font_t* kPrimaryFont = &lv_font_montserrat_32;

// M7: owns the LVGL widget(s) and the backlight (via LGFX's Light_PWM,
// injected by reference — DI, no Singleton, matching the rest of the
// project). TimerFace decides *what* to show (via render()) and
// AppController decides brightness/color *state* (via SetBrightness/
// SetWarmth, driven by its notification state machine); GuiManager only
// knows how to paint whatever it's told. Screen counter-rotation
// (following AttitudeEstimator::Output::screen_angle_deg) is still
// deferred — plan flags it nice-to-have, not required for v1.
class GuiManager {
public:
    explicit GuiManager(LGFX& lcd);

    void SetPrimaryText(const char* text);

    // brightness: 0.0 (off) to 1.0 (full). Scaled to the LGFX/LEDC 0-255
    // range internally.
    void SetBrightness(float brightness);

    // warmth: 0.0 (normal/white text) to 1.0 (fully warm/red) — the
    // "尾聲脈動...色調偏暖/紅" notification. Linear RGB interpolation
    // between white and a warm red, not true HSV hue rotation; simple
    // and sufficient for a single-color text label.
    void SetWarmth(float warmth);

private:
    LGFX& lcd_;
    lv_obj_t* label_;
};
