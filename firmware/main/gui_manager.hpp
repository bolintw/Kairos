#pragma once

#include "lvgl.h"

// Font for the primary timer display, pulled out so it's a one-line
// change to try a different size/family while tuning layout. Note: LVGL
// only compiles in font sizes enabled via menuconfig
// (CONFIG_LV_FONT_MONTSERRAT_*, in sdkconfig) — swapping this to a size
// that isn't enabled there needs that turned on too.
constexpr const lv_font_t* kPrimaryFont = &lv_font_montserrat_32;

// DRAFT, deliberately minimal. Full LVGL integration (screen
// counter-rotation driven by AttitudeEstimator::Output::screen_angle_deg,
// brightness/hue notifications) is M7's scope — see plan. For now this
// exists only so TimerFace::render() has something concrete to call:
// TimerFace decides *what* text to show, GuiManager just draws it. Expect
// this class to grow significantly at M7; not trying to anticipate that
// shape here.
class GuiManager {
public:
    GuiManager();

    void SetPrimaryText(const char* text);

private:
    lv_obj_t* label_;
};
