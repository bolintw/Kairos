#include "gui_manager.hpp"

namespace {
float Clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}
}  // namespace

GuiManager::GuiManager(LGFX& lcd)
    : lcd_(lcd)
{
    label_ = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_font(label_, kPrimaryFont, 0);
    lv_obj_set_style_text_align(label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label_, LV_ALIGN_CENTER, 0, 0);
}

void GuiManager::SetPrimaryText(const char* text)
{
    lv_label_set_text(label_, text);
}

void GuiManager::SetBrightness(float brightness)
{
    lcd_.setBrightness(static_cast<uint8_t>(Clamp01(brightness) * 255.0f + 0.5f));
}

void GuiManager::SetWarmth(float warmth)
{
    const float w = Clamp01(warmth);

    // White at warmth=0, a warm red/orange at warmth=1. Linear RGB
    // interpolation, not true HSV hue rotation — plenty for a single
    // solid-color text label.
    constexpr uint8_t kWarmR = 255, kWarmG = 80, kWarmB = 40;
    const uint8_t r = static_cast<uint8_t>(255.0f + w * (kWarmR - 255));
    const uint8_t g = static_cast<uint8_t>(255.0f + w * (kWarmG - 255));
    const uint8_t b = static_cast<uint8_t>(255.0f + w * (kWarmB - 255));
    lv_obj_set_style_text_color(label_, lv_color_make(r, g, b), 0);
}
