#include "gui_manager.hpp"

namespace {
float Clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

// See gui_manager.hpp's rotation design note — flip this to +1.0f if the
// overlay turns out to spin the wrong way on real hardware.
constexpr float kRotationSign = -1.0f;
}  // namespace

GuiManager::GuiManager(LGFX& lcd)
    : lcd_(lcd)
{
    // Centered on the panel so root_'s own local center coincides with
    // the true screen center (kPanelSizePx/2, kPanelSizePx/2) — pivoting
    // there keeps rotation looking correct without extra offset math.
    root_ = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(root_);  // no border/padding/scrollbar — just a rotation anchor
    lv_obj_set_size(root_, kRootWidthPx, kRootHeightPx);
    lv_obj_set_pos(root_, (kPanelSizePx - kRootWidthPx) / 2, (kPanelSizePx - kRootHeightPx) / 2);
    lv_obj_set_style_transform_pivot_x(root_, kRootWidthPx / 2, 0);
    lv_obj_set_style_transform_pivot_y(root_, kRootHeightPx / 2, 0);

    label_ = lv_label_create(root_);
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

void GuiManager::SetRotationDeg(float screen_angle_deg)
{
    const int32_t rot_0p1_deg = static_cast<int32_t>(kRotationSign * screen_angle_deg * 10.0f);
    lv_obj_set_style_transform_rotation(root_, rot_0p1_deg, 0);
}
