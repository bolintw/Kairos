#include "gui_manager.hpp"

#include <cstring>

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
    // Set directly here, not left to SetWarmth(0.0f)'s side effect —
    // AppController stopped calling SetWarmth (2026-08-25, brightness-only
    // notifications), which silently left label_ on LVGL's default text
    // color (near-invisible on the black background) since nothing else
    // ever set it. SetWarmth() still exists and still works if the color
    // channel comes back, but this baseline can't depend on it being
    // called at all.
    lv_obj_set_style_text_color(label_, lv_color_white(), 0);

    // Plain static circle, no rotation — see the kRingRadiusPx/kRingWidthPx
    // comment in gui_manager.hpp for why this is safe to make full-size
    // unlike root_. Centered the same way as root_ (pivot math doesn't
    // matter here since it never transforms, but centering the object
    // itself on the panel does).
    ring_ = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(ring_);
    lv_obj_set_size(ring_, kRingRadiusPx * 2, kRingRadiusPx * 2);
    lv_obj_set_pos(ring_, (kPanelSizePx - kRingRadiusPx * 2) / 2, (kPanelSizePx - kRingRadiusPx * 2) / 2);
    lv_obj_set_style_radius(ring_, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(ring_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ring_, kRingWidthPx, 0);
    lv_obj_set_style_border_color(ring_, lv_color_white(), 0);
    lv_obj_set_style_border_opa(ring_, LV_OPA_TRANSP, 0);
}

void GuiManager::SetPrimaryText(const char* text)
{
    if (std::strncmp(last_text_, text, sizeof(last_text_)) == 0) {
        return;  // unchanged — see the field comment in gui_manager.hpp
    }
    std::strncpy(last_text_, text, sizeof(last_text_) - 1);
    last_text_[sizeof(last_text_) - 1] = '\0';
    lv_label_set_text(label_, text);
    ++update_count_;
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

void GuiManager::SetAccentColor(lv_color_t color)
{
    lv_obj_set_style_text_color(label_, color, 0);
    lv_obj_set_style_border_color(ring_, color, 0);
}

void GuiManager::SetRingOpacity(uint8_t opa)
{
    if (has_last_ring_opa_ && opa == last_ring_opa_) {
        return;  // unchanged — same reasoning as SetPrimaryText's dirty-check
    }
    last_ring_opa_ = opa;
    has_last_ring_opa_ = true;
    lv_obj_set_style_border_opa(ring_, opa, 0);
    ++update_count_;
}

void GuiManager::SetRotationDeg(float screen_angle_deg)
{
    const int32_t rot_0p1_deg = static_cast<int32_t>(kRotationSign * screen_angle_deg * 10.0f);
    if (has_last_rotation_ && rot_0p1_deg == last_rotation_0p1_deg_) {
        return;  // unchanged (in the 0.1-degree units LVGL sees) — see gui_manager.hpp
    }
    last_rotation_0p1_deg_ = rot_0p1_deg;
    has_last_rotation_ = true;
    lv_obj_set_style_transform_rotation(root_, rot_0p1_deg, 0);
    ++update_count_;
}
