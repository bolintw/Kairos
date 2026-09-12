#include "gui_manager.hpp"

#include <cmath>
#include <cstring>

#include "esp_timer.h"

namespace {
float Clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

// Converts AttitudeEstimator's CCW-positive screen_angle_deg into LVGL's
// fixed CW-positive transform_rotation before negating it to counter-rotate.
constexpr float kRotationSign = 1.0f;

// Floor on how often the expensive rotated redraw is allowed to fire,
// independent of the sensor tick rate. ~30Hz: fast enough to read as
// smooth motion.
constexpr int64_t kRotationUpdateMinIntervalUs = 33 * 1000;

// Battery icon geometry — a horizontal battery icon: a gray outline rect
// with a small terminal nub on the right, kBatteryBlockCount vertical
// segments inside filling left-to-right. Centered on the panel.
constexpr int32_t kBatteryIconWidthPx = 150;
constexpr int32_t kBatteryIconHeightPx = 70;
constexpr int32_t kBatteryIconXPx = (kPanelSizePx - kBatteryIconWidthPx) / 2;
constexpr int32_t kBatteryIconYPx = (kPanelSizePx - kBatteryIconHeightPx) / 2;
// "outline stroke thickness around 5" — the outline's own border stroke.
constexpr int32_t kBatteryBorderWidthPx = 5;
constexpr int32_t kBatteryCornerRadiusPx = 10;
// Terminal nub — the small bump on the right edge every battery icon
// has, same reference image. A plain filled rect in the same gray as the
// outline border, not its own border — small enough that the distinction
// wouldn't read at this size.
constexpr int32_t kBatteryNubWidthPx = 10;
constexpr int32_t kBatteryNubHeightPx = 28;
// Segment layout: inset from the outline's own border by kBatteryPadPx on
// every side, then kBatteryBlockCount equal-width bars with
// kBatterySegGapPx between neighbors, filling the inset area's full
// height.
constexpr int32_t kBatteryPadPx = 6;
constexpr int32_t kBatterySegGapPx = 5;
constexpr int32_t kBatterySegAreaXPx = kBatteryIconXPx + kBatteryBorderWidthPx + kBatteryPadPx;
constexpr int32_t kBatterySegAreaYPx = kBatteryIconYPx + kBatteryBorderWidthPx + kBatteryPadPx;
constexpr int32_t kBatterySegAreaWidthPx = kBatteryIconWidthPx - 2 * (kBatteryBorderWidthPx + kBatteryPadPx);
constexpr int32_t kBatterySegAreaHeightPx = kBatteryIconHeightPx - 2 * (kBatteryBorderWidthPx + kBatteryPadPx);
constexpr int32_t kBatterySegWidthPx =
    (kBatterySegAreaWidthPx - (GuiManager::kBatteryBlockCount - 1) * kBatterySegGapPx) / GuiManager::kBatteryBlockCount;
constexpr int32_t kBatterySegCornerRadiusPx = 3;
constexpr lv_color_t kBatteryGray = LV_COLOR_MAKE(140, 140, 140);
constexpr lv_color_t kBatteryEmptyGray = LV_COLOR_MAKE(60, 60, 60);

// One fixed color per fill count, red (1 segment) -> green (all filled).
lv_color_t BatteryTierColor(int filled_blocks)
{
    switch (filled_blocks) {
        case 1: return lv_color_make(210, 50, 50);    // red
        case 2: return lv_color_make(220, 120, 40);   // orange
        case 3: return lv_color_make(210, 190, 40);   // yellow
        case 4: return lv_color_make(150, 195, 50);   // yellow-green
        default: return lv_color_make(60, 175, 80);   // green (5+)
    }
}
}  // namespace

GuiManager::GuiManager(LGFX& lcd)
    : lcd_(lcd)
{
    // Centered on the panel so root_'s local center coincides with the
    // true screen center — pivoting there keeps rotation correct without
    // extra offset math.
    root_ = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(root_);  // no border/padding/scrollbar — just a rotation anchor
    lv_obj_set_size(root_, kRootWidthPx, kRootHeightPx);
    lv_obj_set_pos(root_, (kPanelSizePx - kRootWidthPx) / 2, (kPanelSizePx - kRootHeightPx) / 2);
    // root_ is a rotation pivot, not a visual mask — by default LVGL
    // clips children to its box, which would cut off wrapped 2-line text.
    lv_obj_add_flag(root_, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_set_style_transform_pivot_x(root_, kRootWidthPx / 2, 0);
    lv_obj_set_style_transform_pivot_y(root_, kRootHeightPx / 2, 0);

    // Bottom-aligned, not centered, to make room for secondary_label_
    // above it — the two line_heights sum to exactly kRootHeightPx.
    label_ = lv_label_create(root_);
    lv_obj_set_style_text_font(label_, kPrimaryFont, 0);
    lv_obj_set_style_text_align(label_, LV_TEXT_ALIGN_CENTER, 0);
    // Nudged 3px toward secondary_label_ to close the visible gap each
    // font's line-height leading leaves even when the boxes are flush.
    lv_obj_align(label_, LV_ALIGN_BOTTOM_MID, 0, -3);
    // Explicit, not left to SetWarmth(0.0f)'s side effect — AppController
    // doesn't always call SetWarmth, and the default text color is
    // near-invisible on the black background.
    lv_obj_set_style_text_color(label_, lv_color_white(), 0);

    // Small built-in font, not the custom primary one, so its line_height
    // is known and small enough to fit above label_ within kRootHeightPx.
    secondary_label_ = lv_label_create(root_);
    lv_obj_set_style_text_font(secondary_label_, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_align(secondary_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(secondary_label_, LV_ALIGN_TOP_MID, 0, 3);  // same nudge as label_, opposite direction
    lv_obj_set_style_text_color(secondary_label_, lv_color_white(), 0);
    // Explicit: a fresh lv_label_t defaults to LVGL's own placeholder
    // text ("Text"), not empty.
    lv_label_set_text(secondary_label_, "");

    // Dim background track: created before ring_/ring_tick_ below so it
    // z-orders underneath both. Always a full circle at fixed low
    // opacity; SetAccentColor() gives it the same hue as the other two.
    ring_track_ = lv_arc_create(lv_screen_active());
    lv_obj_remove_style_all(ring_track_);
    lv_obj_set_size(ring_track_, kRingRadiusPx * 2, kRingRadiusPx * 2);
    lv_obj_set_pos(ring_track_, (kPanelSizePx - kRingRadiusPx * 2) / 2, (kPanelSizePx - kRingRadiusPx * 2) / 2);
    lv_arc_set_bg_angles(ring_track_, 0, 360);
    lv_obj_set_style_arc_width(ring_track_, kRingWidthPx, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(ring_track_, false, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ring_track_, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(ring_track_, kRingTrackOpa, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(ring_track_, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(ring_track_, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(ring_track_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(ring_track_, LV_OBJ_FLAG_HIDDEN);

    // lv_arc, not a plain lv_obj border, since the ring needs a partial
    // circle whose span changes every tick (SetRingProgress()). A fixed
    // -90 base rotation plus recomputed angles each call covers
    // everything needed, so this stays static on lv_screen_active(), not
    // a root_ child. LV_PART_MAIN is the only part drawn.
    ring_ = lv_arc_create(lv_screen_active());
    lv_obj_remove_style_all(ring_);
    lv_obj_set_size(ring_, kRingRadiusPx * 2, kRingRadiusPx * 2);
    lv_obj_set_pos(ring_, (kPanelSizePx - kRingRadiusPx * 2) / 2, (kPanelSizePx - kRingRadiusPx * 2) / 2);
    lv_arc_set_rotation(ring_, -90);  // angle 0 = 12 o'clock, sweeps clockwise — see SetRingProgress()
    lv_arc_set_bg_angles(ring_, 0, 360);
    lv_obj_set_style_arc_width(ring_, kRingWidthPx, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(ring_, false, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ring_, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(ring_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(ring_, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(ring_, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(ring_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(ring_, LV_OBJ_FLAG_HIDDEN);

    // Color follows SetAccentColor() like ring_/ring_track_; this initial
    // white is just the pre-first-render default.
    ring_tick_ = lv_arc_create(lv_screen_active());
    lv_obj_remove_style_all(ring_tick_);
    lv_obj_set_size(ring_tick_, kRingTickRadiusPx * 2, kRingTickRadiusPx * 2);
    lv_obj_set_pos(ring_tick_, (kPanelSizePx - kRingTickRadiusPx * 2) / 2,
                    (kPanelSizePx - kRingTickRadiusPx * 2) / 2);
    lv_arc_set_rotation(ring_tick_, -90);
    lv_arc_set_bg_angles(ring_tick_, 0, 0);
    lv_obj_set_style_arc_width(ring_tick_, kRingTickWidthPx, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(ring_tick_, false, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ring_tick_, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_arc_opa(ring_tick_, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(ring_tick_, LV_OPA_TRANSP, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(ring_tick_, LV_OPA_TRANSP, LV_PART_KNOB);
    lv_obj_remove_flag(ring_tick_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(ring_tick_, LV_OBJ_FLAG_HIDDEN);

    // Battery icon: plain children of lv_screen_active(), same
    // non-rotating treatment as ring_. Hidden by default; ShowBatteryView(true)
    // reveals them. battery_segments_ start as "empty" dim bars —
    // SetBatteryLevel() fills in the actual state before this is shown.

    // Outline: border only, no fill — lv_obj_create's default bg would
    // otherwise show through as a solid gray rectangle behind the segments.
    battery_outline_ = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(battery_outline_);
    lv_obj_set_size(battery_outline_, kBatteryIconWidthPx, kBatteryIconHeightPx);
    lv_obj_set_pos(battery_outline_, kBatteryIconXPx, kBatteryIconYPx);
    lv_obj_set_style_radius(battery_outline_, kBatteryCornerRadiusPx, 0);
    lv_obj_set_style_bg_opa(battery_outline_, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(battery_outline_, kBatteryBorderWidthPx, 0);
    lv_obj_set_style_border_color(battery_outline_, kBatteryGray, 0);
    lv_obj_set_style_border_opa(battery_outline_, LV_OPA_COVER, 0);
    lv_obj_remove_flag(battery_outline_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(battery_outline_, LV_OBJ_FLAG_HIDDEN);

    // Terminal nub — the small bump every battery icon has on its "+" end
    // (see kBatteryNubWidthPx's comment). A plain filled rect, same gray
    // as the outline border.
    battery_nub_ = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(battery_nub_);
    lv_obj_set_size(battery_nub_, kBatteryNubWidthPx, kBatteryNubHeightPx);
    lv_obj_set_pos(battery_nub_, kBatteryIconXPx + kBatteryIconWidthPx,
                    kBatteryIconYPx + (kBatteryIconHeightPx - kBatteryNubHeightPx) / 2);
    lv_obj_set_style_radius(battery_nub_, 3, 0);
    lv_obj_set_style_bg_opa(battery_nub_, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(battery_nub_, kBatteryGray, 0);
    lv_obj_remove_flag(battery_nub_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(battery_nub_, LV_OBJ_FLAG_HIDDEN);

    // Segments: kBatteryBlockCount equal-width bars, left-to-right, inset
    // from the outline's border by kBatteryPadPx — see the geometry
    // constants above.
    for (int i = 0; i < kBatteryBlockCount; ++i) {
        lv_obj_t* seg = lv_obj_create(lv_screen_active());
        lv_obj_remove_style_all(seg);
        lv_obj_set_size(seg, kBatterySegWidthPx, kBatterySegAreaHeightPx);
        lv_obj_set_pos(seg, kBatterySegAreaXPx + i * (kBatterySegWidthPx + kBatterySegGapPx), kBatterySegAreaYPx);
        lv_obj_set_style_radius(seg, kBatterySegCornerRadiusPx, 0);
        lv_obj_set_style_bg_opa(seg, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(seg, kBatteryEmptyGray, 0);  // dim/"empty" default
        lv_obj_remove_flag(seg, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(seg, LV_OBJ_FLAG_HIDDEN);
        battery_segments_[i] = seg;
    }
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
    // Note: a real 2-line string here (e.g. "Low\nBattery") clips its top
    // line on hardware — re-aligning or calling lv_obj_update_layout()
    // after the text change doesn't fix it. Split multi-line content
    // across secondary_label_ + label_ instead (both single-line).
}

void GuiManager::SetSecondaryText(const char* text)
{
    if (std::strncmp(last_secondary_text_, text, sizeof(last_secondary_text_)) == 0) {
        return;  // unchanged — same reasoning as SetPrimaryText's dirty-check
    }
    std::strncpy(last_secondary_text_, text, sizeof(last_secondary_text_) - 1);
    last_secondary_text_[sizeof(last_secondary_text_) - 1] = '\0';
    lv_label_set_text(secondary_label_, text);
    ++update_count_;
}

void GuiManager::SetPrimaryTextOpacity(uint8_t opa)
{
    if (has_last_text_opa_ && opa == last_text_opa_) {
        return;  // unchanged — same reasoning as SetPrimaryText's dirty-check
    }
    last_text_opa_ = opa;
    has_last_text_opa_ = true;
    lv_obj_set_style_text_opa(label_, opa, 0);
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
    lv_obj_set_style_text_color(secondary_label_, color, 0);
    lv_obj_set_style_arc_color(ring_track_, color, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ring_, color, LV_PART_MAIN);
    lv_obj_set_style_arc_color(ring_tick_, color, LV_PART_MAIN);
}

void GuiManager::SetRingOrientation(float face_center_deg)
{
    // -90 reproduces the original fixed behavior for face B
    // (face_center_deg==0). kRotationSign applies the same compensating
    // rotation SetRotationDeg() uses for root_, just computed once here
    // per face switch instead of every tick.
    const int32_t rotation_deg = static_cast<int32_t>(std::lround(-90.0f + kRotationSign * face_center_deg));
    lv_arc_set_rotation(ring_, rotation_deg);
    lv_arc_set_rotation(ring_tick_, rotation_deg);
    ++update_count_;
}

void GuiManager::SetRingProgress(float elapsed_fraction, bool growing)
{
    float frac = elapsed_fraction;
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    const int32_t angle_deg10 = static_cast<int32_t>(frac * 3600.0f + 0.5f);  // tenths of a degree

    if (angle_deg10 == last_ring_angle_deg10_ && growing == last_ring_growing_) {
        return;
    }
    last_ring_angle_deg10_ = angle_deg10;
    last_ring_growing_ = growing;

    const uint16_t boundary_deg = static_cast<uint16_t>(angle_deg10 / 10);
    if (growing) {
        lv_arc_set_bg_angles(ring_, 0, boundary_deg);
    } else {
        lv_arc_set_bg_angles(ring_, boundary_deg, 360);
    }

    // Tick: centered on the boundary angle, kept in [0, 360] by shifting
    // its whole span rather than clamping each edge independently — a
    // per-edge clamp would render only half the tick near boundary_deg≈0,
    // reading as thinner than normal for the first few seconds of every phase.
    float tick_start = static_cast<float>(boundary_deg) - kRingTickHalfSpanDeg;
    float tick_end = static_cast<float>(boundary_deg) + kRingTickHalfSpanDeg;
    if (tick_start < 0.0f) {
        tick_end -= tick_start;  // shift right by the overhang, preserving width
        tick_start = 0.0f;
    } else if (tick_end > 360.0f) {
        tick_start -= (tick_end - 360.0f);  // shift left by the overhang, preserving width
        tick_end = 360.0f;
    }
    lv_arc_set_bg_angles(ring_tick_, static_cast<uint16_t>(tick_start), static_cast<uint16_t>(tick_end));

    ++update_count_;
}

void GuiManager::SetRingOpacity(uint8_t opa)
{
    if (has_last_ring_opa_ && opa == last_ring_opa_) {
        return;  // unchanged — same dirty-check reasoning as the rest of this class
    }
    last_ring_opa_ = opa;
    has_last_ring_opa_ = true;
    lv_obj_set_style_arc_opa(ring_, opa, LV_PART_MAIN);
    ++update_count_;
}

void GuiManager::SetRingVisible(bool visible)
{
    if (has_last_ring_visible_ && visible == last_ring_visible_) {
        return;  // unchanged — same dirty-check reasoning as the rest of this class
    }
    last_ring_visible_ = visible;
    has_last_ring_visible_ = true;
    if (visible) {
        lv_obj_clear_flag(ring_track_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ring_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ring_tick_, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(ring_track_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ring_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ring_tick_, LV_OBJ_FLAG_HIDDEN);
    }
    ++update_count_;
}

void GuiManager::SetRingTickOpacity(uint8_t opa)
{
    if (has_last_ring_tick_opa_ && opa == last_ring_tick_opa_) {
        return;  // unchanged — same dirty-check reasoning as the rest of this class
    }
    last_ring_tick_opa_ = opa;
    has_last_ring_tick_opa_ = true;
    lv_obj_set_style_arc_opa(ring_tick_, opa, LV_PART_MAIN);
    ++update_count_;
}

void GuiManager::SetRotationDeg(float screen_angle_deg)
{
    // Full precision still passed to LVGL below — only the *decision to
    // redraw* is coarsened/throttled, see the header comment.
    const int32_t rot_0p1_deg = static_cast<int32_t>(kRotationSign * screen_angle_deg * 10.0f);
    const int32_t rot_1deg = rot_0p1_deg / 10;

    if (has_last_rotation_ && rot_1deg == last_rotation_1deg_) {
        return;  // hasn't moved a full degree yet
    }
    const int64_t now_us = esp_timer_get_time();
    if (has_last_rotation_ && (now_us - last_rotation_update_us_) < kRotationUpdateMinIntervalUs) {
        return;  // moved, but too soon after the last redraw
    }

    last_rotation_1deg_ = rot_1deg;
    last_rotation_update_us_ = now_us;
    has_last_rotation_ = true;
    lv_obj_set_style_transform_rotation(root_, rot_0p1_deg, 0);
    ++update_count_;
}

void GuiManager::ShowBatteryView(bool show)
{
    if (show == showing_battery_view_) {
        return;  // unchanged — same dirty-check reasoning as the rest of this class
    }
    showing_battery_view_ = show;
    if (show) {
        lv_obj_add_flag(root_, LV_OBJ_FLAG_HIDDEN);
        // SetRingVisible(), not a direct flag manipulation like root_
        // above — this keeps last_ring_visible_ in sync, so the
        // SetRingVisible(true) call UpdateRing() makes once battery view
        // closes (see the `else` branch below) isn't skipped by its own
        // dirty-check for looking like a no-op change.
        SetRingVisible(false);
        lv_obj_clear_flag(battery_outline_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(battery_nub_, LV_OBJ_FLAG_HIDDEN);
        for (int i = 0; i < kBatteryBlockCount; ++i) {
            lv_obj_clear_flag(battery_segments_[i], LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        lv_obj_clear_flag(root_, LV_OBJ_FLAG_HIDDEN);
        // ring_/ring_tick_ deliberately left hidden here, not unhidden —
        // AppController::UpdateRing() runs again the same tick this
        // returns to (see its early-return in Update()) and calls
        // SetRingVisible() with whatever's actually correct for the
        // current face; unhiding unconditionally here risked a one-tick
        // flash of stale content before that correction landed.
        lv_obj_add_flag(battery_outline_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(battery_nub_, LV_OBJ_FLAG_HIDDEN);
        for (int i = 0; i < kBatteryBlockCount; ++i) {
            lv_obj_add_flag(battery_segments_[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
    ++update_count_;
}

void GuiManager::SetBatteryLevel(int filled_blocks)
{
    int clamped = filled_blocks;
    if (clamped < 1) clamped = 1;
    if (clamped > kBatteryBlockCount) clamped = kBatteryBlockCount;

    if (clamped == last_battery_filled_blocks_) {
        return;  // unchanged — same dirty-check reasoning as the rest of this class
    }
    last_battery_filled_blocks_ = clamped;

    const lv_color_t color = BatteryTierColor(clamped);
    for (int i = 0; i < kBatteryBlockCount; ++i) {
        // Fills left-to-right: the first `clamped` segments light up.
        const bool filled = i < clamped;
        lv_obj_set_style_bg_color(battery_segments_[i], filled ? color : kBatteryEmptyGray, 0);
    }
    ++update_count_;
}

void GuiManager::ForceRedraw()
{
    // Re-running init() a second time is safe — LovyanGFX guards the SPI
    // bus setup, so this only redoes the panel-level register writes.
    lcd_.init();
    lv_obj_invalidate(lv_screen_active());
}
