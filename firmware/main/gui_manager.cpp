#include "gui_manager.hpp"

#include <cstring>

#include "esp_timer.h"

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

// See SetRotationDeg()'s comment — a floor on how often the expensive
// rotated redraw is allowed to fire, independent of the sensor tick rate
// (which this whole mechanism exists to stop throttling). ~30Hz: fast
// enough that a flip still reads as smooth motion, far below the 120Hz
// sensor rate this is decoupling the redraw cost from.
constexpr int64_t kRotationUpdateMinIntervalUs = 33 * 1000;

// Battery band geometry (2026-09-08) — see SetBatteryLevel()'s header
// comment. kBatteryBlockCount full-width rectangles stacked vertically to
// exactly cover the panel height, each kPanelSizePx/kBatteryBlockCount
// tall minus kBatteryBandGapPx split across its top/bottom edges so
// adjacent bands show a thin dark seam between them (the "cut" lines) —
// no per-height chord-width math needed, the round bezel already clips
// whatever's drawn to the visible circle.
// 3 -> 14 -> 10 (2026-09-08) — 14 was a bit too thick once the corner
// radius settled down to a gentle fillet (kBandCornerRadiusPx in the
// constructor) instead of the earlier pill-cap shape; landed on 10.
constexpr int32_t kBatteryBandGapPx = 10;

// One fixed color per fill count, red (1 band) -> green
// (GuiManager::kBatteryBlockCount bands) — see SetBatteryLevel()'s
// header comment for why this is a single accent color per level rather
// than a per-band gradient. Starting palette, not tuned against the
// real panel yet.
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
    // Centered on the panel so root_'s own local center coincides with
    // the true screen center (kPanelSizePx/2, kPanelSizePx/2) — pivoting
    // there keeps rotation looking correct without extra offset math.
    root_ = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(root_);  // no border/padding/scrollbar — just a rotation anchor
    lv_obj_set_size(root_, kRootWidthPx, kRootHeightPx);
    lv_obj_set_pos(root_, (kPanelSizePx - kRootWidthPx) / 2, (kPanelSizePx - kRootHeightPx) / 2);
    // root_ is a rotation pivot, not a visual mask (see the class doc) —
    // by default LVGL still clips children to its box, which silently cut
    // off the top of any primary text that wraps to 2 lines (kRootHeightPx
    // is sized for kPrimaryFont's single-line height, see below; two lines
    // is 2x that). Caught via calibration_mode.cpp's "Rotate\nand hold"
    // (2026-09-01) — this flag makes root_ a plain rotation anchor with no
    // clipping, matching what it's actually for.
    lv_obj_add_flag(root_, LV_OBJ_FLAG_OVERFLOW_VISIBLE);
    lv_obj_set_style_transform_pivot_x(root_, kRootWidthPx / 2, 0);
    lv_obj_set_style_transform_pivot_y(root_, kRootHeightPx / 2, 0);

    // Bottom-aligned rather than centered (2026-08-30, was LV_ALIGN_CENTER)
    // to make room for secondary_label_ above it — line_height 44 (this
    // font) + 16 (secondary_label_'s) == kRootHeightPx (60) exactly, so
    // the two stack flush with no gap or overlap and no need to grow
    // root_.
    label_ = lv_label_create(root_);
    lv_obj_set_style_text_font(label_, kPrimaryFont, 0);
    lv_obj_set_style_text_align(label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(label_, LV_ALIGN_BOTTOM_MID, 0, 0);
    // Set directly here, not left to SetWarmth(0.0f)'s side effect —
    // AppController stopped calling SetWarmth (2026-08-25, brightness-only
    // notifications), which silently left label_ on LVGL's default text
    // color (near-invisible on the black background) since nothing else
    // ever set it. SetWarmth() still exists and still works if the color
    // channel comes back, but this baseline can't depend on it being
    // called at all.
    lv_obj_set_style_text_color(label_, lv_color_white(), 0);

    // See gui_manager.hpp's SetSecondaryText comment — small built-in
    // font, not the custom primary one, specifically so its line_height
    // is known and small enough to fit above label_ within kRootHeightPx.
    secondary_label_ = lv_label_create(root_);
    lv_obj_set_style_text_font(secondary_label_, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_align(secondary_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(secondary_label_, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_color(secondary_label_, lv_color_white(), 0);
    // Explicit, same lesson as label_'s text-color init above: a fresh
    // lv_label_t defaults to LVGL's own placeholder text ("Text"), not
    // empty — left alone, that showed up as a real stray line on real
    // hardware the very first time BreathFace's idle screen rendered
    // (2026-08-30), because SetSecondaryText("")'s dirty-check compares
    // against last_secondary_text_'s initial value, which is already ""
    // — so that very first idle-screen call to clear it looked like a
    // no-op and never actually reached lv_label_set_text() at all. Only
    // cleared once some later *real* text ("Ready?") made the tracked
    // value diverge from the actual on-screen "Text" for the first time.
    lv_label_set_text(secondary_label_, "");

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

    // Battery bands (2026-09-08) — see the header's kBatteryBlockCount
    // comment. Plain children of lv_screen_active(), same non-rotating
    // treatment as ring_ above. Hidden by default; ShowBatteryView(true)
    // reveals them. Each starts as an "empty" dim band — SetBatteryLevel()
    // fills in the actual state before the view is ever shown to a user
    // (AppController calls it every tick while showing_battery_ is true).
    // Plain lv_obj rectangles, not a special widget: a full-width bar is
    // exactly what lv_obj_create + a bg color already does, no arc/shape
    // API needed once the shape wanted is horizontal stripes rather than
    // radial slices.
    {
        // Non-uniform band heights (2026-09-08) — equal fifths read too
        // mechanical; the user wants the middle band biggest, tapering
        // toward the top/bottom, with corners just gently filleted rather
        // than the full pill-cap shape tried first (kBandCornerRadiusPx
        // below, not height/2 anymore). Weights are relative, indexed the
        // same as battery_bands_ (i=0 bottom/fills-first .. top),
        // symmetric around the middle (index 2) band. Boundaries computed
        // via a running *cumulative* weight->pixel conversion, each
        // rounded independently only at the cumulative point — not each
        // band's height rounded on its own — so the 5 heights still sum to
        // exactly kPanelSizePx with no stray 1px gap or overlap from
        // independent rounding error.
        constexpr float kBatteryBandWeights[kBatteryBlockCount] = {1.0f, 1.35f, 1.7f, 1.35f, 1.0f};
        float total_weight = 0.0f;
        for (float w : kBatteryBandWeights) total_weight += w;

        int32_t cum_height_px[kBatteryBlockCount + 1];
        cum_height_px[0] = 0;
        float cum_weight = 0.0f;
        for (int i = 0; i < kBatteryBlockCount; ++i) {
            cum_weight += kBatteryBandWeights[i];
            cum_height_px[i + 1] = static_cast<int32_t>(kPanelSizePx * cum_weight / total_weight + 0.5f);
        }
        cum_height_px[kBatteryBlockCount] = kPanelSizePx;  // force exact total, absorb float rounding drift

        constexpr int32_t kBandCornerRadiusPx = 10;
        for (int i = 0; i < kBatteryBlockCount; ++i) {
            // i=0 is the bottom band (see header comment — fills
            // bottom-up), so its pixel range is the last slot counting
            // down from the panel's bottom edge.
            const int32_t band_height_px = cum_height_px[i + 1] - cum_height_px[i];
            const int32_t y_top = kPanelSizePx - cum_height_px[i + 1];
            const int32_t draw_height_px = band_height_px - kBatteryBandGapPx;
            lv_obj_t* band = lv_obj_create(lv_screen_active());
            lv_obj_remove_style_all(band);
            lv_obj_set_size(band, kPanelSizePx, draw_height_px);
            lv_obj_set_pos(band, 0, y_top + kBatteryBandGapPx / 2);
            lv_obj_set_style_radius(band, kBandCornerRadiusPx, 0);
            lv_obj_set_style_bg_opa(band, LV_OPA_COVER, 0);
            lv_obj_set_style_bg_color(band, lv_color_make(60, 60, 60), 0);  // dim/"empty" default
            lv_obj_remove_flag(band, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_add_flag(band, LV_OBJ_FLAG_HIDDEN);
            battery_bands_[i] = band;
        }
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
        lv_obj_add_flag(ring_, LV_OBJ_FLAG_HIDDEN);
        for (int i = 0; i < kBatteryBlockCount; ++i) {
            lv_obj_clear_flag(battery_bands_[i], LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        lv_obj_clear_flag(root_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ring_, LV_OBJ_FLAG_HIDDEN);
        for (int i = 0; i < kBatteryBlockCount; ++i) {
            lv_obj_add_flag(battery_bands_[i], LV_OBJ_FLAG_HIDDEN);
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
    const lv_color_t empty_color = lv_color_make(60, 60, 60);
    for (int i = 0; i < kBatteryBlockCount; ++i) {
        // i=0 is the bottom band (see the constructor's comment) —
        // filling bottom-up means the first `clamped` *lowest* bands light
        // up, which is exactly i < clamped here since i counts up from
        // the bottom.
        const bool filled = i < clamped;
        lv_obj_set_style_bg_color(battery_bands_[i], filled ? color : empty_color, 0);
    }
    ++update_count_;
}

void GuiManager::ForceRedraw()
{
    // Re-run the panel's own init sequence (RST toggle, GC9A01A memory
    // access control / etc. register writes) — added 2026-09-06 after
    // the "just invalidate everything" version alone didn't fix a blank
    // screen post-RunIdleSleep() even after switching to ESP-IDF's
    // automatic light sleep (see gui_manager.hpp's comment). Confirmed
    // safe to call again post-boot by reading LovyanGFX's own source
    // (platforms/esp32/common.cpp): the SPI bus setup is guarded by
    // `if (_spi_dev_handle[spi_host] == nullptr)`, so a second init()
    // call skips spi_bus_initialize()/spi_bus_add_device() entirely and
    // only redoes the panel-level register setup — not a resource leak
    // or a double-init failure. Call this before restoring brightness
    // (AppController::NotifyWokeFromIdleSleep()), since a full panel
    // re-init is exactly the kind of thing that could glitch the
    // backlight state along the way.
    lcd_.init();
    lv_obj_invalidate(lv_screen_active());
}
