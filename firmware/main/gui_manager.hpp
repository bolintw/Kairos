#pragma once

#include "lgfx_config.hpp"
#include "lvgl.h"

// Font for the primary timer display, pulled out so it's a one-line
// change to try a different size/family while tuning layout. Note: LVGL
// only compiles in font sizes enabled via menuconfig
// (CONFIG_LV_FONT_MONTSERRAT_*, in sdkconfig) — swapping this to a size
// that isn't enabled there needs that turned on too.
constexpr const lv_font_t* kPrimaryFont = &lv_font_montserrat_32;

// Panel is 240x240 (see lgfx_config.hpp / main.cpp's lv_display_create).
constexpr int32_t kPanelSizePx = 240;

// Size of root_, the rotating container around the primary label — NOT
// the full panel. See the design note below for why: a full 240x240
// rotating container crashed twice on real hardware (task watchdog,
// 2026-08-24/25). Sized generously for the longest primary text expected
// ("Focus 25:00" at kPrimaryFont) with margin, kept much smaller than the
// panel so LVGL's software rotation — on either its fast matrix path or
// the slow per-pixel fallback — stays within a tractable pixel count and
// (worst case, ARGB8888 fallback) a tractable buffer size
// (kRootWidthPx*kRootHeightPx*4 ~= 47KB, comfortably under the LVGL heap
// — see CONFIG_LV_MEM_SIZE_KILOBYTES in sdkconfig).
constexpr int32_t kRootWidthPx = 200;
constexpr int32_t kRootHeightPx = 60;

// M7: owns the LVGL widget(s) and the backlight (via LGFX's Light_PWM,
// injected by reference — DI, no Singleton, matching the rest of the
// project). TimerFace decides *what* to show (via render()) and
// AppController decides brightness/color *state* (via SetBrightness/
// SetWarmth, driven by its notification state machine); GuiManager only
// knows how to paint whatever it's told.
//
// Screen counter-rotation (2026-08-24, revised 2026-08-25 after three
// hardware crashes): root_ wraps only the primary label — NOT the whole
// screen, and NOT the debug overlay label (that stays a plain fixed
// child of lv_screen_active(), created directly in main.cpp, deliberately
// not rotating). History:
//   1. First attempt made root_ the full 240x240 panel with both labels
//      parented under it: hung (task watchdog) — full-screen software
//      rotation needs a ~225KB ARGB8888 buffer the 64KB LVGL heap can't
//      satisfy, and LVGL's flush loop spins forever re-trying rather
//      than failing gracefully.
//   2. Tried LV_DRAW_TRANSFORM_USE_MATRIX (a faster rotation path that
//      composites via a matrix instead of a big intermediate buffer):
//      still hung — that path only engages when the rotated object's
//      bounding box isn't clipped by the current redraw's dirty rect,
//      which a full-screen container usually is (a label's own text
//      update typically only dirties its own small area), so it kept
//      falling back to the slow buffer path anyway.
//   3. Shrunk root_ to just the primary label's footprint (this version)
//      with LV_DRAW_TRANSFORM_USE_MATRIX still on: crashed instead of
//      hanging (Guru Meditation / LoadProhibited, NULL dest_buf inside
//      LVGL's RGB565_SWAPPED glyph-mask blend). That matrix path appears
//      to not correctly wire up the destination buffer for text glyphs
//      under our custom LV_COLOR_FORMAT_RGB565_SWAPPED (needed to match
//      LovyanGFX's byte order) — looks like an LVGL bug in a fairly
//      unusual combination (non-default matrix path + non-default color
//      format + glyph rendering), not something to work around here.
//      LV_DRAW_TRANSFORM_USE_MATRIX turned back off in sdkconfig; the
//      small root_ alone keeps the older buffer-based path (which does
//      work correctly, just needed to stay small) fast and memory-safe
//      without needing the matrix shortcut.
// SetRotationDeg() takes screen_angle_deg directly — same CW-positive
// convention as AttitudeEstimator — and negates it: LVGL's
// transform_rotation is also CW-positive (matches its arc widget's
// documented convention), so rotating the content by the negative of the
// device's own physical rotation keeps it upright to the user regardless
// of how the device is twisted in hand. Sign not yet confirmed on real
// hardware — flip kRotationSign in the .cpp if it comes out backwards.
class GuiManager {
public:
    explicit GuiManager(LGFX& lcd);

    void SetPrimaryText(const char* text);

    // brightness: 0.0 (off) to 1.0 (full). Scaled to the LGFX/LEDC 0-255
    // range internally.
    void SetBrightness(float brightness);

    // warmth: 0.0 (normal/white text) to 1.0 (fully warm/red) — the
    // "尾聲脈動...色調偏暖/紅" notification. Linear RGB interpolation
    // between white and a warm red, not true HSV hue rotation — simple
    // and sufficient for a single-color text label.
    void SetWarmth(float warmth);

    // Counter-rotates the primary label's small root_ container (see
    // class doc) so it stays upright as the physical device rotates.
    // screen_angle_deg: AttitudeEstimator::Output::screen_angle_deg,
    // straight from the estimator, no filtering applied here.
    void SetRotationDeg(float screen_angle_deg);

    // Counts real LVGL updates (SetPrimaryText/SetRotationDeg calls that
    // weren't skipped by the value-unchanged check below) — for a debug
    // fps readout. Deliberately NOT a count of lvgl_flush_cb calls: root_
    // is taller than one draw-buffer chunk (200x60 vs the ~24-row chunk
    // size for that width), so LVGL's partial-mode redraw splits a single
    // logical update into ~3 flush calls — counting those directly
    // overstates the real update rate by that factor (confirmed on
    // hardware 2026-08-25: a stationary mount still showed ~90-something
    // "fps" from sensor noise crossing the 0.1-degree rotation threshold,
    // which was actually ~30 real updates/sec, 3x-inflated).
    uint32_t GetUpdateCount() const { return update_count_; }

private:
    // SetPrimaryText/SetRotationDeg are called every sensor tick (120Hz)
    // regardless of whether the value actually changed — but
    // lv_label_set_text() has no same-text check of its own (always
    // reallocates + marks dirty, see lv_label.c), and a style setter
    // called with an unchanged value still invalidates. Left unguarded,
    // that's 120 redraws/sec even while the displayed second and angle
    // are both holding still (e.g. paused, or resting stationary) — most
    // of the battery cost this was meant to avoid. Comparing against the
    // last value here (2026-08-25) means a redraw only actually happens
    // when something visibly changed: ticking seconds settle to ~1
    // redraw/sec on their own, a stationary angle settles to ~0.
    char last_text_[32] = "";
    int32_t last_rotation_0p1_deg_ = 0;
    bool has_last_rotation_ = false;
    uint32_t update_count_ = 0;

    LGFX& lcd_;
    lv_obj_t* root_;
    lv_obj_t* label_;
};
