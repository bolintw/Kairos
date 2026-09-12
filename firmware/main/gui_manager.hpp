#pragma once

#include "lgfx_config.hpp"
#include "lvgl.h"

// Space Grotesk Bold, 40px, full printable ASCII range (0x20-0x7E) so any
// string works, not just a hand-curated subset. SIL OFL 1.1, Copyright
// 2020 The Space Grotesk Project Authors; license text at
// SpaceGrotesk-OFL.txt. Regenerate with:
//   npx lv_font_conv --font reference/vendor/fonts/SpaceGrotesk-Bold.ttf
//   --size 40 --bpp 4 --format lvgl --no-compress --range "0x20-0x7E"
//   --lv-font-name font_space_grotesk_bold_40
//   -o firmware/main/font_space_grotesk_bold_40.c
// --no-compress is required (this build doesn't enable
// CONFIG_LV_USE_FONT_COMPRESSED); after regenerating, edit the file's top
// #include to a plain `#include "lvgl.h"`.
extern "C" const lv_font_t font_space_grotesk_bold_40;
constexpr const lv_font_t* kPrimaryFont = &font_space_grotesk_bold_40;

// Panel is 240x240 (see lgfx_config.hpp / main.cpp's lv_display_create).
constexpr int32_t kPanelSizePx = 240;

// Outer progress ring: a thin circular arc plus a small tick mark riding
// its moving edge. Sized close to the panel edge with a bit of margin.
// Neither piece counter-rotates continuously like root_ does — a per-face
// base rotation (re-snapped only on a face switch, see
// SetRingOrientation()) is enough — so both stay plain static lv_arc
// objects on lv_screen_active(), not root_ children.
constexpr int32_t kRingRadiusPx = 112;
constexpr int32_t kRingWidthPx = 7;

// Dim background track: a second, always-full-circle arc drawn behind
// ring_ at low fixed opacity, so the "already elapsed" portion of a
// countdown reads as a dim ring instead of nothing. Shares ring_'s
// geometry and hue; only opacity differs, and SetRingProgress() never
// touches it.
constexpr uint8_t kRingTrackOpa = 70;

// Tick mark: a short straight radial line from 12 o'clock in toward the
// center, at the countdown's current moving boundary. Built as a thin
// lv_arc segment (LVGL has no plain radial-line primitive) with
// arc_rounded=false (flat caps — rounded caps read as a blob instead of
// a line). kRingTickWidthPx is how far inward it reaches from the main
// ring's outer edge; kRingTickThicknessPx is its tangential thickness,
// converted to an angular half-span via arc-length = radius * angle(rad).
constexpr int32_t kRingTickRadiusPx = kRingRadiusPx;
constexpr int32_t kRingTickWidthPx = kRingRadiusPx / 5;
constexpr int32_t kRingTickThicknessPx = 9;
constexpr float kRingTickHalfSpanDeg = (kRingTickThicknessPx * 180.0f) / (2.0f * kRingRadiusPx * 3.14159265f);

// Size of root_, the rotating container around the primary label — NOT
// the full panel (see the class comment for why a full-panel rotation
// isn't viable). Sized for the longest primary text expected ("Focus
// 25:00") with margin, kept small to bound the rotation buffer's cost.
constexpr int32_t kRootWidthPx = 200;
// Primary font's line_height (44) + secondary_label_'s (21) = 65, exactly.
constexpr int32_t kRootHeightPx = 65;

// Owns the LVGL widget(s) and the backlight. TimerFace decides *what* to
// show (via render()); AppController decides brightness/color *state*;
// GuiManager only knows how to paint whatever it's told.
//
// root_ wraps only the primary label, not the whole screen and not the
// debug overlay label — a full-panel rotating container isn't viable on
// this hardware (the software rotation buffer it needs is too large for
// the LVGL heap), and LV_DRAW_TRANSFORM_USE_MATRIX (LVGL's faster
// rotation path) crashes when the rotated object contains live text on
// this LVGL version, so it stays off in sdkconfig; root_'s small buffer-
// based rotation is what's actually used.
//
// SetRotationDeg() takes screen_angle_deg directly (AttitudeEstimator's
// CCW-positive convention) and combines it with kRotationSign
// (gui_manager.cpp) to counter-rotate against LVGL's own fixed
// CW-positive transform_rotation — rotating the content by the negative
// of the device's own physical rotation keeps it upright to the user.
class GuiManager {
public:
    explicit GuiManager(LGFX& lcd);

    void SetPrimaryText(const char* text);

    // A small line above the primary label, same root_ (counter-rotates
    // together). Used for e.g. BreathFace's phase name ("Inhale"/"Hold"/
    // "Exhale") shown permanently alongside its countdown. Uses a small
    // built-in Montserrat font, not the custom primary font.
    void SetSecondaryText(const char* text);

    // Primary label's own text opacity, 0-255 — separate from
    // SetBrightness() (backlight PWM) and SetAccentColor() (hue). Used
    // for fading captions in/out (e.g. PomodoroFace's "Focus"/"Relax").
    // Callers must reset this to 255 when returning to steady-state
    // content, or it stays at whatever the caption last faded to.
    void SetPrimaryTextOpacity(uint8_t opa);

    // brightness: 0.0 (off) to 1.0 (full). Scaled to the LGFX/LEDC 0-255
    // range internally.
    void SetBrightness(float brightness);

    // warmth: 0.0 (normal/white text) to 1.0 (fully warm/red), for an
    // end-of-phase pulse notification. Linear RGB interpolation, not true
    // HSV hue rotation — sufficient for a single-color text label.
    void SetWarmth(float warmth);

    // Sets the primary label, secondary label, and outer ring's color to
    // the same value — TimerFace subclasses call this from render() to
    // communicate face/phase identity (e.g. focus=red, break=green).
    void SetAccentColor(lv_color_t color);

    // Progress ring — see AppController::UpdateRing() for the full
    // policy this renders; this quartet just draws whatever it's told.

    // Re-snaps the ring's "12 o'clock" reference to match the current
    // face — called once per face switch, not every tick.
    // face_center_deg: FaceCenterDeg(current_face_) from app_controller.cpp.
    void SetRingOrientation(float face_center_deg);

    // elapsed_fraction: 0..1 (clamped internally), how far the moving
    // edge has traveled. growing=false (countdown): the ring starts full
    // and the visible arc shrinks. growing=true (count-up): the arc
    // grows from 0. Either way the moving edge and the tick mark sit at
    // elapsed_fraction*360 degrees clockwise from 12 o'clock.
    void SetRingProgress(float elapsed_fraction, bool growing);

    // Main ring opacity, 0-255, independent of SetRingTickOpacity()
    // below — used for BreathFace's own rise/hold/fall envelope.
    void SetRingOpacity(uint8_t opa);

    // Shows or hides the whole ring (main arc + tick together), for
    // states with nothing meaningful to show progress for at all.
    void SetRingVisible(bool visible);

    // Tick/notch opacity only, 0-255 — doesn't move its position, which
    // is whatever SetRingProgress()'s last call set. AppController holds
    // this at 255 while actively counting and blinks it 255/0 while
    // paused.
    void SetRingTickOpacity(uint8_t opa);

    // Counter-rotates the primary label's small root_ container so it
    // stays upright as the physical device rotates. screen_angle_deg
    // comes straight from AttitudeEstimator, unfiltered here.
    //
    // Dirty-checked to a full degree, plus a kRotationUpdateMinIntervalUs
    // floor between actual redraws — root_'s rotated redraw is expensive
    // (near a full-container redraw each time), and sensor noise alone
    // can otherwise trigger it far more often than visibly needed. Full
    // precision is still passed to LVGL; only how often the redraw fires
    // is throttled.
    void SetRotationDeg(float screen_angle_deg);

    // Battery-check gesture display: a horizontal battery icon (gray
    // outline + terminal nub, kBatteryBlockCount segments filling
    // left-to-right). Plain (non-rotating) children of lv_screen_active(),
    // same as ring_ — screen_angle_deg isn't trustworthy while the device
    // is held out of the tracked plane, which is the gesture itself.
    static constexpr int kBatteryBlockCount = 5;

    // Shows or hides the whole battery view: hides root_/ring_ and shows
    // the battery icon, or the reverse. The hidden face keeps updating
    // underneath either way; this only controls what's drawn.
    void ShowBatteryView(bool show);

    // filled_blocks: how many of kBatteryBlockCount segments to fill,
    // left-to-right, clamped to [1, kBatteryBlockCount] — always at
    // least one segment lit, so "blank display" and "battery near-empty"
    // don't look identical. Picks one fixed color per fill level (red at
    // 1 -> green at kBatteryBlockCount).
    void SetBatteryLevel(int filled_blocks);

    // Recovers the display after RunIdleSleep(): re-runs the LCD panel's
    // init sequence and forces every managed widget to redraw, since the
    // panel shows nothing post-wake otherwise. Call before restoring
    // brightness (NotifyWokeFromIdleSleep()) — a full panel re-init
    // could glitch backlight state along the way.
    void ForceRedraw();

    // Counts real LVGL updates (calls that weren't skipped by the
    // value-unchanged check below), for a debug fps readout. Not a count
    // of raw flush calls — root_ spans multiple draw-buffer chunks, so a
    // single logical update triggers several flushes, which would
    // overstate the real update rate.
    uint32_t GetUpdateCount() const { return update_count_; }

private:
    // SetPrimaryText/SetRotationDeg are called every sensor tick
    // regardless of whether the value actually changed, but neither
    // LVGL's label-text nor style setters skip a no-op call on their
    // own. Comparing against the last value here means a redraw only
    // happens when something visibly changed.
    char last_text_[32] = "";
    char last_secondary_text_[16] = "";
    int32_t last_rotation_1deg_ = 0;
    int64_t last_rotation_update_us_ = 0;
    bool has_last_rotation_ = false;
    int32_t last_ring_angle_deg10_ = -1;  // tenths of a degree; -1 = never set, forces the first SetRingProgress() to draw
    bool last_ring_growing_ = false;
    bool last_ring_visible_ = false;
    bool has_last_ring_visible_ = false;
    uint8_t last_ring_opa_ = 0;
    bool has_last_ring_opa_ = false;
    uint8_t last_ring_tick_opa_ = 0;
    bool has_last_ring_tick_opa_ = false;
    uint8_t last_text_opa_ = 0;
    bool has_last_text_opa_ = false;
    uint32_t update_count_ = 0;

    bool showing_battery_view_ = false;
    int last_battery_filled_blocks_ = 0;  // 0 = never set, forces the first real SetBatteryLevel() to draw

    LGFX& lcd_;
    lv_obj_t* root_;
    lv_obj_t* secondary_label_;
    lv_obj_t* label_;
    lv_obj_t* ring_track_;
    lv_obj_t* ring_;
    lv_obj_t* ring_tick_;
    lv_obj_t* battery_outline_;
    lv_obj_t* battery_nub_;
    lv_obj_t* battery_segments_[kBatteryBlockCount];
};
