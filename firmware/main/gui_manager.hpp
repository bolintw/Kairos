#pragma once

#include "lgfx_config.hpp"
#include "lvgl.h"

// Space Grotesk Bold, 40px, full printable ASCII (2026-08-25 "UI polish"
// pass — LVGL's built-in Montserrat has no bold weight and was judged not
// elegant enough for the primary display). Generated via lv_font_conv
// from reference/vendor/fonts/SpaceGrotesk-Bold.ttf — SIL OFL 1.1,
// Copyright 2020 The Space Grotesk Project Authors; license text tracked
// alongside the generated font source at SpaceGrotesk-OFL.txt (kept
// in-tree, unlike reference/vendor/ which is gitignored, since the OFL
// requires the license to travel with any distributed copy of the font,
// including this subsetted embedded form). Originally hand-curated down
// to just "0123456789:FocusRelax" — the exact characters in use at the
// time — but that broke the moment a string outside that set showed up:
// calibration_mode.cpp's "Calib in %d"/"Hold still..."/"Calibrated"
// rendered as boxes on real hardware (2026-08-26) because none of its
// letters were in the font. Switched to the full 0x20-0x7E range instead
// of trying to keep a hand-curated charset in sync with every string in
// the codebase — ~460KB of free flash made the size difference (about
// 25KB more) not worth the fragility. --no-compress is required:
// lv_font_conv's default RLE-compressed glyph format needs
// CONFIG_LV_USE_FONT_COMPRESSED, which this build doesn't enable —
// compressed glyphs silently rendered as nothing (2026-08-25, caught on
// real hardware: ring/debug overlay both fine, primary digits just
// invisible). Regenerate (all one command, wrapped here only for line
// length) with: npx lv_font_conv --font
// reference/vendor/fonts/SpaceGrotesk-Bold.ttf --size 40 --bpp 4
// --format lvgl --no-compress --range "0x20-0x7E"
// --lv-font-name font_space_grotesk_bold_40
// -o firmware/main/font_space_grotesk_bold_40.c
// if the size/weight ever needs to change — then edit the
// generated file's top #include block to a plain `#include "lvgl.h"`
// (see the comment left in font_space_grotesk_bold_40.c for why: the
// tool's default ifdef only resolves correctly from inside the lvgl
// component tree, not from main/).
extern "C" const lv_font_t font_space_grotesk_bold_40;
constexpr const lv_font_t* kPrimaryFont = &font_space_grotesk_bold_40;

// Panel is 240x240 (see lgfx_config.hpp / main.cpp's lv_display_create).
constexpr int32_t kPanelSizePx = 240;

// Outer ring (2026-08-25, "UI polish" pass): a plain circular border, shown
// while paused / near a phase's end, hidden while running mid-phase — see
// AppController's UpdateRing() for the actual show/hide/blink policy, this
// is just the geometry. Sized close to the panel edge with a bit of
// margin so it doesn't get clipped. Unlike root_ above, ring_ does NOT
// need to counter-rotate — a circle is rotationally symmetric, so it's a
// plain static lv_obj on lv_screen_active(), not a root_ child, and never
// touches the transform/matrix code path that caused the three rotation
// crashes documented below.
constexpr int32_t kRingRadiusPx = 112;
constexpr int32_t kRingWidthPx = 6;

// Size of root_, the rotating container around the primary label — NOT
// the full panel. See the design note below for why: a full 240x240
// rotating container crashed twice on real hardware (task watchdog,
// 2026-08-24/25). Sized generously for the longest primary text expected
// ("Focus 25:00" at kPrimaryFont) with margin, kept much smaller than the
// panel so LVGL's software rotation — on either its fast matrix path or
// the slow per-pixel fallback — stays within a tractable pixel count and
// (worst case, ARGB8888 fallback) a tractable buffer size
// (kRootWidthPx*kRootHeightPx*4 ~= 52KB — see CONFIG_LV_MEM_SIZE_KILOBYTES
// in sdkconfig, currently 256KB, so this is comfortably small; the crash
// history above is about the original *240x240* attempt (225KB against a
// then-64KB heap), not about this container being sized at all — see the
// height note just below for what actually sets kRootHeightPx now).
constexpr int32_t kRootWidthPx = 200;
// height = primary font's line_height (44) + secondary_label_'s
// (lv_font_montserrat_18, line_height 21) = 65, exactly, no slack —
// see SetSecondaryText's comment. Was a plain 60 (room for one line only)
// until secondary_label_ existed (2026-08-30); grown just enough for the
// second line, not to some round/generous number, since every extra
// pixel here is extra ARGB8888 buffer cost per the note above.
constexpr int32_t kRootHeightPx = 65;

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
//      hanging (Guru Meditation / LoadProhibited, NULL-ish dest_buf inside
//      LVGL's RGB565_SWAPPED glyph-mask blend). LV_DRAW_TRANSFORM_USE_MATRIX
//      turned back off in sdkconfig; the small root_ alone keeps the older
//      buffer-based path (which does work correctly, just needed to stay
//      small) fast and memory-safe without needing the matrix shortcut.
//   4. Retried matrix (2026-09-06) after a main-loop timing breakdown
//      showed root_'s buffer-based rotation redraw dominating (~97% of
//      every second, throttling the whole sensor loop to ~10-14Hz instead
//      of its designed ~120Hz) — theory this time was that the crash was
//      specific to LV_COLOR_FORMAT_RGB565_SWAPPED (a non-default LVGL
//      color format), so switched to plain LV_COLOR_FORMAT_RGB565 +
//      lcd.setSwapBytes(true) (letting LovyanGFX handle the byte order
//      instead) before re-enabling the matrix flag. Crashed anyway — same
//      Guru Meditation, same draw_letter_cb -> lv_draw_sw_blend ->
//      lv_draw_sw_blend_color_to_rgb565 (the *non*-swapped variant this
//      time) via refr_obj_matrix, confirming the bug is in LVGL 9.5.0's
//      matrix-transformed text-glyph rendering itself — independent of
//      color format, so switching formats was never going to dodge it.
//      Reverted both the color-format experiment and the matrix flag.
//      Conclusion: the matrix path is not usable for a rotated object
//      that contains live text on this LVGL version, full stop — not a
//      config mistake to keep retrying. Any further throughput fix needs
//      to either avoid re-rendering text glyphs every angle change (e.g.
//      rasterize the label to a static bitmap once, rotate *that* via
//      matrix instead of live glyph draws) or decouple rendering from the
//      sensor loop entirely (a separate task/core, so root_'s redraw cost
//      stops blocking IMU/AttitudeEstimator/AppController regardless of
//      how expensive it stays) — neither attempted yet as of this note.
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

    // A small line above the primary label, same root_ (counter-rotates
    // together, stays upright with it) — added 2026-08-30 for
    // BreathFace's phase name ("Inhale"/"Hold"/"Exhale"). First attempt
    // had the phase name replace the countdown as a fading caption, same
    // trick as PomodoroFace's Focus/Relax — worked fine for PomodoroFace
    // (one transition per 25+ minute phase) but at BreathFace's pace (a
    // new phase every 2-8s) the word-then-number swap itself became the
    // abrupt thing, on top of the number jump it was meant to soften.
    // Showing both permanently side by side removes the swap entirely
    // instead of softening it. Uses a small built-in Montserrat size, not
    // the custom primary font — originally lv_font_montserrat_14 (reusing
    // the debug overlay's font, needing no root_ growth at all: 16+44 ==
    // the then-kRootHeightPx of 60), bumped to lv_font_montserrat_18
    // the same day once the user found 14 hard to read — its line_height
    // (21px) plus the primary font's (44px) is exactly kRootHeightPx
    // (65, grown from 60 for this), so the two labels still stack with
    // zero slack. root_ growing 5px is a ~5KB larger worst-case rotation
    // buffer, nowhere near the scale that caused the original 240x240
    // rotation crashes (see the class comment) — the risk that history
    // warns about is a buffer sized like the *whole panel*, not a few
    // extra pixels on an already-small container.
    void SetSecondaryText(const char* text);

    // Primary label's own text opacity, 0-255 — separate from
    // SetBrightness() (backlight PWM, whole-screen) and SetAccentColor()
    // (hue). Added 2026-08-26 for PomodoroFace's phase-transition caption
    // ("Focus"/"Relax"): a plain text swap read as too abrupt next to the
    // ring's smooth cosine breathing, so the caption fades itself in and
    // back out the same way — see PomodoroFace::render(). Callers must
    // reset this to 255 when going back to steady-state content, or it'll
    // still be sitting at whatever the caption last faded to.
    void SetPrimaryTextOpacity(uint8_t opa);

    // brightness: 0.0 (off) to 1.0 (full). Scaled to the LGFX/LEDC 0-255
    // range internally.
    void SetBrightness(float brightness);

    // warmth: 0.0 (normal/white text) to 1.0 (fully warm/red) — the
    // "end-of-phase pulse... shifts toward a warm/red tint" notification.
    // Linear RGB interpolation
    // between white and a warm red, not true HSV hue rotation — simple
    // and sufficient for a single-color text label.
    void SetWarmth(float warmth);

    // Accent color (2026-08-25, extended 2026-08-30 to the secondary
    // label too): sets the primary label's text color, the secondary
    // label's text color, AND the outer ring's border color to the same
    // value — TimerFace subclasses call this from render() to
    // communicate face/phase identity (focus=red, break=green,
    // count-up=blue, breathe phases=green/gold/violet) now that the
    // primary text itself is numbers-only. Independent of SetWarmth()
    // above (still unused, kept in case a separate tint channel comes
    // back) and independent of SetBrightness() (backlight PWM, not pixel
    // color — the two never fight).
    void SetAccentColor(lv_color_t color);

    // Sets the outer ring's border opacity, 0 (invisible) to 255 (fully
    // opaque) — see AppController::UpdateRing() for the policy (paused /
    // near-phase-end solid / last-5s breathing). A single continuous
    // control rather than a visible/hidden toggle so the last-5s "breathe"
    // (2026-08-25, replaced a hard on/off blink the user found too
    // harsh) can drive it as a smooth sine wave instead of a snap.
    // Geometry (radius/width) is fixed at compile time via
    // kRingRadiusPx/kRingWidthPx above.
    void SetRingOpacity(uint8_t opa);

    // Counter-rotates the primary label's small root_ container (see
    // class doc) so it stays upright as the physical device rotates.
    // screen_angle_deg: AttitudeEstimator::Output::screen_angle_deg,
    // straight from the estimator, no filtering applied here.
    //
    // Dirty-check widened from 0.1deg to 1deg + a kRotationUpdateMinIntervalUs
    // floor between actual redraws (2026-09-06) — root_'s rotation uses
    // LVGL's slow per-pixel software rotation path (the fast matrix path
    // crashes under our RGB565_SWAPPED color format + text glyphs, see the
    // class doc's crash history), and its rotated bounding box (200x65 at
    // an arbitrary angle, diagonal ~209px) needs close to a full-height
    // redraw every time it fires. The 0.1deg check alone was already known
    // to fire ~30x/sec from plain sensor noise on a stationary mount (see
    // GetUpdateCount()'s comment, confirmed 2026-08-25) — a per-second main
    // loop timing breakdown (2026-09-06) tied that directly to the whole
    // sensor loop being throttled to ~10-14Hz instead of its designed
    // ~120Hz. Neither change touches what angle actually gets rendered
    // (still full precision) — only how often the expensive redraw is
    // allowed to happen.
    void SetRotationDeg(float screen_angle_deg);

    // Recovers the display after RunIdleSleep() (M9, sleep_mode.hpp):
    // re-runs the LCD panel's own init sequence (lcd_.init() — confirmed
    // safe to call twice, see gui_manager.cpp) and forces every managed
    // widget to redraw on the next lv_timer_handler() call, bypassing the
    // value-unchanged checks below. Added 2026-09-06 after the panel
    // showed nothing at all post-wake, backlight aside, across two
    // different sleep implementations (manual esp_light_sleep_start(),
    // then ESP-IDF's automatic PM-driven light sleep) — the fact that
    // both hit the identical symptom points at something about the
    // physical light-sleep transition itself needing the panel
    // re-initialized, not at either sleep entry mechanism specifically.
    // Call before restoring brightness (NotifyWokeFromIdleSleep()) — a
    // full panel re-init could glitch backlight state along the way, so
    // brightness needs to be reapplied after this, not before.
    void ForceRedraw();

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
    char last_secondary_text_[16] = "";
    int32_t last_rotation_1deg_ = 0;
    int64_t last_rotation_update_us_ = 0;
    bool has_last_rotation_ = false;
    uint8_t last_ring_opa_ = 0;
    bool has_last_ring_opa_ = false;
    uint8_t last_text_opa_ = 0;
    bool has_last_text_opa_ = false;
    uint32_t update_count_ = 0;

    LGFX& lcd_;
    lv_obj_t* root_;
    lv_obj_t* secondary_label_;
    lv_obj_t* label_;
    lv_obj_t* ring_;
};
