#include "app_controller.hpp"

#include <cmath>

#include "pomodoro_face.hpp"
#include "reserved_face.hpp"
#include "stopwatch_face.hpp"

namespace {
constexpr uint32_t kFocusMsA = 25 * 60 * 1000;
constexpr uint32_t kBreakMsA = 5 * 60 * 1000;
constexpr uint32_t kFocusMsB = 50 * 60 * 1000;
constexpr uint32_t kBreakMsB = 10 * 60 * 1000;

// Temporary (2026-08-25): D borrowed for a short Pomodoro so the
// brightness/notification changes (fade, end-of-focus ramp, always-bright
// break) can be tested in ~2 minutes instead of waiting out a real 25+
// minute phase. D's real behavior is still undecided (see CreateFace) —
// swap back to ReservedFace once done testing.
constexpr uint32_t kFocusMsD = 60 * 1000;  // 1:00
constexpr uint32_t kBreakMsD = 30 * 1000;  // 0:30

// See app_controller.hpp's design note 1 for why 80 degrees produces the
// intended asymmetric enter/leave band by itself.
constexpr float kFaceHysteresisLeaveDeg = 80.0f;

// Brightness/notification tuning — starting points per the plan's own
// note that these need real usage to validate, not to be locked down
// here. See app_controller.hpp design note 5.
constexpr float kDimmedBrightness = 0.3f;        // immersion-dim floor while running
constexpr uint32_t kFadeToDimMs = 10 * 1000;      // ~10s fade after becoming bright
// Last ~30s of a focus-like (non-break) phase: ramp UP to full bright,
// reaching it within ~3s (faster than the dim-fade, so it reads clearly
// as a distinct event) and holding there until the phase actually
// changes — replaces an earlier breathing-pulse design (2026-08-25,
// user's redesign) that never shipped past a first draft.
constexpr uint32_t kFocusEndRampWindowMs = 30 * 1000;
constexpr uint32_t kFocusEndRampMs = 3 * 1000;
constexpr uint32_t kLongIdleTimeoutMs = 5 * 60 * 1000;  // minutes-scale, paused-only, backlight off

// See app_controller.hpp design note 6 — window after a face switch
// during which a tap is ignored, absorbing flip-induced tap-engine
// false triggers instead of letting them immediately start the timer.
constexpr uint32_t kTapMuteAfterSwitchMs = 1000;

float FaceCenterDeg(AppController::Face face)
{
    switch (face) {
        case AppController::Face::kA: return -90.0f;
        case AppController::Face::kB: return 0.0f;
        case AppController::Face::kC: return 90.0f;
        case AppController::Face::kD: return 180.0f;
    }
    return 0.0f;
}

// Shortest angular distance between two angles, always >= 0.
float AngularDistanceDeg(float a_deg, float b_deg)
{
    float diff = a_deg - b_deg;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return std::fabs(diff);
}

// Nearest-center classification, no hysteresis — used once QuantizeFace
// decides the angle has moved far enough to justify leaving current_face_.
AppController::Face NearestFace(float screen_angle_deg)
{
    float deg = screen_angle_deg;
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    if (deg < 45.0f || deg >= 315.0f) return AppController::Face::kB;
    if (deg < 135.0f) return AppController::Face::kC;
    if (deg < 225.0f) return AppController::Face::kD;
    return AppController::Face::kA;
}
}  // namespace

AppController::AppController(GuiManager& gui_manager)
    : gui_manager_(gui_manager)
{
}

AppController::Face AppController::QuantizeFace(float screen_angle_deg) const
{
    if (AngularDistanceDeg(screen_angle_deg, FaceCenterDeg(current_face_)) < kFaceHysteresisLeaveDeg) {
        return current_face_;
    }
    return NearestFace(screen_angle_deg);
}

std::unique_ptr<TimerFace> AppController::CreateFace(Face face)
{
    switch (face) {
        case Face::kA: return std::make_unique<PomodoroFace>(kFocusMsA, kBreakMsA);
        case Face::kB: return std::make_unique<PomodoroFace>(kFocusMsB, kBreakMsB);
        case Face::kC: return std::make_unique<StopwatchFace>();
        case Face::kD: return std::make_unique<PomodoroFace>(kFocusMsD, kBreakMsD);  // temporary test face, see kFocusMsD above
    }
    return nullptr;
}

void AppController::Update(const AttitudeEstimator::Output& attitude, uint32_t dt_ms)
{
    const Face quantized = QuantizeFace(attitude.screen_angle_deg);

    // Commit the instant quantized disagrees with the confirmed face (or
    // there's no face yet, at boot) — no settle wait. Went through two
    // earlier stages: originally gated on `!attitude.is_moving` (only
    // commit once motion stopped), which itself needed a `was_disturbed_`
    // latch removed on 2026-08-24 (see git history) because it broke on
    // slow final corrections. The `is_moving` gate itself was kept a bit
    // longer as a hedge against angle overshoot during a flip — before
    // the gyro scale fix + alpha tuning, a fast rotation could transiently
    // read 30-40 degrees past its true angle, so switching immediately
    // risked triggering on a bogus mid-flip reading. With that fixed
    // (see qmi8658.hpp, kComplementaryAlpha), the overshoot that
    // motivated waiting is gone, and the user found immediate switching
    // feels better on hardware (2026-08-25) — QuantizeFace's own 80-degree
    // hysteresis is enough to reject resting noise on its own, gating on
    // is_moving too was redundant, same shape of issue as the latch fix.
    // Residual trade-off: a fast swipe that passes *through* a face's
    // zone on the way to another one now commits (and resets) that
    // passed-through face too, instead of only the final settled one —
    // accepted as fine for how the device is actually being flipped.
    if (!current_ || quantized != current_face_) {
        if (current_) {
            current_->onExit();
        }
        current_face_ = quantized;
        current_ = CreateFace(current_face_);
        if (current_) {
            current_->onEnter();
        }

        // A flip should always read as an obvious bright event,
        // regardless of edge detection below (the new face always
        // starts paused, so is_running true->false/false->true
        // wouldn't reliably fire "just became bright" on its own —
        // see design note 5).
        brightness_ = 1.0f;
        bright_phase_elapsed_ms_ = 0;
        paused_elapsed_ms_ = 0;
        prev_is_running_ = false;
        prev_has_target_ = false;
        prev_remaining_ms_ = 0;

        // See design note 6: absorb flip-induced tap-engine false
        // triggers instead of letting them start the timer immediately.
        tap_mute_remaining_ms_ = kTapMuteAfterSwitchMs;
    } else if (tap_mute_remaining_ms_ > 0) {
        tap_mute_remaining_ms_ = dt_ms < tap_mute_remaining_ms_ ? tap_mute_remaining_ms_ - dt_ms : 0;
    }

    if (current_) {
        current_->onTick(dt_ms);
    }
    UpdateBrightness(dt_ms, attitude.is_moving);
    if (current_) {
        current_->render(gui_manager_);
    }
}

void AppController::UpdateBrightness(uint32_t dt_ms, bool is_moving)
{
    if (!current_) {
        gui_manager_.SetBrightness(0.0f);
        gui_manager_.SetWarmth(0.0f);
        return;
    }

    const TimerFace::Status status = current_->GetStatus();

    const bool just_started = status.is_running && !prev_is_running_;
    const bool just_paused = !status.is_running && prev_is_running_;
    // remaining_ms jumping up by more than this tick's dt_ms means a new
    // phase began (e.g. focus -> break) rather than just ticking down.
    const bool phase_changed = status.has_target && prev_has_target_ &&
                                status.remaining_ms > prev_remaining_ms_ + dt_ms;

    // Tapping, flipping (via just_started/just_paused/phase_changed) and
    // plain movement all count as "the user is engaging with the device
    // right now" — see design note 5. Folding them into one flag means
    // spinning the device without crossing a face boundary resets the
    // same fade/idle clocks a tap or a real flip would.
    const bool interacting = just_started || just_paused || phase_changed || is_moving;
    if (interacting) {
        brightness_ = 1.0f;
        bright_phase_elapsed_ms_ = 0;
        paused_elapsed_ms_ = 0;
    }

    if (status.is_running) {
        paused_elapsed_ms_ = 0;
        bright_phase_elapsed_ms_ += dt_ms;

        const bool in_focus_end_ramp = status.has_target && !status.is_break_phase &&
                                        status.remaining_ms <= kFocusEndRampWindowMs;
        if (status.is_break_phase) {
            // Stays fully bright for the whole break — no fade, nothing
            // to ramp toward, it's already there.
            brightness_ = 1.0f;
        } else if (in_focus_end_ramp) {
            const uint32_t time_in_window = kFocusEndRampWindowMs - status.remaining_ms;
            const float ramp_t_raw = static_cast<float>(time_in_window) / static_cast<float>(kFocusEndRampMs);
            const float ramp_t = ramp_t_raw < 1.0f ? ramp_t_raw : 1.0f;
            brightness_ = kDimmedBrightness + ramp_t * (1.0f - kDimmedBrightness);
        } else if (!interacting) {
            // Linear fade: t is elapsed/kFadeToDimMs (0 -> 1), brightness_
            // is a straight interpolation between 1.0 and kDimmedBrightness.
            const float t = static_cast<float>(bright_phase_elapsed_ms_) / static_cast<float>(kFadeToDimMs);
            const float clamped_t = t < 1.0f ? t : 1.0f;
            brightness_ = 1.0f + clamped_t * (kDimmedBrightness - 1.0f);
        }
        // else: interacting already set brightness_=1.0 above; the fade
        // begins next tick, not this one.
    } else {
        paused_elapsed_ms_ += dt_ms;
        if (paused_elapsed_ms_ >= kLongIdleTimeoutMs) {
            brightness_ = 0.0f;
        }
    }

    gui_manager_.SetBrightness(brightness_);

    prev_is_running_ = status.is_running;
    prev_has_target_ = status.has_target;
    prev_remaining_ms_ = status.remaining_ms;
}

void AppController::OnTap()
{
    if (tap_mute_remaining_ms_ > 0) {
        return;  // see design note 6 — absorbing a flip's residual vibration
    }
    if (current_) {
        current_->onTap();
    }
}
