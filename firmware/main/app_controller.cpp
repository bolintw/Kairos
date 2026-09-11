#include "app_controller.hpp"

#include <cmath>

#include "breath_face.hpp"
#include "pomodoro_face.hpp"
#include "stopwatch_face.hpp"

namespace {
// 25/5 min -> 1min/30s (2026-09-09, temporary) — real values are a pain
// to sit through while testing the new countdown ring's erosion/tick
// behavior on hardware; face B (50/10 min) is left at its real duration
// as the "actually usable" Pomodoro option in the meantime. Revert once
// the ring itself is settled.
constexpr uint32_t kFocusMsA = 1 * 60 * 1000;
constexpr uint32_t kBreakMsA = 30 * 1000;
constexpr uint32_t kFocusMsB = 50 * 60 * 1000;
constexpr uint32_t kBreakMsB = 10 * 60 * 1000;

// See app_controller.hpp's design note 1 for why this single value
// produces both the leave and return bands by itself (leave at this
// distance from the current face's own center, return within
// 90-this-value of the new face's center, since centers are 90 deg
// apart). 80 -> 70 -> 65 (2026-08-25): with the new low-pass filter's
// convergence lag (attitude_estimator.cpp, kLowPassAlpha), the settled
// angle after a flip was landing close enough to the leave/return
// boundary that returning felt sluggish. 70 (return band +-20) still
// felt a bit slow; 65 widens it to +-25. Leaving a face gets marginally
// easier too (65 deg swing instead of 80) as a side effect of the same
// single parameter.
constexpr float kFaceHysteresisLeaveDeg = 65.0f;

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

// Idle-sleep sequence (2026-09-06, M9) — see app_controller.hpp design
// note 9. Replaces an earlier flat 5-minute "dim to off while paused"
// timeout: once ShouldEnterIdleSleep() exists to actually put the device
// into light sleep, there's no reason to wait minutes first — these are
// all just seconds. kIdlePreDimHoldMs is the same idea as kFadeToDimMs
// above but for "paused and ignored", not "running": grace period before
// assuming the user's actually done with the device, not just paused
// mid-thought. kIdleFadeToDimMs fades to kDimmedBrightness (not all the
// way to 0) and is intentionally quicker than kFadeToDimMs's leisurely
// immersion-fade — this is a "winding down" cue, not that. kIdleDimHoldMs
// holds at kDimmedBrightness, still normal operation (fast tap-poll rate,
// not yet the coarser light-sleep nap cadence) in case the device is
// still being handled. Only once all of that has elapsed does brightness
// finally snap the rest of the way to 0 and ShouldEnterIdleSleep() go
// true in the same tick — see the fade math below. All of these are
// starting points, not yet validated against real use like
// kFadeToDimMs/kDimmedBrightness were.
constexpr uint32_t kIdlePreDimHoldMs = 10 * 1000;
constexpr uint32_t kIdleFadeToDimMs = 3 * 1000;
constexpr uint32_t kIdleDimHoldMs = 5 * 1000;
constexpr uint32_t kIdleSleepThresholdMs = kIdlePreDimHoldMs + kIdleFadeToDimMs + kIdleDimHoldMs;  // ~18s total

// See app_controller.hpp design note 6 — window after a face switch
// during which a tap is ignored, absorbing flip-induced tap-engine
// false triggers instead of letting them immediately start the timer.
// 400 -> 1000 -> 500 -> 800 (2026-09-06): the 1000ms version was tuned
// before realizing the countdown started at commit (often mid-swing,
// still moving), wasting most of the window before settling. Once it only
// counted down once !attitude.is_moving (Update()), the full window
// consistently applied from the moment it's needed, so 500 felt like
// enough on the open bench. In the assembled enclosure, a flip still
// often left enough residual wobble/knock after !is_moving first went
// true (case rattling, hand releasing contact) to trip the tap engine and
// start the timer right on arrival — 500ms of settled time wasn't quite
// covering that. Widened to 800.
constexpr uint32_t kTapMuteAfterSwitchMs = 800;

// See app_controller.hpp design note 7 — the paused ring-tick's on/off
// blink. 2026-09-09: tried a smooth cosine breathing fade first (reusing
// the ring's old "last few seconds" effect's exact shape), then a slower
// version of the same fade after it read as too fast/anxious — the user's
// actual ask was neither: revert to the *older* mechanism this app
// already tried once before, a hard on/off blink (2026-08-25's first-ever
// ring-breathing version, itself later replaced for being "too harsh" —
// but that was in a different role, an urgent last-seconds cue; here,
// wanted back specifically for this calmer "just paused" indicator).
// kRingBlinkHalfPeriodMs matches that original's exact cadence
// (`(remaining_ms/1000) % 2`, i.e. a flat 1 real second on, 1 off).
constexpr uint32_t kRingBlinkHalfPeriodMs = 1000;

// See app_controller.hpp design note 7 — count-up ring wraps once per
// real hour.
constexpr uint32_t kRingCountUpPeriodMs = 3600u * 1000u;

// See app_controller.hpp design note 10. Asymmetric on purpose: still a
// deliberate hold to enter (500ms — 2000ms->500ms 2026-09-08, first-hardware-
// pass feedback that 2s felt slow now that the entry threshold itself
// (kAzInvalidEnterThresholdG=0.7g) already does most of the work rejecting
// accidental triggers), but essentially instant to leave (0ms — set the
// device back down and it's back immediately, no hold needed).
constexpr uint32_t kBatteryViewEnterMs = 500;
constexpr uint32_t kBatteryViewExitMs = 0;

// Placeholder until real ADC/voltage-LUT code exists (see design note
// 10's "known gap" paragraph) — fixed at "middle" so the gesture and
// rendering can be tested end-to-end on hardware before real battery data
// is wired in. 1-indexed, matches GuiManager::SetBatteryLevel()'s range.
constexpr int kStubBatteryFilledBlocks = 3;

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
        case Face::kD: return std::make_unique<BreathFace>();
    }
    return nullptr;
}

void AppController::Update(const AttitudeEstimator::Output& attitude, uint32_t dt_ms)
{
    UpdateBatteryView(attitude, dt_ms);
    if (showing_battery_) {
        // Underlying face keeps its own clock correct (design note 10 —
        // nothing to save/restore on the way back out), but face
        // switching/brightness-fade/ring/render are all skipped this tick;
        // the battery view owns the screen instead.
        if (current_) {
            current_->onTick(dt_ms);
        }
        gui_manager_.SetBrightness(1.0f);
        gui_manager_.SetRingVisible(false);
        gui_manager_.SetBatteryLevel(kStubBatteryFilledBlocks);  // TODO: real ADC reading, see design note 10
        return;
    }

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

        // Re-snap the ring's "12 o'clock" to this face's own upright
        // orientation — see design note 7 and GuiManager::SetRingOrientation()'s
        // comment. Once per switch, not every tick: the ring deliberately
        // doesn't counter-rotate continuously the way root_ does.
        gui_manager_.SetRingOrientation(FaceCenterDeg(current_face_));

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
    } else if (tap_mute_remaining_ms_ > 0 && !attitude.is_moving) {
        // Only count down once actually settled — commits fire the
        // instant quantized changes (see above), which can be mid-swing,
        // still moving. Counting down through that motion wasted most of
        // the window before the residual-vibration risk this exists for
        // even starts; holding it at full while is_moving stays true
        // means the full window applies from the moment it's needed.
        tap_mute_remaining_ms_ = dt_ms < tap_mute_remaining_ms_ ? tap_mute_remaining_ms_ - dt_ms : 0;
    }

    if (current_) {
        current_->onTick(dt_ms);
    }
    UpdateBrightness(dt_ms, attitude.is_moving);
    UpdateRing(dt_ms);
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
        if (paused_elapsed_ms_ >= kIdleSleepThresholdMs) {
            // Full ~18s sequence has played out and we're handing off to
            // RunIdleSleep() this same tick (main.cpp checks
            // ShouldEnterIdleSleep() right after this call returns) — cut
            // the rest of the way to fully off.
            brightness_ = 0.0f;
        } else if (paused_elapsed_ms_ >= kIdlePreDimHoldMs) {
            // Same shape as the running-branch fade above (linear t,
            // clamped at 1), just faster and heading to kDimmedBrightness
            // over kIdleFadeToDimMs, then held there (t stays clamped at 1)
            // through kIdleDimHoldMs until the branch above takes over.
            const uint32_t fade_elapsed_ms = paused_elapsed_ms_ - kIdlePreDimHoldMs;
            const float t = static_cast<float>(fade_elapsed_ms) / static_cast<float>(kIdleFadeToDimMs);
            const float clamped_t = t < 1.0f ? t : 1.0f;
            brightness_ = 1.0f + clamped_t * (kDimmedBrightness - 1.0f);
        }
    }

    gui_manager_.SetBrightness(brightness_);

    prev_is_running_ = status.is_running;
    prev_has_target_ = status.has_target;
    prev_remaining_ms_ = status.remaining_ms;
}

bool AppController::ShouldEnterIdleSleep() const
{
    return paused_elapsed_ms_ >= kIdleSleepThresholdMs;
}

void AppController::NotifyWokeFromIdleSleep()
{
    paused_elapsed_ms_ = 0;
    brightness_ = 1.0f;
    gui_manager_.SetBrightness(brightness_);
}

void AppController::UpdateRing(uint32_t dt_ms)
{
    if (!current_) {
        gui_manager_.SetRingVisible(false);
        ring_pause_blink_elapsed_ms_ = 0;
        return;
    }

    const TimerFace::Status status = current_->GetStatus();

    if (!status.has_target && !status.is_count_up) {
        // Nothing to show progress for — e.g. BreathFace's true idle
        // screen (see its GetStatus()). Distinct from "the ring reads
        // 0%/100%", which the branches below already cover on their own.
        gui_manager_.SetRingVisible(false);
        ring_pause_blink_elapsed_ms_ = 0;
        return;
    }
    gui_manager_.SetRingVisible(true);

    // elapsed_fraction: how far through the current phase (countdown) or
    // current hour (count-up) things are, 0..1 — see design note 7 for
    // the two shapes this drives in GuiManager::SetRingProgress().
    float elapsed_fraction;
    if (status.has_target) {
        elapsed_fraction = status.target_ms > 0
            ? 1.0f - static_cast<float>(status.remaining_ms) / static_cast<float>(status.target_ms)
            : 1.0f;  // defensive only — a real has_target phase should never report target_ms==0
    } else {
        // is_count_up: remaining_ms is repurposed to carry elapsed ms this
        // run (see Status's own field comment).
        const uint32_t elapsed_in_period_ms = status.remaining_ms % kRingCountUpPeriodMs;
        elapsed_fraction = static_cast<float>(elapsed_in_period_ms) / static_cast<float>(kRingCountUpPeriodMs);
    }
    gui_manager_.SetRingProgress(elapsed_fraction, /*growing=*/status.is_count_up);

    if (status.is_running) {
        // Reset here, not just left alone, so a *later* pause always
        // starts its blink from "on" rather than resuming wherever an
        // earlier pause happened to leave ring_pause_blink_elapsed_ms_.
        ring_pause_blink_elapsed_ms_ = 0;
        gui_manager_.SetRingTickOpacity(255);
        return;
    }

    // Not running: only show the tick at all once there's genuine
    // progress to pause *from* (2026-09-09) — a brand-new face (or a
    // phase that just auto-advanced) starts paused with
    // elapsed_fraction==0, and shouldn't show the tick yet at all; it
    // first appears once the phase actually starts running, then blinks
    // if paused again later. The ring itself already reads as
    // unambiguously fresh on its own (a full or empty circle, whichever
    // mode) without the tick doing anything on top of it.
    constexpr float kMinElapsedFractionForTick = 0.001f;  // guards against float noise landing exactly at 0
    if (elapsed_fraction <= kMinElapsedFractionForTick) {
        ring_pause_blink_elapsed_ms_ = 0;
        gui_manager_.SetRingTickOpacity(0);
        return;
    }

    // Paused, with real progress already made: hard on/off blink — see
    // design note 7 for why this isn't the smooth breathing fade anymore.
    // Driven by a real wall-clock accumulator (advanced here by dt_ms)
    // rather than remaining_ms, which is frozen while paused — the whole
    // reason this needs its own timer instead of reusing the original
    // `(remaining_ms/1000) % 2` formula directly.
    ring_pause_blink_elapsed_ms_ += dt_ms;
    const bool blink_on = (ring_pause_blink_elapsed_ms_ / kRingBlinkHalfPeriodMs) % 2 == 0;
    gui_manager_.SetRingTickOpacity(blink_on ? 255 : 0);
}

void AppController::OnTap()
{
    if (showing_battery_) {
        return;  // see design note 10 — ignore taps while checking battery
    }
    if (tap_mute_remaining_ms_ > 0) {
        return;  // see design note 6 — absorbing a flip's residual vibration
    }
    if (current_) {
        current_->onTap();
    }
}

void AppController::UpdateBatteryView(const AttitudeEstimator::Output& attitude, uint32_t dt_ms)
{
    // See design note 10: battery_view_hold_ms_ counts continuous time in
    // the state *opposite* showing_battery_'s current value, reset the
    // instant attitude.in_valid_plane agrees with the current state
    // again. attitude.in_valid_plane is already hysteresis-debounced at
    // the source (AttitudeEstimator), so a single stray tick right at a
    // threshold can't happen here in the first place.
    if (!showing_battery_) {
        if (!attitude.in_valid_plane) {
            battery_view_hold_ms_ += dt_ms;
            if (battery_view_hold_ms_ >= kBatteryViewEnterMs) {
                showing_battery_ = true;
                battery_view_hold_ms_ = 0;
                gui_manager_.ShowBatteryView(true);
            }
        } else {
            battery_view_hold_ms_ = 0;
        }
    } else {
        if (attitude.in_valid_plane) {
            battery_view_hold_ms_ += dt_ms;
            if (battery_view_hold_ms_ >= kBatteryViewExitMs) {
                showing_battery_ = false;
                battery_view_hold_ms_ = 0;
                gui_manager_.ShowBatteryView(false);
            }
        } else {
            battery_view_hold_ms_ = 0;
        }
    }
}
