#include "app_controller.hpp"

#include <cmath>

#include "breath_face.hpp"
#include "pomodoro_face.hpp"
#include "stopwatch_face.hpp"

namespace {
constexpr uint32_t kFocusMsA = 25 * 60 * 1000;
constexpr uint32_t kBreakMsA = 5 * 60 * 1000;
constexpr uint32_t kFocusMsB = 50 * 60 * 1000;
constexpr uint32_t kBreakMsB = 10 * 60 * 1000;

// Hysteresis: leave the current face once past this distance from its
// own center; return once within (90-this) of the new face's center —
// see app_controller.hpp design note 1.
constexpr float kFaceHysteresisLeaveDeg = 65.0f;

// Brightness/notification tuning — see app_controller.hpp design note 5.
constexpr float kDimmedBrightness = 0.3f;        // immersion-dim floor while running
constexpr uint32_t kFadeToDimMs = 10 * 1000;      // ~10s fade after becoming bright
// Last ~30s of a focus-like (non-break) phase: ramp UP to full bright
// within ~3s (faster than the dim-fade, so it reads as a distinct event)
// and hold until the phase changes.
constexpr uint32_t kFocusEndRampWindowMs = 30 * 1000;
constexpr uint32_t kFocusEndRampMs = 3 * 1000;

// Idle-sleep sequence — see app_controller.hpp design note 9.
// kIdlePreDimHoldMs: grace period before assuming the user is done.
// kIdleFadeToDimMs: fades to kDimmedBrightness, quicker than
// kFadeToDimMs's leisurely immersion-fade — a "winding down" cue.
// kIdleDimHoldMs: holds at kDimmedBrightness, still normal operation.
// Brightness then snaps to 0 and ShouldEnterIdleSleep() goes true.
constexpr uint32_t kIdlePreDimHoldMs = 10 * 1000;
constexpr uint32_t kIdleFadeToDimMs = 3 * 1000;
constexpr uint32_t kIdleDimHoldMs = 5 * 1000;
constexpr uint32_t kIdleSleepThresholdMs = kIdlePreDimHoldMs + kIdleFadeToDimMs + kIdleDimHoldMs;  // ~18s total

// Shared by UpdateBrightness()'s paused-idle fade and the low-battery
// warning screen's own idle timer, so both drive the same curve.
// idle_elapsed_ms >= kIdleSleepThresholdMs is also the cue for the
// caller to treat this as "ready for idle sleep" (ShouldEnterIdleSleep()).
float IdleBrightnessCurve(uint32_t idle_elapsed_ms)
{
    if (idle_elapsed_ms >= kIdleSleepThresholdMs) {
        return 0.0f;
    }
    if (idle_elapsed_ms >= kIdlePreDimHoldMs) {
        const uint32_t fade_elapsed_ms = idle_elapsed_ms - kIdlePreDimHoldMs;
        const float t = static_cast<float>(fade_elapsed_ms) / static_cast<float>(kIdleFadeToDimMs);
        const float clamped_t = t < 1.0f ? t : 1.0f;
        return 1.0f + clamped_t * (kDimmedBrightness - 1.0f);
    }
    return 1.0f;
}

// Low-battery warning hysteresis — see app_controller.hpp design note 11.
constexpr float kLowBatteryEnterV = 3.3f;
constexpr float kLowBatteryExitV = 3.4f;
const lv_color_t kLowBatteryColor = lv_color_make(240, 150, 20);  // warning orange-amber, distinct from every TimerFace accent

// Window after a face switch during which a tap is ignored, absorbing
// flip-induced tap-engine false triggers — see design note 6.
constexpr uint32_t kTapMuteAfterSwitchMs = 800;

// The paused ring-tick's on/off blink period — see design note 7.
constexpr uint32_t kRingBlinkHalfPeriodMs = 1000;

// Count-up ring wraps once per real hour — see design note 7.
constexpr uint32_t kRingCountUpPeriodMs = 3600u * 1000u;

// Battery-check gesture hold times — see design note 10. Asymmetric on
// purpose: a deliberate hold to enter, but essentially instant to leave.
constexpr uint32_t kBatteryViewEnterMs = 500;
constexpr uint32_t kBatteryViewExitMs = 0;

// LiPo usable range (1S, full ~4.2V, discharge floor ~3.0V) — see design
// note 10 for why this is a plain linear split, not a discharge-curve LUT.
constexpr float kBatteryEmptyV = 3.0f;
constexpr float kBatteryFullV = 4.2f;

// 1-indexed, matches GuiManager::SetBatteryLevel()'s range — clamped
// there too, but clamping here as well keeps this function's own output
// meaningful in isolation (e.g. if ever logged/tested directly).
int BatteryVoltageToFilledBlocks(float voltage_v)
{
    float fraction = (voltage_v - kBatteryEmptyV) / (kBatteryFullV - kBatteryEmptyV);
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    const int blocks = 1 + static_cast<int>(fraction * (GuiManager::kBatteryBlockCount - 1) + 0.5f);
    return blocks;
}

// A/B/C/D pinned directly to content (A=-90 is specifically "the 25/5
// pomodoro face"), not to a physical mounting spot — face-to-enclosure
// mapping is still provisional, see design note 1.
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

    // Matches FaceCenterDeg above.
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
        // Underlying face keeps ticking (design note 10); face
        // switching/ring/render are skipped, the battery view owns the
        // screen instead, running the same idle-dim-then-sleep timeline
        // as everything else via its own counter.
        if (current_) {
            current_->onTick(dt_ms);
        }
        if (attitude.is_moving) {
            battery_view_idle_elapsed_ms_ = 0;
        } else {
            battery_view_idle_elapsed_ms_ += dt_ms;
        }
        gui_manager_.SetBrightness(IdleBrightnessCurve(battery_view_idle_elapsed_ms_));
        gui_manager_.SetRingVisible(false);
        gui_manager_.SetBatteryLevel(BatteryVoltageToFilledBlocks(battery_voltage_v_));
        return;
    }

    UpdateLowBatteryHysteresis();
    if (showing_low_battery_) {
        // Forced "please charge" override, NOT exempt from idle-dim-then-
        // sleep the way showing_battery_ is. Unlike showing_battery_,
        // current_ is force-paused on entry (UpdateLowBatteryHysteresis())
        // and can't be resumed until this clears — see design note 11.
        if (current_) {
            current_->onTick(dt_ms);
        }
        // Countdown lock: once voltage is critical, motion no longer
        // resets this, so continuous handling can't hold the device
        // awake indefinitely below the real-damage edge.
        if (attitude.is_moving && battery_voltage_v_ >= kCriticalBatteryEnterV) {
            low_battery_idle_elapsed_ms_ = 0;
        } else {
            low_battery_idle_elapsed_ms_ += dt_ms;
        }
        gui_manager_.SetBrightness(IdleBrightnessCurve(low_battery_idle_elapsed_ms_));
        gui_manager_.SetRingVisible(false);
        gui_manager_.SetAccentColor(kLowBatteryColor);
        gui_manager_.SetPrimaryTextOpacity(255);  // undo whatever face was mid-fade when this screen took over
        // "Low"/"Battery" split across the two single-line labels — see
        // GuiManager::SetPrimaryText()'s comment for why a single 2-line
        // string doesn't work here.
        gui_manager_.SetSecondaryText("Low");
        gui_manager_.SetPrimaryText("Battery");
        return;
    }

    const Face quantized = QuantizeFace(attitude.screen_angle_deg);

    // Commit the instant quantized disagrees with the confirmed face (or
    // there's no face yet, at boot) — see design note 2.
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
        // orientation, once per switch not every tick — see design note 7.
        gui_manager_.SetRingOrientation(FaceCenterDeg(current_face_));

        // A flip always reads as an obvious bright event — see design note 5.
        brightness_ = 1.0f;
        bright_phase_elapsed_ms_ = 0;
        paused_elapsed_ms_ = 0;
        prev_is_running_ = false;
        prev_has_target_ = false;
        prev_remaining_ms_ = 0;

        tap_mute_remaining_ms_ = kTapMuteAfterSwitchMs;  // see design note 6
    } else if (tap_mute_remaining_ms_ > 0 && !attitude.is_moving) {
        // Only count down once actually settled — see design note 6.
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

    // Tapping, flipping, and plain movement all count as "the user is
    // engaging with the device right now" — see design note 5.
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
        brightness_ = IdleBrightnessCurve(paused_elapsed_ms_);
    }

    gui_manager_.SetBrightness(brightness_);

    prev_is_running_ = status.is_running;
    prev_has_target_ = status.has_target;
    prev_remaining_ms_ = status.remaining_ms;
}

bool AppController::ShouldEnterIdleSleep() const
{
    return paused_elapsed_ms_ >= kIdleSleepThresholdMs ||
           (showing_low_battery_ && low_battery_idle_elapsed_ms_ >= kIdleSleepThresholdMs) ||
           (showing_battery_ && battery_view_idle_elapsed_ms_ >= kIdleSleepThresholdMs);
}

void AppController::NotifyWokeFromIdleSleep()
{
    paused_elapsed_ms_ = 0;
    low_battery_idle_elapsed_ms_ = 0;
    battery_view_idle_elapsed_ms_ = 0;
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
        gui_manager_.SetRingVisible(false);  // nothing to show progress for
        ring_pause_blink_elapsed_ms_ = 0;
        return;
    }
    gui_manager_.SetRingVisible(true);

    // 0..1, how far through the current phase (countdown) or current
    // hour (count-up) things are — see design note 7.
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
        // Reset so a *later* pause always starts its blink from "on".
        ring_pause_blink_elapsed_ms_ = 0;
        gui_manager_.SetRingTickOpacity(255);
        return;
    }

    // Not running: only show the tick once there's genuine progress to
    // pause *from* — a fresh face/phase (elapsed_fraction==0) hides it
    // entirely until the phase actually starts running.
    constexpr float kMinElapsedFractionForTick = 0.001f;  // guards against float noise landing exactly at 0
    if (elapsed_fraction <= kMinElapsedFractionForTick) {
        ring_pause_blink_elapsed_ms_ = 0;
        gui_manager_.SetRingTickOpacity(0);
        return;
    }

    // Paused with real progress: hard on/off blink, driven by a real
    // wall-clock accumulator since remaining_ms is frozen while paused.
    ring_pause_blink_elapsed_ms_ += dt_ms;
    const bool blink_on = (ring_pause_blink_elapsed_ms_ / kRingBlinkHalfPeriodMs) % 2 == 0;
    gui_manager_.SetRingTickOpacity(blink_on ? 255 : 0);
}

void AppController::OnTap()
{
    if (showing_battery_) {
        // Doesn't reach the underlying face, but counts as "paying
        // attention" for this view's own idle timer — see design note 10.
        battery_view_idle_elapsed_ms_ = 0;
        return;
    }
    if (showing_low_battery_) {
        // Swallowed rather than forwarded to current_->onTap() — see
        // design note 11. Below kCriticalBatteryEnterV the reset itself
        // is withheld too (countdown lock).
        if (battery_voltage_v_ >= kCriticalBatteryEnterV) {
            low_battery_idle_elapsed_ms_ = 0;
        }
        return;
    }
    if (tap_mute_remaining_ms_ > 0) {
        return;  // see design note 6
    }
    if (current_) {
        current_->onTap();
    }
}

void AppController::UpdateBatteryView(const AttitudeEstimator::Output& attitude, uint32_t dt_ms)
{
    // battery_view_hold_ms_ counts continuous time in the state opposite
    // showing_battery_'s current value, reset once attitude.in_valid_plane
    // agrees with the current state again — see design note 10.
    if (!showing_battery_) {
        if (!attitude.in_valid_plane) {
            battery_view_hold_ms_ += dt_ms;
            if (battery_view_hold_ms_ >= kBatteryViewEnterMs) {
                showing_battery_ = true;
                battery_view_hold_ms_ = 0;
                battery_view_idle_elapsed_ms_ = 0;  // fresh bright-then-dim cycle starting now
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

void AppController::UpdateLowBatteryHysteresis()
{
    // Plain two-threshold hysteresis, no hold timer — voltage doesn't
    // bounce the way a gesture's in_valid_plane does.
    if (showing_low_battery_) {
        if (battery_voltage_v_ >= kLowBatteryExitV) {
            showing_low_battery_ = false;
        }
    } else {
        if (battery_voltage_v_ < kLowBatteryEnterV) {
            showing_low_battery_ = true;
            low_battery_idle_elapsed_ms_ = 0;

            // Force-pause a running timer via a synthetic tap, reusing
            // each face's own onTap() pause logic — only fires while
            // genuinely running, since the same onTap() would otherwise
            // start it instead.
            if (current_ && current_->GetStatus().is_running) {
                current_->onTap();
            }
        }
    }
}
