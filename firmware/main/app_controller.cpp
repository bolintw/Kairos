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

// See app_controller.hpp's design note 1 for why 80 degrees produces the
// intended asymmetric enter/leave band by itself.
constexpr float kFaceHysteresisLeaveDeg = 80.0f;

// Brightness/notification tuning — starting points per the plan's own
// note that these need real usage to validate, not to be locked down
// here. See app_controller.hpp design note 5.
constexpr float kDimmedBrightness = 0.3f;        // immersion-dim floor while running
constexpr uint32_t kFadeToDimMs = 10 * 1000;      // ~10s fade after becoming bright
constexpr uint32_t kEndPulseWindowMs = 10 * 1000; // last ~10s of a phase with a target
constexpr uint32_t kPulsePeriodMs = 1500;         // one breathing dark-bright-dark cycle
constexpr float kPulseLowBrightness = 0.3f;
constexpr uint32_t kLongIdleTimeoutMs = 5 * 60 * 1000;  // minutes-scale, paused-only, backlight off

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
        case Face::kD: return std::make_unique<ReservedFace>();  // debug placeholder, see design note 4
    }
    return nullptr;
}

void AppController::Update(const AttitudeEstimator::Output& attitude, uint32_t dt_ms)
{
    const Face quantized = QuantizeFace(attitude.screen_angle_deg);

    // Commit as soon as we're at rest and quantized disagrees with the
    // confirmed face (or there's no face yet, at boot) — see design note 2
    // for why this doesn't need to also confirm is_moving was observed
    // true at some point: QuantizeFace's 80-degree hysteresis already
    // proves real movement happened, so gating on the instantaneous gyro
    // threshold too was redundant, and broke on a slow final correction
    // that crossed the boundary without ever exceeding that threshold —
    // was_disturbed_ never latched, so the commit below never ran, even
    // though quantized was already correct.
    if (!attitude.is_moving && (!current_ || quantized != current_face_)) {
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
    }

    if (current_) {
        current_->onTick(dt_ms);
    }
    UpdateBrightness(dt_ms);
    if (current_) {
        current_->render(gui_manager_);
    }
}

void AppController::UpdateBrightness(uint32_t dt_ms)
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

    if (just_started || phase_changed) {
        brightness_ = 1.0f;
        bright_phase_elapsed_ms_ = 0;
    }
    if (just_paused) {
        brightness_ = 1.0f;  // 暫停時立即轉亮
    }

    float warmth = 0.0f;

    if (status.is_running) {
        paused_elapsed_ms_ = 0;
        bright_phase_elapsed_ms_ += dt_ms;

        const bool in_end_pulse = status.has_target && status.remaining_ms <= kEndPulseWindowMs;
        if (in_end_pulse) {
            const uint32_t t = bright_phase_elapsed_ms_ % kPulsePeriodMs;
            const float phase = static_cast<float>(t) / static_cast<float>(kPulsePeriodMs);
            const float triangle = phase < 0.5f ? (phase * 2.0f) : (2.0f - phase * 2.0f);
            brightness_ = kPulseLowBrightness + triangle * (1.0f - kPulseLowBrightness);
            warmth = 1.0f;
        } else if (!just_started && !phase_changed) {
            const float t = static_cast<float>(bright_phase_elapsed_ms_) / static_cast<float>(kFadeToDimMs);
            const float clamped_t = t < 1.0f ? t : 1.0f;
            brightness_ = 1.0f + clamped_t * (kDimmedBrightness - 1.0f);
        }
        // else: just_started/phase_changed already set brightness_=1.0
        // above; the fade begins next tick, not this one.
    } else {
        paused_elapsed_ms_ += dt_ms;
        if (paused_elapsed_ms_ >= kLongIdleTimeoutMs) {
            brightness_ = 0.0f;
        }
    }

    gui_manager_.SetBrightness(brightness_);
    gui_manager_.SetWarmth(warmth);

    prev_is_running_ = status.is_running;
    prev_has_target_ = status.has_target;
    prev_remaining_ms_ = status.remaining_ms;
}

void AppController::OnTap()
{
    if (current_) {
        current_->onTap();
    }
}
