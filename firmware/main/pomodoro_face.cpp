#include "pomodoro_face.hpp"

#include <cmath>
#include <cstdio>

#include "gui_manager.hpp"

namespace {
// Accent colors (2026-08-25 "UI polish" pass): focus=red, break=green — the
// primary text dropped its "Focus"/"Break" word prefix in favor of color
// alone carrying that distinction, applied to both the label and the
// outer ring via GuiManager::SetAccentColor().
const lv_color_t kColorFocus = lv_color_make(255, 60, 60);
const lv_color_t kColorBreak = lv_color_make(35, 150, 65);  // darker than the initial pick, 2026-08-25

// See pomodoro_face.hpp's transition_remaining_ms_ comment.
constexpr uint32_t kTransitionMs = 2500;
}  // namespace

PomodoroFace::PomodoroFace(uint32_t focus_ms, uint32_t break_ms)
    : focus_ms_(focus_ms), break_ms_(break_ms)
{
}

void PomodoroFace::onEnter()
{
    phase_ = Phase::kFocus;
    remaining_ms_ = focus_ms_;
    running_ = false;
    transition_remaining_ms_ = 0;  // no caption on a fresh flip onto this face
}

void PomodoroFace::onExit()
{
}

void PomodoroFace::onTick(uint32_t dt_ms)
{
    // Unconditional, ahead of the running_ gate below — the transition
    // caption is an announcement, not part of the countdown, so it plays
    // out on wall-clock time even through a pause (see the field comment
    // in pomodoro_face.hpp).
    if (transition_remaining_ms_ > 0) {
        transition_remaining_ms_ = dt_ms < transition_remaining_ms_ ? transition_remaining_ms_ - dt_ms : 0;
    }

    if (!running_) return;

    // Looped rather than a single if-check: a dt_ms larger than the
    // remaining time in the current phase should walk through as many
    // phase transitions as it spans, not just one — matters for tests
    // fast-forwarding with large dt_ms values, and is correct regardless
    // even though real loop ticks are far smaller than a phase duration.
    while (dt_ms >= remaining_ms_) {
        dt_ms -= remaining_ms_;
        AdvancePhase();
    }
    remaining_ms_ -= dt_ms;
}

void PomodoroFace::onTap()
{
    running_ = !running_;
}

void PomodoroFace::AdvancePhase()
{
    if (phase_ == Phase::kFocus) {
        phase_ = Phase::kBreak;
        remaining_ms_ = break_ms_;
    } else {
        phase_ = Phase::kFocus;
        remaining_ms_ = focus_ms_;
    }
    // running_ deliberately untouched — auto-loop, see header.
    transition_remaining_ms_ = kTransitionMs;
}

void PomodoroFace::render(GuiManager& gui)
{
    gui.SetAccentColor(phase_ == Phase::kFocus ? kColorFocus : kColorBreak);
    // Defensive, not cosmetic: GuiManager's secondary-line text persists
    // across face switches, so flipping here from BreathFace (which uses
    // it for its phase name) would otherwise leave a stale "Inhale"/
    // "Hold"/"Exhale" floating above this face's own countdown.
    gui.SetSecondaryText("");

    if (transition_remaining_ms_ > 0) {
        // Fade the caption itself in and back out over the window instead
        // of popping it in as flat text — a plain swap read as too abrupt
        // next to the ring's smooth breathing (2026-08-26, caught on
        // hardware). One sine hump across the whole window: 0 at the
        // first frame (elapsed=0), peaks at the midpoint, back to 0 on
        // the last frame right before render() falls through to the
        // numeric display below.
        constexpr float kPi = 3.14159265359f;
        const uint32_t elapsed_ms = kTransitionMs - transition_remaining_ms_;
        const float t = static_cast<float>(elapsed_ms) / static_cast<float>(kTransitionMs);
        const uint8_t opa = static_cast<uint8_t>(std::sin(kPi * t) * 255.0f);
        gui.SetPrimaryTextOpacity(opa);
        gui.SetPrimaryText(phase_ == Phase::kFocus ? "Focus" : "Relax");
        return;
    }

    gui.SetPrimaryTextOpacity(255);  // undo the caption's fade, in case it just ended

    const uint32_t total_seconds = remaining_ms_ / 1000;
    const uint32_t minutes = total_seconds / 60;
    const uint32_t seconds = total_seconds % 60;

    char buf[24];
    std::snprintf(buf, sizeof(buf), "%02u:%02u",
                   static_cast<unsigned int>(minutes), static_cast<unsigned int>(seconds));
    gui.SetPrimaryText(buf);
}

TimerFace::Status PomodoroFace::GetStatus() const
{
    return Status{running_, /*has_target=*/true, remaining_ms_, phase_ == Phase::kBreak};
}
