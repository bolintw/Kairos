#include "pomodoro_face.hpp"

#include <cstdio>

#include "gui_manager.hpp"

PomodoroFace::PomodoroFace(uint32_t focus_ms, uint32_t break_ms)
    : focus_ms_(focus_ms), break_ms_(break_ms)
{
}

void PomodoroFace::onEnter()
{
    phase_ = Phase::kFocus;
    remaining_ms_ = focus_ms_;
    running_ = false;
}

void PomodoroFace::onExit()
{
}

void PomodoroFace::onTick(uint32_t dt_ms)
{
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
}

void PomodoroFace::render(GuiManager& gui)
{
    const uint32_t total_seconds = remaining_ms_ / 1000;
    const uint32_t minutes = total_seconds / 60;
    const uint32_t seconds = total_seconds % 60;

    char buf[24];
    std::snprintf(buf, sizeof(buf), "%s %02u:%02u",
                   phase_ == Phase::kFocus ? "Focus" : "Break",
                   static_cast<unsigned int>(minutes), static_cast<unsigned int>(seconds));
    gui.SetPrimaryText(buf);
}

TimerFace::Status PomodoroFace::GetStatus() const
{
    return Status{running_, /*has_target=*/true, remaining_ms_};
}
