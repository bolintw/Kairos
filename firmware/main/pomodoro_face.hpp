#pragma once

#include <cstdint>

#include "timer_face.hpp"

// One class parameterized by focus/break duration rather than
// Pomodoro25_5/Pomodoro50_10 subclasses, since the two variants only
// differ by two numbers. The Factory instantiates this with 25/5 min for
// face A, 50/10 min for face B.
class PomodoroFace : public TimerFace {
public:
    PomodoroFace(uint32_t focus_ms, uint32_t break_ms);

    void onEnter() override;   // reset to Phase::kFocus, full duration, paused
    void onExit() override;
    void onTick(uint32_t dt_ms) override;
    void onTap() override;     // toggle running/paused
    void render(GuiManager& gui) override;
    Status GetStatus() const override;

private:
    enum class Phase { kFocus, kBreak };

    // Focus<->break both auto-advance and loop indefinitely; an
    // unattended device just keeps cycling (the low-battery safety net
    // handles that case at the system level, not here).
    void AdvancePhase();

    uint32_t focus_ms_;
    uint32_t break_ms_;
    Phase phase_ = Phase::kFocus;
    uint32_t remaining_ms_ = 0;
    bool running_ = false;

    // While nonzero, render() shows "Focus"/"Relax" instead of the MM:SS
    // countdown. Decremented in onTick() even while paused — it's an
    // announcement of an event that already happened, not part of the
    // countdown itself.
    uint32_t transition_remaining_ms_ = 0;
};
