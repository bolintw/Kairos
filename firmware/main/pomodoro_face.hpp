#pragma once

#include <cstdint>

#include "timer_face.hpp"

// One concrete class parameterized by focus/break duration, rather than
// a base class with Pomodoro25_5/Pomodoro50_10 subclasses — the two
// variants have no behavioral difference, only two different numbers, so
// subclassing would just be boilerplate forwarding constructor args (see
// discussion — agreed 2026-08-23). The Factory instantiates this with
// 25/5 min for face A, 50/10 min for face B; host tests can pass
// arbitrary short durations to exercise the phase transition without
// waiting real minutes.
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

    // Resolved 2026-08-23: focus<->break both auto-advance (no tap
    // needed) and loop indefinitely — running an unattended device
    // through repeated cycles is accepted; the safety net lives at the
    // system level (low-battery forced sleep, see plan's M9), not here.
    void AdvancePhase();

    uint32_t focus_ms_;
    uint32_t break_ms_;
    Phase phase_ = Phase::kFocus;
    uint32_t remaining_ms_ = 0;
    bool running_ = false;
};
