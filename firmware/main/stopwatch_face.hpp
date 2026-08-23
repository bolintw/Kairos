#pragma once

#include "timer_face.hpp"

// Simplest face: counts elapsed running time up from zero, no target
// duration, no phase transitions. First face implemented to validate the
// AppController<->TimerFace tap/tick/render pipeline end to end before
// adding the more involved Pomodoro faces.
class StopwatchFace : public TimerFace {
public:
    void onEnter() override;
    void onExit() override;
    void onTick(uint32_t dt_ms) override;
    void onTap() override;
    void render(GuiManager& gui) override;

private:
    uint32_t elapsed_ms_ = 0;
    bool running_ = false;
};
