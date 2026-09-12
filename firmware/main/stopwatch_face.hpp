#pragma once

#include "timer_face.hpp"

// Counts elapsed running time up from zero, no target duration, no phase
// transitions.
class StopwatchFace : public TimerFace {
public:
    void onEnter() override;
    void onExit() override;
    void onTick(uint32_t dt_ms) override;
    void onTap() override;
    void render(GuiManager& gui) override;
    Status GetStatus() const override;

private:
    uint32_t elapsed_ms_ = 0;
    bool running_ = false;
};
