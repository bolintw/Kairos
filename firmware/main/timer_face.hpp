#pragma once

#include <cstdint>

class GuiManager;  // forward-declared — concrete rendering is out of
                    // scope until M7; this header only needs the type
                    // name to declare render()'s signature.

// DRAFT — interface as already sketched in the plan; writing it here to
// confirm it as final before building StopwatchFace against it.
//
// A TimerFace only knows its own counting logic and what to display —
// nothing about other faces, attitude/hysteresis, brightness, or
// persistence. AppController owns all of that and drives this interface.
class TimerFace {
public:
    virtual void onEnter() = 0;               // flip into this face: reset and start paused
    virtual void onExit() = 0;
    virtual void onTick(uint32_t dt_ms) = 0;   // called every loop tick regardless of running state
    virtual void onTap() = 0;                  // toggle running/paused
    virtual void render(GuiManager& gui) = 0;
    virtual ~TimerFace() = default;
};
