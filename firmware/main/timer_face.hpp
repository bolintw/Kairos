#pragma once

#include <cstdint>

class GuiManager;  // forward-declared — only the type name is needed here

// A TimerFace only knows its own counting logic and what to display —
// nothing about other faces, attitude/hysteresis, brightness, or
// persistence. AppController owns all of that and drives this interface.
class TimerFace {
public:
    // Just facts about the face's current state — AppController
    // interprets these into brightness/ring/phase-transition behavior.
    struct Status {
        bool is_running;
        bool has_target;         // false for StopwatchFace (no target duration)
        uint32_t remaining_ms;   // has_target: ms left in the phase. is_count_up: ms elapsed instead. Else unused.
        bool is_break_phase;     // "break-like" (stays bright) vs "focus-like" (fades then ramps up near the end)
        uint32_t target_ms;      // full duration of the current phase; 0 if !has_target
        bool is_count_up;        // true for a genuine count-up face (e.g. StopwatchFace)
    };

    virtual void onEnter() = 0;               // flip into this face: reset and start paused
    virtual void onExit() = 0;
    virtual void onTick(uint32_t dt_ms) = 0;   // called every loop tick regardless of running state
    virtual void onTap() = 0;                  // toggle running/paused
    virtual void render(GuiManager& gui) = 0;
    virtual Status GetStatus() const = 0;
    virtual ~TimerFace() = default;
};
