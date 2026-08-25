#pragma once

#include <cstdint>

class GuiManager;  // forward-declared — concrete rendering is out of
                    // scope until M7; this header only needs the type
                    // name to declare render()'s signature.

// A TimerFace only knows its own counting logic and what to display —
// nothing about other faces, attitude/hysteresis, brightness, or
// persistence. AppController owns all of that and drives this interface.
class TimerFace {
public:
    // Added 2026-08-23 for M7's brightness/notification logic (plan's
    // "亮度作為通知系統"): AppController needs to know whether the face
    // is running (漸暗 starts on run, 暫停轉亮 on pause — detected by
    // AppController watching is_running edges tick to tick, not a
    // dedicated "just started" flag here) and, if it has a target
    // duration, how much is left (尾聲提示). StopwatchFace has no
    // target, so has_target=false and remaining_ms is meaningless.
    // is_break_phase (added 2026-08-25) lets AppController tell a
    // "focus-like" phase (fades while running, ramps to full bright near
    // the end) from a "break-like" one (stays fully bright throughout) —
    // still just a fact about which phase this is, not a brightness
    // policy; StopwatchFace has no phases, so always false. Deliberately
    // just facts, no brightness/phase-transition semantics here — that
    // interpretation stays in AppController.
    struct Status {
        bool is_running;
        bool has_target;
        uint32_t remaining_ms;
        bool is_break_phase;
    };

    virtual void onEnter() = 0;               // flip into this face: reset and start paused
    virtual void onExit() = 0;
    virtual void onTick(uint32_t dt_ms) = 0;   // called every loop tick regardless of running state
    virtual void onTap() = 0;                  // toggle running/paused
    virtual void render(GuiManager& gui) = 0;
    virtual Status GetStatus() const = 0;
    virtual ~TimerFace() = default;
};
