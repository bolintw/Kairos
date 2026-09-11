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
    // "brightness as the notification system"): AppController needs to
    // know whether the face is running (dims on run, brightens on pause
    // — detected by AppController watching is_running edges tick to
    // tick, not a dedicated "just started" flag here) and, if it has a
    // target duration, how much is left (for an end-of-phase cue).
    // StopwatchFace has no
    // target, so has_target=false and remaining_ms is meaningless.
    // is_break_phase (added 2026-08-25) lets AppController tell a
    // "focus-like" phase (fades while running, ramps to full bright near
    // the end) from a "break-like" one (stays fully bright throughout) —
    // still just a fact about which phase this is, not a brightness
    // policy; StopwatchFace has no phases, so always false. Deliberately
    // just facts, no brightness/phase-transition semantics here — that
    // interpretation stays in AppController.
    // target_ms/is_count_up added 2026-09-09 for AppController's progress
    // ring redesign (see its UpdateRing() design note): the ring needs to
    // know not just *how much time is left*, but *what fraction of the
    // whole* that is, which target_ms supplies for the has_target case
    // (elapsed = target_ms - remaining_ms). is_count_up distinguishes a
    // genuine count-up face (StopwatchFace — grows a ring from empty, one
    // hour per revolution) from "no active phase at all" (BreathFace's
    // idle screen also reports has_target=false, but isn't counting
    // anything up either — the ring stays hidden there); when
    // is_count_up is true, remaining_ms is repurposed to carry ms elapsed
    // in the current run instead of its usual "ms left" meaning, since
    // it's otherwise unused for a face with no target.
    struct Status {
        bool is_running;
        bool has_target;
        uint32_t remaining_ms;   // has_target: ms left in the current phase. is_count_up (has_target false): ms elapsed in the current run. Neither: unused/0.
        bool is_break_phase;
        uint32_t target_ms;      // has_target: full duration of the current phase. Else 0/unused.
        bool is_count_up;
    };

    virtual void onEnter() = 0;               // flip into this face: reset and start paused
    virtual void onExit() = 0;
    virtual void onTick(uint32_t dt_ms) = 0;   // called every loop tick regardless of running state
    virtual void onTap() = 0;                  // toggle running/paused
    virtual void render(GuiManager& gui) = 0;
    virtual Status GetStatus() const = 0;
    virtual ~TimerFace() = default;
};
