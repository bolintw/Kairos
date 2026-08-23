#pragma once

#include <memory>

#include "attitude_estimator.hpp"
#include "gui_manager.hpp"
#include "timer_face.hpp"

// DRAFT — face-switching + Factory. Revised 2026-08-23 after hardware
// testing surfaced a real bug in the first version (see below).
//
// Design:
//
// 1. Quantize AttitudeEstimator::Output::screen_angle_deg into A/B/C/D,
//    centered on -90/0/90/180 (A/B/C/D respectively, set 2026-08-23).
//    Physical mapping is provisional pending M10 enclosure geometry —
//    trivial to change later. A=PomodoroFace(25,5), B=PomodoroFace(50,10),
//    C=StopwatchFace, D=nullptr (see CreateFace).
//
//    Angle hysteresis (added 2026-08-23, user's drone flight-controller
//    background): quantization alone would flip-flop if the settled
//    angle sits near a 45-degree boundary (small accel noise while
//    resting is enough — alpha=0.8 still applies 20% accel weight every
//    tick). QuantizeFace() is stateful: it only moves off current_face_
//    once the angle is more than kFaceHysteresisLeaveDeg (80) away from
//    current_face_'s own center; otherwise it stays put. That single rule
//    produces both halves of the intended band by construction — e.g.
//    currently on B (center 0): stays B until angle passes -80/+80
//    (matches "leave at ±80"), and once on A or C, doesn't come back to B
//    until within 10 of B's center (80 away from A/C's own center —
//    same rule, just measured from the other side).
//
// 2. Reset (onExit/onEnter) fires only when the device actually crossed
//    into a *different* quantized face at some point during a disturbance
//    and then settled — not on every is_moving blip. First version fired
//    on any settle-after-motion, including settling back to the SAME
//    face; on hardware, a plain tap's own vibration was enough to cross
//    is_moving's threshold, so pressing tap-to-pause was intermittently
//    read as a full reset instead. Tracked via current_face_ (the
//    confirmed face) and crossed_face_ (set the instant the quantized
//    face differs from current_face_ while disturbed, cleared on
//    settle). Boot is bootstrapped the same way: crossed_face_ starts
//    true so the first settle after boot always creates a face.
//
// 3. Taps are forwarded unconditionally — NOT gated on is_moving (that
//    was the first version's attempted fix for accidental tap-engine
//    triggers mid-flip, but it made legitimate taps near a settle feel
//    unresponsive, and is no longer needed: with (2) fixed, a tap that
//    lands during real motion just toggles the pre-flip face, and gets
//    overwritten by onEnter()'s reset moments later if a genuine flip is
//    confirmed anyway).
//
// 4. Face D has no defined behavior yet (plan: 待定，暫緩實作). Rather
//    than a placeholder concrete TimerFace, `current_` is simply nullptr
//    while on face D; Update()/OnTap() no-op in that case (brightness set
//    to 0 — nothing to show). Revisit once D's functionality is decided.
//
// 5. Brightness/notification state machine (M7, plan's "亮度作為通知系
//    統"), driven off TimerFace::Status polled each tick, edge-detected
//    here (is_running true->false/false->true, remaining_ms jumping up
//    = a new phase started):
//      - face entry (onEnter() just called): snap to full bright,
//        regardless of the new face's initial is_running (always paused
//        on entry, but a flip should read as an obvious bright event)
//      - just started running, or a phase just changed: snap to full
//        bright, then fade toward a dimmed level over ~10s while running
//      - within the last ~10s of a phase with a target duration: breathing
//        brightness pulse + warm/red tint, overriding the fade
//      - just paused: snap to full bright immediately
//      - paused continuously past a much longer (minutes-scale) idle
//        timeout: dim to fully off, independent of the ~10s timeout above
//    TimerFace never sees any of this — GetStatus() is facts only.
//
// AttitudeEstimator is NOT held by reference here — main.cpp calls
// AttitudeEstimator::Update() once per tick (single call site, avoids
// double-integrating the gyro angle) and passes the resulting Output in.
class AppController {
public:
    enum class Face { kA, kB, kC, kD };

    explicit AppController(GuiManager& gui_manager);

    // Call once per sensor tick with the latest attitude output and
    // elapsed time since the previous call.
    void Update(const AttitudeEstimator::Output& attitude, uint32_t dt_ms);

    // Call when the tap engine reports a new tap.
    void OnTap();

private:
    Face QuantizeFace(float screen_angle_deg) const;  // stateful — reads current_face_, see design note 1 above
    std::unique_ptr<TimerFace> CreateFace(Face face);  // the "Factory"
    void UpdateBrightness(uint32_t dt_ms);             // see design note 5 above

    GuiManager& gui_manager_;
    std::unique_ptr<TimerFace> current_;  // nullptr while on face D
    Face current_face_ = Face::kA;         // confirmed face; meaningless until first settle
    bool was_disturbed_ = true;            // starts true so boot's first settle is handled
    bool crossed_face_ = true;             // starts true so boot's first settle creates a face

    float brightness_ = 1.0f;
    uint32_t bright_phase_elapsed_ms_ = 0;  // ms since the last "became bright" event, drives the fade
    uint32_t paused_elapsed_ms_ = 0;        // ms continuously paused, drives the long idle timeout
    bool prev_is_running_ = false;
    bool prev_has_target_ = false;
    uint32_t prev_remaining_ms_ = 0;
};
