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
// 2. Reset (onExit/onEnter) fires only when, at rest, the quantized face
//    disagrees with current_face_ (the confirmed face) — not on every
//    is_moving blip. First version fired on any settle-after-motion,
//    including settling back to the SAME face; on hardware, a plain tap's
//    own vibration was enough to cross is_moving's threshold, so
//    pressing tap-to-pause was intermittently read as a full reset
//    instead. Fixed by comparing quantized to current_face_ directly
//    instead of resetting unconditionally on every settle.
//
//    Second version (still buggy, fixed 2026-08-24) additionally required
//    catching is_moving==true at some point before allowing the commit,
//    on the theory that this would filter out spurious settle events.
//    That extra gate was redundant — QuantizeFace's 80-degree hysteresis
//    is already proof real movement happened — and it broke on a slow
//    final correction across the boundary that never exceeded the
//    is_moving gyro-rate threshold: the commit was gated behind a latch
//    that only a fast-enough motion could set, so a gentle return to a
//    face stayed stuck until some later unrelated fast flip happened to
//    re-arm it. Fixed by comparing quantized to current_face_ directly,
//    still gated on `!attitude.is_moving` (only commit once settled).
//
//    Third version (2026-08-25): dropped the `!attitude.is_moving` gate
//    entirely — commits the instant quantized disagrees with
//    current_face_, moving or not. That gate was originally kept as a
//    hedge against gyro angle overshoot during a flip (pre gyro-scale-fix
//    /alpha-tuning, a fast rotation could transiently read 30-40 degrees
//    past true, so switching mid-motion risked triggering on a bogus
//    reading); once that overshoot was fixed, the user tried removing it
//    on hardware and preferred the immediate feel. QuantizeFace's own
//    80-degree hysteresis still rejects resting noise on its own — the
//    is_moving gate was redundant on top of it, same shape of fix as the
//    latch removal above. Trade-off accepted: a fast swipe that passes
//    *through* a face's zone on the way to another one now commits (and
//    resets) that passed-through face too, not just the final settled
//    one. Boot is bootstrapped via `!current_` (no face exists yet)
//    rather than a separate "always commit once" flag.
//
// 3. Taps are forwarded unconditionally — NOT gated on is_moving (that
//    was the first version's attempted fix for accidental tap-engine
//    triggers mid-flip, but it made legitimate taps near a settle feel
//    unresponsive, and is no longer needed: with (2) fixed, a tap that
//    lands during real motion just toggles the pre-flip face, and gets
//    overwritten by onEnter()'s reset moments later if a genuine flip is
//    confirmed anyway).
//
// 4. Face D has no defined behavior yet (plan: 待定，暫緩實作). Backed by
//    ReservedFace (added 2026-08-24) — a placeholder TimerFace that just
//    renders "Reserved" and no-ops everything else — so a flip to D is
//    visible on screen instead of indistinguishable from current_ being
//    null. Revisit once D's functionality is decided; `current_` is no
//    longer expected to be null in normal operation.
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
    Face current_face_ = Face::kA;         // confirmed face; meaningless until current_ is set

    float brightness_ = 1.0f;
    uint32_t bright_phase_elapsed_ms_ = 0;  // ms since the last "became bright" event, drives the fade
    uint32_t paused_elapsed_ms_ = 0;        // ms continuously paused, drives the long idle timeout
    bool prev_is_running_ = false;
    bool prev_has_target_ = false;
    uint32_t prev_remaining_ms_ = 0;
};
