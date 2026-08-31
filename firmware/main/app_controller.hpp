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
//    C=StopwatchFace, D=BreathFace (see CreateFace).
//
//    Angle hysteresis (added 2026-08-23, user's drone flight-controller
//    background): quantization alone would flip-flop if the settled
//    angle sits near a 45-degree boundary (small accel noise while
//    resting is enough). QuantizeFace() is stateful: it only moves off
//    current_face_ once the angle is more than kFaceHysteresisLeaveDeg
//    away from current_face_'s own center; otherwise it stays put. That
//    single rule produces both halves of the intended band by
//    construction — e.g. currently on B (center 0): stays B until angle
//    passes -kFaceHysteresisLeaveDeg/+kFaceHysteresisLeaveDeg ("leave"),
//    and once on A or C, doesn't come back to B until within
//    (90-kFaceHysteresisLeaveDeg) of B's center ("return" —
//    kFaceHysteresisLeaveDeg away from A/C's own center, same rule, just
//    measured from the other side). Currently 65 (see app_controller.cpp
//    for the 80->70->65 history), so leave at +-65, return within +-25.
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
// 4. Face D's behavior was undecided for a while (plan: undecided, deferred),
//    backed in the meantime first by ReservedFace (a placeholder that
//    just rendered "Reserved") and then briefly by a second PomodoroFace
//    instance for fast iteration on the M7 brightness/notification work.
//    Settled 2026-08-30: BreathFace, a guided 4-7-8-style breathing
//    exercise — see its own header for why it's a distinct class rather
//    than another PomodoroFace variant. `current_` is not expected to be
//    null in normal operation.
//
// 5. Brightness/notification state machine (M7, plan's "brightness as
//    the notification system"), driven off TimerFace::Status polled
//    each tick (is_running true->false/false->true, remaining_ms
//    jumping up = a new phase started) PLUS attitude.is_moving (added
//    2026-08-25) — a single
//    `interacting` flag folds all of these together: "the user is
//    engaging with the device right now", whether that's tapping,
//    flipping faces, or just spinning it in their hand without crossing
//    a face boundary:
//      - face entry (onEnter() just called): snap to full bright,
//        regardless of the new face's initial is_running (always paused
//        on entry, but a flip should read as an obvious bright event) —
//        handled directly in Update(), not part of `interacting` below
//      - interacting while running: snap to full bright and restart the
//        ~10s linear fade toward a dimmed level — so idly spinning the
//        device to watch the rotation animation, with no face change,
//        doesn't let the screen dim out from under you
//      - break phase (Status::is_break_phase): stays fully bright for the
//        entire phase, no fade — accepted battery cost for now, revisit
//        once real battery life is measured (M9)
//      - last ~30s of a focus-like (non-break) phase with a target
//        duration: ramps UP to full bright, reaching it within ~3s
//        (faster than the ~10s dim-fade, so it reads as a distinct
//        event) and holding there until the phase actually changes —
//        deliberately NOT interrupted by mere movement, since drawing
//        attention is the whole point of this window. Replaces an
//        earlier breathing dark-bright-dark pulse design (2026-08-25,
//        user's redesign after using it) — ramping toward brighter reads
//        more clearly and is easier on the eyes than oscillating. The
//        warm/red color tint that used to accompany this was dropped the
//        same day to keep the notification channel to brightness alone
//        for now — GuiManager::SetWarmth() still exists if it comes back
//      - interacting while paused: snap to full bright and restart the
//        long idle timeout below
//      - paused with no interaction for a much longer (minutes-scale)
//        idle timeout: dim to fully off, independent of the ~10s timeout
//        above
//    TimerFace never sees any of this — GetStatus() is facts only;
//    AttitudeEstimator likewise has no notion of brightness.
//
// 6. Tap mute window (2026-08-25): OnTap() is ignored for
//    kTapMuteAfterSwitchMs after a face switch settles — the user found
//    a flip often lands with enough residual wobble/vibration to trip
//    the tap engine an instant later, immediately starting the timer on
//    a face they just arrived at (expected to get worse once the device
//    is inside an enclosure, more surface area to knock). tap_mute_
//    remaining_ms_ is armed to the window length on every face commit,
//    but only counts down while !attitude.is_moving — commits fire the
//    instant quantized changes (design note 2), often still mid-swing,
//    so counting down unconditionally from the commit moment could burn
//    through most of the window before the device actually stops moving,
//    which is when the residual-vibration risk this exists for actually
//    starts. Holding the countdown at full while still moving means the
//    whole window applies from the moment it's needed. OnTap() no-ops
//    while it's nonzero.
//
//    Briefly generalized (same day) to arm/hold on *any*
//    attitude.is_moving, not just a face-switch commit, on the theory
//    that any handling deserves the same grace period. Reverted after
//    hardware testing: a light finger tap on the screen alone was enough
//    to flip is_moving 0->1->0 (matches design note 2's history — a
//    tap's vibration crossing is_moving isn't hypothetical on this
//    hardware, it's already caused one bug before). Under the
//    any-movement version that re-arms the mute window from the tap's
//    own vibration, making the timer noticeably harder to trigger by
//    tapping at all, not just a rare rapid-retap edge case. Scoping the
//    arm back to face-switch commits avoids this: it doesn't re-arm from
//    an ordinary tap's own is_moving blip once the window has already
//    reached 0, since nothing there triggers on is_moving alone.
//
//    Deliberately app-level and orthogonal to the tap engine's own
//    internal detection windows
//    (peak_window/tap_window/d_tap_window in main.cpp's ConfigureTap
//    call) — those shape what counts as a tap at all, this just ignores
//    genuine taps for a moment after a flip.
//
// 7. Outer ring (2026-08-25, "UI polish" pass, revised 2026-08-26): a
//    second notification channel alongside brightness — GuiManager's
//    ring_ (see its header). Purely a function of the current
//    TimerFace::Status snapshot each tick, no elapsed-time state of its
//    own, unlike UpdateBrightness's fade/idle timers — "paused" and
//    "remaining_ms" are already facts available every tick.
//
//    Settled meaning (2026-08-26): the ring means exactly one thing,
//    "paused" (solid), at any point in a phase — checked first in
//    UpdateRing(), unconditionally, before anything else — PLUS a
//    distinct breathing cue in the closing kRingBreathWindowSec seconds
//    while still running. Originally also solid for a much wider ~30s
//    "approaching the end" window regardless of running/paused — dropped
//    same day the caption (note 8) shipped, once the user noticed a
//    paused ring and a merely-near-the-end running ring looked identical
//    in that window, making it impossible to tell from the ring alone
//    whether a pause during those 30s had actually registered. The
//    wider "approaching the end" cue still exists, just moved entirely
//    to brightness (kFocusEndRampWindowMs) — the ring no longer
//    double-duties as that signal.
//
//    Because the `!is_running` check runs first and unconditionally,
//    pausing during the breathing window snaps straight to solid 255
//    with no special-casing needed, and resuming falls back into the
//    breathing branch and picks the wave up from wherever it already
//    was — remaining_ms is frozen while paused, so nothing needs to
//    remember "the brightness before pausing", it's just still there.
//
//    The breathing itself was originally a hard on/off blink derived
//    from (remaining_ms/1000) % 2; the user found that too harsh on real
//    hardware, replaced same day with a smooth fade (SetRingOpacity, a
//    cosine wave over remaining_ms % 1000) — still derived straight from
//    remaining_ms rather than a separate accumulating timer, so it can't
//    drift out of phase with the digits.
//
// 8. Phase-transition caption (2026-08-26): entirely inside PomodoroFace,
//    not AppController — see pomodoro_face.hpp's transition_remaining_ms_
//    field comment. Mentioned here only because design note 7 above
//    references it: the two features share a screen but were built to be
//    fully orthogonal (the caption doesn't know the ring exists, and vice
//    versa), which is what let each one get simplified/fixed
//    independently without touching the other.
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

    // Call when the tap engine reports a new tap. Ignored for a short
    // window right after a face switch — see design note 6.
    void OnTap();

private:
    Face QuantizeFace(float screen_angle_deg) const;  // stateful — reads current_face_, see design note 1 above
    std::unique_ptr<TimerFace> CreateFace(Face face);  // the "Factory"
    void UpdateBrightness(uint32_t dt_ms, bool is_moving);  // see design note 5 above
    void UpdateRing();  // see design note 7 above

    GuiManager& gui_manager_;
    std::unique_ptr<TimerFace> current_;  // nullptr while on face D
    Face current_face_ = Face::kA;         // confirmed face; meaningless until current_ is set

    float brightness_ = 1.0f;
    uint32_t bright_phase_elapsed_ms_ = 0;  // ms since the last "became bright" event, drives the fade
    uint32_t paused_elapsed_ms_ = 0;        // ms continuously paused, drives the long idle timeout
    bool prev_is_running_ = false;
    bool prev_has_target_ = false;
    uint32_t prev_remaining_ms_ = 0;

    uint32_t tap_mute_remaining_ms_ = 0;  // see design note 6
};
