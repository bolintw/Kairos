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
//    centered on -90/0/90/180 (A/B/C/D respectively, set 2026-08-23; these
//    numbers briefly swapped for A/C then reverted the same day, 2026-09-11
//    — see FaceCenterDeg's comment in app_controller.cpp — when
//    screen_angle_deg's CW->CCW-positive flip meant keeping these numbers
//    fixed to their content changes which physical twist reaches each one).
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
//        idle-sleep countdown below
//      - paused with no interaction for kIdlePreDimHoldMs (2026-09-06,
//        replaces an earlier minutes-scale "dim to off" timeout that
//        never got past a first draft): fast-fade to fully off over
//        kIdleFadeToOffMs, then hold off for kIdleOffHoldMs more —
//        ShouldEnterIdleSleep() becomes true once that whole sequence
//        (kIdlePreDimHoldMs+kIdleFadeToOffMs+kIdleOffHoldMs, ~18s) has
//        elapsed. main.cpp's loop checks this every tick and, once true,
//        calls the blocking RunIdleSleep() (sleep_mode.hpp) and then
//        NotifyWokeFromIdleSleep() once it returns. Reuses
//        paused_elapsed_ms_ directly rather than a separate timer field —
//        it already tracks exactly "ms continuously paused, reset on any
//        interaction", which is exactly what this needs too.
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
// 7. Outer ring (2026-08-25, "UI polish" pass; redesigned 2026-09-09 into
//    a progress ring): a second notification channel alongside
//    brightness — GuiManager's ring_/ring_tick_ (see their header
//    comment). Original (2026-08-25/26) meaning — solid while paused,
//    a closing-seconds breathing cue while running near the end,
//    otherwise hidden — replaced entirely: the user wanted the ring to
//    show *actual progress*, not just a late-stage cue.
//
//    New meaning: the ring's visible arc directly tracks how far through
//    the current phase (or, for a count-up face, the current hour) things
//    are, clockwise from 12 o'clock:
//      - Countdown (TimerFace::Status::has_target — PomodoroFace,
//        BreathFace's every phase including kReady): starts as a full
//        circle and *erodes* — the eaten portion (gone, starting at 12
//        o'clock) grows clockwise as remaining_ms falls, reaching fully
//        empty exactly at remaining_ms=0. elapsed_fraction =
//        1 - remaining_ms/target_ms (target_ms is new on Status, 2026-09-09
//        — the ring needs the phase's *original* duration, which
//        remaining_ms alone can't supply).
//      - Count-up (Status::is_count_up — StopwatchFace only): the
//        opposite shape, *grows* from nothing at 12 o'clock, one full
//        revolution per hour, wrapping back to empty and starting over —
//        elapsed_fraction = (elapsed_ms mod 1 hour) / 1 hour. remaining_ms
//        is repurposed on Status to carry elapsed_ms for this case (see
//        its own field comment) since it's otherwise unused/meaningless
//        for a face with no target.
//      - Neither flag set (BreathFace's true kIdle screen; no current_ at
//        all) — GuiManager::SetRingVisible(false), no progress to show.
//
//    Both shapes share the exact same "moving edge" angle
//    (elapsed_fraction*360 degrees clockwise from 12), which is also
//    where the small tick mark (ring_tick_) sits — GuiManager's
//    SetRingProgress() positions both from that one angle each call, see
//    its own comment for the eroding-vs-growing arc-placement difference.
//    The ring itself is a pure function of Status each tick (elapsed_ms/
//    remaining_ms/target_ms), same as before — no state of its own beyond
//    what GuiManager's own dirty-check needs.
//
//    Tick geometry (2026-09-09, revised same day from hardware feedback):
//    reaches inward from the ring toward the center (kRingTickWidthPx,
//    roughly a fifth of kRingRadiusPx), not outward past the ring's outer
//    edge — first version poked outward, user wanted the opposite
//    direction.
//
//    Per-face orientation (2026-09-09, same feedback round): the ring's
//    own "12 o'clock" now re-snaps to match whichever face is showing —
//    GuiManager::SetRingOrientation(FaceCenterDeg(current_face_)), called
//    once at every face-switch commit (see Update() below), not every
//    tick. First version left the ring's rotation fixed at face B's
//    orientation always, so on any other face its 12 o'clock pointed
//    somewhere that wasn't actually "up" for that face's own upright
//    content — e.g. on face A (FaceCenterDeg=-90, reached by rotating the
//    device 90 degrees/CW from B as of 2026-09-11's CCW-positive
//    screen_angle_deg convention — was CCW pre-flip, same -90 number both
//    times; see FaceCenterDeg's own comment in app_controller.cpp for why
//    the number stayed pinned to content instead of following the sign
//    flip), the fixed reference physically landed at what would be B's
//    own 9 o'clock. Re-snapping per face,
//    instead of continuously tracking screen_angle_deg the way root_
//    does, keeps this off the transform/matrix crash path (see
//    GuiManager's class doc) while still reading correctly once a flip
//    settles.
//
//    Running vs paused — the ring freezes at whatever erosion/growth
//    state it was at, it does NOT restore/reset on pause (that was an
//    explicit requirement: pausing right at the very start of a countdown
//    must not look identical to a fresh, never-started one). The *tick*
//    is what signals running vs paused instead, and goes through three
//    states rather than two (2026-09-09, final form after two rounds of
//    hardware feedback):
//      - Fresh, never (yet) started — elapsed_fraction==0, whether that's
//        a just-entered face or a phase that just auto-advanced (e.g.
//        focus->break): tick hidden entirely (opacity 0). The ring itself
//        (a full or empty circle, unambiguous on its own depending on
//        mode) is already a clear enough "this is fresh" signal without
//        the tick doing anything on top of it.
//      - Running: tick visible, steady/full opacity, riding the moving
//        edge.
//      - Paused with real progress already made (elapsed_fraction > 0):
//        tick blinks, hard on/off (kRingBlinkHalfPeriodMs — see its own
//        comment for why this ended up a flat blink rather than a smooth
//        fade, after two earlier attempts at the latter).
//    First version (screen-off / vanish while paused, no blink at all)
//    read as not obvious enough on hardware — a blink is a much stronger
//    cue than presence-vs-absence, the classic VCR/DVD "steady while
//    playing, blinking while paused" convention.
//
//    Can't reuse remaining_ms to drive the blink's phase the way the
//    ring's old breathing effect drove its wave off
//    `remaining_ms % kRingBreathPeriodMs` — remaining_ms is frozen while
//    paused, which is the whole point here. ring_pause_blink_elapsed_ms_
//    is a real wall-clock accumulator instead (advances by dt_ms only
//    while genuinely blinking, reset to 0 in both other states — running,
//    or fresh-and-unstarted — so a later real pause always starts its
//    blink "on" rather than resuming wherever an earlier pause happened
//    to leave off).
//
//    BreathFace's render() used to override the ring itself right after
//    UpdateRing() ran (a bespoke rise/hold/fall opacity envelope, "the
//    ring IS the exercise") — removed 2026-09-09: the new generic
//    countdown ring already does the same job (every breath phase has a
//    real target_ms), just running faster since these phases are seconds
//    long, not minutes. One mechanism for every has_target face now,
//    instead of PomodoroFace/generic-countdown using one and BreathFace
//    quietly overriding it with another.
//
// 8. Phase-transition caption (2026-08-26): entirely inside PomodoroFace,
//    not AppController — see pomodoro_face.hpp's transition_remaining_ms_
//    field comment. Mentioned here only because design note 7 above
//    references it: the two features share a screen but were built to be
//    fully orthogonal (the caption doesn't know the ring exists, and vice
//    versa), which is what let each one get simplified/fixed
//    independently without touching the other.
//
// 9. Idle sleep (2026-09-06, M9): the ~18s paused/idle sequence described
//    in design note 5 ends with ShouldEnterIdleSleep() going true, at
//    which point main.cpp switches the IMU into Wake-on-Motion mode
//    (Qmi8658::EnterWakeOnMotion() — see its comment for why WoM, not the
//    tap engine: a real tap's INT2 signal is a brief pulse, too short for
//    light sleep's GPIO wakeup to reliably catch on real hardware, where
//    WoM's held-level signal isn't) and calls the blocking
//    RunIdleSleep(imu) (sleep_mode.hpp) — repeated real light sleeps,
//    each one either woken directly by a motion event on IMU_INT2 or a
//    ~1s backstop timer — followed by restoring the tap engine and
//    NotifyWokeFromIdleSleep() once it returns. Deliberately NOT deep
//    sleep: light sleep resumes execution right where RunIdleSleep() left
//    off rather than rebooting, so current_'s face/phase/remaining time
//    and AttitudeEstimator's angle are simply still there when we come
//    back — no state to save or restore. See
//    gravity_timer_project_plan.md's M9 notes for the fuller comparison
//    against *deep* sleep + Wake-on-Motion (blocked there specifically:
//    IMU_INT1/INT2 aren't wired to an RTC-capable GPIO, which deep
//    sleep's ext0/ext1 wakeup requires but light sleep's GPIO wakeup
//    doesn't — that's what makes WoM usable here at all) and deep sleep +
//    ULP-RISC-V bit-bang I2C (works, but far more implementation/
//    debugging cost for savings that are hard to feel against this
//    path's already-huge improvement).
//
//    Motion-only wake (2026-09-06, was tap-only, was tap-or-rotation
//    before that): trade-off accepted knowingly — WoM wakes on any
//    sufficiently large accelerometer slope, not specifically a tap
//    (being picked up, the desk being knocked, etc. all wake it too).
//    Whatever event caused RunIdleSleep() to return is consumed by that
//    function itself — it's never forwarded to OnTap() — so the device
//    always comes back paused, never straight into running. Reaching in
//    and toggling running_ requires a distinct, subsequent tap once back
//    in the normal loop. Deliberate: resuming a timer just because the
//    device woke up would mean an incidental bump could silently start a
//    session — and matters more now than it did for tap-only wake, since
//    a wider set of events can trigger this wake at all.
//
//    Double-tap-to-wake (2026-09-07): the trade-off above turned out to
//    bite harder than expected on real hardware — even with
//    Qmi8658::EnterWakeOnMotion()'s threshold maxed out at the register's
//    255 ceiling, an incidental hand bump near the device was still
//    enough to trigger a wake. Fixed one level below this class, entirely
//    inside main.cpp's idle-sleep block: a WoM trigger no longer wakes on
//    its own. It's a silent pre-wake — screen stays off, ShouldEnterIdleSleep()
//    stays satisfied — until a second, genuinely separate tap lands within
//    a short window (kWomConfirmWindowMs) right after. Only that combination
//    calls NotifyWokeFromIdleSleep(). This is now a deliberate two-tap
//    wake gesture, not just a debounce — a single tap can't satisfy both
//    stages back-to-back on this hardware (the tap engine isn't running
//    yet at the instant of the physical tap; still in WoM mode until
//    RunIdleSleep() returns), so waking the device for real always takes
//    two distinct taps: one to leave WoM/light-sleep, one to confirm.
//    Combined with the "distinct subsequent tap to resume" rule just
//    above, running a paused timer from a full idle sleep now takes
//    three taps total: two to wake the screen, one more to actually
//    start it — all in service of the same goal, an incidental bump
//    should never be mistaken for intent.
//
// 10. Battery-check gesture (2026-09-08): picking the device up and
//     holding it (tilting it out of the tracked rotation plane —
//     AttitudeEstimator::Output::in_valid_plane going false, now
//     hysteresis-debounced at the source, see that class's
//     kAzInvalidEnterThresholdG/kAzValidReturnThresholdG) for
//     kBatteryViewEnterMs shows a 5-block battery-level gauge instead of
//     the current face; holding the device back in-plane for
//     kBatteryViewExitMs returns to normal. Deliberately NOT a Face:
//     showing_battery_ doesn't touch current_/current_face_ at all — the
//     underlying face's onTick() keeps running the whole time (a
//     Pomodoro phase keeps counting down while you check the battery),
//     only render() is swapped out (GuiManager::ShowBatteryView()) and
//     face-switch/tap logic is suppressed for the duration, same "nothing
//     to save/restore" shape as idle sleep (design note 9) — there's just
//     nothing state-worthy about "the screen currently shows a different
//     thing".
//
//     Why hysteresis had to move into AttitudeEstimator rather than
//     living here like the other debouncing in this file
//     (kFaceHysteresisLeaveDeg, kTapMuteAfterSwitchMs): those work on a
//     single already-hysteresis'd or edge-triggered signal, but
//     in_valid_plane wasn't hysteresis'd at all before this — a plain
//     single-threshold boolean bouncing across its boundary would keep
//     resetting battery_view_hold_ms_'s accumulation to 0, since this
//     class only sees the boolean, not the underlying |AZ| value needed
//     to build a band on top of it. Fixing it at the source also quietly
//     improves AttitudeEstimator's own internal accel-trust fallback
//     (Update()'s in_valid_plane branch), which had the identical
//     boundary-chatter exposure already, just never mattered enough to
//     notice before this.
//
//     Entry threshold intentionally not a small resting tilt (0.7g) —
//     this is meant to require a real "pick it up and hold it at an
//     angle" motion, not fire from a light nudge. Exit threshold (0.3g)
//     matches the original pre-hysteresis single value, chosen to make
//     returning to normal comparatively easy once you set the device back
//     down.
//
//     Known gap, not hidden: while showing_battery_, idle-sleep's own
//     countdown (design note 9) is untouched — it only ever advances
//     while the underlying face reports !is_running anyway, so checking
//     the battery while a timer is actively running never idle-sleeps
//     regardless. The plan-doc's low-battery-safety-net design calls for
//     "stays lit while charging, sleeps anyway if not" — not implemented
//     yet, since that needs a real voltage-trend read over time (no STAT
//     pin wired to a GPIO on this board) that doesn't exist until the ADC
//     side of this feature is built; revisit once it is. Battery level
//     itself is a stub for the same reason — see kStubBatteryFilledBlocks
//     in app_controller.cpp.
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

    // True once the paused/idle timeout has faded the screen fully off
    // and held it there — main.cpp's loop checks this every tick and, if
    // true, calls the blocking RunIdleSleep() and then
    // NotifyWokeFromIdleSleep() once it returns. See design note 9.
    bool ShouldEnterIdleSleep() const;

    // Call once after RunIdleSleep() returns: resets the idle countdown
    // and snaps brightness back up immediately, same as any other
    // interaction-while-paused event (design note 5) — without this the
    // screen would stay black until the next tick's fade math happened
    // to catch up.
    void NotifyWokeFromIdleSleep();

private:
    Face QuantizeFace(float screen_angle_deg) const;  // stateful — reads current_face_, see design note 1 above
    std::unique_ptr<TimerFace> CreateFace(Face face);  // the "Factory"
    void UpdateBrightness(uint32_t dt_ms, bool is_moving);  // see design note 5 above
    void UpdateRing(uint32_t dt_ms);  // see design note 7 above — dt_ms drives the paused tick's blink timer
    void UpdateBatteryView(const AttitudeEstimator::Output& attitude, uint32_t dt_ms);  // see design note 10 above

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

    uint32_t ring_pause_blink_elapsed_ms_ = 0;  // see design note 7 — real wall-clock ms, not tied to remaining_ms

    bool showing_battery_ = false;         // see design note 10
    uint32_t battery_view_hold_ms_ = 0;    // ms continuously in the state opposite showing_battery_'s current value
};
