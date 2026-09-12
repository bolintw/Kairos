#pragma once

#include <memory>

#include "attitude_estimator.hpp"
#include "gui_manager.hpp"
#include "timer_face.hpp"

// Owns face-switching (hysteresis + Factory), brightness/ring
// notifications, idle sleep, and the battery-check/low-battery screens.
//
// Design:
//
// 1. Quantizes screen_angle_deg into A/B/C/D, centered on -90/0/90/180.
//    A=PomodoroFace(25,5), B=PomodoroFace(50,10), C=StopwatchFace,
//    D=BreathFace (see CreateFace). QuantizeFace() is stateful hysteresis:
//    it only leaves current_face_ once the angle is more than
//    kFaceHysteresisLeaveDeg past current_face_'s own center, and only
//    returns once within (90-kFaceHysteresisLeaveDeg) of the target's
//    center — avoids flip-flopping near a boundary from resting noise.
//
// 2. A face commit (onExit/onEnter) fires the instant the quantized face
//    disagrees with current_face_, moving or not — QuantizeFace's own
//    hysteresis is enough proof real movement happened. A fast swipe
//    that passes through a face's zone on the way to another commits
//    (and resets) that passed-through face too.
//
// 3. Taps are forwarded unconditionally, not gated on is_moving — a tap
//    landing mid-flip just toggles the pre-flip face and gets
//    overwritten by onEnter()'s reset if a genuine flip is confirmed.
//
// 4. `current_` is not expected to be null in normal operation.
//
// 5. Brightness/notification state machine, driven off TimerFace::Status
//    each tick plus attitude.is_moving — an `interacting` flag folds
//    tapping, flipping, and idle rotation together:
//      - face entry: snap to full bright
//      - interacting while running: snap to full bright, restart the
//        ~10s fade toward the dimmed level
//      - break phase: stays fully bright for the whole phase, no fade
//      - last ~30s of a focus-like phase: ramps UP to full bright within
//        ~3s and holds until the phase changes, not interrupted by
//        movement
//      - interacting while paused: snap to full bright, restart the
//        idle-sleep countdown
//      - paused with no interaction for kIdlePreDimHoldMs: fast-fade to
//        off over kIdleFadeToOffMs, then hold off for kIdleOffHoldMs —
//        ShouldEnterIdleSleep() goes true once the whole ~18s sequence
//        elapses. main.cpp checks this every tick and, once true, calls
//        the blocking RunIdleSleep() then NotifyWokeFromIdleSleep().
//    TimerFace/AttitudeEstimator have no notion of brightness.
//
// 6. Tap mute window: OnTap() is ignored for kTapMuteAfterSwitchMs after
//    a face switch commits — a flip often lands with residual wobble
//    that would otherwise immediately start the timer on the face just
//    arrived at. Armed only on a face-switch commit (not on any
//    is_moving blip — a light tap alone can cross that threshold too),
//    and only counts down while !attitude.is_moving. Orthogonal to the
//    tap engine's own detection windows in main.cpp's ConfigureTap call.
//
// 7. Outer progress ring, a second notification channel alongside
//    brightness. Visible arc tracks progress clockwise from 12 o'clock:
//      - Countdown (Status::has_target): starts full and erodes —
//        elapsed_fraction = 1 - remaining_ms/target_ms.
//      - Count-up (Status::is_count_up): grows from empty, one
//        revolution per hour — elapsed_fraction = (elapsed_ms mod 1h)/1h.
//      - Neither flag set: ring hidden.
//    The tick mark rides the same moving-edge angle. Ring freezes (does
//    not reset) on pause — the tick signals running/paused instead, with
//    three states: fresh/unstarted (hidden), running (steady), paused
//    with real progress (hard blink, kRingBlinkHalfPeriodMs). Orientation
//    re-snaps to the current face (GuiManager::SetRingOrientation()) once
//    per face-switch commit, not every tick.
//
// 8. PomodoroFace's phase-transition caption lives entirely in
//    pomodoro_face.hpp — orthogonal to the ring above; neither knows
//    about the other.
//
// 9. Idle sleep: once ShouldEnterIdleSleep() goes true, main.cpp switches
//    the IMU into Wake-on-Motion mode and calls the blocking
//    RunIdleSleep(imu) (repeated real light sleeps), then restores the
//    tap engine and calls NotifyWokeFromIdleSleep(). Light sleep, not
//    deep sleep, so current_'s state and the fused angle are simply
//    still there on return.
//
//    WoM wakes on any sufficiently large accelerometer slope, not
//    specifically a tap, so whatever woke RunIdleSleep() is consumed
//    internally and never forwarded to OnTap() — the device always comes
//    back paused, requiring a distinct subsequent tap to resume.
//
//    Double-tap-to-wake: a WoM trigger alone doesn't wake the device —
//    it's a silent pre-wake (screen stays off) until a second, genuinely
//    separate tap lands within kWomConfirmWindowMs. Only that combination
//    calls NotifyWokeFromIdleSleep(). Resuming a paused timer from full
//    idle sleep is three taps total: two to wake the screen, one more to
//    start it.
//
// 10. Battery-check gesture: holding the device tilted out of the
//     tracked plane (AttitudeEstimator::Output::in_valid_plane false,
//     hysteresis-debounced at the source) for kBatteryViewEnterMs shows a
//     5-block gauge instead of the current face; holding it back in-plane
//     for kBatteryViewExitMs returns to normal. Not a Face:
//     showing_battery_ doesn't touch current_/current_face_ — the
//     underlying face keeps ticking, only render() is swapped
//     (GuiManager::ShowBatteryView()) and face-switch/tap logic is
//     suppressed. BatteryVoltageToFilledBlocks() (app_controller.cpp)
//     does a plain linear split of the LiPo's 3.0-4.2V usable range
//     across the 5 blocks — not lab-accurate, good enough for an
//     at-a-glance gauge. Runs its own idle-sleep timeline
//     (battery_view_idle_elapsed_ms_) so leaving the device tilted
//     doesn't keep it lit indefinitely; unlike the low-battery screen
//     below, the underlying face is left running, not force-paused.
//
// 11. Low-battery warning screen — stage 1 of the two-stage low-battery
//     safety net (stage 2, forced deep sleep, lives entirely in main.cpp
//     since it's a reboot-based mechanism with nothing for AppController
//     to own). Voltage-driven: evaluated every tick against
//     battery_voltage_v_ with its own hysteresis pair
//     (kLowBatteryEnterV/kLowBatteryExitV). showing_low_battery_ forces a
//     "please charge" screen over whatever was showing, and is NOT exempt
//     from idle-sleep the way showing_battery_ is — staying lit
//     indefinitely works against the point of the screen. Runs the same
//     bright-hold/fade/dim-hold/cut-to-off timeline as design note 5
//     against its own low_battery_idle_elapsed_ms_ counter.
//     UpdateLowBatteryHysteresis() force-pauses a running timer on entry
//     (synthetic onTap()) and OnTap() swallows real taps while the screen
//     is showing.
//
//     Countdown lock below kCriticalBatteryEnterV: once voltage is this
//     low, neither motion nor a tap resets low_battery_idle_elapsed_ms_
//     anymore — it counts straight down regardless of interaction, so
//     the device can't be kept awake indefinitely below the voltage
//     where continued operation risks real LiPo damage. Above
//     kCriticalBatteryEnterV but still below kLowBatteryEnterV,
//     interaction resets the timer as usual.
//
// AttitudeEstimator is NOT held by reference here — main.cpp calls
// AttitudeEstimator::Update() once per tick (single call site, avoids
// double-integrating the gyro angle) and passes the resulting Output in.
class AppController {
public:
    enum class Face { kA, kB, kC, kD };

    // Stage 2 of the low-battery safety net's entry threshold — the
    // actual deep-sleep trigger lives in main.cpp, which reads this same
    // constant so the two can't drift apart.
    static constexpr float kCriticalBatteryEnterV = 3.0f;

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

    // Caches the latest real battery voltage (main.cpp reads
    // BatteryMonitor on its own slow cadence, not every sensor tick) for
    // the battery gauge / low-battery screen to use.
    void SetBatteryVoltage(float voltage_v) { battery_voltage_v_ = voltage_v; }

private:
    Face QuantizeFace(float screen_angle_deg) const;  // stateful — reads current_face_, see design note 1 above
    std::unique_ptr<TimerFace> CreateFace(Face face);  // the "Factory"
    void UpdateBrightness(uint32_t dt_ms, bool is_moving);  // see design note 5 above
    void UpdateRing(uint32_t dt_ms);  // see design note 7 above — dt_ms drives the paused tick's blink timer
    void UpdateBatteryView(const AttitudeEstimator::Output& attitude, uint32_t dt_ms);  // see design note 10 above
    void UpdateLowBatteryHysteresis();  // see design note 11 above — just flips showing_low_battery_, no rendering

    GuiManager& gui_manager_;
    std::unique_ptr<TimerFace> current_;
    Face current_face_ = Face::kA;         // confirmed face; meaningless until current_ is set

    float brightness_ = 1.0f;
    uint32_t bright_phase_elapsed_ms_ = 0;  // ms since the last "became bright" event, drives the fade
    uint32_t paused_elapsed_ms_ = 0;        // ms continuously paused, drives the long idle timeout
    bool prev_is_running_ = false;
    bool prev_has_target_ = false;
    uint32_t prev_remaining_ms_ = 0;

    uint32_t tap_mute_remaining_ms_ = 0;  // see design note 6

    uint32_t ring_pause_blink_elapsed_ms_ = 0;  // real wall-clock ms, not tied to remaining_ms

    bool showing_battery_ = false;         // see design note 10
    uint32_t battery_view_hold_ms_ = 0;    // ms continuously in the state opposite showing_battery_'s current value
    uint32_t battery_view_idle_elapsed_ms_ = 0;  // ms continuously shown, drives the same fade/sleep curve as paused_elapsed_ms_
    float battery_voltage_v_ = 4.2f;  // defaults to "full", not 0 — avoids an alarming false-empty reading before the first real ADC read

    bool showing_low_battery_ = false;         // see design note 11
    uint32_t low_battery_idle_elapsed_ms_ = 0;  // ms continuously shown, drives the same fade/sleep curve as paused_elapsed_ms_
};
