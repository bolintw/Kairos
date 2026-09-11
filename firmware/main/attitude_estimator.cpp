#include "attitude_estimator.hpp"

#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.0f / kPi;

// Tuning starting points, not yet validated against real usage — see the
// header's OPEN items this resolves. Adjust while watching the debug
// overlay and re-flashing.
constexpr float kGyroMovingThresholdDps = 20.0f;
// Hysteresis band (2026-09-08, for the battery-check gesture — see
// app_controller.hpp's design note on showing_battery_): a single 0.3g
// threshold flip-flopped right at the boundary, which matters more now
// than it used to since AppController debounces entry/exit with a
// multi-second hold timer — any single-tick bounce back across a plain
// threshold resets that timer's accumulation to 0. Split into two:
// kAzInvalidEnterThresholdG (must clear this to leave the valid plane)
// well above kAzValidReturnThresholdG (must drop back under this to
// return) — 0.7g/0.3g chosen deliberately far apart (this is meant to be
// a real "pick the device up and hold it at an angle" gesture, not a
// small resting tilt) rather than a narrow band just wide enough to stop
// chatter. Also used by Update()'s own accel-trust fallback below, not
// just the exposed Output::in_valid_plane — same signal, one mechanism,
// benefits both call sites (see AttitudeEstimator::in_valid_plane_'s
// field comment).
// 0.7/0.3 -> 0.9/0.5 (2026-09-08, first-hardware-pass feedback) — same
// 0.4g band width, shifted up: entry needs an even more deliberate hold-up
// motion, exit still noticeably easier than entry but no longer as easy
// as the original single 0.3g threshold was.
constexpr float kAzInvalidEnterThresholdG = 0.9f;
constexpr float kAzValidReturnThresholdG = 0.5f;
// History: 0.98 -> 0.5 -> 0.8 -> 0.95 -> 0.9 (2026-08-24, final for now).
// 0.98 took ~17s to converge (matched the earlier "low tens of seconds"
// hardware report). 0.5
// converged fast (~0.5s) but felt too accel-dominant/vibration-sensitive.
// 0.8 was a middle ground (~1.5s). Once the gyro full-scale-range bug was
// fixed (CTRL3 scale now correctly matches +-256dps, see qmi8658.hpp),
// the overshoot that motivated trusting accel more went away, and 0.95
// felt close to the user's prior flight-controller tuning experience —
// but tuned down slightly to 0.9 on further hardware testing, still
// gyro-dominant with a touch more accel correction than 0.95.
// 0.9 -> 0.8 (2026-09-01, alongside the adaptive weighting just below):
// this value now only governs the at-rest end of the ramp (see
// AccelTrustWeight) rather than a flat per-tick weight during motion too,
// and kAccelLowPassAlpha's heavy smoothing (~37ms tau) means the
// stationary accel reading it's blending toward is much steadier than
// when this was last tuned — safe to trust it more once genuinely still,
// which shortens the final settle-in after a flip. Confirmed on hardware.
//
// Converted from a flat alpha to kComplementaryTauMs below (2026-09-06) —
// see that constant's comment for why.

// Adaptive complementary filter (2026-09-01): (1 - kComplementaryAlpha)
// above is no longer applied as a flat per-tick accel weight — it's now
// the weight used only once the device is essentially stationary, scaled
// down toward 0 as rotation speeds up. Reasoning: the accel weight isn't
// really "how much do we trust accel" in the abstract, it's "how close is
// filtered_accel_g_ to the device's actual current orientation right
// now" — and kAccelLowPassAlpha's heavy smoothing (~37ms tau, see its own
// comment) means that gap grows with rotation speed, not just existing
// at some fixed size. Blending a fixed 10%/tick toward a value that lags
// further behind during a fast flip pulls the (accurate, fast) gyro
// track backward the whole time the device is moving — read on hardware
// as a consistent ~5 degree undershoot on quick 90-degree flips, then a
// visible "damped" ease-in to the correct angle over the following
// ~100-150ms (roughly kAccelLowPassAlpha's settling time) once gyro rate
// drops back to ~0 and the blend keeps nudging toward accel's now-caught-
// up value. Trusting accel less while |gz| is high sidesteps pulling
// toward a value known to be stale, without touching
// kGyroLowPassAlpha/kAccelLowPassAlpha themselves (different concern —
// those smooth a single tick's raw sample, this decides whether THIS
// tick's accel-derived angle should factor into the output at all).
// kGyroSpeedFullTrustDps/kGyroSpeedZeroTrustDps are the new tuning knobs;
// initial guesses, not yet validated on hardware like the constants
// above were — watch the debug overlay's gz reading during a normal flip
// to sanity-check where they should sit.
constexpr float kGyroSpeedFullTrustDps = 5.0f;   // below this, treat as
                                                   // stationary: full
                                                   // (1-kComplementaryAlpha)
                                                   // accel weight, same as
                                                   // the old fixed behavior
constexpr float kGyroSpeedZeroTrustDps = 60.0f;  // at/above this, accel
                                                   // weight -> 0: trust
                                                   // gyro alone while
                                                   // genuinely rotating

// Single-pole low-pass on the raw samples — see the header's field
// comment. Was one shared kLowPassAlpha for both accel and gyro through
// 0.8 -> 0.2 -> 0.4 -> 0.6 (0.8/0.2/0.4 on 2026-08-25, 0.6 on 2026-08-26
// after enclosure testing: 0.8 was backwards from the intended smoothing
// strength ~5ms tau, barely any effect; 0.2 ~37ms tau visibly steadied
// resting jitter but made active rotation noticeably laggier; 0.4 ~16ms
// tau traded back some smoothing for responsiveness but still read as
// sluggish mounted in the enclosure; 0.6 ~9ms tau continued that
// direction). Split into two separate constants 2026-08-31: with a
// single shared value, pushing gyro's alpha up for responsiveness forced
// accel's noise immunity down right along with it, even though accel
// only ever contributes kComplementaryAlpha's 10%-per-tick correction —
// it was never the signal driving felt responsiveness, so there was no
// reason its filtering had to track gyro's. That coupling is very
// likely why hand-held vibration started reading as "a bad angle that
// takes a while to recover from" around the same time — accel's own
// filtering had gotten weaker as a side effect of chasing gyro
// responsiveness, not because accel itself needed to respond faster.
//
// kGyroLowPassAlpha: 0.6 -> 0.75 (2026-08-31, ~9ms -> ~6ms tau at
// 120Hz) — still felt a bit laggy at 0.6 once separated from accel;
// this is deliberately near the low-filtering end of what's been tried,
// since gyro drives essentially all of the felt rotation responsiveness
// (see kComplementaryAlpha's 90/10 split above).
//
// Time-based, not a flat per-tick alpha (2026-09-06): all three of
// kGyroLowPassAlpha/kAccelLowPassAlpha/kComplementaryAlpha above were
// fixed fractions applied once per Update() *call*, not per unit time —
// fine as long as the caller ticks at a roughly constant rate close to
// what they were tuned at (~8.33ms/120Hz), which a main-loop timing
// investigation (see gravity_timer_project_plan.md's M9 section) found is
// NOT reliably true on real hardware — the sensor loop measured ~13-15Hz
// there, not 120Hz. At 1/10th the tuned tick rate, a flat per-call alpha
// makes the *real-time* smoothing/blending 10x slower than it was tuned
// to feel like, which is a real suspect behind the "reversal, then slow
// creep back" feel investigated around the same time — not proven to be
// the whole story, but decoupling filter behavior from tick rate is
// correct regardless of how that turns out, and doesn't require the loop
// speed problem to be fixed first to pay off.
//
// tau (ms) is the actual time-invariant quantity a first-order low-pass
// is tuned by; alpha at a given dt is derived from it via ExpAlpha()
// below (alpha = 1 - exp(-dt/tau)), not looked up as a fixed constant.
// The tau values here are exactly what the *old* alphas already implied
// at the 120Hz rate they were tuned at (tau = -dt_ref/ln(1-alpha),
// dt_ref=1000/120ms) — confirmed against this file's own pre-existing
// "~6ms"/"~37ms" comments above rather than computed fresh, so switching
// to this scheme reproduces the exact same feel at 120Hz and only changes
// behavior when the real tick rate drifts from that.
constexpr float kGyroLowPassTauMs = 6.0f;
// kAccelLowPassAlpha: 0.6 -> 0.2 (2026-08-31, back to the value that
// felt too laggy in the old *shared* scheme — not laggy here, since
// accel was never the fast-response signal to begin with). ~37ms tau:
// heavier smoothing specifically to reject the vibration/shock content
// blamed above, at essentially no cost to rotation feel.
constexpr float kAccelLowPassTauMs = 37.35f;
// Same underlying number as kAccelLowPassTauMs (both alphas were 0.2 at
// the 120Hz tuning point — see kComplementaryAlpha's own history above),
// kept as a separate named constant since it governs a conceptually
// different thing (the complementary blend's at-rest weight, not a raw-
// sample smoothing filter) and could end up tuned independently later.
constexpr float kComplementaryTauMs = 37.35f;

float ExpAlpha(float dt_ms, float tau_ms)
{
    return 1.0f - std::exp(-dt_ms / tau_ms);
}

float LowPass(float new_x, float old_x, float alpha)
{
    return alpha * new_x + (1.0f - alpha) * old_x;
}

float WrapDeg180(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

// Nudges `base` toward `target` by `target_weight` of the shortest angular
// distance between them. A plain linear blend of two raw angles breaks
// near the +-180 wraparound (e.g. averaging +179 and -179 gives ~0,
// instead of ~180); going through the wrapped difference avoids that.
float BlendTowardAngle(float base, float target, float target_weight)
{
    const float diff = WrapDeg180(target - base);
    return WrapDeg180(base + target_weight * diff);
}

float GyroMagnitudeDps(float gx, float gy, float gz)
{
    return std::sqrt(gx * gx + gy * gy + gz * gz);
}

// See kGyroSpeedFullTrustDps/kGyroSpeedZeroTrustDps's comment for why this
// exists. Linear ramp from full trust (kComplementaryAlpha's usual
// 1-kComplementaryAlpha weight) at/below kGyroSpeedFullTrustDps, down to
// zero trust at/above kGyroSpeedZeroTrustDps. gz_abs_dps is the in-plane
// rotation axis specifically (not the 3-axis GyroMagnitudeDps above,
// which mixes in off-axis GX/GY meant for a different question — "is the
// device being picked up" — this is about how fast the tracked angle
// itself is currently changing).
// dt_ms added (2026-09-06) — see kGyroLowPassTauMs's comment. The old
// "(1.0f - kComplementaryAlpha)" full-trust ceiling is now
// ExpAlpha(dt_ms, kComplementaryTauMs), the time-correct version of the
// same number.
float AccelTrustWeight(float gz_abs_dps, float dt_ms)
{
    float trust = 1.0f - (gz_abs_dps - kGyroSpeedFullTrustDps) /
                              (kGyroSpeedZeroTrustDps - kGyroSpeedFullTrustDps);
    if (trust < 0.0f) trust = 0.0f;
    if (trust > 1.0f) trust = 1.0f;
    return ExpAlpha(dt_ms, kComplementaryTauMs) * trust;
}

// Shared by Update() and SeedInitialAngle() — same formula, see the
// header's sign-convention comment. Takes ax/ay directly (not a Sample)
// so Update() can pass its low-pass-filtered values.
//
// atan2(ax, -ay), offset ADDED (2026-09-11, was atan2(-ax,-ay) with the
// offset SUBTRACTED — see the header's sign-convention history): this
// exact pairing — negate only the ax argument, and add instead of
// subtract — is deliberate, not an arbitrary equivalent rewrite. It makes
// the new formula produce exactly the negation of the old one for every
// input (raw_new(ax,ay) = -raw_old(ax,ay), a fixed algebraic identity),
// which means any face_a_offset_deg already calibrated and stored in NVS
// under the *old* convention still nulls out correctly here — no
// recalibration needed by this sign flip alone. (calibration_mode.cpp's
// own RawAccelAngleDeg helper, which computes what gets stored as
// face_a_offset_deg in the first place, deliberately still uses the old
// atan2(-ax,-ay) form for exactly this reason — see its comment.)
float AngleFromAccel(float ax, float ay, float face_a_offset_deg)
{
    const float raw_accel_angle = std::atan2(ax, -ay) * kRadToDeg;
    return WrapDeg180(raw_accel_angle + face_a_offset_deg);
}

}  // namespace

AttitudeEstimator::AttitudeEstimator(float face_a_offset_deg, float accel_bias_x_g, float accel_bias_y_g)
    : face_a_offset_deg_(face_a_offset_deg),
      accel_bias_x_g_(accel_bias_x_g),
      accel_bias_y_g_(accel_bias_y_g)
{
}

void AttitudeEstimator::CalibrateGyroZeroOffset(const Sample& stationary_sample)
{
    gyro_bias_dps_[0] = stationary_sample.gyro_dps[0];
    gyro_bias_dps_[1] = stationary_sample.gyro_dps[1];
    gyro_bias_dps_[2] = stationary_sample.gyro_dps[2];
}

void AttitudeEstimator::SeedInitialAngle(const Sample& sample)
{
    // One-shot check at boot, before in_valid_plane_'s hysteresis has any
    // history to run on — uses the (lower/stricter-for-"valid") return
    // threshold directly as a simple "is this sample trustworthy enough
    // to seed from" gate, and also seeds in_valid_plane_ itself so the
    // very first real Update() call's hysteresis starts from a state that
    // actually matches the boot orientation instead of always assuming
    // true.
    const bool in_valid_plane = std::fabs(sample.accel_g[2]) < kAzValidReturnThresholdG;
    in_valid_plane_ = in_valid_plane;
    if (!in_valid_plane) {
        return;  // leave angle_deg_ at its default; Update() converges normally
    }
    angle_deg_ = AngleFromAccel(sample.accel_g[0] - accel_bias_x_g_, sample.accel_g[1] - accel_bias_y_g_,
                                 face_a_offset_deg_);
}

AttitudeEstimator::Output AttitudeEstimator::Update(const Sample& sample, uint32_t dt_ms)
{
    const float dt_ms_f = static_cast<float>(dt_ms);

    // Low-pass the raw sample first — everything below reads the
    // filtered values, never the raw sample directly. See the header's
    // field comment for why (and for kComplementaryTauMs vs
    // kGyroLowPassTauMs/kAccelLowPassTauMs being different things).
    if (has_filtered_sample_) {
        const float accel_alpha = ExpAlpha(dt_ms_f, kAccelLowPassTauMs);
        const float gyro_alpha = ExpAlpha(dt_ms_f, kGyroLowPassTauMs);
        for (int i = 0; i < 3; ++i) {
            filtered_accel_g_[i] = LowPass(sample.accel_g[i], filtered_accel_g_[i], accel_alpha);
            filtered_gyro_dps_[i] = LowPass(sample.gyro_dps[i], filtered_gyro_dps_[i], gyro_alpha);
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            filtered_accel_g_[i] = sample.accel_g[i];
            filtered_gyro_dps_[i] = sample.gyro_dps[i];
        }
        has_filtered_sample_ = true;
    }

    const float gx = filtered_gyro_dps_[0] - gyro_bias_dps_[0];
    const float gy = filtered_gyro_dps_[1] - gyro_bias_dps_[1];
    const float gz = filtered_gyro_dps_[2] - gyro_bias_dps_[2];

    // Sign flipped 2026-08-23 to CW=positive, flipped back 2026-09-11 to
    // CCW=positive (see the header's sign-convention history) — CCW
    // rotation (as seen by the user) now *increases* screen_angle_deg, CW
    // decreases it. The underlying hardware fact is unchanged — CCW still
    // reads as negative GZ — only which direction we call "positive"
    // changed, so both this gyro term and AngleFromAccel's formula flip
    // together.
    const float angle_from_gyro =
        WrapDeg180(angle_deg_ - gz * (static_cast<float>(dt_ms) / 1000.0f));

    // Schmitt-trigger hysteresis on |AZ| — see kAzInvalidEnterThresholdG's
    // comment. in_valid_plane_ only flips when the *current* threshold for
    // its *current* state is crossed; otherwise it holds.
    if (in_valid_plane_) {
        if (std::fabs(filtered_accel_g_[2]) >= kAzInvalidEnterThresholdG) {
            in_valid_plane_ = false;
        }
    } else {
        if (std::fabs(filtered_accel_g_[2]) < kAzValidReturnThresholdG) {
            in_valid_plane_ = true;
        }
    }
    const bool in_valid_plane = in_valid_plane_;

    // Computed unconditionally now (2026-09-06), not just inside the
    // in_valid_plane branch — see Output::debug_accel_only_angle_deg. Cheap
    // (one atan2), and having it even when !in_valid_plane lets the debug
    // log show what accel was reading right up to/through a flip, not just
    // once the blend starts trusting it again.
    const float angle_from_accel = AngleFromAccel(filtered_accel_g_[0] - accel_bias_x_g_,
                                                    filtered_accel_g_[1] - accel_bias_y_g_, face_a_offset_deg_);

    if (in_valid_plane) {
        angle_deg_ = BlendTowardAngle(angle_from_gyro, angle_from_accel, AccelTrustWeight(std::fabs(gz), dt_ms_f));
    } else {
        // Accel isn't trustworthy while the screen isn't facing the
        // user — fall back to pure gyro integration for this tick.
        angle_deg_ = angle_from_gyro;
    }

    Output out;
    out.screen_angle_deg = angle_deg_;
    out.is_moving = GyroMagnitudeDps(gx, gy, gz) > kGyroMovingThresholdDps;
    out.in_valid_plane = in_valid_plane;
    out.debug_gyro_only_angle_deg = angle_from_gyro;
    out.debug_accel_only_angle_deg = angle_from_accel;
    out.debug_gz_dps = gz;
    return out;
}
