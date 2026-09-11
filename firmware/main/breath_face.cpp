#include "breath_face.hpp"

#include <cmath>
#include <cstdio>

#include "gui_manager.hpp"

namespace {
// See pomodoro_face.cpp's kColorFocus/kColorBreak for the same convention
// — one accent color per state, applied to both the label and the ring.
// Here it's one color per breathing phase instead of per face: green
// (inhale) does reuse PomodoroFace's break-green hue family deliberately
// (both read as "restful"), gold (hold) and violet (exhale) are new,
// distinct from every other face's color (2026-08-30 design chat).
const lv_color_t kColorInhale = lv_color_make(60, 200, 120);
const lv_color_t kColorHold = lv_color_make(230, 180, 60);
const lv_color_t kColorExhale = lv_color_make(150, 110, 220);

struct CycleSpec {
    uint32_t inhale_s;
    uint32_t hold_s;
    uint32_t exhale_s;
};

// 2-3-4 / 3-5-6 / 4-7-8 / 4-7-8 (user's design, 2026-08-26 chat) — see
// breath_face.hpp's class comment for why this isn't the "official" flat
// 4-7-8 protocol.
constexpr CycleSpec kCycles[] = {
    {2, 3, 4},
    {3, 5, 6},
    {4, 7, 8},
    {4, 7, 8},
};
constexpr uint32_t kNumCycles = sizeof(kCycles) / sizeof(kCycles[0]);

// See breath_face.hpp's Phase::kReady comment.
constexpr uint32_t kReadySeconds = 3;

// See breath_face.hpp's done_caption_remaining_ms_ comment.
constexpr uint32_t kDoneCaptionMs = 2000;

// Same sine-hump fade PomodoroFace's caption uses (0 at elapsed=0, peaks
// at the midpoint, 0 again at elapsed=total) — used for the done caption.
uint8_t SineFadeOpacity(uint32_t elapsed_ms, uint32_t total_ms)
{
    constexpr float kPi = 3.14159265359f;
    const float t = total_ms > 0 ? static_cast<float>(elapsed_ms) / static_cast<float>(total_ms) : 0.0f;
    return static_cast<uint8_t>(std::sin(kPi * t) * 255.0f);
}
}  // namespace

void BreathFace::onEnter()
{
    phase_ = Phase::kIdle;
    cycle_index_ = 0;
    phase_remaining_ms_ = 0;
    running_ = false;
    done_caption_remaining_ms_ = 0;
}

void BreathFace::onExit()
{
}

void BreathFace::onTick(uint32_t dt_ms)
{
    // The done caption is an announcement, not part of any countdown —
    // see its field comment in breath_face.hpp — so it decrements
    // unconditionally here, ahead of the running_ gate below. This
    // matters because it only ever plays while running_ is already false
    // (right after the last phase ends), so gating it behind that check
    // would mean it never ticks down at all.
    if (done_caption_remaining_ms_ > 0) {
        done_caption_remaining_ms_ = dt_ms < done_caption_remaining_ms_ ? done_caption_remaining_ms_ - dt_ms : 0;
    }

    if (!running_) return;

    // Looped rather than a single if-check, same reasoning as
    // PomodoroFace::onTick — a dt_ms spanning multiple short phases (the
    // 2s/3s ones especially) should walk through all of them, not just
    // one.
    while (running_ && dt_ms >= phase_remaining_ms_) {
        dt_ms -= phase_remaining_ms_;
        AdvancePhase();
    }
    if (running_) {
        phase_remaining_ms_ -= dt_ms;
    }
}

void BreathFace::onTap()
{
    if (phase_ == Phase::kIdle) {
        // Start a fresh session — a 3s "Ready" beat first (see
        // Phase::kReady), then AdvancePhase() carries it into cycle 0's
        // inhale once that runs out.
        cycle_index_ = 0;
        phase_ = Phase::kReady;
        phase_remaining_ms_ = kReadySeconds * 1000;
        running_ = true;
        return;
    }
    // Mid-session: plain pause/resume, same convention as every other
    // face — position (phase_/cycle_index_/phase_remaining_ms_) is left
    // untouched either way.
    running_ = !running_;
}

void BreathFace::AdvancePhase()
{
    const CycleSpec& cycle = kCycles[cycle_index_];
    switch (phase_) {
        case Phase::kReady:
            phase_ = Phase::kInhale;
            phase_remaining_ms_ = cycle.inhale_s * 1000;  // cycle_index_ is 0 here, set by onTap()
            break;
        case Phase::kInhale:
            phase_ = Phase::kHold;
            phase_remaining_ms_ = cycle.hold_s * 1000;
            break;
        case Phase::kHold:
            phase_ = Phase::kExhale;
            phase_remaining_ms_ = cycle.exhale_s * 1000;
            break;
        case Phase::kExhale:
            ++cycle_index_;
            if (cycle_index_ >= kNumCycles) {
                // Session complete — stop on its own rather than looping;
                // see the class comment for why this differs from
                // PomodoroFace's forever-loop.
                phase_ = Phase::kIdle;
                cycle_index_ = 0;
                running_ = false;
                done_caption_remaining_ms_ = kDoneCaptionMs;
            } else {
                phase_ = Phase::kInhale;
                phase_remaining_ms_ = kCycles[cycle_index_].inhale_s * 1000;
            }
            break;
        case Phase::kIdle:
            break;  // onTick only calls AdvancePhase() while running_, never reached from kIdle
    }
}

uint32_t BreathFace::PhaseTotalMs() const
{
    const CycleSpec& cycle = kCycles[cycle_index_];
    switch (phase_) {
        case Phase::kReady:  return kReadySeconds * 1000;
        case Phase::kInhale: return cycle.inhale_s * 1000;
        case Phase::kHold:   return cycle.hold_s * 1000;
        case Phase::kExhale: return cycle.exhale_s * 1000;
        case Phase::kIdle:   return 0;
    }
    return 0;
}

void BreathFace::render(GuiManager& gui)
{
    if (phase_ == Phase::kIdle) {
        // No ring here — GetStatus() reports has_target=false and
        // is_count_up=false while idle (no phase active, and this isn't a
        // count-up face either), which is exactly what tells
        // AppController's ring logic to hide the ring entirely rather
        // than render either progress mode.
        gui.SetAccentColor(kColorInhale);
        gui.SetSecondaryText("");
        if (done_caption_remaining_ms_ > 0) {
            const uint8_t opa = SineFadeOpacity(kDoneCaptionMs - done_caption_remaining_ms_, kDoneCaptionMs);
            gui.SetPrimaryTextOpacity(opa);
            gui.SetPrimaryText("Good Job");
            return;
        }
        gui.SetPrimaryTextOpacity(255);  // undo the done caption's fade, in case it just ended
        gui.SetPrimaryText("Breath");
        return;
    }

    // kReady reuses kColorInhale — it's leading into the first inhale
    // anyway, and a 3s prep beat doesn't need its own color in a palette
    // that's already at three deliberately-distinct hues.
    const lv_color_t color = phase_ == Phase::kReady  ? kColorInhale
                            : phase_ == Phase::kInhale ? kColorInhale
                            : phase_ == Phase::kHold   ? kColorHold
                                                        : kColorExhale;
    gui.SetAccentColor(color);

    // Phase name permanently above the countdown, not a fading caption
    // that replaces it (2026-08-30, first attempt reverted): at this
    // pace — a new phase every 2-8s — swapping word for number was itself
    // the abrupt thing the user flagged, on top of the number jump it was
    // meant to soften. Showing both together removes the swap instead of
    // softening it; no timer of its own needed, GuiManager dirty-checks
    // unchanged text on its own.
    gui.SetSecondaryText(phase_ == Phase::kReady  ? "Ready?"
                        : phase_ == Phase::kInhale ? "Inhale"
                        : phase_ == Phase::kHold   ? "Hold"
                                                    : "Exhale");
    gui.SetPrimaryTextOpacity(255);  // undo the done caption's fade, in case a fresh session started right after one ended

    // Ceiling, not truncating: phase_remaining_ms_ counts down to 0, and
    // rounding up means the display shows e.g. "4" for the entire first
    // second of a 4s phase instead of skipping straight to "3" — matches
    // how a countdown reads intuitively (see the phase start moment,
    // where phase_remaining_ms_ == PhaseTotalMs() exactly and this must
    // still show the full duration, not one less).
    const uint32_t seconds_left = (phase_remaining_ms_ + 999) / 1000;
    // Sized for the theoretical uint32_t %u worst case, not just the real
    // 1-8 range in use — GCC's -Wformat-truncation reasons about the
    // parameter's static type, not its actual runtime values, and warns
    // (as an error, -Werror here) on a buffer sized to just the values
    // this face actually produces.
    char buf[12];
    std::snprintf(buf, sizeof(buf), "%u", static_cast<unsigned int>(seconds_left));
    gui.SetPrimaryText(buf);

    // The ring IS the exercise here, not a secondary cue (unlike every
    // other face's ring usage) — see the class comment. Briefly replaced
    // (2026-09-09) by AppController's generic countdown-erosion ring
    // (every has_target phase gets one now, kReady included), then
    // restored the same day — the user specifically wanted this face's
    // own rise-through-inhale/hold-flat/fall-through-exhale opacity
    // envelope back, not the erosion visual. Runs AFTER AppController's
    // own UpdateRing() in the tick sequence (see main.cpp/app_controller.cpp
    // call order), so this always has the final say while a phase is
    // active — same override pattern PomodoroFace's caption already uses
    // for text opacity. SetRingProgress(0, eroding) forces the arc back to
    // a full circle (UpdateRing() would otherwise have set a partial span
    // moments earlier this same tick); SetRingTickOpacity(0) hides the
    // tick, which has no meaning for this envelope — it only makes sense
    // alongside the erosion visual this face isn't using.
    gui.SetRingVisible(true);
    gui.SetRingProgress(0.0f, /*growing=*/false);
    gui.SetRingTickOpacity(0);

    const uint32_t phase_total_ms = PhaseTotalMs();
    const uint32_t elapsed_ms = phase_total_ms - phase_remaining_ms_;
    const float t = phase_total_ms > 0
                         ? static_cast<float>(elapsed_ms) / static_cast<float>(phase_total_ms)
                         : 1.0f;

    uint8_t opa = 255;
    if (phase_ == Phase::kReady) {
        opa = 0;  // nothing breath-related happening yet — see Phase::kReady's comment
    } else if (phase_ == Phase::kInhale) {
        opa = static_cast<uint8_t>(t * 255.0f);
    } else if (phase_ == Phase::kExhale) {
        opa = static_cast<uint8_t>((1.0f - t) * 255.0f);
    }
    // kHold: stays at the default 255 above — steady, nothing rising or falling.
    gui.SetRingOpacity(opa);
}

TimerFace::Status BreathFace::GetStatus() const
{
    // is_break_phase reused as "stay fully bright" (see its own comment
    // in timer_face.hpp) for the whole time a session is running — the
    // user needs the display legible through a still 7s hold, when
    // there's no tap or movement to otherwise count as "interacting" and
    // keep AppController's normal fade-while-idle logic from dimming it
    // mid-exercise. Deliberately still tied to running_, not phase_ —
    // unlike has_target just below, this one's whole point is "actively
    // mid-exercise right now."
    //
    // has_target: was `= running_` (2026-09-09, changed) — under the old
    // ring policy that only mattered while running anyway (a paused ring
    // just went solid regardless), but the new countdown ring needs to
    // keep showing the *frozen* erosion state while paused mid-phase, not
    // report has_target=false and have AppController conclude there's
    // nothing to show. `phase_ != kIdle` is true for the whole time a
    // phase is selected, running or not; only true idle (no session, or
    // one just ended) reports false.
    return Status{running_, /*has_target=*/phase_ != Phase::kIdle, phase_remaining_ms_, /*is_break_phase=*/running_,
                  /*target_ms=*/PhaseTotalMs(), /*is_count_up=*/false};
}
