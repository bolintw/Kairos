#pragma once

#include <cstdint>

#include "timer_face.hpp"

// D face's real function (2026-08-30, resolving app_controller.hpp design
// note 4's "no defined behavior yet") — a guided 4-7-8 breathing exercise
// (inhale/hold/exhale), not another countdown timer. Distinct enough from
// PomodoroFace to warrant its own class rather than another parameterized
// instance: three asymmetric phases per cycle (not one duration), a fixed
// number of cycles that stops on its own rather than auto-looping forever
// (a breathing exercise is meant to end, unlike focus<->break), and the
// outer ring is the exercise itself here, not a secondary notification —
// see render().
//
// First two cycles ramp up (2-3-4, 3-5-6) before settling into the real
// 4-7-8-4-7-8 seconds for cycles 3-4 (user's design, 2026-08-26 chat) —
// not the "official" protocol, which keeps all cycles at 4-7-8, but a
// gentler on-ramp for someone reaching for this mid-workday rather than
// an experienced practitioner.
class BreathFace : public TimerFace {
public:
    BreathFace() = default;

    void onEnter() override;   // reset to Phase::kIdle, paused, ready for a tap to start
    void onExit() override;
    void onTick(uint32_t dt_ms) override;
    void onTap() override;     // kIdle: start a fresh session. Mid-session: pause/resume, same as every other face.
    void render(GuiManager& gui) override;
    Status GetStatus() const override;

private:
    // kReady (2026-08-30): a fixed 3s "Ready" countdown onTap() inserts
    // between kIdle and the first kInhale — tapping to start otherwise
    // dropped straight into the first inhale with no chance to settle.
    // Treated as a real phase sharing the same render()/AdvancePhase()
    // machinery as kInhale/kHold/kExhale (running_, brightness held full,
    // pause/resume all fall out for free), not a special case — the ring
    // just stays hidden through it (see render()), since nothing breath-
    // related is happening yet.
    enum class Phase { kIdle, kReady, kInhale, kHold, kExhale };

    // Cycle-duration table (2-3-4/3-5-6/4-7-8/4-7-8) is an implementation
    // detail, defined in breath_face.cpp's anonymous namespace rather
    // than as a class member here — nothing outside AdvancePhase()/
    // PhaseTotalMs()/onTap() needs to know its shape.
    void AdvancePhase();  // kReady->first inhale, or inhale->hold->exhale->(next cycle's inhale, or kIdle after the last one)
    uint32_t PhaseTotalMs() const;  // full duration of whichever phase_ is current, for render()'s ring envelope

    Phase phase_ = Phase::kIdle;
    uint32_t cycle_index_ = 0;
    uint32_t phase_remaining_ms_ = 0;
    bool running_ = false;

    // Completion caption (2026-08-30): "Good Job" for kDoneCaptionMs once
    // all cycles finish, before render() falls back to the idle "Breath"
    // screen. First attempt also had a per-phase fading caption
    // ("Inhale"/"Hold"/"Exhale" replacing the countdown for ~0.7s at each
    // phase boundary, same trick as PomodoroFace's Focus/Relax) — dropped
    // the same day: at BreathFace's pace (a new phase every 2-8s) the
    // word-then-number swap was itself the abrupt thing the user then
    // flagged, on top of the number jump it was meant to soften. Replaced
    // with GuiManager::SetSecondaryText() showing the phase name
    // permanently above the countdown instead — see render() — which
    // needs no timer of its own. This field is unrelated to that: it's a
    // one-shot end-of-session moment, not a per-phase announcement, kept
    // as its own field rather than a repurposed one to keep those two
    // occasions from being confused with each other.
    uint32_t done_caption_remaining_ms_ = 0;
};
