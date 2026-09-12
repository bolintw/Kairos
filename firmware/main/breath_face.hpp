#pragma once

#include <cstdint>

#include "timer_face.hpp"

// A guided 4-7-8 breathing exercise (inhale/hold/exhale), not another
// countdown timer: three asymmetric phases per cycle, a fixed number of
// cycles that stops on its own instead of looping forever, and the outer
// ring is the exercise itself, not a secondary notification (see
// render()).
//
// First two cycles ramp up (2-3-4, 3-5-6) before settling into the real
// 4-7-8 for cycles 3-4 — a gentler on-ramp than the "official" protocol's
// flat 4-7-8 throughout.
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
    // kReady: a fixed 3s "Ready" countdown onTap() inserts between kIdle
    // and the first kInhale, so starting doesn't drop straight into the
    // first inhale. Shares the same render()/AdvancePhase() machinery as
    // kInhale/kHold/kExhale; the ring just stays hidden through it.
    enum class Phase { kIdle, kReady, kInhale, kHold, kExhale };

    void AdvancePhase();  // kReady->first inhale, or inhale->hold->exhale->(next cycle, or kIdle after the last one)
    uint32_t PhaseTotalMs() const;  // full duration of the current phase, for render()'s ring envelope

    Phase phase_ = Phase::kIdle;
    uint32_t cycle_index_ = 0;
    uint32_t phase_remaining_ms_ = 0;
    bool running_ = false;

    // "Good Job" caption shown for this many ms once all cycles finish,
    // before render() falls back to the idle "Breath" screen.
    uint32_t done_caption_remaining_ms_ = 0;
};
