#include "breath_face.hpp"

#include <cmath>
#include <cstdio>

#include "gui_manager.hpp"

namespace {
// One accent color per breathing phase, applied to both the label and the ring.
const lv_color_t kColorInhale = lv_color_make(60, 200, 120);
const lv_color_t kColorHold = lv_color_make(230, 180, 60);
const lv_color_t kColorExhale = lv_color_make(150, 110, 220);

struct CycleSpec {
    uint32_t inhale_s;
    uint32_t hold_s;
    uint32_t exhale_s;
};

constexpr CycleSpec kCycles[] = {
    {2, 3, 4},
    {3, 5, 6},
    {4, 7, 8},
    {4, 7, 8},
};
constexpr uint32_t kNumCycles = sizeof(kCycles) / sizeof(kCycles[0]);
constexpr uint32_t kReadySeconds = 3;
constexpr uint32_t kDoneCaptionMs = 2000;

// Sine-hump fade: 0 at elapsed=0, peaks at the midpoint, 0 again at elapsed=total.
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
    // Decremented unconditionally — it only ever plays while running_ is
    // already false, so gating it behind that check would mean it never
    // ticks down.
    if (done_caption_remaining_ms_ > 0) {
        done_caption_remaining_ms_ = dt_ms < done_caption_remaining_ms_ ? done_caption_remaining_ms_ - dt_ms : 0;
    }

    if (!running_) return;

    // Looped so a dt_ms spanning multiple short phases (2s/3s ones
    // especially) walks through all of them, not just one.
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
        // Start a fresh session with a 3s "Ready" beat first; AdvancePhase()
        // carries it into cycle 0's inhale once that runs out.
        cycle_index_ = 0;
        phase_ = Phase::kReady;
        phase_remaining_ms_ = kReadySeconds * 1000;
        running_ = true;
        return;
    }
    running_ = !running_;  // mid-session: plain pause/resume
}

void BreathFace::AdvancePhase()
{
    const CycleSpec& cycle = kCycles[cycle_index_];
    switch (phase_) {
        case Phase::kReady:
            phase_ = Phase::kInhale;
            phase_remaining_ms_ = cycle.inhale_s * 1000;
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
            break;
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

    // kReady reuses kColorInhale — it's leading into the first inhale anyway.
    const lv_color_t color = phase_ == Phase::kReady  ? kColorInhale
                            : phase_ == Phase::kInhale ? kColorInhale
                            : phase_ == Phase::kHold   ? kColorHold
                                                        : kColorExhale;
    gui.SetAccentColor(color);

    // Phase name shown permanently above the countdown.
    gui.SetSecondaryText(phase_ == Phase::kReady  ? "Ready?"
                        : phase_ == Phase::kInhale ? "Inhale"
                        : phase_ == Phase::kHold   ? "Hold"
                                                    : "Exhale");
    gui.SetPrimaryTextOpacity(255);

    // Round up so the display shows the full duration (e.g. "4" for a 4s
    // phase) instead of skipping straight to "3".
    const uint32_t seconds_left = (phase_remaining_ms_ + 999) / 1000;
    char buf[12];  // sized for uint32_t's %u worst case, not just the real 1-8 range, to avoid -Wformat-truncation
    std::snprintf(buf, sizeof(buf), "%u", static_cast<unsigned int>(seconds_left));
    gui.SetPrimaryText(buf);

    // The ring IS the exercise here (rise through inhale, flat through
    // hold, fall through exhale), not AppController's generic countdown-
    // erosion visual. Runs after AppController's own UpdateRing() in the
    // tick sequence, so this has the final say while a phase is active.
    // SetRingProgress(0, false) forces the arc back to a full circle;
    // SetRingTickOpacity(0) hides the erosion tick, unused here.
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
    // is_break_phase="stay fully bright" while running, so a still 7s
    // hold (no tap/movement to register as "interacting") doesn't dim.
    // has_target uses phase_ != kIdle rather than running_, so the ring
    // keeps showing the frozen state while paused mid-phase too.
    return Status{running_, /*has_target=*/phase_ != Phase::kIdle, phase_remaining_ms_, /*is_break_phase=*/running_,
                  /*target_ms=*/PhaseTotalMs(), /*is_count_up=*/false};
}
