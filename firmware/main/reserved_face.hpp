#pragma once

#include "timer_face.hpp"

// Placeholder for face D, whose real functionality is still undecided (see
// app_controller.hpp design note 4). Exists so D has a visible, debuggable
// face instead of a silent nullptr — before this, there was no way to tell
// from the screen alone whether a flip to D had actually been detected.
// Replace with real behavior once D's functionality is decided.
class ReservedFace : public TimerFace {
public:
    void onEnter() override;
    void onExit() override;
    void onTick(uint32_t dt_ms) override;
    void onTap() override;
    void render(GuiManager& gui) override;
    Status GetStatus() const override;
};
