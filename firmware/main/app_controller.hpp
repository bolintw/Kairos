#pragma once

#include <memory>

#include "gui_manager.hpp"
#include "timer_face.hpp"

// DRAFT — deliberately scoped down for this first pass. Flip-driven face
// switching (hysteresis + quantizing AttitudeEstimator's screen_angle_deg
// into A/B/C/D) and the Factory that would build the right TimerFace for
// each are NOT implemented yet: AppController currently just owns a
// single hardcoded StopwatchFace, so the tap/tick/render pipeline can be
// validated on hardware first. Attitude-driven switching, Factory, and
// NVS persistence (design already settled — see plan's M5 persistence
// notes) all get added once more faces exist. At that point
// AppController will take an AttitudeEstimator::Output per tick rather
// than an AttitudeEstimator reference — main.cpp already owns the single
// Update() call on the sensor-owning loop, so AppController shouldn't
// also hold a live reference and call Update() itself; that would
// double-integrate the gyro angle.
//
// GuiManager is injected by reference (DI, no Singleton), matching the
// pattern already used elsewhere in this project.
class AppController {
public:
    explicit AppController(GuiManager& gui_manager);

    // Call once per loop tick with elapsed time since the previous call.
    void Update(uint32_t dt_ms);

    // Call when the tap engine reports a new tap.
    void OnTap();

private:
    GuiManager& gui_manager_;
    std::unique_ptr<TimerFace> current_;
};
