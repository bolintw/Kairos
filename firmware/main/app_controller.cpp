#include "app_controller.hpp"

#include <cmath>

#include "pomodoro_face.hpp"
#include "stopwatch_face.hpp"

namespace {
constexpr uint32_t kFocusMsA = 25 * 60 * 1000;
constexpr uint32_t kBreakMsA = 5 * 60 * 1000;
constexpr uint32_t kFocusMsB = 50 * 60 * 1000;
constexpr uint32_t kBreakMsB = 10 * 60 * 1000;

// See app_controller.hpp's design note 1 for why 80 degrees produces the
// intended asymmetric enter/leave band by itself.
constexpr float kFaceHysteresisLeaveDeg = 80.0f;

float FaceCenterDeg(AppController::Face face)
{
    switch (face) {
        case AppController::Face::kA: return -90.0f;
        case AppController::Face::kB: return 0.0f;
        case AppController::Face::kC: return 90.0f;
        case AppController::Face::kD: return 180.0f;
    }
    return 0.0f;
}

// Shortest angular distance between two angles, always >= 0.
float AngularDistanceDeg(float a_deg, float b_deg)
{
    float diff = a_deg - b_deg;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return std::fabs(diff);
}

// Nearest-center classification, no hysteresis — used once QuantizeFace
// decides the angle has moved far enough to justify leaving current_face_.
AppController::Face NearestFace(float screen_angle_deg)
{
    float deg = screen_angle_deg;
    while (deg < 0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    if (deg < 45.0f || deg >= 315.0f) return AppController::Face::kB;
    if (deg < 135.0f) return AppController::Face::kC;
    if (deg < 225.0f) return AppController::Face::kD;
    return AppController::Face::kA;
}
}  // namespace

AppController::AppController(GuiManager& gui_manager)
    : gui_manager_(gui_manager)
{
}

AppController::Face AppController::QuantizeFace(float screen_angle_deg) const
{
    if (AngularDistanceDeg(screen_angle_deg, FaceCenterDeg(current_face_)) < kFaceHysteresisLeaveDeg) {
        return current_face_;
    }
    return NearestFace(screen_angle_deg);
}

std::unique_ptr<TimerFace> AppController::CreateFace(Face face)
{
    switch (face) {
        case Face::kA: return std::make_unique<PomodoroFace>(kFocusMsA, kBreakMsA);
        case Face::kB: return std::make_unique<PomodoroFace>(kFocusMsB, kBreakMsB);
        case Face::kC: return std::make_unique<StopwatchFace>();
        case Face::kD: return nullptr;  // no defined behavior yet
    }
    return nullptr;
}

void AppController::Update(const AttitudeEstimator::Output& attitude, uint32_t dt_ms)
{
    const Face quantized = QuantizeFace(attitude.screen_angle_deg);

    if (attitude.is_moving) {
        was_disturbed_ = true;
        if (quantized != current_face_) {
            crossed_face_ = true;
        }
    } else if (was_disturbed_) {
        if (crossed_face_) {
            if (current_) {
                current_->onExit();
            }
            current_face_ = quantized;
            current_ = CreateFace(current_face_);
            if (current_) {
                current_->onEnter();
            }
        }
        was_disturbed_ = false;
        crossed_face_ = false;
    }

    if (current_) {
        current_->onTick(dt_ms);
        current_->render(gui_manager_);
    }
}

void AppController::OnTap()
{
    if (current_) {
        current_->onTap();
    }
}
