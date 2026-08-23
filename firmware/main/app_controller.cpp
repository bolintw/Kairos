#include "app_controller.hpp"

#include "stopwatch_face.hpp"

AppController::AppController(GuiManager& gui_manager)
    : gui_manager_(gui_manager)
{
    current_ = std::make_unique<StopwatchFace>();
    current_->onEnter();
}

void AppController::Update(uint32_t dt_ms)
{
    current_->onTick(dt_ms);
    current_->render(gui_manager_);
}

void AppController::OnTap()
{
    current_->onTap();
}
