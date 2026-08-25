#include "stopwatch_face.hpp"

#include <cstdio>

#include "gui_manager.hpp"

void StopwatchFace::onEnter()
{
    elapsed_ms_ = 0;
    running_ = false;
}

void StopwatchFace::onExit()
{
}

void StopwatchFace::onTick(uint32_t dt_ms)
{
    if (running_) {
        elapsed_ms_ += dt_ms;
    }
}

void StopwatchFace::onTap()
{
    running_ = !running_;
}

void StopwatchFace::render(GuiManager& gui)
{
    const uint32_t total_seconds = elapsed_ms_ / 1000;
    const uint32_t minutes = total_seconds / 60;
    const uint32_t seconds = total_seconds % 60;

    // Plain libc snprintf, not LVGL's builtin one — the %f-support
    // limitation noted in main.cpp only applies to lv_label_set_text_fmt.
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02u:%02u",
                   static_cast<unsigned int>(minutes), static_cast<unsigned int>(seconds));
    gui.SetPrimaryText(buf);
}

TimerFace::Status StopwatchFace::GetStatus() const
{
    return Status{running_, /*has_target=*/false, /*remaining_ms=*/0, /*is_break_phase=*/false};
}
