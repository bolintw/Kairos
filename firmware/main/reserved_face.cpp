#include "reserved_face.hpp"

#include "gui_manager.hpp"

void ReservedFace::onEnter()
{
}

void ReservedFace::onExit()
{
}

void ReservedFace::onTick(uint32_t dt_ms)
{
}

void ReservedFace::onTap()
{
}

void ReservedFace::render(GuiManager& gui)
{
    gui.SetPrimaryText("Reserved");
}

TimerFace::Status ReservedFace::GetStatus() const
{
    return Status{/*is_running=*/false, /*has_target=*/false, /*remaining_ms=*/0};
}
