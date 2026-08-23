#include "gui_manager.hpp"

GuiManager::GuiManager()
{
    label_ = lv_label_create(lv_screen_active());
    lv_obj_set_style_text_font(label_, kPrimaryFont, 0);
    lv_obj_set_style_text_align(label_, LV_TEXT_ALIGN_CENTER, 0);
    // Sits in the gap between the debug overlay's top label and bottom
    // chart (see main.cpp) rather than dead center, so both stay visible
    // at once while StopwatchFace is being validated on hardware.
    lv_obj_align(label_, LV_ALIGN_CENTER, 0, -30);
}

void GuiManager::SetPrimaryText(const char* text)
{
    lv_label_set_text(label_, text);
}
