#pragma once

#include "qmi8658.hpp"

// Blocking idle-power-saving routine (M9) — called once
// AppController::ShouldEnterIdleSleep() goes true, i.e. the screen has
// already faded fully off and held there for a bit (see
// app_controller.hpp design note 9). Repeatedly sleeps (real light sleep,
// woken by either a tap on IMU_INT2 or a ~1s backstop timer — see the
// .cpp) and polls for a tap in between, returning as soon as one is
// detected.
//
// Tap-only wake (2026-09-06, was tap-or-rotation): the caller must have
// already switched the IMU into accel-only Low Power mode
// (Qmi8658::SetLowPowerAccelOnly(true)) before calling this — gyro's own
// current draw barely depends on ODR (see that method's comment), so
// disabling it entirely was the only real lever for getting idle current
// down, and it means there's no gyro sample to check a rotation against
// here anymore. Restoring normal 6DOF mode afterward is also the
// caller's job.
//
// Calls esp_light_sleep_start() directly in a loop, with a real GPIO
// wakeup source on IMU_INT2 (2026-09-06, was a plain vTaskDelay() nap
// relying on FreeRTOS tickless idle + ESP-IDF's automatic PM light sleep
// for a while — see sleep_mode.cpp's class comment for the two failed
// GPIO-fast-wake attempts on *that* automatic path that led back here,
// and gravity_timer_project_plan.md's M9 section for the full writeup).
//
// An even earlier version also called esp_light_sleep_start() directly
// and left the LCD (and the separate, non-GuiManager debug overlay
// label) permanently blank after the very first sleep — that's why the
// automatic-PM detour happened in between. Root cause was never pinned
// down at the time, but turned out not to matter: the actual fix
// (GuiManager::ForceRedraw(), re-running lcd.init() on wake) was added
// while still on the automatic path and fixed a blank screen there too —
// same symptom, same fix, regardless of which mechanism triggers the
// sleep, which is why calling esp_light_sleep_start() manually again now
// isn't expected to reopen that issue. main.cpp still calls
// GuiManager::ForceRedraw() unconditionally after this returns.
//
// Deliberately light sleep, not esp_deep_sleep_start() in any form:
// light sleep resumes execution in place rather than rebooting, so every
// other object in the app (current_'s face/phase/remaining time,
// AttitudeEstimator's angle) is simply still there when this returns —
// no state to save/restore. See gravity_timer_project_plan.md's M9 notes
// for the full comparison against deep sleep + Wake-on-Motion (blocked:
// IMU_INT1/INT2 aren't wired to an RTC-capable GPIO, and the board's
// header doesn't expose them for a bodge wire either) and deep sleep +
// ULP-RISC-V bit-bang I2C (works, but far more implementation/debugging
// cost for savings that are hard to feel against this path's
// already-200x-plus improvement).
//
// Touches nothing on screen — the display is already off by the time
// AppController calls this, and nothing here should light it back up
// (that's AppController::NotifyWokeFromIdleSleep()'s job, called once
// this returns).
void RunIdleSleep(Qmi8658& imu);
