#pragma once

#include "battery_monitor.hpp"
#include "qmi8658.hpp"

// Which of the two things RunIdleSleep() can return for made it stop
// sleeping — see that function's comment. kCriticalBattery added
// 2026-09-12 alongside the low-battery safety net's stage 2
// (main.cpp's kCriticalBatteryEnterV/AppController::kCriticalBatteryEnterV):
// this loop can now sit here for a long time with nobody ever waking it,
// during which battery voltage keeps draining at light sleep's own
// ~0.8-0.9mA — without checking for that here, a device already asleep
// when it crossed the critical threshold would just never notice and
// stay in light sleep indefinitely, defeating stage 2 entirely for
// exactly the case (nobody touching the device) it exists to protect
// against.
enum class IdleSleepWakeReason { kMotion, kCriticalBattery };

// Blocking idle-power-saving routine (M9) — called once
// AppController::ShouldEnterIdleSleep() goes true, i.e. the screen has
// already faded fully off and held there for a bit (see
// app_controller.hpp design note 9). Repeatedly sleeps (real light sleep,
// woken by either a Wake-on-Motion event on IMU_INT2 or a ~1s backstop
// timer — see the .cpp) and polls for one in between, returning as soon
// as one is detected — or, checked far less often on the same backstop
// timer (see the .cpp's kBatteryCheckIntervalUs), as soon as
// battery_monitor reads below critical_battery_v. The caller (main.cpp)
// is expected to check the return value and go straight to deep sleep on
// kCriticalBattery, skipping the normal WoM-confirm dance entirely — see
// its own idle-sleep block.
//
// Wake-on-Motion, not tap (2026-09-06, wom-wake-mode branch — was
// tap-engine-based on main): the caller must have already switched the
// IMU into WoM mode (Qmi8658::EnterWakeOnMotion()) before calling this —
// see that method's comment for why (in short: a real tap's INT2 signal
// is too brief a pulse for light sleep's GPIO wakeup to reliably catch;
// WoM's is a held level instead). Restoring the tap engine afterward is
// the caller's job (ExitWakeOnMotion() + a fresh ConfigureTap() call).
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
// for the full comparison against *deep* sleep + Wake-on-Motion
// (blocked: IMU_INT1/INT2 aren't wired to an RTC-capable GPIO, and the
// board's header doesn't expose them for a bodge wire either — that
// restriction is specific to deep sleep's ext0/ext1 wakeup, which only
// looks at RTC GPIOs; light sleep's GPIO wakeup used here has no such
// restriction, which is exactly why WoM is usable at all in this
// project) and deep sleep + ULP-RISC-V bit-bang I2C (works, but far more
// implementation/debugging cost for savings that are hard to feel
// against this path's already-200x-plus improvement).
//
// Touches nothing on screen — the display is already off by the time
// AppController calls this, and nothing here should light it back up
// (that's AppController::NotifyWokeFromIdleSleep()'s job, called once
// this returns).
IdleSleepWakeReason RunIdleSleep(Qmi8658& imu, BatteryMonitor& battery_monitor, float critical_battery_v);
