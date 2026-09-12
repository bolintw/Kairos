#pragma once

#include "battery_monitor.hpp"
#include "qmi8658.hpp"

// Why RunIdleSleep() stopped sleeping: real motion, or battery voltage
// crossed the critical threshold while nobody was around to wake it.
enum class IdleSleepWakeReason { kMotion, kCriticalBattery };

// Blocking idle-power-saving routine, called once
// AppController::ShouldEnterIdleSleep() goes true. Repeatedly light-sleeps
// (woken by a Wake-on-Motion event on IMU_INT2 or a ~1s backstop timer)
// and returns as soon as motion is detected, or as soon as
// battery_monitor reads below critical_battery_v (checked less often, on
// the same backstop timer). The caller is expected to check the return
// value and go straight to deep sleep on kCriticalBattery, skipping the
// normal WoM-confirm dance.
//
// Requires the IMU already switched into WoM mode
// (Qmi8658::EnterWakeOnMotion()) before calling — a plain tap's INT2
// pulse is too brief for light sleep's GPIO wakeup to catch reliably,
// WoM's held level isn't. Restoring the tap engine afterward
// (ExitWakeOnMotion() + ConfigureTap()) is the caller's job.
//
// Light sleep, not deep sleep: execution resumes in place, so every
// other object in the app (current face/phase/remaining time, fused
// angle) is simply still there when this returns — no state to
// save/restore. Touches nothing on screen; the display is already off
// when this is called, and NotifyWokeFromIdleSleep() handles turning it
// back on once this returns. main.cpp calls GuiManager::ForceRedraw()
// unconditionally after this returns to recover the panel state.
IdleSleepWakeReason RunIdleSleep(Qmi8658& imu, BatteryMonitor& battery_monitor, float critical_battery_v);
