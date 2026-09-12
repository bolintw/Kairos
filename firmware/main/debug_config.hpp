#pragma once

// Shared master switch for the on-screen debug overlay
// (main.cpp's kDebugOverlayEnabled) plus every serial log that narrates
// the same sleep/wake/tap story — sleep_mode.cpp's kSleepDebugLogEnabled,
// qmi8658.hpp's kQmi8658DebugLogEnabled, and main.cpp's own
// kWomConfirmLatencyLogEnabled. Each stays its own locally-named
// constant; they just all resolve to this one value so flipping this
// single flag turns the whole group on or off together.
//
// Does NOT cover main.cpp's kAttitudeDebugLogEnabled/
// kLoopTimingLogEnabled — different diagnostic subjects (attitude-fusion
// tuning, main-loop throughput) that would interleave unrelated streams
// into the same UART if turned on alongside this.
constexpr bool kDebugEnabled = false;
