#pragma once

// Shared master switch (2026-09-12) for the on-screen debug overlay
// (main.cpp's kDebugOverlayEnabled) plus every serial log that narrates
// the same sleep/wake/tap story — sleep_mode.cpp's kSleepDebugLogEnabled,
// qmi8658.hpp's kQmi8658DebugLogEnabled (covers both PollTapEvent and
// PollWomEvent), and main.cpp's own kWomConfirmLatencyLogEnabled. Each of
// those stays its own locally-named constant, with its own explanatory
// comment at its definition site — they just all resolve to this one
// value now instead of an independent literal, so flipping this single
// flag turns the whole group on or off together, per the user's own
// request once the wake/sleep/tap work settled down and the on-screen
// overlay was no longer needed for day-to-day use.
//
// Deliberately does NOT also cover main.cpp's kAttitudeDebugLogEnabled/
// kLoopTimingLogEnabled — different diagnostic subjects entirely
// (attitude-fusion tuning, main-loop throughput), and turning those on
// alongside this would interleave unrelated streams into the same UART,
// defeating whichever one you actually meant to read (see main.cpp's own
// "Flag layout note" for that still-applicable reasoning — this shared
// flag only narrows that note's scope for the sleep/wake/tap subset,
// doesn't override it).
constexpr bool kDebugEnabled = false;
