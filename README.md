# Kairos

A gravity-triggered desktop Pomodoro / countdown timer. No buttons for the
core interaction — flip the device to a different face to switch what it's
timing, tap it to start or pause. Built on a Waveshare ESP32-S3-LCD-1.28
round display, in modern C++ on ESP-IDF.

Kairos is also a personal practice ground for two things at once: getting
hands-on with embedded C++ on real hardware, and calibrating how well an AI
coding agent (Claude) holds up on a project that grows past a single
session — architecture discipline, whether its design choices survive
contact with real hardware bugs, and how to review its output in a domain
I'm not already fluent in. Every commit that's been verified on real
hardware and authored primarily by the agent is committed under
`Claude Sonnet 5 <noreply@anthropic.com>`, so that history is easy to
filter and read back later (`git log --author="Claude Sonnet 5"`).

## Hardware

- [Waveshare ESP32-S3-LCD-1.28](https://www.waveshare.com/esp32-s3-lcd-1.28.htm) — round 240x240 GC9A01A SPI display, QMI8658 6-axis IMU, ESP32-S3. ([Wiki](https://www.waveshare.com/wiki/ESP32-S3-LCD-1.28))
- Non-touch variant; all interaction is gesture-driven (flip / tap), read via the onboard IMU.

## How it works

The device has no fixed "up" — the screen always faces you, and you rotate
it in your hand like a dial. Four quadrants (A/B/C/D) each hold a different
timer face; rotating past a hysteresis threshold switches which one is
active. A single tap toggles start/pause on whichever face is showing.
Screen brightness itself doubles as the notification channel — it fades
during a focus session, snaps back on any interaction, and ramps up again
as a phase is about to end — since there's no speaker or vibration motor.

## Building

Requires [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/index.html) v6.0 targeting `esp32s3`.

```sh
git submodule update --init --recursive
cd firmware
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

## Status

Actively developed, not yet in an enclosure. See the milestone list and
design notes inline in the source (`firmware/main/*.hpp` design comments
carry a lot of the "why", not just the "what").

## License

[PolyForm Noncommercial 1.0.0](LICENSE) — free to use and modify for any
noncommercial purpose; commercial use is not permitted.
