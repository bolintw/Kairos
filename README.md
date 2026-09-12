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

## Demo

<!--
  GIF slots — drop files into docs/media/ with these names and uncomment
  the corresponding line. Suggested shots:
    - flip.gif           Rotating the device between faces (A/B/C/D)
    - tap.gif            Tap to start/pause
    - charge.gif         Opening the enclosure / plugging in USB-C to charge
    - battery-check.gif  The pick-up-and-hold battery-check gesture
-->
<!-- ![Flip to switch faces](docs/media/flip.gif) -->
<!-- ![Tap to start/pause](docs/media/tap.gif) -->
<!-- ![Charging](docs/media/charge.gif) -->
<!-- ![Battery-check gesture](docs/media/battery-check.gif) -->

## Hardware

- [Waveshare ESP32-S3-LCD-1.28](https://www.waveshare.com/esp32-s3-lcd-1.28.htm) — round 240x240 GC9A01A SPI display, QMI8658 6-axis IMU, ESP32-S3. ([Wiki](https://www.waveshare.com/wiki/ESP32-S3-LCD-1.28))
- Non-touch variant; all interaction is gesture-driven (flip / tap), read via the onboard IMU.

## How it works

The device has no fixed "up" — the screen always faces you, and you rotate
it in your hand like a dial. Four quadrants (A/B/C/D) each hold a different
timer face — two Pomodoro variants, a stopwatch, and a guided 4-7-8
breathing exercise — rotating past a hysteresis threshold switches which
one is active. A single tap toggles start/pause on whichever face is
showing. Screen brightness itself doubles as the notification channel — it
fades during a focus session, snaps back on any interaction, and ramps up
again as a phase is about to end — since there's no speaker or vibration
motor.

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

Actively developed, assembled in a 3D-printed enclosure, and running
day-to-day on battery. Idle power management is done: real light sleep
(~0.8-0.9mA, down from ~70-90mA active) woken by the IMU's native
Wake-on-Motion mode rather than software polling, with a double-tap
gesture to confirm an intentional wake. Battery voltage is sensed via ADC
and backs a two-stage low-battery safety net — a forced warning screen
first, then a genuine deep-sleep/reboot below a critical threshold, with
hysteresis tuned against a real charging-induced voltage jump. Current
focus is retuning tap-gesture sensitivity now that the enclosure has
changed how vibration reaches the IMU. See the milestone list and design
notes inline in the source (`firmware/main/*.hpp` design comments carry a
lot of the "why", not just the "what").

## License

[PolyForm Noncommercial 1.0.0](LICENSE) — free to use and modify for any
noncommercial purpose; commercial use is not permitted.

The primary display font is [Space Grotesk](https://github.com/floriankarsten/space-grotesk),
Copyright 2020 The Space Grotesk Project Authors, licensed under the
[SIL Open Font License 1.1](firmware/main/SpaceGrotesk-OFL.txt).
