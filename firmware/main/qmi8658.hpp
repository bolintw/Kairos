#pragma once

#include <cstdint>
#include <cstdio>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "debug_config.hpp"

// Minimal QMI8658 accel+gyro+tap reader for the M3/M4 debug overlay.
// Accel/gyro register addresses and init sequence are ported from
// Waveshare's official Arduino demo for this exact board
// (ESP32-S3-LCD-1.28-Demo.zip, QMI8658.cpp), which doesn't cover tap
// detection. The Tap Engine protocol (CTRL9 command handshake, CAL1-4
// register layout, TAP_STATUS bit layout) is ported instead from
// lewisxhe/SensorLib's SensorQMI8658.hpp — a widely used community
// driver for this exact chip — rather than re-derived from the
// datasheet PDF, which isn't reliably text-extractable here.
//
// Self-contained: owns its own I2C bus + device, separate from the
// one-shot bus RunI2cScan() creates and tears down in i2c_scan.hpp.
// Whether AttitudeEstimator (M5) shares a bus with this or replaces it
// entirely is an open design question for that milestone, not decided
// here — this class exists only to drive the M3/M4 debug overlay.
//
// Gates PollWomEvent()'s/PollTapEvent()'s STATUS1 (and TAP_STATUS)
// prints — real errors (I2C bus/device creation failure, soft reset not
// confirming, CTRL9 handshake failure) always print regardless, those
// aren't routine noise. This is a header included from multiple .cpp
// files, so this constant deliberately isn't a class member — a plain
// file-scope constexpr gets its own internal-linkage copy per
// translation unit, no ODR issue, no need to plumb it through the
// constructor just to toggle a printf. Tied to the shared kDebugEnabled
// (debug_config.hpp, 2026-09-12) alongside sleep_mode.cpp's
// kSleepDebugLogEnabled and main.cpp's overlay/WoM-latency log; see that
// header's comment for why this group merges while main.cpp's
// kAttitudeDebugLogEnabled/kLoopTimingLogEnabled (its own flag-layout
// note) stay independent.
constexpr bool kQmi8658DebugLogEnabled = kDebugEnabled;

class Qmi8658 {
public:
    struct Sample {
        float accel_g[3];    // X, Y, Z in units of g, +-8g range
        float gyro_dps[3];   // X, Y, Z in degrees/sec, +-256dps range
    };

    enum class TapEvent { kNone, kSingle, kDouble };

    Qmi8658(gpio_num_t sda, gpio_num_t scl, uint16_t addr = 0x6B)
    {
        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = I2C_NUM_0;
        bus_cfg.sda_io_num = sda;
        bus_cfg.scl_io_num = scl;
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = true;
        if (i2c_new_master_bus(&bus_cfg, &bus_) != ESP_OK) {
            printf("Qmi8658: failed to create I2C bus\n");
            return;
        }

        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = addr;
        dev_cfg.scl_speed_hz = 400000;
        if (i2c_master_bus_add_device(bus_, &dev_cfg, &dev_) != ESP_OK) {
            printf("Qmi8658: failed to add device at 0x%02X\n", addr);
            return;
        }

        // Soft reset (2026-09-07, wom-wake-mode branch) — added after a
        // WoM debugging session (main.cpp's WOM_EDGE_TEST) that captured
        // zero GPIO edges on either INT pin across many real taps, with
        // no obvious register-level explanation. This chip is a separate
        // I2C device that a re-flash or ESP32 reset does *not* reset —
        // after a full day of repeated Enter/ExitWakeOnMotion and tap-
        // engine reconfiguration cycles across many test iterations
        // without a real power cycle in between, there was no guarantee
        // it was still in a clean, known state by the time any of that
        // testing started. Datasheet Section 5.9/7.4 (Table 27): write
        // 0xB0 to RESET (0x60) — the datasheet's own prose section (7.4)
        // actually says "0x0B" for the same operation, contradicting its
        // own register table (5.9); went with 0xB0, cross-checked against
        // lewisxhe/SensorLib (QMI8658_REG_RESET_DEFAULT), the same
        // community driver this file's tap-engine protocol was ported
        // from. Confirmed via RSTResult (0x4D) reading 0x80 on success;
        // up to 15ms for the process per the datasheet, polled here with
        // margin. Not gated behind dev_ the way other methods are — this
        // runs before dev_ semantically "exists" as a working device from
        // this class's perspective, but the handle itself is already
        // valid at this point.
        WriteReg(kRegReset, 0xB0);
        bool reset_ok = false;
        for (int i = 0; i < 20; ++i) {  // ~200ms at 10ms/iteration, well past the documented 15ms max
            vTaskDelay(pdMS_TO_TICKS(10));
            uint8_t result = 0;
            if (ReadRegs(kRegResetResult, &result, 1) && result == 0x80) {
                reset_ok = true;
                break;
            }
        }
        if (!reset_ok) {
            printf("Qmi8658: soft reset did not confirm (RSTResult != 0x80) — proceeding anyway\n");
        }

        // Sequence and register values match QMI8658_init() /
        // QMI8658_config_acc() / QMI8658_config_gyro() in the vendor demo:
        // Ctrl1=0x60, Ctrl2 = accel +-8g @ 1000Hz, Ctrl5=0x00 (LPF/HPF off
        // — the vendor code computes LPF bits but then unconditionally
        // overwrites them with 0 before the write), Ctrl7 = enable
        // accel+gyro.
        //
        // 0x60 -> 0x78 (2026-09-07, wom-wake-mode branch) — after the
        // WOM_EDGE_TEST diagnostic (main.cpp) found zero GPIO edges on
        // either INT pin across 5 rounds despite confirmed genuine WoM
        // detection (STATUS1.WoM latching correctly, negative-control
        // clean), a second AI model consulted on this found that the
        // QMI8658A datasheet (sister part, same die family) documents
        // CTRL1 bit3/bit4 as INT1_EN/INT2_EN — "0: pin is high-Z, 1: pin
        // output enabled", default 0 — while our QMI8658C Rev A datasheet
        // marks those same two bits "Reserved" (a document gap, not a
        // different chip behavior, per that analysis — matches this
        // project's own already-found Section 5.9/7.4 RESET-value
        // contradiction, i.e. this specific datasheet copy is known
        // unreliable in more than one place). 0x60 = 0110_0000 leaves both
        // bits 0 — every INT pin left high-Z this whole time would fully
        // explain "zero edges no matter what else changes" across every
        // WoM test round without needing any of those other rounds' fixes
        // to have been wrong. 0x78 = 0x60 | 0x18 sets both bits 3 and 4,
        // enabling both INT1 and INT2 outputs; deliberately ORed rather
        // than a fresh literal so bits 5/6 (whatever they are — not
        // re-derived here, unrelated to this change) stay exactly as the
        // vendor demo had them.
        //
        // Caveat this doesn't explain (told to me straight, not hidden):
        // GPIO48 was observed carrying real ~900Hz DRDY-synced activity
        // earlier this session while CTRL1 was still 0x60 — if INT2 were
        // genuinely high-Z the whole time, that pulse shouldn't have
        // reached the physical pin either. Possible the C-die routes DRDY
        // through a different path than discrete WoM/tap events, but
        // that's not confirmed either way — this is a cheap, high-signal
        // test regardless of that open question, not a certainty.
        //
        // CONFIRMED (2026-09-07) — this was the actual bug. With this fix
        // plus CAL1_H reverted to 0x40 below, WOM_EDGE_TEST captured 107
        // edges on GPIO48 across 5 real taps (6 clearly separated bursts,
        // each ~7-19 rapid toggles over ~10-20ms — one physical tap's
        // accel spike apparently crosses the WoM threshold on several
        // consecutive 1000Hz samples, each independently toggling the
        // line, not one clean toggle per "event" the way Section 12.4's
        // wording alone suggested), where every prior round captured
        // zero. INT2 was simply never driving the pin before this.
        WriteReg(kRegCtrl1, 0x78);
        WriteReg(kRegCtrl2, 0x23);  // +-8g range, 1000Hz ODR
        WriteReg(kRegCtrl5, 0x00);
        // CTRL3 gFS<2:0> is bits[6:4] (QMI8658C datasheet Rev 0.6, Table
        // 24, p.27): 000=16, 001=32, 010=64, 011=128, 100=256, 101=512,
        // 110=1024, 111=2048 dps. 0x43 = 0100_0011 -> bits[6:4]=100 ->
        // *256dps*. This was originally a bug (code assumed 512dps, see
        // git history 2026-08-24) causing every dps reading to be 2x true
        // value — kept at 256dps deliberately for a while afterward
        // (narrower range = more LSB/dps = less dps noise for the same
        // ADC noise floor), on the assumption typical desk-flip rates
        // wouldn't approach 256dps. That assumption didn't hold up: a
        // 2026-09-07 angle-graphing session (kAttitudeDebugLogEnabled,
        // main.cpp) caught real flips peaking at ~250-252 dps — within
        // ~2% of the 256dps ceiling, on ordinary flips, not a
        // deliberately hard one. Worse, the main loop's tick period is
        // still ~90-140ms (see gravity_timer_project_plan.md's main-loop
        // throughput notes) — far coarser than kGyroLowPassTauMs's 6ms
        // time constant, so that filter barely smooths anything at this
        // tick rate and a peak between two samples could be missed
        // entirely, meaning the true instantaneous peak could already be
        // higher than what got logged. 0x43 -> 0x53 (sets bit4, bits[6:4]
        // 100->101 i.e. 256->512dps) trades some of that noise-floor
        // margin back for headroom against silent clipping (clipping
        // under-reads the angle rather than overshooting it — a quieter,
        // easier-to-miss failure than the overshoot bug this range change
        // originally fixed), matching this comment's own predicted
        // trigger for revisiting it. kGyroLsbPerDps below updated to
        // match; bits[3:0]=0011 (1000Hz ODR) unchanged.
        WriteReg(kRegCtrl3, 0x53);  // +-512dps range, 1000Hz ODR
        WriteReg(kRegCtrl5, 0x00);
        WriteReg(kRegCtrl7, 0x03);  // accel + gyro enable
    }

    ~Qmi8658()
    {
        if (dev_) i2c_master_bus_rm_device(dev_);
        if (bus_) i2c_del_master_bus(bus_);
    }

    Qmi8658(const Qmi8658&) = delete;
    Qmi8658& operator=(const Qmi8658&) = delete;

    bool Read(Sample& out)
    {
        if (!dev_) return false;

        uint8_t buf[12];
        if (!ReadRegs(kRegAxL, buf, sizeof(buf))) return false;

        int16_t raw[6];
        for (int i = 0; i < 6; ++i) {
            raw[i] = static_cast<int16_t>(
                (static_cast<uint16_t>(buf[i * 2 + 1]) << 8) | buf[i * 2]);
        }

        for (int i = 0; i < 3; ++i) {
            out.accel_g[i] = raw[i] / kAccelLsbPerG;
            out.gyro_dps[i] = raw[3 + i] / kGyroLsbPerDps;
        }
        return true;
    }

    // Configures the hardware Tap Engine. Values here are M4's starting
    // point for calibration, not a finished tune — see main.cpp for the
    // actual numbers in use and where they came from.
    //
    // priority: which axis wins when peaks land simultaneously (0 = X>Y>Z,
    //   per SensorLib's TapDetectionPriority enum).
    // peak_window/tap_window/d_tap_window: durations in *samples* at the
    //   configured accel ODR (1000Hz here).
    // alpha/gamma: smoothing ratios for the engine's internal running
    //   averages, unitless, not ODR-dependent.
    // peak_mag_thr_g2/udm_thr_g2: peak and quiet thresholds in g^2.
    void ConfigureTap(uint8_t priority, uint8_t peak_window, uint16_t tap_window,
                       uint16_t d_tap_window, float alpha, float gamma,
                       float peak_mag_thr_g2, float udm_thr_g2)
    {
        if (!dev_) return;

        // The CAL1-4 registers double as tap-config scratch space, so
        // accel/gyro are paused while writing them (matches SensorLib's
        // configTap, which does the same before restoring CTRL7).
        WriteReg(kRegCtrl7, 0x00);

        WriteReg(kRegCal1L, peak_window);
        WriteReg(kRegCal1H, priority);
        WriteReg(kRegCal2L, tap_window & 0xFF);
        WriteReg(kRegCal2H, (tap_window >> 8) & 0xFF);
        WriteReg(kRegCal3L, d_tap_window & 0xFF);
        WriteReg(kRegCal3H, (d_tap_window >> 8) & 0xFF);
        WriteReg(kRegCal4H, 0x01);
        WriteCommandAndWait(kCmdConfigureTap);

        WriteReg(kRegCal1L, static_cast<uint8_t>(alpha * 128));
        WriteReg(kRegCal1H, static_cast<uint8_t>(gamma * 128));

        // Resolution is documented as 0.001 g^2/LSB, so value = thr * 1000.
        const uint16_t peak_val = static_cast<uint16_t>(peak_mag_thr_g2 * 1000.0f + 0.5f);
        WriteReg(kRegCal2L, peak_val & 0xFF);
        WriteReg(kRegCal2H, (peak_val >> 8) & 0xFF);

        const uint16_t udm_val = static_cast<uint16_t>(udm_thr_g2 * 1000.0f + 0.5f);
        WriteReg(kRegCal3L, udm_val & 0xFF);
        WriteReg(kRegCal3H, (udm_val >> 8) & 0xFF);
        WriteReg(kRegCal4H, 0x02);
        WriteCommandAndWait(kCmdConfigureTap);

        WriteReg(kRegCtrl7, 0x03);  // re-enable accel + gyro
        WriteReg(kRegCtrl8, 0x01);  // enable tap detection (bit 0)
    }

    // Switches between the constructor's normal 6DOF setup (accel+gyro
    // both enabled) and accel-only — added 2026-09-06 for M9 idle sleep.
    // Gyro draws roughly the same current regardless of ODR (QMI8658C.pdf
    // Table 16 — cost of driving the MEMS resonator itself, not
    // sample-rate-dependent), so disabling it is the only real lever for
    // gyro's share; accel-only High-Resolution at 1000Hz (Table 15) is
    // ~182uA versus the ~1mA this project measured with both enabled —
    // most of the win, from gyro alone. Accepted trade-off: no gyro
    // samples means no rotation-based wake while in this mode, tap
    // detection only (still works — tap detection only ever needed
    // accel, see ConfigureTap()).
    //
    // Deliberately does NOT also drop the accel ODR the way two earlier
    // versions of this function did (Low Power mode at 128Hz, then
    // High-Resolution at 125Hz) — both broke tap detection outright on
    // hardware, not just made it less sensitive. In hindsight the ODR
    // reduction was barely worth it anyway (134-182uA either way per
    // Table 15, accel's own current barely varies across its
    // High-Resolution ODR range) next to the risk: the tap engine's
    // alpha/gamma coefficients (see ConfigureTap()'s comment) are the
    // chip's own per-sample EMA weights, so they carry a real-time time
    // constant that depends on ODR the same way this project's own
    // software low-pass filters do (attitude_estimator.cpp) — dropping
    // ODR ~7x means their effective time constants stretch ~7x too, and
    // at 125Hz the math for the *original* gamma's implied time constant
    // (~4.46ms at the real 896.8Hz 6DOF rate) works out to needing
    // gamma > 1, not a representable coefficient at all. Keeping the
    // accel ODR unchanged (CTRL2 stays 0x23 in both branches below, only
    // CTRL7's enable bits move) sidesteps this entirely: the tap engine
    // keeps running with the exact same tuning that's already been
    // validated on hardware, whether or not gyro is also enabled.
    //
    // Toggling CTRL7 (this function's only job) latches a spurious
    // STATUS1 tap flag, same quirk main.cpp already discards one
    // PollTapEvent() for right after boot's ConfigureTap() call (which
    // also toggles CTRL7). Callers of SetLowPowerAccelOnly(true) need to
    // do the same discard before trusting the next real PollTapEvent() —
    // skipped without it, RunIdleSleep() woke itself up immediately every
    // time, reading that latched flag as a real tap (caught on hardware).
    // enable=true also sets CTRL7.bit5 (DRDY_DIS, 2026-09-06) — datasheet
    // Section 6.3: with DRDY_DIS=0 (the default, and what enable=false
    // leaves it at), the accelerometer's Data-Ready signal is *also*
    // routed to INT2/GPIO48 (same pin CTRL8.bit6=0 already sends tap
    // events to, see ConfigureTap()'s CTRL8 write), pulsing at the accel
    // ODR the whole time this mode is active. A GPIO-interrupt-driven
    // wake (sleep_mode.cpp's diagnostic, replacing RunIdleSleep()'s fixed
    // 1s poll — see plan doc's M9 section) needs INT2 to only pulse on a
    // real tap; left at the default, the ESP32 would wake on every accel
    // sample instead, defeating the whole point. Setting DRDY_DIS=1 here
    // blocks DRDY from INT2 without touching accel sampling itself. Not
    // set when enable=false since normal 6DOF operation never reads
    // INT2 at all currently — harmless either way there, left at its
    // default for now.
    void SetLowPowerAccelOnly(bool enable)
    {
        if (!dev_) return;
        WriteReg(kRegCtrl7, enable ? 0x21 : 0x03);  // accel-only+DRDY_DIS vs accel+gyro; CTRL2/tap CAL registers untouched
    }

    // Wake-on-Motion mode (2026-09-06, M9 — wom-wake-mode branch) —
    // datasheet Section 12. Tried first as the tap engine + light sleep's
    // GPIO wakeup (sleep_mode.cpp), but a real tap's INT2 signal is a
    // brief pulse "synced with DRDY" (Section 10.5, ~1ms at this ODR) —
    // on hardware, ESP_SLEEP_WAKEUP_GPIO never once fired across several
    // real taps, every wake fell through to the 1s timer backstop
    // instead, consistent with the pulse being too short for the
    // level-wakeup detector to catch reliably. WoM is different: Section
    // 12.4 — "For each WoM event, the state of the selected interrupt
    // line is toggled" — from the configured initial value (0 here), a
    // real event drives INT2 to 1 and *holds* it there until the host
    // reads STATUS1 (PollWomEvent() below), not a pulse. That's exactly
    // the level-shaped signal GPIO wakeup needs.
    //
    // Trade-off accepted knowingly: WoM wakes on any sufficiently large
    // accelerometer slope, not specifically a tap — being picked up, the
    // desk being knocked, etc. all wake the device too. Matches this
    // project's "anything that wakes the device comes back paused,
    // requires a real second tap to resume" model (AppController design
    // note 9) reasonably well — a spurious wake just means one extra
    // paused screen, not a false start.
    //
    // EnterWakeOnMotion()/ExitWakeOnMotion() follow the datasheet's own
    // two configuration sequences exactly (Section 12.5 enter, 12.6
    // exit) rather than a single toggle, since the two procedures aren't
    // quite symmetric (exit deliberately leaves CTRL7 disabled — see
    // ExitWakeOnMotion()'s comment). threshold_mg/blanking_samples are
    // starting points (like every other threshold in this file), not a
    // finished tune — watch for both missed wakes (raise threshold too
    // high) and false wakes from desk vibration (too low) on real
    // hardware. CAL1_L/CAL1_H are the same scratch registers
    // ConfigureTap() uses for its own settings, just carrying a different
    // meaning depending on which CTRL9 command follows them — not
    // documented whether the tap engine's own already-latched thresholds
    // survive a WoM configuration cycle in between, so the caller is
    // expected to just re-run ConfigureTap() after ExitWakeOnMotion()
    // rather than assume they did.
    void EnterWakeOnMotion(uint8_t threshold_mg, uint8_t blanking_samples)
    {
        if (!dev_) return;
        WriteReg(kRegCtrl7, 0x00);  // disable all sensors — required before configuring WoM (datasheet 12.5)
        // Explicit CTRL8=0x00 (2026-09-07) — not in the datasheet's own
        // Figure 25 configuration sequence, added after WOM_EDGE_TEST
        // (main.cpp) showed STATUS1.WoM genuinely latching only on real
        // motion (confirmed via a no-touch negative control — see git
        // history) while *neither* physical INT pin ever showed an edge.
        // Section 6 says WoM mode's INT pin behavior "follows the
        // configuration of WoM", implying it overrides whatever CTRL8
        // (still 0x01 — tap engine "enabled" — left over from the last
        // ConfigureTap() call, since this function never otherwise
        // touches CTRL8) says. Testing whether that's actually true on
        // this chip, or whether the tap engine still enabled in CTRL8
        // is what's preventing WoM's own routing from reaching the pin.
        WriteReg(kRegCtrl8, 0x00);
        WriteReg(kRegCal1L, threshold_mg);
        // bits[7:6] select interrupt pin + initial value (Table 39: "01"
        // = INT2/init-0, "11" = INT2/init-1, "00" = INT1/init-0, "10" =
        // INT1/init-1); bits[5:0] is the blanking time in accelerometer
        // samples (max 63, ~63ms at this device's 1000Hz accel ODR) —
        // screens out startup transients right after enabling, same
        // spirit as the external discard loop main.cpp already needs for
        // the tap engine's own CTRL7-toggle quirk (see
        // SetLowPowerAccelOnly()'s comment), but built into the chip
        // this time.
        //
        // 0x40 -> 0x80 -> back to 0x40 (2026-09-07): tried 0x80 (bit7 set
        // instead of bit6) for one round after 0x40 produced zero edges,
        // on the theory the 2-bit field might be read the other way
        // round — that round also produced zero edges, but a second AI
        // model consulted on the whole investigation cross-checked this
        // specific field against both the QST reference driver's own enum
        // (bit6 selects the pin, bit7 the initial value — matching 0x40's
        // reading, not 0x80's) and lewisxhe/SensorLib's macros, and judged
        // 0x40 was the correct encoding all along. Combined with the
        // CTRL1 INT1_EN/INT2_EN finding just above (both INT pins were
        // simply high-Z this whole time, in every round including this
        // one), that fully accounts for round 5 also showing zero edges
        // without the bit-order guess itself having been wrong. Reverted
        // back to 0x40 accordingly, now testing CTRL1's fix in isolation
        // against the encoding this project's own tap-engine-adjacent
        // reasoning already had right the first time.
        WriteReg(kRegCal1H, static_cast<uint8_t>(0x40 | (blanking_samples & 0x3F)));
        // Checked now (2026-09-07) — round 1 of the WOM_EDGE_TEST
        // diagnostic (main.cpp) captured zero edges on IMU_INT2 across 5
        // real taps despite STATUS1.WoM latching by the end, which could
        // mean this handshake silently failed rather than the interrupt
        // just going to the wrong pin. Ruling that in or out directly.
        if (!WriteCommandAndWait(kCmdWriteWomSetting)) {
            printf("Qmi8658::EnterWakeOnMotion: CTRL9 handshake failed\n");
        }
        // Accel-only enable — WoM still needs the accelerometer running
        // internally to evaluate motion (datasheet 12.3), even though "no
        // sensor data is generated" in the normal DRDY sense while in
        // this mode (Section 6). Same accel-only rationale as
        // SetLowPowerAccelOnly(true), not repeated here.
        WriteReg(kRegCtrl7, 0x01);
    }

    void ExitWakeOnMotion()
    {
        if (!dev_) return;
        WriteReg(kRegCtrl7, 0x00);  // disable all sensors (datasheet 12.6)
        WriteReg(kRegCal1L, 0x00);  // 0x00 disables WoM, returns INT pins to normal function (Table 39)
        if (!WriteCommandAndWait(kCmdWriteWomSetting)) {
            printf("Qmi8658::ExitWakeOnMotion: CTRL9 handshake failed\n");
        }
        // Deliberately leaves CTRL7 at 0x00 (sensors disabled) rather
        // than restoring 6DOF here — the caller is expected to
        // immediately call ConfigureTap() afterward (see this method's
        // class-level comment), which starts with its own CTRL7=0x00 and
        // ends by restoring CTRL7=0x03 anyway; toggling CTRL7 off then
        // back on here just to have ConfigureTap() toggle it off again
        // moments later would be pure waste.
    }

    // Polls for a Wake-on-Motion event. STATUS1.bit2 (WoM) — reading
    // STATUS1 clears the bit and resets INT2 back to its configured
    // initial value (datasheet 12.4), the same read-clears shape as
    // PollTapEvent()'s STATUS1.bit1 below.
    bool PollWomEvent()
    {
        if (!dev_) return false;
        uint8_t status1 = 0;
        if (!ReadRegs(kRegStatus1, &status1, 1)) return false;
        if (kQmi8658DebugLogEnabled && status1 != 0) {
            printf("Qmi8658::PollWomEvent: STATUS1=0x%02X\n", status1);
        }
        return (status1 & 0x04) != 0;
    }

    // Polls for a new tap event. TAP_STATUS (0x59) holds the *type* of the
    // most recent tap, but its value doesn't change between two same-type
    // taps in a row, so diffing it directly misses repeats — that was this
    // function's first version, and it's why only one tap ever registered.
    // STATUS1 bit1 is the actual "a new tap happened since last check"
    // signal (inferred from SensorLib's update(), which polls STATUS1
    // this way in a loop and correctly counts repeated taps — not
    // confirmed against the datasheet directly, but the alternative
    // matches observed behavior exactly). Only read TAP_STATUS for the
    // single/double detail once STATUS1 says a new event is there.
    TapEvent PollTapEvent()
    {
        if (!dev_) return TapEvent::kNone;

        uint8_t status1 = 0;
        if (!ReadRegs(kRegStatus1, &status1, 1)) return TapEvent::kNone;
        // Diagnostic (2026-09-06, M9 idle-sleep debugging; gated behind
        // kQmi8658DebugLogEnabled 2026-09-07) — print the raw byte
        // whenever anything in it is set, not just the TAP bit we act on,
        // so a run that spuriously wakes RunIdleSleep() shows exactly
        // what STATUS1 actually looked like at that moment instead of us
        // continuing to guess. Cheap to leave in even when enabled:
        // status1 reads all-zero the overwhelming majority of the time in
        // normal use, so this doesn't spam the log outside of taps/
        // whatever this turns out to be.
        if (kQmi8658DebugLogEnabled && status1 != 0) {
            printf("Qmi8658::PollTapEvent: STATUS1=0x%02X\n", status1);
        }
        if ((status1 & 0x02) == 0) return TapEvent::kNone;

        uint8_t tap_status = 0;
        if (!ReadRegs(kRegTapStatus, &tap_status, 1)) return TapEvent::kNone;
        if (kQmi8658DebugLogEnabled) {
            printf("Qmi8658::PollTapEvent: TAP bit set, TAP_STATUS=0x%02X\n", tap_status);
        }

        switch (tap_status & 0x03) {
            case 1: return TapEvent::kSingle;
            case 2: return TapEvent::kDouble;
            default: return TapEvent::kNone;
        }
    }

private:
    static constexpr uint8_t kRegCtrl1 = 0x02;
    static constexpr uint8_t kRegCtrl2 = 0x03;
    static constexpr uint8_t kRegCtrl3 = 0x04;
    static constexpr uint8_t kRegCtrl5 = 0x06;
    static constexpr uint8_t kRegCtrl7 = 0x08;
    static constexpr uint8_t kRegCtrl8 = 0x09;
    static constexpr uint8_t kRegCtrl9 = 0x0A;
    static constexpr uint8_t kRegCal1L = 0x0B;
    static constexpr uint8_t kRegCal1H = 0x0C;
    static constexpr uint8_t kRegCal2L = 0x0D;
    static constexpr uint8_t kRegCal2H = 0x0E;
    static constexpr uint8_t kRegCal3L = 0x0F;
    static constexpr uint8_t kRegCal3H = 0x10;
    static constexpr uint8_t kRegCal4H = 0x12;
    static constexpr uint8_t kRegStatusInt = 0x2D;
    static constexpr uint8_t kRegStatus1 = 0x2F;
    static constexpr uint8_t kRegTapStatus = 0x59;
    static constexpr uint8_t kRegAxL = 0x35;
    static constexpr uint8_t kRegReset = 0x60;
    static constexpr uint8_t kRegResetResult = 0x4D;

    static constexpr uint8_t kCmdAck = 0x00;
    static constexpr uint8_t kCmdConfigureTap = 0x0C;
    static constexpr uint8_t kCmdWriteWomSetting = 0x08;

    static constexpr float kAccelLsbPerG = 4096.0f;   // +-8g range
    static constexpr float kGyroLsbPerDps = 64.0f;    // +-512dps range (32768/512), see CTRL3=0x53 above

    void WriteReg(uint8_t reg, uint8_t value)
    {
        const uint8_t payload[2] = {reg, value};
        i2c_master_transmit(dev_, payload, sizeof(payload), 50);
    }

    bool ReadRegs(uint8_t reg, uint8_t* buf, size_t len)
    {
        return i2c_master_transmit_receive(dev_, &reg, 1, buf, len, 50) == ESP_OK;
    }

    // CTRL9 host command handshake: write the command, wait for the
    // "done" bit (STATUS_INT bit7), ack it, wait for the bit to clear.
    bool WriteCommandAndWait(uint8_t cmd)
    {
        WriteReg(kRegCtrl9, cmd);
        if (!WaitForStatusIntBit(true)) return false;
        // (2026-09-07: this used to print gpio_get_level(GPIO_NUM_47) here
        // as a zero-risk check of CTRL1.bit3/INT1_EN, per datasheet 6.2's
        // "host can check the INT1 pin high level for the handshake" — it
        // read 1, confirming the enable-bit theory before the WOM_EDGE_TEST
        // round that settled it for real (107 edges on GPIO48, correlated
        // with 5 real taps). Removed now that its question is answered;
        // see the constructor's CTRL1 comment and EnterWakeOnMotion()'s
        // CAL1_H comment for the full writeup.)
        WriteReg(kRegCtrl9, kCmdAck);
        return WaitForStatusIntBit(false);
    }

    bool WaitForStatusIntBit(bool want_set)
    {
        for (int i = 0; i < 200; ++i) {  // ~200ms timeout at 1ms/iteration
            uint8_t val = 0;
            if (!ReadRegs(kRegStatusInt, &val, 1)) return false;
            if (((val & 0x80) != 0) == want_set) return true;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        return false;
    }

    i2c_master_bus_handle_t bus_ = nullptr;
    i2c_master_dev_handle_t dev_ = nullptr;
};
