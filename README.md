# Echometer

A pocket-sized, battery-powered **ultrasonic bat detector**. It listens for
bat calls above 19 kHz, and when it hears one it records a 3.84-second
full-spectrum clip at **256 ksps** into PSRAM and serves it over its own Wi-Fi
network. Open a phone browser, download the WAV, and import it straight into
the Wildlife Acoustics **Echo Meter** app — the files carry GUANO metadata and
are accepted as Echo Meter Touch 2 Pro recordings.

No SD card, no cables, no custom phone app, no laptop in a field at midnight.

This is **v4**, the fourth board in the series. It is the first one that is
genuinely portable and the first one that doesn't clip on a close bat: the
analog front end has **six switchable gain steps from 22× to 782×**, chosen
from the web page or tracked automatically, and the whole thing runs off a
protected LiPo with charger and boost on board. See
[Design history](#design-history) for how it got here.

---

## Contents

- [Specifications](#specifications)
- [How it works](#how-it-works)
- [Repository layout](#repository-layout)
- [Getting started](#getting-started)
- [Using it](#using-it)
- [Hardware in brief](#hardware-in-brief)
  - [Switchable gain](#switchable-gain)
  - [Power](#power)
  - [Pinout](#pinout)
- [Firmware in brief](#firmware-in-brief)
- [Errata — read before ordering the PCB](#errata--read-before-ordering-the-pcb)
- [Design history](#design-history)
- [Roadmap](#roadmap)
- [References](#references)

---

## Specifications

| | |
|---|---|
| Detection band | 19 – 128 kHz |
| Sampling | 256 ksps, 12-bit, external SAR ADC (ADCS7476) |
| Frequency resolution | 125 Hz/bin (2048-point FFT, 8 ms frames) |
| Analog gain | 6 steps, **22× – 782×** (26.9 – 57.9 dB), manual or AGC |
| Microphone | Knowles SPU0410LR5H-QB MEMS, −38 dBV/Pa, usable to ≈ 80 kHz |
| Clip length | 3.84 s (983 040 samples, 1.97 MB) |
| Clip storage | up to 8 PSRAM slots — 4 on an 8 MB module; RAM-only, lost at power-off |
| Transfer | open Wi-Fi AP `echometer` → `http://192.168.4.1` |
| File format | 16-bit mono WAV + GUANO chunk, `CHIRP_YYYYMMDD_HHMMSS.wav` |
| Compute | ESP32-S3 DevKitC-1, octal PSRAM, ESP-IDF ≥ 5.2 |
| Power | 2000 mAh LiPo, TP4056 charger @ 1 A, DW01/FS8205A protection, MT3608 boost |
| PCB | 4-layer, KiCad 10, gerbers in [`hardware/fab/`](hardware/fab/) |

## How it works

```text
                        ┌── 74HC4053 mux ──┐
                        │  (3 × GPIO)      │
                        ▼                  ▼
  ┌────────┐      ┌───────────┐      ┌──────────┐      ┌───────────┐
  │ conical│      │  OPA838   │      │  TL972   │      │ ADCS7476  │
──│  horn  │─────▶│  ×4.25/34 │─────▶│ ×5.2…23  │─────▶│ 12-bit    │
  │        │ MEMS │  stage 1  │  HP  │ stage 2  │  AA  │ 256 ksps  │
  └────────┘  mic └───────────┘      └──────────┘      └─────┬─────┘
                                                             │ I²S RX DMA
                                                             ▼
   Safari ◀── HTTP ◀── SoftAP ◀── PSRAM clip ◀── trigger ◀── 2048-pt FFT
     │                             4 × 3.84 s      >19 kHz    (esp-dsp)
     ▼                                             narrowband
  Echo Meter app                                   3 frames
```

1. Press **BOOT** to arm. The LED flashes green whenever ultrasound is present.
2. Every 2048-sample buffer is FFT'd. A detection needs a **narrowband** peak
   above 19 kHz that **persists** across 3 consecutive frames — broadband
   transients (a finger on the board, contact noise) are rejected.
3. On detection the clip streams into a PSRAM slot for 3.84 s. The LED goes
   completely silent during the write — its own drive current used to print
   evenly-spaced clicks into the spectrogram.
4. Blue flash when saved, then back to idle. Join Wi-Fi `echometer`, browse to
   `http://192.168.4.1`, download or delete clips, and set the gain.

The interesting trick is the ADC clocking. The ESP32-S3's internal ADC-DMA path
tops out near 83 ksps, nowhere near enough for a bat. So an external SAR ADC is
driven by the **I²S peripheral pretending to be an SPI master**: in 16-bit
stereo master mode the WS line toggles every 16 bit-clocks, so WS becomes the
ADC's `CS#` and BCK its `SCLK`. RX DMA captures every sample with zero
per-sample CPU, jitter-free, at an 8.192 MHz bit clock.

## Repository layout

| Path | Contents |
|---|---|
| [`hardware/`](hardware/) | KiCad 10 project — schematic, 4-layer PCB, gerbers, BOM, horn generator. See [hardware/README.md](hardware/README.md) for the analog design. |
| [`firmware/`](firmware/) | ESP-IDF application. See [firmware/README.md](firmware/README.md) for the detection logic and bring-up lessons. |
| [`firmware/main/archive/`](firmware/main/archive/) | Earlier firmware variants kept for reference — v3's original, an SD-card recorder, a board bring-up test. |

## Getting started

**You need:** an ESP32-S3-DevKitC-1 with **octal** PSRAM, the assembled v4
board, a 2000 mAh LiPo with a JST-PH lead, and ESP-IDF ≥ 5.2.

```bash
cd firmware
. ~/software/esp-idf/export.sh              # adjust to your ESP-IDF path
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor        # the COM (UART) USB-C port
```

The DevKitC-1 has two USB-C ports. **COM** (CP2102N → UART0) does flashing and
the log console and always works. If a bad flash ever bricks the app: hold
BOOT, tap RESET, release → ROM download mode, reflash over COM.

Before you build the board, read
[Errata](#errata--read-before-ordering-the-pcb) — item 1 is blocking.

## Using it

| Action | What happens |
|---|---|
| Press BOOT | Arm. LED flashes green while ultrasound is present. |
| Detection | LED silent, 3.84 s clip written to PSRAM, then blue flash. |
| Press BOOT while armed | Disarm back to idle. Presses during a write are ignored. |
| Join `echometer` (open network) | AP is up in every state. |
| `http://192.168.4.1` | Clip list, newest first: download, delete, set gain, toggle AGC. |

Clips live in RAM only — **they are lost at power-off**. Download before you
disconnect the battery.

To get a clip into the Echo Meter app on iOS: download in Safari, then
Files → share sheet → *Copy to Echo Meter*.

---

## Hardware in brief

Everything below is the summary. Full derivations — op-amp selection, mic
specs, the input common-mode budget, every filter corner, the power design,
the pinout, the acoustic horn and the bat call-level budgets — are in
[hardware/README.md](hardware/README.md).

Single channel, single 3.3 V analog rail, everything biased to mid-rail
(VREF ≈ 1.65 V). Stage 1 is a **TI OPA838** — 300 MHz GBW, decompensated,
1.8 nV/√Hz — because gain × 120 kHz has to fit inside the gain-bandwidth
product. Stage 2 is a **TI TL972**, rail-to-rail in and out, which drives the
ADC to full swing on 3.3 V. Both stages are non-inverting with a 1 kΩ leg to
VREF. The band is ≈ 19 kHz – 128 kHz: two 10 nF high-passes stacked at the
bottom, a 72 kHz RC anti-alias filter ahead of the ADC at the top.

### Switchable gain

A **74HC4053** analog mux switches extra feedback resistors **in parallel**
with a permanent one at each stage's summing node — one switch on stage 1, two
on stage 2. Parallel resistors *lower* the gain, the permanent resistors mean
the loop is never open, and the mux's Y0-when-LOW polarity means the state the
ESP32's GPIOs sit in at reset is **minimum gain, which cannot clip**.

| **Total** | dB | Clips near | Use |
|---|---|---|---|
| **22×** | 26.9 | 105 dB SPL | cave / roost, bats at arm's length |
| **51×** | 34.2 | 98 dB SPL | |
| **98×** | 39.8 | 92 dB SPL | general close work |
| **210×** | 46.4 | 86 dB SPL | default at cold boot |
| **408×** | 52.2 | 80 dB SPL | |
| **782×** | 57.9 | 74 dB SPL | distant passes, ~30 m |

Steps are 5.6–7.3 dB apart. "Clips near" is the input SPL at which the TL972
output hits its rail. The range stops at 782× because past ~800× you are
amplifying the microphone's own noise rather than signal — further reach has to
come from narrowband processing, not from the front end.

Resistor values, the switch-state table, why the three switches are split 1/2
across the stages, and why the switches sit at the summing node:
[hardware/README.md](hardware/README.md#gain--how-the-switching-works).

### Power

USB charger → cell → protection → boost to 5.31 V for the module, with the
analog rail taken off separately. The one decision worth naming here: **the
analog LDO is fed from the battery, not from the boost.** The TPS74801's
separate BIAS pin runs its control loop from +5 V so the pass FET can be driven
above VIN, which lets IN sit at the raw cell voltage. The win isn't efficiency,
it's noise — the analog rail never sees the MT3608's 1.2 MHz ripple, which no
LDO's PSRR is good at rejecting.

Two things to know before you use it:

- **Charging while running works**, but the load current means the TP4056 may
  never reach its termination threshold, so it can sit in CV at 4.2 V and the
  green LED may never come on. Don't leave it plugged in for days.
- **There is no power switch.** The board runs from the moment the cell is
  connected — unplug J6 to stop it. See errata 5 to store it charged.

Rail diagram, component values, the 5.31 V boost-margin calculation and where
to add a switch if you want one: [hardware/README.md](hardware/README.md#power).

### Pinout

ADC on GPIO 4 / 5 / 6 (I²S WS / BCK / DIN), gain select on GPIO 38 / 37 / 36
(⚠ **see errata 1**), RGB LED on 48, BOOT button on 0. Full header-pin table:
[hardware/README.md](hardware/README.md#pinout--esp32-s3-devkitc-1).

---

## Firmware in brief

ESP-IDF application, single `main.c`. Full detail — state machine, WAV/GUANO
construction, sdkconfig requirements and the bring-up lessons — is in
[firmware/README.md](firmware/README.md).

| State | Entered by | LED |
|---|---|---|
| IDLE | boot / disarm | off |
| DETECT | BOOT press | green flash while ultrasound present |
| RECORD | detection persists 3 frames | **silent** for the 3.84 s write |
| after RECORD | write finished | blue flash ~1.2 s → IDLE |

**Detection.** 2048-point real FFT per buffer via esp-dsp. Trigger requires, on
3 consecutive buffers: a bin above 19 kHz (bin ≥ 152) over
`ULTRASONIC_THRESH_COUNTS`, **and** a peak ≥ 20× the average power of the rest
of the >19 kHz band. The tonality test is what rejects broadband impulses; the
persistence test is what rejects momentary ones.

**Gain control.**

- **Web:** `http://192.168.4.1` → tap a gain. Picking one by hand turns the AGC
  off, and the choice is saved to NVS so it survives a power cycle.
- **AGC:** fast attack, slow decay. Any frame peaking above 90 % of full scale
  drops the gain one step immediately; the gain only steps back up after ~1 s
  (120 frames) below 10 %. Asymmetric on purpose — clipping happens in the
  **analog** domain before the ADC, so a clipped call is unrecoverable, while a
  symmetric AGC would pump on every gap between calls.
- **Frozen during RECORD.** A gain step mid-clip would put a discontinuity in
  the waveform and make the clip useless for absolute amplitude. The gain in
  force is latched at trigger time and written into the clip's GUANO metadata
  (`Note` plus `Echometer|Gain`), so SPL can be reconstructed offline.
- After every gain change the detector skips 2 frames while the mux's charge
  injection settles out of the summing node, so a switch is never mistaken for
  a call.

**HTTP endpoints:** `/` (clip list), `/dl`, `/del`, `/gain?i=N`, `/agc?on=0|1`.

**A note on the device identity in the metadata.** The Echo Meter app validates
the GUANO `Model` field against known Wildlife Acoustics modules and rejects
anything else, so the firmware writes `Model: Echo Meter Touch 2 Pro` — a
format this recorder genuinely matches (256 kHz, 16-bit, full-spectrum mono).
True provenance is kept in the GUANO `Note` field. Keep that in mind before
sharing files anywhere that trusts the device tag.

---

## Errata — read before ordering the PCB

Known problems with the v4 board as drawn. **Item 1 is blocking** — fix it in
the schematic before you send gerbers. The rest are things to know before you
build, not stop-ships.

**1. Two gain-select signals land on the octal-PSRAM bus.** With
`CONFIG_SPIRAM_MODE_OCT=y`, GPIO35/36/37 belong to the PSRAM controller and
cannot be used for anything else, but the PCB routes Gain S2 to J12.11
(GPIO37) and Gain S3 to J12.12 (GPIO36). The firmware already assumes the
corrected pins — `PIN_GAIN_S1/S2/S3` are GPIO 38/39/40 — so an uncorrected
board will not switch gain.
**Fix:** move S2 to J12.9 (GPIO39) and S3 to J12.8 (GPIO40). GPIO39/40 are
MTCK/MTDO, free here only because the DevKitC-1 debugs over the built-in
USB-Serial-JTAG on GPIO19/20. See
[firmware/README.md](firmware/README.md#5-gpio-choice-is-constrained-by-psram-mode).

**2. Stage 1's lowest gain step sits below the OPA838's minimum stable gain.**
The OPA838 is decompensated, with a minimum stable gain of +7; stage 1's low
setting is 4.25. The design relies on C14 (27 pF across R13) to provide the
compensation the topology needs, and 27 pF is small enough that stray
capacitance matters. If the three low gain steps ring, peak or oscillate,
this is the first place to look.

**3. The +5 V rail has ~390 mV of bias margin, and it needs verifying.** The
boost is set to 5.31 V because the TPS74801's BIAS pin must stay at least
1.62 V above the 3.3 V output, i.e. ≥ 4.92 V. Measure the sag on a real board:
if +5 V dips below 4.92 V during a Wi-Fi TX burst, the LDO loses bias headroom
and the analog rail glitches during exactly the moments you transmit.

**4. The SD footprint (J5) is +5 V and unbuffered.** Pin 5 is fed +5 V and the
SPI lines are not level-shifted, so J5 accepts an SD **module** with its own
regulator and level shifters — never a bare card socket. The current firmware
does not use it; leave it unpopulated unless you are building the archived
recorder.

**5. There is no power switch, and the cell will over-discharge in storage.**
The boost's EN is tied to its own input, so the board runs from the moment the
cell is connected. Standby drain as built is 50–120 mA, which flattens
2000 mAh in 17–40 hours and then parks the cell at the DW01's 2.4 V cutoff —
bad for lithium.
**Fix:** unplug J6 for storage and store the cell charged. If you add a switch,
put it in the `Lipo+ → L1 / U8.5` branch, **not** on the boost's EN pin — the
MT3608 is asynchronous, so `Lipo+ → L1 → D3 → +5V` is a permanent DC path that
survives disabling the boost. See
[hardware/README.md](hardware/README.md#charging-and-switching-both-with-caveats).

**6. Several schematic symbols carry stale part numbers.** R13, R23/R25, R24
and R26 inherit a 24 kΩ `Part#` / `LCSC Part #` from their previous value, and
C16 inherits a 10 nF one. Those fields are cleared in the checked-in
[`hardware/v4.csv`](hardware/v4.csv), so **ordering from the CSV is safe** —
but fix the symbol properties before the next `kicad-cli sch export bom`.

---

## Design history

Four boards, each one built to answer the question the previous one raised.

| | v1 | v2 | v3 | v4 |
|---|---|---|---|---|
| **Compute** | STM32F401CBU6 | STM32F446RET6 (Nucleo) | ESP32-S3 | ESP32-S3 |
| **Sampling** | internal ADC, free-running DMA | internal ADC, TIM2-gated **240 ksps** | external ADCS7476 via I²S, **256 ksps** | same |
| **Detection** | none — raw recorder | CMSIS-DSP FFT, >19 kHz threshold | + tonality + persistence | same |
| **Storage** | micro-SD (SPI1, FatFS) | micro-SD (SPI3, FatFS) | 4 × PSRAM slots | up to 8 slots |
| **Retrieval** | pull the card / USB CDC | pull the card / UART + Python | **Wi-Fi + Safari + GUANO WAV** | same |
| **Gain** | analog board, fixed | fixed 121× | fixed 625× | **6 steps, 22×–782× + AGC** |
| **Listening** | heterodyne + FD to a speaker | — | — | — |
| **Power** | 9 V in, LM2574 + LDOs | dev-board USB | USB (browned out) | **LiPo + charger + protection + boost** |

### v1 — does the analog even work?

A custom STM32F401CBU6 mainboard plus a separate analog board carrying the
classic hobbyist bat-detector chain: SPU0410 MEMS mic → LM358 preamp → CD4066
mixer switched by a TLC555 → LM386 → speaker, i.e. **heterodyne and frequency
division straight to your ears**. Two small preamp boards (one OPA838, one
TL972) were spun alongside it to compare op-amps on real signals.

The MCU side ran FreeRTOS with three tasks: ADC1 in continuous DMA into a
double buffer, a ring buffer feeding FatFS on a micro-SD over SPI1, and a
debounced button. Power was 9 V in through an LM2574 buck and NCP718/NCP163
LDOs.

**What it proved:** the microphone, the horn and the amplifier chain all work,
and you can hear bats. **What it cost:** hearing a bat isn't recording one —
the heterodyne path destroys the signal it demodulates. There was no detection
logic at all, so recording was start/stop by hand. The ADC free-ran on a
software start with no timer trigger, so the sample clock had jitter. And
splitting analog and digital across two boards joined by 4-pin headers was an
invitation to pickup.

**→ v2:** drop the speaker path entirely, go full-spectrum digital, put the ADC
on a hardware time base, and add a real FFT detector.

### v2 — detect it, don't just hear it

A Nucleo-F446RE with a custom analog + SD daughterboard: SPU0410 →
**OPA838 → TL972**, single 3.3 V rail, mid-rail bias — the two-stage topology
that survives into v4. TIM2 TRGO gates ADC1 at exactly **240 ksps** into a
2 × 2048 DMA double buffer, and each half-buffer goes through
`arm_rfft_fast_f32` from CMSIS-DSP: 117 Hz/bin, trigger on any bin ≥ 163
(>19 kHz) above 50 counts. Detect and record are two states with different LED
blink rates; recording writes raw `uint16` samples to FatFS over SPI3 with an
`f_sync` every 100 writes. Python tools (`bat_record.py`, `bat_viewer.py`,
`bat_waveform.py`) pull captures over the serial port and plot them.

Gain went from 11× per stage (**121×** total) to 25× per stage (**625×**)
during bring-up. At 121× real calls sat barely above the noise floor; the 5×
jump is what moved detection from marginal to reliable.

**What it cost:** 240 ksps means a 120 kHz Nyquist, and that was the internal
ADC's practical ceiling — the target species here, *Rhinolophus pusillus*, calls
at ~108 kHz, right at the edge. Getting files off meant pulling the card or
dumping over serial. And the gain was still a soldered constant.

**→ v3:** an external ADC fast enough to have real margin above 108 kHz, and a
way to get files onto a phone in the field.

### v3 — fast enough, and wireless

ESP32-S3 with an external **ADCS7476** 12-bit SAR clocked by the I²S
"WS-as-CS" trick at **256 ksps** — zero per-sample CPU, 128 kHz Nyquist.
Detection moved to esp-dsp and gained the two tests that make it usable
outdoors: **tonality** (peak bin ≥ 20× the mean power of the rest of the band)
and **persistence** (3 consecutive frames). Clips go to PSRAM — four 3.84 s
slots in 8 MB — and are served over a SoftAP as WAVs with a **GUANO** metadata
chunk that the Echo Meter iPhone app accepts. Two mics and two full channels,
fixed 625×.

That combination is what made it a real detector: no card, no cable, and a
phone app that already knows how to draw a spectrogram.

**What it cost, and this is the whole reason v4 exists:**

- **625× clipped constantly.** Roughly 9 out of 10 close calls railed the
  second stage, smearing the spectrogram into broadband bars and false
  harmonics. Clipping happens in the analog domain, so no amount of digital
  processing recovers it — and there was no way to turn the gain down.
- **It browned out.** Wi-Fi RF calibration and TX bursts (300–400 mA on top of
  octal PSRAM) tripped the brownout detector on marginal USB supplies. Inline
  USB "noise filter" dongles made it *worse* — series resistance is exactly the
  problem. Downloads, being the heaviest sustained RF load, failed first.
- **A floating ADC input false-triggered on touch.** Fixed in firmware by the
  tonality and persistence tests; the hardware fix was left for the next spin.
- **The LED printed clicks into the spectrogram.** The WS2812's RMT refresh
  coupled into the front end, so the firmware learned to keep it silent for the
  whole capture window.
- **The mic-supply filter did nothing.** 10 Ω + 110 nF corners at 159 kHz —
  above the band it was supposed to clean.

**→ v4:** make the gain adjustable, and stop feeding a 400 mA radio through a
thin cable.

### v4 — portable, and it doesn't clip

| Area | v3 | v4 |
|---|---|---|
| Gain | fixed 625× | **6 steps, 22×–782×**, web-selectable + AGC |
| Gain control | — | 74HC4053 mux at both summing nodes, 3 GPIOs |
| Channels | 2 mics, 2× OPA838 + 2× TL972 | 1 mic, 1× OPA838 + 1× TL972 — gain replaces the second channel |
| Power source | USB only | LiPo 2000 mAh + charger + protection + boost |
| Charging | — | TP4056 @ 1 A, red/green status LEDs |
| Cell protection | — | DW01 + FS8205A, low-side |
| 3V3 analog rail | LDO from 5 V (boost output) | LDO **from the battery**, bias from 5 V |
| Mic supply filter | 10 Ω + 110 nF (159 kHz — did nothing) | **100 Ω + 10 µF (159 Hz)** |
| Anti-alias | 100 Ω + 10 nF (159 kHz, above Nyquist) | **100 Ω + 22 nF (72 kHz)** |
| Wi-Fi burst reservoir | external | **470 µF on board** at the module header |
| Floating ADC input | v3 erratum — false triggers | driven from the op amp through 100 Ω; can't float |
| Clip metadata | timestamp only | **gain recorded in GUANO** |

The trade worth naming: v4 gives up v3's second channel. Two mics at two fixed
gains is the other classic way to get wide dynamic range — digitise both, keep
whichever didn't clip, zero AGC artifacts. One switched channel costs a mux and
some AGC logic but buys 31 dB of range instead of one fixed step, and it frees
the board area that paid for the battery.

## Roadmap

Carried forward, in rough order of value:

- **Ring buffer for pre-trigger audio.** The clip currently starts at the
  triggering buffer, so the calls *before* detection fired are lost. This is
  the single biggest capture upgrade left — every serious passive recorder
  keeps a few seconds of pre-roll.
- **Adaptive trigger.** A running noise-floor estimate (exponential moving
  average of ultrasonic-band energy) rather than the fixed
  `ULTRASONIC_THRESH_COUNTS`. Note that with variable gain a fixed threshold
  now means a *different* absolute sensitivity at each step, which makes this
  more pressing than it was in v3.
- **Call-shape gating.** Require a swept FM component, plausible duration
  (~2–20 ms) and inter-pulse interval, to reject bush-crickets and machinery.
  The tonality + persistence test is a first cut at this.
- **Duty cycling** for unattended overnight runs — light sleep between checks,
  or a low-power always-on trigger path that wakes full recording on a hit.
- **Pause the AP during capture**, so no TX spike or RF pickup can coincide
  with the 3.84 s sampling window.

For reference, AudioMoth uses a plain amplitude threshold; Song Meter /
Kaleidoscope use band-limited energy triggers; Anabat uses zero-crossing
frequency division; and the most advanced run an on-device CNN
(e.g. BatDetect2) to detect and classify in real time.

## References

- [GUANO metadata specification](https://guano-md.org)
- TI [OPA838](https://www.ti.com/lit/gpn/opa838), TL972, ADCS7476, TPS74801 datasheets
- Knowles SPU0410LR5H-QB datasheet and Ultrasonic Application Note AN17
- [AudioMoth](https://www.openacousticdevices.info/) — reference low-power design
- [BatDetect2](https://github.com/macaodha/batdetect2) — CNN call detection and classification
