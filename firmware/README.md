# Firmware — ESP32-S3 chirp catcher

ESP-IDF (≥ 5.2) application. On a button press it arms an FFT-based ultrasonic
detector; on detection it records a 3.84 s clip at 256 ksps into PSRAM and
serves it as an Echo-Meter-compatible WAV over a Wi-Fi SoftAP. The analog gain
is switched from the same web page, or tracked automatically.

Everything lives in [`main/main.c`](main/main.c).

## Build & flash

```bash
. ~/software/esp-idf/export.sh   # ESP-IDF >= 5.2
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

The DevKitC-1 has two USB-C ports: **COM** (CP2102N → UART0) is used for
flashing *and* the log console and always works; **USB** (native OTG,
GPIO19/20) is unused by the current firmware. If a bad flash ever bricks the
app: hold BOOT, tap RESET, release — ROM download mode, reflash over COM.

Key sdkconfig settings (already in the checked-in `sdkconfig`):

| Setting | Value | Why |
|---|---|---|
| `CONFIG_SPIRAM` + `_MODE_OCT` + `_SPEED_80M` | y | octal PSRAM for clip slots (quad modules: use `_MODE_QUAD`, 2 MB → 1 slot) |
| `CONFIG_ESP_CONSOLE_UART_DEFAULT` (UART0) | y | console on the COM port |
| `CONFIG_PARTITION_TABLE_CUSTOM` | `partitions.csv` | 4 MB factory app |

Wi-Fi TX power is left at the default max — see lesson 2 for why the earlier
cap was removed.

## State machine & LED

| State | Entered by | LED |
|---|---|---|
| IDLE | boot / disarm | off |
| DETECT (armed) | BOOT button press | off while quiet, **green flash** while ultrasonic is being detected |
| RECORD | detection persists 3 FFT frames | **off/silent** for the 3.84 s write (see below) |
| after RECORD | write finished | **blue flash** ~1.2 s, then off → IDLE |
| error (no PSRAM slot, …) | — | red blink |

**Why the LED is off during RECORD:** the WS2812 is driven over RMT on GPIO48,
and every refresh is a pulse burst that couples into the high-gain analog front
end — a periodic LED refresh during capture printed evenly-spaced broadband
clicks into the spectrogram. So the LED only ever refreshes *outside* the
sampling window: green while detecting (before the clip), silent during the
write, blue flash after. `LED_OFF` touches the strip once on entry then stays
completely idle.

Pressing the button in DETECT disarms back to IDLE; presses during RECORD are
ignored. Wi-Fi (`echometer`, open, `http://192.168.4.1`) is up in every state.

## Signal path

- **ADC via I²S "WS-as-CS".** The S3's internal ADC tops out around 83 ksps,
  far too slow. An external ADCS7476 (12-bit SAR) is clocked by the I²S
  peripheral in 16-bit stereo master mode — WS toggles every 16 bit-clocks and
  drives the ADC's CS, BCK drives SCLK, and RX DMA captures SDATA with zero
  per-sample CPU. At 256 ksps the bit clock is 8.192 MHz; usable Nyquist is
  128 kHz. One 16-bit sample per WS period, so the "left" slot is the data and
  the right is discarded.
- **Bit alignment.** The ADC emits 4 leading zeros then D11..D0, and standard
  I²S delays MSB by one BCK, so the word arrives shifted. Mask and shift in
  firmware rather than fighting the peripheral.
- **Detection.** 2048-point real FFT (esp-dsp) per buffer — 125 Hz/bin, 8 ms
  per frame. Trigger requires, on `DETECT_CONSEC` = 3 consecutive buffers: a
  bin above 19 kHz (bin ≥ 152) over `ULTRASONIC_THRESH_COUNTS`, **and** a peak
  ≥ `CHIRP_TONALITY_MIN` = 20× the average power of the rest of the >19 kHz
  band. See lesson 3 for why both tests exist.
- **Clips.** `CLIP_BUFS` = 480 buffers = 983 040 samples = 3.84 s = 1.97 MB.
  Slots are allocated from PSRAM at boot — up to `MAX_CLIP_SLOTS` = 8, stopping
  when free PSRAM would drop below `SPIRAM_RESERVE` = 512 kB, so an 8 MB module
  yields four. Oldest is overwritten. **Clips are RAM-only and lost at
  power-off.**
- **WAV serving.** 44-byte PCM header + samples + a trailing **GUANO** metadata
  chunk; filenames `CHIRP_YYYYMMDD_HHMMSS.wav`. There is no RTC on board — the
  clock is seeded from the firmware build time at boot.

## Gain control

New in v4. Three GPIOs drive a 74HC4053 that switches feedback resistors in
parallel at both amplifier stages; see
[../hardware/README.md](../hardware/README.md) for the analog side.

| | |
|---|---|
| `PIN_GAIN_S1/S2/S3` | GPIO **38 / 39 / 40** (note: the PCB routes S2/S3 to 37/36 — see errata 1 in the top-level README) |
| `GAIN_STEPS[]` | the six-step table, each with its 4053 levels, total gain and approximate clipping SPL |
| `GAIN_DEFAULT` | index 3 = 210× — a sane cold start |
| `gain_apply()` / `gain_init()` | drives the three GPIOs; restores the saved step from NVS |
| `agc_update()` | fast-attack / slow-decay tracker, returns true when it moved |
| `/gain?i=N` | set a step, and turn the AGC off |
| `/agc?on=0\|1` | toggle the AGC |
| `clip_slot_t.gain` | latched at trigger, shown in the clip list, written to GUANO |

**The AGC is asymmetric on purpose.** Any frame peaking above
`AGC_ATTACK_LEVEL` (90 % of full scale) drops the gain one step *immediately*;
the gain only steps back up after `AGC_DECAY_FRAMES` = 120 frames (~1 s) below
`AGC_DECAY_LEVEL` (10 %). Clipping happens in the analog domain, before the
ADC, so a clipped call is unrecoverable and worth reacting to instantly — while
a symmetric AGC would pump on every gap between calls.

**The gain is frozen during RECORD.** A step mid-clip would put a discontinuity
in the waveform and make the clip useless for absolute amplitude. The step in
force is latched at trigger time and written into GUANO (`Note` plus a
namespaced `Echometer|Gain` field), so SPL can be reconstructed offline.

**Settling.** After every gain change the detector skips `GAIN_SETTLE_FRAMES`
= 2 frames while the mux's charge injection settles out of the summing node,
so a switch is never mistaken for a call.

**Persistence.** The chosen step and the AGC on/off flag are stored in NVS
under the `echometer` namespace, so a hand-picked gain survives a power cycle.

## HTTP interface

| Endpoint | Purpose |
|---|---|
| `/` | clip list, newest first, with gain shown per clip; gain and AGC controls |
| `/dl?…` | download a clip as WAV |
| `/del?…` | delete a clip |
| `/gain?i=N` | select gain step N (0–5); implicitly disables the AGC |
| `/agc?on=0\|1` | enable/disable the AGC |

The server uses 30 s send/receive timeouts so a brief link stall doesn't kill a
2 MB download.

## `main/archive/`

Older firmware variants, kept for reference (not built):

- `main_v3.c` — the unmodified v3 application, before the gain layer. Useful as
  a diff base.
- `main_sd.c` — the original SD-card / USB-CDC recorder (detect → record raw
  samples to FatFS or stream over TinyUSB CDC). Needs `sdmmc fatfs vfs` in
  CMake REQUIRES plus the `espressif/esp_tinyusb` managed component, TinyUSB
  CDC enabled in menuconfig, and a **FAT32** (not exFAT) SD card.
- `main_test.c` — minimal board bring-up test (button → sample + FFT → green
  LED on detect, ADC stats on the console).

To build one, copy it over `main/main.c` and restore the dependencies noted in
`main/CMakeLists.txt`.

---

## Lessons learned

### 1. Getting WAVs accepted by the Echo Meter iPhone app

- A bare PCM WAV is **rejected**. The app wants **GUANO metadata**
  ([guano-md.org](https://guano-md.org)): a `guan` RIFF chunk with `Timestamp`,
  `Samplerate`, `Length`, `Make`, `Model`.
- With honest `Make: DIY / Model: echometer-v4` the import fails with
  *"invalid module type"* — the app validates `Model` against known Wildlife
  Acoustics modules. Identifying as `Echo Meter Touch 2 Pro` (whose format —
  256 kHz, 16-bit, full-spectrum mono — this firmware genuinely matches) makes
  imports succeed. True provenance is kept in a GUANO `Note` field. Keep this
  in mind before sharing files anywhere that trusts the device tag.
- 256 ksps was chosen over v2's 240 ksps partly for this compatibility.
- Their filename convention `PREFIX_YYYYMMDD_HHMMSS.wav` is also worth matching
  — sessions are organised by parsed timestamp.
- Import path on iOS: Files → share sheet → *Copy to Echo Meter*. The
  `Imported Files Session.kml` the app produces is a GPS **export**, not an
  import requirement.
- iPhone file transfer without a custom app: BLE is a dead end (the S3 has no
  Bluetooth Classic; iOS has no standard BLE file-receive; nRF Connect only
  logs hex at ~10 kB/s). **SoftAP + a tiny HTTP server + Safari** is the way.

### 2. Power / brownout

Mostly fixed in v4's hardware, but the reasoning is worth keeping:

- Enabling Wi-Fi *and* octal PSRAM pushed a marginal USB supply over the edge:
  `E BOD: Brownout detector was triggered` right at Wi-Fi RF calibration (the
  biggest current spike, 300–400 mA on top of everything else).
- An inline USB "noise filter" dongle made it worse — those add series
  resistance and limit current. Straight into a PC port worked.
- **Capping TX power was tried and removed.** `CONFIG_ESP_PHY_MAX_WIFI_TX_POWER`
  = 10 dBm (plus a runtime `esp_wifi_set_max_tx_power()`) did soften the
  calibration spike, but a weaker link forces retransmits that keep the radio
  on *longer* — 2 MB clip downloads got slow and aborted mid-file. Both were
  removed; TX power runs at the default max and power is fixed on the supply
  side instead.
- Downloads are the heaviest sustained RF load the board ever sees — heavier
  than boot — so if boot is marginal, downloads fail first.
- The firmware still starts Wi-Fi ~2 s after boot (see `app_main`) so the
  boot/PSRAM inrush and the RF-cal spike don't stack. v4 adds a 470 µF
  reservoir at the module header and a battery instead of a cable.

### 3. False triggers from a floating analog input

- Touching v3's floating ADC input false-triggered the detector every time: a
  finger/contact transient is a **broadband impulse** that lifts every FFT bin
  at once, and a single-bin amplitude threshold can't tell it from a call.
- Firmware fix, both in the DETECT loop: **tonality** — the peak bin must be
  ≥ 20× the mean power of the rest of the >19 kHz band; **persistence** — 3
  consecutive frames (~26 ms) must pass before recording. Real chirps are
  narrowband and sustained; touches are flat and momentary.
- v4 fixes it in hardware too: the ADC input is driven from the TL972 through
  R17 (100 Ω) and cannot float. The two firmware tests are still worth keeping
  — they also reject wind, handling noise and machinery transients.

### 4. ESP-IDF / toolchain gotchas

- `led_strip` 2.x uses `.led_pixel_format = LED_PIXEL_FORMAT_GRB`; the
  `color_component_format` field only exists in 3.x. Match the code to the
  version in `dependencies.lock`, not to the latest docs.
- A **trailing space** typed into the menuconfig field for the custom partition
  CSV produced `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv "` and a
  baffling ninja error (`'…/partitions.csv ' … missing and no known rule to
  make it`). menuconfig keeps whatever you type.
- TinyUSB CDC is **not** enabled by default (`CONFIG_TINYUSB_CDC_ENABLED`) —
  the archived USB-streaming main silently needs it, plus the secondary console
  set to *None*, since USB-Serial-JTAG and TinyUSB share GPIO19/20.
- SD cards for the archived recorder must be **FAT32/MBR** (`mkfs.vfat -F 32`);
  cards over 32 GB ship exFAT, which IDF's FatFS won't mount by default.

### 5. GPIO choice is constrained by PSRAM mode

With `CONFIG_SPIRAM_MODE_OCT=y`, **GPIO35/36/37 belong to the PSRAM
controller** and cannot be used for anything else. This is why the gain-select
pins are 38/39/40 in firmware even though the v4 PCB routes two of them to
36/37 — see errata 1 in the [top-level README](../README.md#errata--read-before-ordering-the-pcb).
GPIO39/40 are MTCK/MTDO, free here only because the DevKitC-1 debugs over the
built-in USB-Serial-JTAG on GPIO19/20.

---

## Still to do

- **Ring buffer for pre-trigger audio** — the clip currently starts at the
  triggering buffer, so the calls *before* detection fired are lost. The single
  biggest capture upgrade left.
- **Adaptive trigger** — a running noise-floor estimate rather than the fixed
  `ULTRASONIC_THRESH_COUNTS`. With variable gain, a fixed count threshold means
  a *different* absolute sensitivity at each step, so this matters more now
  than it did in v3.
- **Call-shape gating** — require a swept FM component, plausible duration
  (~2–20 ms) and inter-pulse interval, to reject bush-crickets and machinery.
- **Duty cycling** for unattended overnight runs.
- **Pause the AP during the 3.84 s capture**, so no TX spike or RF pickup can
  coincide with sampling.
