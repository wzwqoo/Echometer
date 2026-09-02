# Hardware — analog front end, power, PCB and acoustics

KiCad 10 project (`v4.kicad_pro`, schematic + 4-layer PCB), fabrication outputs
in [`fab/`](fab/), BOM in [`v4.csv`](v4.csv), and the parametric acoustic-horn
generator [`ultrasonic_horn_freecad.py`](ultrasonic_horn_freecad.py).

The schematic is split over two sheets: `v4.kicad_sch` (root) and
`MCU.kicad_sch` (everything — front end, ADC, power, module headers).

> Read the errata in the [top-level README](../README.md#errata--read-before-ordering-the-pcb)
> before ordering. Item 1 is blocking: two of the three gain-select signals land
> on the octal-PSRAM bus and must be moved.

---

## Analog front end

### Transducer

**Knowles SPU0410LR5H-QB**, one fitted. An analog MEMS mic with usable response
through the target bat band; its raw output is small — tens of millivolts for a
loud, close call — so it must be amplified and biased into the ADC's input
range before sampling.

| Parameter | Value |
|---|---|
| Type | analog MEMS (SiSonic), bottom-port, omnidirectional |
| Sensitivity | −38 dBV/Pa (±3 dB) ≈ 12.6 mV/Pa @ 94 dB SPL, 1 kHz |
| SNR | 63 dB(A) → self-noise ≈ 31 dB(A) SPL |
| Frequency response | ~100 Hz → usable to ≈ 80 kHz (rising resonance ~20–25 kHz) |
| Max acoustic input | ≈ 120 dB SPL |
| Supply / current | 1.5–3.6 V / ~123 µA |
| Output impedance | ~200 Ω |
| Package | 3.76 × 2.95 × 1.10 mm, 0.25 mm port |

The 0.25 mm acoustic port sits at the throat of the conical horn.

### Op-amp choice

The amplifier must pass the full ultrasonic band **and** apply large gain
without rolling off, which sets a hard requirement on gain–bandwidth product:
`GBW ≥ gain × 120 kHz`. Two devices:

| Stage | Part | Why |
|---|---|---|
| 1st gain | **TI OPA838** (OPA838IDBVR) | 300 MHz GBW, decompensated (min stable gain +7), 1.8 nV/√Hz, ~1 mA |
| 2nd gain / ADC driver | **TI TL972** (TL972IDR) | dual rail-to-rail I/O, 12 MHz GBW — drives the ADC to full swing on 3.3 V |

Both run single-supply and bias the signal to **mid-rail (VREF ≈ 1.65 V)** so
the AC waveform swings symmetrically inside the ADC's 0–3.3 V window. A **TI
TPS74801** low-noise LDO supplies the analog rail and VREF, so digital
switching noise doesn't couple into the µV-level mic signal.

Note the OPA838's decompensation: its minimum stable gain is +7, which is a
real constraint on the gain-step table below — stage 1's low setting of 4.25
would be unstable on its own, and is only usable because C14 (27 pF across
R13) provides the compensation the topology needs.

### Gain — how the switching works

Both stages are non-inverting with a 1 kΩ leg to VREF. Each has a
**permanent** feedback resistor, and a **74HC4053** analog mux switches further
resistors in **parallel** with it, which *lowers* the gain.

| Stage | Amp | Rg | Permanent Rf | Switched in parallel | Gains |
|---|---|---|---|---|---|
| 1 | OPA838 (U3) | R6 = 1 k | R13 = 33 k | R26 = 3k6 via switch 1 | 34 / 4.25 |
| 2 | TL972B (U2) | R12 = 1 k | R25 = 22 k | R23 = 22 k (sw 2), R24 = 6k8 (sw 3) | 23 / 12 / 6.2 / 5.2 |

Three design decisions are load-bearing here:

**The permanent resistors are never switched out.** The loop is closed in every
mux state, including the ~300 ms before firmware runs and the instant the mux
transitions. Nothing can rail.

**The polarity is chosen so reset means quiet.** The 4053 selects Y0 — resistor
connected, parallel combination, *lower* gain — when its select pin is **LOW**.
GPIOs sit low at reset, so the board wakes at minimum gain, which cannot clip.

**The switches sit at the summing node**, mux common to the op-amp inverting
input and the resistors to Y0. Both sides of a closed switch are therefore at
virtual ground, so the mux's on-resistance sees an almost constant voltage.
That is what keeps the 4053's on-resistance nonlinearity out of the signal —
put the switch anywhere the voltage swings and it becomes a distortion source.

**Why the switches are split 1 / 2 across the stages.** Switching resistors in
parallel adds conductances, so the achievable ratio within one stage is bounded
— a single stage can only span about 18 dB however you pick the values. Two
stages multiply, giving the full 31 dB.

### The six steps

| S1 | S2 | S3 | Stage 1 | Stage 2 | **Total** | dB | Clips near |
|---|---|---|---|---|---|---|---|
| 0 | 0 | 0 | 4.25 | 5.2 | **22×** | 26.9 | 105 dB SPL |
| 0 | 0 | 1 | 4.25 | 12 | **51×** | 34.2 | 98 dB SPL |
| 0 | 1 | 1 | 4.25 | 23 | **98×** | 39.8 | 92 dB SPL |
| 1 | 1 | 0 | 34 | 6.2 | **210×** | 46.4 | 86 dB SPL |
| 1 | 0 | 1 | 34 | 12 | **408×** | 52.2 | 80 dB SPL |
| 1 | 1 | 1 | 34 | 23 | **782×** | 57.9 | 74 dB SPL |

Steps are 5.6–7.3 dB apart. "Clips near" is the input SPL at which the TL972
output hits its rail, for the SPU0410 at −38 dBV/Pa with ~1.45 V of peak swing.

Three switches give 8 states, not 6. The two omitted (177× and 26×) sit within
1.5 dB of a neighbour, and that bunching is unavoidable: the four states of two
parallel switches always satisfy `G(both) = G(a) + G(b) − G(neither)`, so they
cannot be spaced evenly.

### Input common-mode headroom — checked, and why VREF must stay mid-rail

Neither amp has a rail-to-rail *input* stage, and both stages are
non-inverting, so the + input carries signal rather than sitting still. On a
3.3 V rail:

| | Spec | Window on 3.3 V | Worst-case swing here | Margin |
|---|---|---|---|---|
| TL972 (U2) | V<sub>ICM</sub> = VCC− + 1.15 … VCC+ − 1.15 | **1.15 – 2.15 V** | 1.371 – 1.929 V (at A2 = 5.2) | 221 mV |
| OPA838 (U3) | VS− − 0 … VS+ − 1.3 (over temp) | **0 – 2.00 V** | 1.584 – 1.716 V | 284 mV |

Both pass. The TL972's is the tighter one and it appears at the *lowest* gain
step, where the output is clipping at ±1.45 V and its + input therefore has to
swing ±279 mV. The OPA838 is never stressed because its + input only ever
carries the mic-level signal, at most ±66 mV — the mic itself runs out
(AOP ≈ 120 dB SPL) at about the same point its 2.0 V ceiling would be reached.

The useful consequence: the safe window is the **intersection**, 1.15 – 2.00 V,
and VREF at 1.65 V sits near its centre. Do not "improve" headroom by moving
VREF down — 1.15 V is a hard floor for the TL972, and anything below ~1.4 V
starts eating the margin you gained at the top.

### Why the gain stops at 782×

The SPU0410's own noise integrated over 20–120 kHz is around **38 dB SPL**, and
a bat at 30 m arrives at roughly 30 dB SPL. Past ~800× you are amplifying mic
noise, not signal.

v3's jump from 121× to 625× helped because at 121× the ADC's LSB was still
competing with the mic noise; at 782× the mic dominates and more gain buys
nothing. Further range has to come from narrowband processing — FFT bin energy,
matched filter — not from the front end.

### Filtering — ultrasonic only

| Element | Value | Corner | Purpose |
|---|---|---|---|
| C12 + (R3 + mic Zout ≈ 1.3 k) | 10 nF | 12.2 kHz | HP, mic → stage 1 |
| C13 + (R15 + R10 ≈ 1.1 k) | 10 nF | 14.5 kHz | HP, stage 1 → stage 2 |
| *both together* | | **≈ 19 kHz −3 dB** | −33 dB at 2 kHz |
| C14 across R13 | 27 pF | 179 kHz | stage-1 LP / compensation |
| C15 across R25 | 27 pF | 268 kHz | stage-2 LP / compensation |
| R17 + C16 | 100 Ω + 22 nF | 72 kHz | anti-alias ahead of the ADC |
| R18 + C19 | 100 Ω + 10 µF | 159 Hz | mic **supply** filter (not signal) |

Two of these look similar and do opposite jobs, and v3 got one of them wrong.
R18/C19 is a **power-supply** filter on the mic's VDD pin — it wants its corner
far *below* the band, and 159 Hz gives about 48 dB of rejection at 40 kHz.
v3 used 10 Ω + 110 nF, which corners at 159 kHz and therefore did essentially
nothing. C12/C13 are the **signal** high-pass and want their corners *at* the
band edge.

The anti-alias filter had the same class of bug in v3: 100 Ω + 10 nF corners at
159 kHz, above the 128 kHz Nyquist, so it wasn't anti-aliasing anything. 22 nF
brings it to 72 kHz.

Keeping R6/R12 at 1 kΩ is what let both 27 pF compensation caps stay. Scaling
the network up 10× for resistor accuracy would have demanded ~2 pF caps, which
are not buildable against stray capacitance. Mux on-resistance instead adds
1.5 % error at the 33 k tap and 4 % at the 6k8 tap — irrelevant for a detector.

---

## Digital acquisition

Sampling is done by an **external 12-bit SAR ADC (ADCS7476, rated 1 MSPS)** run
at **256 ksps → 128 kHz Nyquist**, covering the full bat band. The ESP32-S3's
built-in ADC-DMA path tops out near 83 ksps — far too slow — so the external
ADC is clocked with a hardware trick:

- The **I²S peripheral** runs as master, 16-bit **stereo**, so its **WS line
  toggles every 16 bit-clocks**.
- **WS → ADC CS**, **BCK → SCLK**, **SDATA → I²S DIN**. WS-low frames each
  conversion (16 clocks readout); WS-high is the re-track gap.
- I²S RX DMA captures every sample jitter-free with **zero per-sample CPU**.
  At 256 ksps the bit clock is `256000 × 2 × 16 = 8.192 MHz`.

Buffers of **2048 samples** (one FFT frame, 8 ms) feed the detector —
125 Hz/bin resolution; the detection band starts at bin 152 (19 kHz). See
[../firmware/README.md](../firmware/README.md) for the detection logic and the
bit-alignment quirk in the returned word.

The ADC input is driven from the TL972 output through R17 (100 Ω), so unlike
v3 it can never float. A floating ADC input is an antenna, and touching it
false-triggered v3's detector every time.

### Signal levels & detection budget

| Quantity | Value |
|---|---|
| ADC | 12-bit, full scale 4095 counts over 3.3 V |
| Bias point | VREF ≈ 1.65 V |
| Raw transducer signal (close call) | ~33–50 mV |
| Gain range | 22× – 782×, six steps |
| Detection band | > 19 kHz |
| Known interferer | ~50 kHz USB switching noise falls **inside** the band |

The FFT detection threshold (`ULTRASONIC_THRESH_COUNTS`) sits a few dB above
the *amplified* noise floor. Because the gain is now variable, that fixed
count threshold represents a different absolute sensitivity at each step — see
the roadmap note about an adaptive trigger.

---

## Power

```text
USB ──▶ TP4056 ──▶ [cell] ──▶ DW01/FS8205 ──▶ Lipo+ ─┬─▶ MT3608 ──▶ +5V ──▶ ESP32 module
                                                     │              └──▶ TPS74801 BIAS
                                                     └──────────────────▶ TPS74801 IN ──▶ +3V3 analog
```

| Block | Part | Setting |
|---|---|---|
| Charger | TP4056 (U5) | R7 = 1.2 k → **1 A** (0.5 C into 2000 mAh); thermal pad on GND |
| Protection | DW01 (U6) + FS8205A (U7) | low-side, R20 100 Ω + C25 100 nF, R8 1 k on CS |
| Boost | MT3608 (U8) | L1 4.7 µH, D3 SS34, R21 150 k / R22 19.1 k → **5.31 V** |
| Analog LDO | TPS74801 (U10) | IN from Lipo+, BIAS + EN from +5 V, R11 31.2 k / R9 10 k → **3.30 V** |
| Wi-Fi reservoir | C28 470 µF | at the module header |
| Status | D1 red / D2 green | TP4056 CHRG / STDBY |

### The LDO is fed from the battery, not from the boost

That is the whole point of the TPS74801's separate BIAS pin. BIAS runs the
control loop and the gate driver, so the pass FET can be driven *above* VIN.
That lets IN sit at the raw cell voltage (3.0–4.2 V) and drop only ~50 mV at
the ~25 mA the analog side draws.

The win isn't efficiency — it's noise. Routed the obvious way (LDO fed from the
boost output), the analog rail inherits the MT3608's 1.2 MHz switching ripple,
and no LDO's PSRR is good at 1.2 MHz. Feeding it from the cell means the analog
rail never sees the switcher at all.

**Boost output is 5.31 V, not 5.0 V, and that is deliberate.** BIAS must stay
at least 1.62 V above the 3.3 V output, i.e. ≥ 4.92 V. At 5.1 V there was only
180 mV of margin — about what a Wi-Fi TX burst eats. Measure the sag on a real
board; if +5 V dips below 4.92 V during TX, the LDO loses bias headroom and the
analog rail glitches during exactly the moments you transmit.

### Charging and switching, both with caveats

**Charging while running works** — the cell buffers everything — but the load
current means the TP4056 may never see its termination threshold. It can sit in
CV at 4.2 V indefinitely and the green LED may never come on. Don't leave it
plugged in for days.

**There is no power switch.** The boost's EN is tied to its own input, so the
board runs from the moment the cell is connected; unplug J6 to stop it. Standby
drain as built is 50–120 mA, which flattens 2000 mAh in 17–40 hours and then
parks the cell at the DW01's 2.4 V cutoff — bad for storage.

If you add a switch, put it in the **`Lipo+ → L1 / U8.5` branch**, not on the
boost's EN pin. The MT3608 is asynchronous, so `Lipo+ → L1 → D3 → +5V` is a
permanent DC path: disabling the boost leaves the module's regulator powered
anyway. Breaking the boost input kills that path and still leaves the charger
connected.

---

## Pinout — ESP32-S3 DevKitC-1

| Function | Header pin | GPIO |
|---|---|---|
| ADC CS (I²S WS) | J11.4 | 4 |
| ADC SCLK (I²S BCK) | J11.5 | 5 |
| ADC SDATA (I²S DIN) | J11.6 | 6 |
| SD nCS / MOSI / SCK / MISO | J11.16–19 | 10 / 11 / 12 / 13 |
| +5 V / GND | J11.21 / J11.22 | — |
| Gain S1 | J12.10 | 38 |
| Gain S2 | J12.11 | 37 ⚠ move to J12.9 (GPIO39) |
| Gain S3 | J12.12 | 36 ⚠ move to J12.8 (GPIO40) |
| RGB LED | on module | 48 |
| Button | on module | 0 (BOOT) |

The SD lines are wired but unused by the current firmware; J5 must be an SD
**module** with its own regulator and level shifters, since it is fed +5 V on
pin 5 and the SPI lines are unbuffered.

---

## Acoustics — the conical horn

[`ultrasonic_horn_freecad.py`](ultrasonic_horn_freecad.py) is a parametric
generator for a conical horn that concentrates incoming ultrasound onto the mic
port. It is free SNR, ahead of the electronics.

| Parameter | Value |
|---|---|
| Profile | conical |
| Throat Ø (mic end) | 0.30 mm (just over the 0.25 mm mic port) |
| Mouth Ø (open end) | 28 mm (≈ 3.5 × wavelength at 45 kHz) |
| Length | 50 mm |
| Flare half-angle | ≈ 15.9° |
| Wall thickness | 0.5 mm |
| Optimised band | 40–50 kHz |
| Print | resin 25 µm (vertical), or FDM 0.12 mm / 100 % infill |

At 108 kHz the 28 mm mouth is ~8.8 wavelengths across — well above cutoff, so
the horn still delivers strong on-axis gain, just with a narrower beam that
must be aimed more precisely.

---

## Bat call levels, directivity and detection range

> Engineering **estimates** for sizing the front end — not species-accurate
> field data.

### Source levels

| Call type / example | Peak freq | SL @ 10 cm | ≈ @ 1 m | Notes |
|---|---|---|---|---|
| Loud open-air hawkers (*Nyctalus*, *Eptesicus*, *Pipistrellus*) | 25–50 kHz | 120–130 dB SPL | 100–110 dB SPL | loudest; FM/QCF sweeps |
| Horseshoe bats (*Rhinolophus*, CF) | 80–110 kHz | 105–120 dB SPL | 85–100 dB SPL | constant-frequency |
| "Whispering" gleaners (*Myotis*, *Plecotus*) | 40–90 kHz | 85–100 dB SPL | 65–80 dB SPL | very quiet, short range |

Cross-reference this against the "clips near" column in the gain table: a loud
hawker at a few metres wants the 22× or 51× step, while a distant pass wants
782×. That 31 dB spread is exactly the range v3's single fixed 625× could not
cover.

### Target species — least horseshoe bat (*Rhinolophus pusillus*)

| Parameter | Value |
|---|---|
| Call type | CF–FM (long constant-frequency tone + short FM tails) |
| Dominant CF | ≈ 106–112 kHz (~108 kHz typical) |
| Source level | moderate, ~80–90 dB SPL @ 1 m |
| FFT bin at 108 kHz | ≈ 864 (of 1024) — comfortably under the 128 kHz Nyquist |
| Est. on-axis range | **~7–9 m** (air absorption ~3.9 dB/m at this frequency) |

Consequences for the design:

- **A fast external ADC is mandatory.** At 108 kHz the call needs > 216 ksps;
  an MCU's internal ADC at 83 ksps could not see this species at all, and even
  v2's 240 ksps left it at the very edge of the band. Hence the ADCS7476 at
  256 ksps.
- **The narrowband CF tone is the easy case for the FFT detector** — its energy
  piles into one bin. Narrowing the detection window to ~100–115 kHz (bins
  ~800–920) would target this species specifically and reject the ~50 kHz USB
  noise interferer entirely.

### Directivity

| Source of directivity | Typical | Effect |
|---|---|---|
| Bat call emission | forward lobe, −3 dB half-angle ~30–40° | a bat not facing you arrives 10–20 dB weaker |
| Bare SPU0410 mic | omnidirectional | no aiming, no acoustic gain |
| With conical horn | directional lobe | +on-axis SNR/range, must be pointed |

### Estimated detection range

Loud bat, SL = 110 dB SPL @ 1 m on-axis; minimum detectable ≈ 40 dB SPL:

| Frequency | Air absorption α | Est. max range |
|---|---|---|
| 20 kHz | ~0.6 dB/m | ~60 m |
| 40 kHz | ~1.3 dB/m | ~30 m |
| 80 kHz | ~2.9 dB/m | ~16 m |
| 120 kHz | ~4.2 dB/m | ~12 m |

Optimistic ceilings — quiet bats, off-axis calls, and weather cut these
substantially.

---

## Bill of materials

Full BOM in [`v4.csv`](v4.csv), regenerated from the schematic with
`kicad-cli sch export bom`. The parts that define the design:

| Part | Qty | Role |
|---|---|---|
| SPU0410LR5H-QB | 1 | MEMS ultrasonic microphone |
| OPA838IDBVR | 1 | 1st gain stage (300 MHz GBW) |
| TL972IDR | 1 | 2nd gain / ADC driver (RRIO) |
| 74HC4053D | 1 | 3× SPDT analog mux — the gain switch |
| ADCS7476AIMFX/NOPB | 1 | 12-bit 1 MSPS SAR ADC |
| TPS74801DRCR | 1 | low-noise LDO (analog rail / VREF) |
| TP4056 | 1 | LiPo charger, 1 A |
| DW01 + FS8205A | 1 + 1 | cell protection |
| MT3608 | 1 | boost to 5.31 V |
| conical horn (`ultrasonic_horn_freecad.py`) | 1 | acoustic gain / directivity |

**Known BOM issue:** several passive symbols still carry the `Part#` and
`LCSC Part #` of their *previous* value — R13, R23/R25, R24 and R26 inherit a
24 kΩ part number, and C16 inherits a 10 nF one. Those fields are cleared in
the checked-in `v4.csv`, so ordering from it is safe, but fix the symbol
properties before the next export.

`altium_ascii_to_kicad.py` is a helper for importing Altium ASCII footprints
into the KiCad library; it is not part of the build.
