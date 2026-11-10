# UV-K5 Classic LBJ / POCSAG Receiver

`lbjradio` branch: receive Chinese railway LBJ/POCSAG (1200 baud) signals on the
classic UV-K5 without a K1, a PC, or an external module.

## How it works

- Front end: the BK4819 **direct-FSK 1.2K** engine (the same engine AirCopy
  uses), not the K1's PA4 audio sampler.
- Decoder: `app/lbj_dec.c`, ported from the K1 LBJ app. BCH(31,21), 2-bit
  correction, sync tolerance 3, 0..31 bit-shift search.
- Modulation: `MODULATION_LBJ`, receive-only; TX is disabled by default.

## Status (important)

**This is a diagnostic/WIP branch, not a finished decoder.**

- The decoder core is verified: K1's `model_rx.py` decodes both real IQ
  recordings with **16/16 clean codewords, 0 errors**.
- The radio itself does not decode the real signal yet. The BK4819 hardware
  slicer produces **6-7 bit errors per 32-bit codeword (~19% BER)** on this
  +/-2.3 kHz signal; 2-bit BCH cannot recover that.
- The screen usually shows `WAIT SHORT...` or `A-------`. The raw FIFO shows
  sync/address distances around D5-D8.
- Next step: **PA9 audio tap + K1 software demod** (low-pass, matched filter,
  adaptive threshold, DPLL). Not implemented yet.

## Attempts so far

The TDCS receiver on this firmware already decodes real signals through the
BK4819 FSK engine, so LBJ started from the same path:

- **BK4819 direct-FSK 1.2K engine** with the losehu MDC1200 RX recipe
  (`REG_58=0x3F03`, `REG_5C=0x5625`), POCSAG sync in `REG_5A/5B`, 64-byte
  payload in `REG_5D`, one-word FIFO threshold in `REG_5E`.
- **Nine RX front-end profiles (C0-C8)**: AirCopy values, TDCS clock
  recovery, Roger MDC RX gain, CRC on/off, FSK level variants, the C7
  POCSAG P front end, and C8 with a 12.5 kHz IF. `UP`/`DOWN` cycles them on
  air.
- **AFC enabled** for the FSK slicer.
- **MSB/LSB FIFO order** and **inverted polarity** (`M/L/I/J`).
- **Full RX re-arm per batch**.
- **Decoder hardening**: 2-bit BCH(31,21), sync tolerance 3, 0..31 bit-shift
  search, known-address Hamming distance, short-block buffer.
- **Raw FIFO dump** on the K-head UART for offline replay.
- **Offline IQ analysis** with K1 `model_rx.py` on two real recordings.
- **K1 v5.6.3/v5.6.4 receive fixes ported**: 2 s squelch hold, stop-time
  batch flush, sampler stop before a receiver reconfigure, C8 narrow IF, and
  the shift shown on screen is the one the decoder used. The K1 `PK_SHIFT`,
  overlay carrier hold and detail-digit gates do not apply to the BK4819
  hardware slicer.

## Problems found

- The signal is valid: carrier -12.42 kHz (SDR offset), deviation about
  +/-2.3 kHz, baud about 1200.75, DC about -200 Hz.
- K1's integer demodulator decodes both recordings with **16/16 clean
  codewords, 0 errors** after mixing to baseband, low-pass filtering and FM
  demodulation. Decimating the -12.5 kHz-centred signal directly corrupts the
  demodulation.
- The K5 raw FIFO shows sync distances **D4-D8** (mostly D6-D7) and address
  distances **D5-D7**. A real address needs **D0-D2**.
- The errors are uniform: 47-48% ones, 0.5 transitions/bit, longest run
  8-13 bits. This rules out polarity, bit order, framing and a gross
  frequency offset.
- Fixed-threshold slicing gives about **15-16 errors per 32 bits**; the K1
  chain (1500 Hz low-pass, 16-sample matched filter, adaptive peak/valley
  threshold, DPLL) gives 0. The BK4819 hardware slicer has none of those.
- Register attempts (`REG_43`, `REG_5C`, `REG_58`, `REG_70`, `REG_59`,
  `REG_5D`, `REG_5E`, `REG_2B`, `REG_7E`, `REG_4B`, C1 clock recovery, C7
  P front end) did not get below the ~19% BER.
- The C7 profile produced repeating `2BDC F2B1` / `57B8 E563` words, i.e.
  garbage, not a better slice.
- `addrDist > 2` used to force the decode offset to 0 while the screen kept
  showing the measured shift. Fixed: the decoder now uses the measured shift,
  so `S<shift>` matches what was decoded.

## Why it is blocked

The BK4819 FSK engine has a fixed hardware slicer: no matched filter, no
adaptive threshold and no DPLL. This signal's +/-2.3 kHz deviation leaves too
little margin at the slicer, so every codeword arrives with 6-7 bit errors and
2-bit BCH cannot recover it. More register tuning cannot add those DSP blocks.

The only remaining K5-only path is to AC-couple the BK4819 AF output
(U4 LM4890 `IN+`) to PA9/ADC_CH4 through a 100 nF capacitor and run the K1
`pa4_sampler.c` + `lbj_dual.c` software demodulator on the MCU, reusing
`lbj_dec`. That requires opening the radio and soldering; it is not implemented.

## Flash budget

- Current LBJ build: **58992 bytes**, about **2.4 KiB free**.
- TDCS and LBJ do not fit together.
- The PA9 sampler + software demod needs roughly 4-6 KiB, so a dedicated LBJ
  build would have to disable FM radio and UART.

## Build

```bash
make ENABLE_TDCS=0 ENABLE_LBJ=1 ENABLE_LBJ_TX=0 TARGET=f4hwn.lbj
```

- `ENABLE_LBJ_TX=1` enables the PTT test transmitter (fixed `1234000` /
  function 3 frame).
- LBJ and TDCS **cannot be enabled together**; they do not fit in 60 KiB.
- Prebuilt firmware: `compiled-firmware/f4hwn.lbj.bin` (58992 bytes, version
  string `LBJ v1.0.3`).

## Usage

1. Set a VFO's modulation to `LBJ`. The main screen becomes the LBJ decode page.
2. Screen layout:
   - `A<short-block address> S<shift> D<address distance>`
   - `IRQ xxxx ST xxxx`
   - the first six FIFO words of the last batch
   - the BCD message (two lines)
   - `F<frames> C<profile> 5C=xx <order>`
3. `UP` / `DOWN`: cycle the nine RX front-end profiles (C0-C8).
4. `*`: cycle bit order/polarity `M/L/I/J`.
5. `PTT`: swallowed when `ENABLE_LBJ_TX=0`; sends the fixed test frame when
   `ENABLE_LBJ_TX=1`.
6. The FSK engine stops while the squelch is closed, so stale FIFO words cannot
   pile up.
7. The short block `1233999/1234000` is kept in its own buffer so the detail
   block `1234001/1234002` cannot overwrite it.

## UART raw dump

2.5 mm K-head, **38400 8N1**. One line per second while batches arrive:

```text
LBJ C<cfg> <M|L|I|J> S<shift> D<dist> IRQ<xxxx> ST<xxxx>
W0: eight 16-bit words
W1: ...
W2: ...
W3: ...
```

The raw FIFO words are dumped before `rev16`/inversion, so a PC can replay
every shift, bit order and polarity offline.

## RX profiles (C0-C8)

| # | REG_58 | REG_5C | REG_70 | REG_43 | Notes |
|---|---|---|---|---|---|
| 0 | `00C1` | `AA30` | `00E0` | `3628` | AirCopy values, 25 kHz IF |
| 1 | `00C1` | `5625` | `00E0` | `3628` | TDCS real-signal clock recovery |
| 2 | `37C3` | `AA30` | `00E0` | `3628` | Roger MDC RX, gain 11 |
| 3 | `37C3` | `5625` | `00E0` | `3628` | Combination |
| 4 | `00C3` | `5625` | `00E0` | `3628` | RX mode variant |
| 5 | `00C1` | `5665` | `00E0` | `3628` | CRC on, alternate clock recovery |
| 6 | `00C1` | `5625` | `00C3` | `3628` | FSK level variant |
| 7 | `03C1` | `5625` | `0000` | `47A8` | POCSAG P front end |
| 8 | `00C1` | `5625` | `00E0` | `3648` | Narrow IF, 12.5 kHz |

## Files

| File | Purpose |
|---|---|
| `app/lbj.c` / `app/lbj.h` | Front end, profiles, FIFO, display, UART, PTT |
| `app/lbj_dec.c` / `app/lbj_dec.h` | POCSAG BCH / address / BCD |
| `radio.c` / `radio.h` | `MODULATION_LBJ`, AFC, RX filter bypass, TX inhibit |
| `app/app.c` | Interrupt hand-off, 10 ms tick, PTT hook |
| `ui/main.c` | LBJ page takes over the main screen |
| `tools/lbj_wav_test.py` | Offline decoder regression |
| `compiled-firmware/f4hwn.lbj.bin` | Prebuilt LBJ firmware |

## Offline test

```bash
python tools/lbj_wav_test.py recording.wav
```

- The WAV must be a 1200-baud POCSAG discriminator/audio recording.
- The script resamples to 19.2 kHz, removes DC, slices at every bit phase and
  verifies the 2-bit BCH and the 0..31 bit-shift search.
- This is a decoder test, not an RF test.

## References

- K1 LBJ: `uv-k1-k5v3-fw`, `App/apps/lbj/`, `App/app/lbj_dual.c`,
  `App/app/pa4_sampler.c`; the receive fixes follow K1 `d46a72ab` (v5.6.3-test)
  and `a5455e2c` (v5.6.4)
- Branch: `lbjradio` (current HEAD)

## Known limitations

- The BK4819 hardware slicer produces ~19% BER on the real signal; 2-bit BCH
  cannot recover it.
- There is no PA9 audio tap, so software demodulation is not implemented.
- LBJ and TDCS do not fit in flash at the same time.
- PTT transmits only the fixed test frame, not a complete LBJ protocol.
