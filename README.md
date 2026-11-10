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

## Build

```bash
make ENABLE_TDCS=0 ENABLE_LBJ=1 ENABLE_LBJ_TX=0 TARGET=f4hwn.lbj
```

- `ENABLE_LBJ_TX=1` enables the PTT test transmitter (fixed `1234000` /
  function 3 frame).
- LBJ and TDCS **cannot be enabled together**; they do not fit in 60 KiB.
- Prebuilt firmware: `compiled-firmware/f4hwn.lbj.bin` (59068 bytes).

## Usage

1. Set a VFO's modulation to `LBJ`. The main screen becomes the LBJ decode page.
2. Screen layout:
   - `A<short-block address> S<shift> D<address distance>`
   - `IRQ xxxx ST xxxx`
   - the first six FIFO words of the last batch
   - the BCD message (two lines)
   - `F<frames> C<profile> 5C=xx <order>`
3. `UP` / `DOWN`: cycle the eight RX front-end profiles (C0-C7).
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

## RX profiles (C0-C7)

| # | REG_58 | REG_5C | REG_70 | Notes |
|---|---|---|---|---|
| 0 | `00C1` | `AA30` | `00E0` | AirCopy values |
| 1 | `00C1` | `5625` | `00E0` | TDCS real-signal clock recovery |
| 2 | `37C3` | `AA30` | `00E0` | Roger MDC RX, gain 11 |
| 3 | `37C3` | `5625` | `00E0` | Combination |
| 4 | `00C3` | `5625` | `00E0` | RX mode variant |
| 5 | `00C1` | `5665` | `00E0` | CRC on, alternate clock recovery |
| 6 | `00C1` | `5625` | `00C3` | FSK level variant |
| 7 | `03C1` | `5625` | `0000` | POCSAG P front end |

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
  `App/app/pa4_sampler.c`
- Branch: `lbjradio`, commit `ce5d1d7`

## Known limitations

- The BK4819 hardware slicer produces ~19% BER on the real signal; 2-bit BCH
  cannot recover it.
- There is no PA9 audio tap, so software demodulation is not implemented.
- LBJ and TDCS do not fit in flash at the same time.
- PTT transmits only the fixed test frame, not a complete LBJ protocol.
