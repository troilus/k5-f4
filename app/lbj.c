/* LBJ / POCSAG receive for the classic UV-K5 - see lbj.h.
 *
 * The K1 LBJRadio samples the BK4819 AF output on PA4 at 19.2 kHz.  The
 * classic UV-K5 has no audio-to-ADC path, so this port uses the BK4819 FSK
 * engine in direct-FSK 1.2K mode (the AirCopy engine) and feeds the FIFO
 * codewords to the K1 POCSAG decoder (lbj_dec.c).
 *
 * The default path is raw FSK: REG_59 = 0x1060 turns the hardware sync off,
 * so the engine only slices at 1200 baud and every word reaches the FIFO.
 * lbj_dec.c hunts the sync word, the polarity and the BCH in software, which
 * is what makes the real signal decodable on the stock hardware.  The old
 * hardware-sync batch path is kept behind ENABLE_LBJ_HWSYNC=1.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "lbj.h"
#include "lbj_dec.h"

#include "audio.h"
#include "driver/bk4819.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "driver/system.h"
#include "driver/uart.h"
#include "frequencies.h"
#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/helper.h"
#include "ui/main.h"
#include "ui/ui.h"

/* BK4819 interrupt bits (REG_02 / REG_3F). */
#define LBJ_IRQ_SYNC   0x0002u
#define LBJ_IRQ_FIFO   0x1000u
#define LBJ_IRQ_DONE   0x2000u
#define LBJ_IRQ_MASK   (LBJ_IRQ_SYNC | LBJ_IRQ_FIFO | LBJ_IRQ_DONE)

/* POCSAG batch payload: 16 codewords = 64 bytes = 32 16-bit FIFO words.
 * The engine strips the 4-byte sync; REG_5D is the payload length. */
#define LBJ_BATCH_WORDS  32u
#define LBJ_BATCH_BITS   (LBJ_BATCH_WORDS * 16u)

#define POCSAG_SYNC      0x7CD215D8u
#define POCSAG_IDLE      0x7A89C197u

/* Raw FSK mode is the default.  REG_59 = 0x1060 keeps the FSK receiver on
 * (bit 12) and the 7-byte preamble (bits 6:4 = 6), but leaves the hardware
 * sync length at 0.  The BK4819 then behaves as a plain 1200 baud slicer: every
 * sliced word lands in the FIFO, no sync has to match in hardware, and
 * lbj_dec.c hunts the POCSAG sync word, the polarity and the BCH in software.
 * That is how the K1 LBJRadio works and it is the only path that survives the
 * hardware slicer's error rate on the real signal.
 *
 * Build with ENABLE_LBJ_HWSYNC=1 to get the old hardware-sync/DONE batch path
 * back for an A/B comparison. */
#ifndef ENABLE_LBJ_HWSYNC
#define LBJ_REG59_RAW    0x1060u
#endif

static struct {
    bool     on;
    bool     dirty;
    uint16_t sec, subSec;
    lbj_rx_t rx;
    uint16_t raw[LBJ_BATCH_WORDS];
    uint8_t  rawCount;   /* words shown on screen (last completed batch) */
    uint8_t  fill;       /* words collected for the batch in progress     */
    uint16_t status;
    uint16_t lastIrq;
    uint16_t frames;
    char     bcd[72];        /* last report of any address             */
    uint8_t  bcdLen;
    char     bcdShort[72];   /* last 1233999/1234000 short block       */
    uint8_t  bcdShortLen;
    uint32_t shortAddr;      /* address of that short block, 0 = none  */
    uint32_t addr;
    uint32_t lastFreq;
    uint8_t  cfg;        /* LBJ_CFG index                          */
    uint8_t  order;      /* 0 M, 1 L, 2 I, 3 J (bit 0 LSB, bit 1 inverted) */
    uint8_t  shift;      /* bit offset of the closest known address */
    uint8_t  addrDist;   /* Hamming distance to that address word   */
    uint16_t lostTicks;  /* 10 ms ticks the squelch has been closed */
    uint8_t  txState;    /* 0 idle, 1 sending, 2 sent, 3 denied    */
    uint16_t txTick;
#ifndef ENABLE_LBJ_HWSYNC
    /* Raw-mode diagnostics, the same numbers the reference DIAG page shows:
     * the interrupt mix, the FIFO word count and the decoder's own counters
     * (sync hits, BCH clean/repaired vs uncorrectable). */
    uint16_t irqCount, fskCount, otherCount, finCount;
    uint16_t fifoWords, ovCount;
    uint8_t  diag;       /* 1 = the DIAG page is on screen          */
#endif
} g;

/* RX tuning table.  The real LBJ signal does not decode with the AirCopy
 * values, so UP/DOWN cycles these on air.  REG_72 (1200 baud), REG_5A/5B
 * (POCSAG sync), REG_5D (64-byte payload) and REG_5E stay fixed.
 * reg43 is the RF/IF filter: 0x3628 = 25 kHz, 0x3648 = 12.5 kHz, and the
 * POCSAG P profile keeps its 7.5 kHz RF / 12.5 kHz IF value. */
typedef struct {
    uint16_t reg58;
    uint16_t reg5c;
    uint16_t reg70;
    uint16_t reg43;
    uint8_t  flags;      /* 1 = POCSAG P front end (C7) */
} lbj_cfg_t;

static const lbj_cfg_t LBJ_CFG[] = {
    { 0x00C1u, 0xAA30u, 0x00E0u, 0x3628u, 0u },  /* 0: current AirCopy values   */
    { 0x00C1u, 0x5625u, 0x00E0u, 0x3628u, 0u },  /* 1: TDCS real-signal clock   */
    { 0x37C3u, 0xAA30u, 0x00E0u, 0x3628u, 0u },  /* 2: Roger MDC RX, gain 11    */
    { 0x37C3u, 0x5625u, 0x00E0u, 0x3628u, 0u },  /* 3: combination              */
    { 0x00C3u, 0x5625u, 0x00E0u, 0x3628u, 0u },  /* 4: RX mode variant          */
    { 0x00C1u, 0x5665u, 0x00E0u, 0x3628u, 0u },  /* 5: CRC on, other clock rec. */
    { 0x00C1u, 0x5625u, 0x00C3u, 0x3628u, 0u },  /* 6: FSK level variant        */
    { 0x03C1u, 0x5625u, 0x0000u, 0x47A8u, 1u },  /* 7: POCSAG P front end       */
    { 0x00C1u, 0x5625u, 0x00E0u, 0x3648u, 0u },  /* 8: narrow IF, 12.5 kHz      */
};

#define LBJ_CFG_COUNT ((uint8_t)(sizeof(LBJ_CFG) / sizeof(LBJ_CFG[0])))

/* ---- small formatting helpers ------------------------------------------ */

static char *put(char *o, const char *s)
{
    while (*s)
        *o++ = *s++;
    return o;
}

static char *putu(char *o, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0;

    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v);
    while (n)
        *o++ = tmp[--n];
    return o;
}

static char *puthex4(char *o, uint16_t v)
{
    static const char h[] = "0123456789ABCDEF";

    *o++ = h[(v >> 12) & 0x0Fu];
    *o++ = h[(v >>  8) & 0x0Fu];
    *o++ = h[(v >>  4) & 0x0Fu];
    *o++ = h[ v        & 0x0Fu];
    return o;
}

static char *puthex2(char *o, uint8_t v)
{
    static const char h[] = "0123456789ABCDEF";

    *o++ = h[(v >> 4) & 0x0Fu];
    *o++ = h[ v       & 0x0Fu];
    return o;
}

/* Some BK4819 parts shift each FIFO word out LSB first.  rev16 turns the
 * received word back into the MSB-first air order POCSAG uses. */
static uint16_t rev16(uint16_t v)
{
    v = (uint16_t)(((v & 0x5555u) << 1) | ((v >> 1) & 0x5555u));
    v = (uint16_t)(((v & 0x3333u) << 2) | ((v >> 2) & 0x3333u));
    v = (uint16_t)(((v & 0x0F0Fu) << 4) | ((v >> 4) & 0x0F0Fu));
    return (uint16_t)((v << 8) | (v >> 8));
}

#ifdef ENABLE_LBJ_HWSYNC
/* Bit at index `i` of the 512-bit batch payload, MSB first inside each word. */
static uint8_t lbj_payload_bit(const uint16_t *w, uint16_t i)
{
    return (uint8_t)((w[i >> 4] >> (15u - (i & 15u))) & 1u);
}

/* 32 bits starting at `bit`, zero-padded past the end of the batch. */
static uint32_t lbj_word_at(const uint16_t *w, uint16_t bit)
{
    uint32_t cw = 0;
    for (uint8_t b = 0; b < 32u; b++) {
        const uint16_t i = (uint16_t)(bit + b);
        cw = (cw << 1) | (i < LBJ_BATCH_BITS ? lbj_payload_bit(w, i) : 0u);
    }
    return cw;
}

/* Known LBJ address codewords: 1233999 and 1234000, all four functions.
 * 1234001/1234002 share the 1234000 codewords; the frame slot, not the
 * codeword, selects the exact address.  These are constants from the POCSAG
 * BCH encoder, so comparing against them is a measurement, not a heuristic. */
static const uint32_t LBJ_ADDR_CW[8] = {
    0x4B5122C4u, 0x4B512C17u, 0x4B5131B2u, 0x4B513F61u,
    0x4B514723u, 0x4B5149F0u, 0x4B515455u, 0x4B515A86u
};

/* 32-bit popcount (same as lbj_dec.c; kept local so the measurement does not
 * depend on the decoder's internals). */
static uint8_t lbj_pc32(uint32_t x)
{
    x = x - ((x >> 1) & 0x55555555u);
    x = (x & 0x33333333u) + ((x >> 2) & 0x33333333u);
    x = (x + (x >> 4)) & 0x0F0F0F0Fu;
    return (uint8_t)((x * 0x01010101u) >> 24);
}

/* Scan all 32 shifts and all 16 codewords for the closest known LBJ address
 * codeword.  Returns the minimum Hamming distance and stores the shift that
 * produced it.  D <= 2 means the address word is present at that shift;
 * a large D means the raw bits are noise or the front end is wrong. */
static uint8_t lbj_measure_addr(const uint16_t *w, bool inv, uint8_t *shift)
{
    uint8_t bestD = 32u;
    uint8_t bestS = 0u;

    for (uint8_t sh = 0; sh < 32u; sh++) {
        for (uint8_t j = 0; j < 16u; j++) {
            uint32_t cw = lbj_word_at(w, (uint16_t)(sh + j * 32u));
            if (inv)
                cw = ~cw;

            for (uint8_t k = 0; k < 8u; k++) {
                const uint8_t d = lbj_pc32(cw ^ LBJ_ADDR_CW[k]);
                if (d < bestD) {
                    bestD = d;
                    bestS = sh;
                    if (!d) {
                        *shift = bestS;
                        return bestD;
                    }
                }
            }
        }
    }

    *shift = bestS;
    return bestD;
}
#endif /* ENABLE_LBJ_HWSYNC */

/* ---- FSK engine -------------------------------------------------------- */

/* Apply the RX front end selected by the config.  C0..C6 keep the original
 * AirCopy/RAW values; C7 (flags=1) is the POCSAG P profile from the BK4819
 * datasheet and the Manawyrm POCSAG transmitter:
 *   - 7.5 kHz RF filter instead of 6 kHz
 *   - 64-byte payload instead of 63
 *   - RX FIFO almost-full at 4 words
 *   - DC filter and ALC off, FM sub-audio filters off
 *   - 7-byte preamble comes from BK4819_PrepareFSKReceive() itself */
static void lbj_apply_frontend(const lbj_cfg_t *c)
{
    const bool p = c->flags != 0u;

    BK4819_WriteRegister(BK4819_REG_5D, p ? 0x4000u : 0x3F00u);
    BK4819_WriteRegister(BK4819_REG_5E, p ? 0x0204u : 0x0201u);
    BK4819_WriteRegister(BK4819_REG_43, c->reg43);
    BK4819_WriteRegister(BK4819_REG_3D, 0x2AABu);

    uint16_t reg2b = BK4819_ReadRegister(BK4819_REG_2B);
    if (p) {
        reg2b = (uint16_t)((reg2b & 0xF8F8u) | 0x0707u);
    } else {
        reg2b |= (1u << 10) | (1u << 9) | (1u << 8);
        reg2b &= ~((1u << 2) | (1u << 1) | (1u << 0));
    }
    BK4819_WriteRegister(BK4819_REG_2B, reg2b);

    if (p) {
        const uint16_t reg7e = BK4819_ReadRegister(BK4819_REG_7E);
        BK4819_WriteRegister(BK4819_REG_7E, (uint16_t)(reg7e & 0xFFC0u));
    }
}

#ifdef ENABLE_LBJ_HWSYNC
static void lbj_fsk_rearm(void)
{
    /* Full AirCopy-style reset between batches.  The light 0x4068/0x3068
     * re-arm left the FIFO and the bit clock in whatever state the previous
     * frame ended in, so after a couple of reports the codeword alignment
     * drifted and only a radio restart recovered.  PrepareFSKReceive()
     * resets the engine, clears the FIFO and re-enables RX. */
    BK4819_PrepareFSKReceive();

    lbj_apply_frontend(&LBJ_CFG[g.cfg % LBJ_CFG_COUNT]);

    const uint16_t mask = BK4819_ReadRegister(BK4819_REG_3F);
    BK4819_WriteRegister(BK4819_REG_3F, (uint16_t)(mask | LBJ_IRQ_MASK));
}
#else
/* Program the BK4819 as a plain slicer: FSK RX on, hardware sync off, FIFO
 * almost-full at one word so the main loop sees every sliced word.  Called
 * once per start, not per batch - the engine stays up until the squelch
 * closes, the channel changes or PTT is pressed. */
static void lbj_raw_config(const lbj_cfg_t *c)
{
    BK4819_ResetFSK();                    /* REG_3F = 0, REG_59 = 0x0068, idle */

    BK4819_WriteRegister(BK4819_REG_70, c->reg70);
    BK4819_WriteRegister(BK4819_REG_72, 0x3065u);   /* tone2 1200 baud       */
    BK4819_WriteRegister(BK4819_REG_58, c->reg58);
    BK4819_WriteRegister(BK4819_REG_5C, c->reg5c);

    BK4819_RX_TurnOn();                   /* REG_30 = the RX chain (BFF1)  */
    lbj_apply_frontend(c);

    /* One-word FIFO threshold and no hardware sync: REG_5D/5A/5B are unused
     * in this mode, REG_59 goes last so the FIFO starts clean. */
    BK4819_WriteRegister(BK4819_REG_5E, 0x0201u);
    BK4819_WriteRegister(BK4819_REG_59, LBJ_REG59_RAW);
}
#endif /* ENABLE_LBJ_HWSYNC */

static void lbj_start(void)
{
    if (g.on)
        return;

    memset(&g.rx, 0, sizeof(g.rx));

    const lbj_cfg_t *c = &LBJ_CFG[g.cfg % LBJ_CFG_COUNT];

#ifdef ENABLE_LBJ_HWSYNC
    BK4819_ResetFSK();

    /* Direct FSK 1.2K, 1200 baud, POCSAG sync, no CRC, 64-byte payload. */
    BK4819_WriteRegister(BK4819_REG_70, c->reg70);
    BK4819_WriteRegister(BK4819_REG_72, 0x3065u);   /* tone2 1200 baud       */
    BK4819_WriteRegister(BK4819_REG_58, c->reg58);
    BK4819_WriteRegister(BK4819_REG_5A, 0x7CD2u);   /* POCSAG sync 7CD215D8  */
    BK4819_WriteRegister(BK4819_REG_5B, 0x15D8u);
    BK4819_WriteRegister(BK4819_REG_5C, c->reg5c);

    BK4819_PrepareFSKReceive();
    lbj_apply_frontend(c);
#else
    lbj_raw_config(c);

    g.irqCount   = 0u;
    g.fskCount   = 0u;
    g.otherCount = 0u;
    g.finCount   = 0u;
    g.fifoWords  = 0u;
    g.ovCount    = 0u;
#endif

    const uint16_t mask = BK4819_ReadRegister(BK4819_REG_3F);
    BK4819_WriteRegister(BK4819_REG_3F, (uint16_t)(mask | LBJ_IRQ_MASK));

    g.on       = true;
    g.rawCount = 0;
    g.fill     = 0;
    g.lastFreq = gRxVfo->pRX->Frequency;
}

#ifdef ENABLE_LBJ_HWSYNC
static uint8_t lbj_decode_batch(uint8_t nwords, bool complete);
#endif

void LBJ_StopSampler(void)
{
    if (!g.on)
        return;

    /* Drain what the engine already handed over.  A PTT, a channel change or
     * a squelch close can land in the last codewords of a batch, and then the
     * hardware re-arm never runs: decode the complete codewords we have so the
     * report is not lost.  No re-arm and no dump on this path. */
#ifdef ENABLE_LBJ_HWSYNC
    lbj_decode_batch(g.fill, false);
#else
    lbj_process(&g.rx);      /* close the report the stream is in the middle of */
#endif

    g.on = false;

    BK4819_WriteRegister(BK4819_REG_59, 0x0000u);
    BK4819_WriteRegister(BK4819_REG_58, 0x0000u);
    BK4819_WriteRegister(BK4819_REG_70, 0x0000u);

    const uint16_t mask = BK4819_ReadRegister(BK4819_REG_3F);
    BK4819_WriteRegister(BK4819_REG_3F, (uint16_t)(mask & ~LBJ_IRQ_MASK));
}

/* ---- decoder output ---------------------------------------------------- */

void lbj_emit_msg(const lbj_rx_t *r, const char *bcd, uint16_t len)
{
    uint16_t n = len;

    if (n > (uint16_t)(sizeof(g.bcd) - 1u))
        n = (uint16_t)(sizeof(g.bcd) - 1u);

    memcpy(g.bcd, bcd, n);
    g.bcd[n] = '\0';
    g.bcdLen = (uint8_t)n;
    g.addr   = r->addr;

    /* A train sends the 15-char short block (1233999/1234000) followed by a
     * 30..65-char detail block (1234001/1234002).  Keep the short block in
     * its own buffer so the detail page cannot overwrite the train number /
     * speed / km readout with '?' when only the detail block fails. */
    if (r->addr == 1233999u || r->addr == 1234000u) {
        memcpy(g.bcdShort, bcd, n);
        g.bcdShort[n] = '\0';
        g.bcdShortLen = (uint8_t)n;
        g.shortAddr   = r->addr;
    }

#ifndef ENABLE_LBJ_HWSYNC
    g.frames++;          /* raw mode: one report, not one hardware batch */
#endif

    g.dirty  = true;
}

/* ---- transmitter ------------------------------------------------------- */

#ifdef ENABLE_LBJ_TX
/* POCSAG batch for the fixed LBJ test message:
 *   address 1234000, function 3, short block "123400080 01234".
 * Slot 0 is the address word, slots 1..3 the message words, slots 4..15 the
 * idle word.  These are the corrected codewords the K5 RX decodes. */
static const uint32_t LBJ_TX_BATCH[16] = {
    0x4B515A86u, 0xC2610751u, 0x80081E6Au, 0x842613B7u,
    0x7A89C197u, 0x7A89C197u, 0x7A89C197u, 0x7A89C197u,
    0x7A89C197u, 0x7A89C197u, 0x7A89C197u, 0x7A89C197u,
    0x7A89C197u, 0x7A89C197u, 0x7A89C197u, 0x7A89C197u,
};

static void lbj_tx_config(void)
{
    BK4819_ResetFSK();

    BK4819_WriteRegister(BK4819_REG_70, 0x00C3u);   /* tone2 on, gain       */
    BK4819_WriteRegister(BK4819_REG_72, 0x3065u);   /* 1200 baud            */
    BK4819_WriteRegister(BK4819_REG_58, 0x00C1u);   /* FSK 1.2K TX/RX       */
    BK4819_WriteRegister(BK4819_REG_5C, 0xAA30u);   /* no CRC               */
    BK4819_WriteRegister(BK4819_REG_5D, 0x3F00u);   /* 64 bytes = 16 words  */
    BK4819_WriteRegister(BK4819_REG_5E, 0x3204u);
    BK4819_WriteRegister(BK4819_REG_5A, 0x7CD2u);   /* POCSAG sync          */
    BK4819_WriteRegister(BK4819_REG_5B, 0x15D8u);
}

static void lbj_tx_wait(void)
{
    for (uint16_t t = 0; t < 400u; t++) {
        if (BK4819_ReadRegister(BK4819_REG_0C) & 1u)
            break;
        SYSTEM_DelayMs(5);
    }
    BK4819_WriteRegister(BK4819_REG_02, 0x0000u);
}

static void lbj_tx_send(void)
{
    if (!LBJ_RxVfoIsLbj())
        return;

    if (gBatteryDisplayLevel == 0 || gBatteryDisplayLevel > 6) {
        g.txState   = 3u;
        gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
        g.dirty     = true;
        return;
    }

    /* The branch keeps the CN HAM F Lock for every normal mode.  The LBJ
     * test transmission is the one deliberate exception: release the lock
     * only while this frame is on air, then put it back. */
    const uint8_t savedLock = gSetting_F_LOCK;
    gSetting_F_LOCK = F_LOCK_NONE;

    if (TX_freq_check(gCurrentVfo->pTX->Frequency) != 0) {
        gSetting_F_LOCK = savedLock;
        g.txState   = 3u;
        gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
        g.dirty     = true;
        return;
    }

    LBJ_StopSampler();

    g.txState = 1u;
    g.txTick  = 0u;

    /* Configure the FSK modem before keying the PA (the beam/tdcstx order). */
    lbj_tx_config();

    RADIO_SetTxParameters();

    BK4819_WriteRegister(BK4819_REG_51, 0x0000u);   /* no CTCSS/DCS        */

    /* Packet 1: a long preamble.  The engine's own preamble is only 7
     * bytes; POCSAG wants 576 bits (72 bytes).  The sync word is set to
     * 0xAAAAAAAA here so the engine's 4-byte sync is just more 1010...
     * preamble, not a premature POCSAG sync.  Total: 7 + 4 + 72 bytes. */
    BK4819_WriteRegister(BK4819_REG_5A, 0xAAAAu);
    BK4819_WriteRegister(BK4819_REG_5B, 0xAAAAu);
    BK4819_WriteRegister(BK4819_REG_5D, 0x4700u);   /* 72 bytes            */
    BK4819_WriteRegister(BK4819_REG_3F, 0x8000u);
    BK4819_WriteRegister(BK4819_REG_59, 0x8068u);
    BK4819_WriteRegister(BK4819_REG_59, 0x0068u);

    for (uint8_t i = 0; i < 36u; i++)
        BK4819_WriteRegister(BK4819_REG_5F, 0xAAAAu);

    SYSTEM_DelayMs(20);
    BK4819_WriteRegister(BK4819_REG_59, 0x2868u);
    lbj_tx_wait();

    /* Packet 2: the real POCSAG sync and the 16 codewords. */
    BK4819_WriteRegister(BK4819_REG_5A, 0x7CD2u);
    BK4819_WriteRegister(BK4819_REG_5B, 0x15D8u);
    BK4819_WriteRegister(BK4819_REG_5D, 0x3F00u);   /* 64 bytes            */
    BK4819_WriteRegister(BK4819_REG_3F, 0x8000u);
    BK4819_WriteRegister(BK4819_REG_59, 0x8068u);
    BK4819_WriteRegister(BK4819_REG_59, 0x0068u);

    for (uint8_t i = 0; i < 16u; i++) {
        BK4819_WriteRegister(BK4819_REG_5F,
                             (uint16_t)(LBJ_TX_BATCH[i] >> 16));
        BK4819_WriteRegister(BK4819_REG_5F,
                             (uint16_t)LBJ_TX_BATCH[i]);
    }

    SYSTEM_DelayMs(20);
    BK4819_WriteRegister(BK4819_REG_59, 0x2868u);   /* FSK TX enable       */
    lbj_tx_wait();

    SYSTEM_DelayMs(20);
    BK4819_ResetFSK();
    RADIO_SetupRegisters(true);

    gSetting_F_LOCK = savedLock;

    g.txState   = 2u;
    g.txTick    = 0u;
    gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
    g.dirty     = true;
}
#endif /* ENABLE_LBJ_TX */

bool LBJ_HandleKey(uint8_t key, bool pressed, bool held)
{
    if (!LBJ_ScreenActive())
        return false;

    if (key == KEY_PTT) {
#ifdef ENABLE_LBJ_TX
        if (pressed && !held)
            lbj_tx_send();
#endif
        return true;      /* receive-only build: swallow PTT */
    }

#ifndef ENABLE_LBJ_HWSYNC
    /* Long STAR press: the DIAG page with the same numbers the reference build
     * shows.  KEY_2 zeroes the counters so a capture starts from a clean
     * state.  Both are raw-mode only; the batch path has nothing to show. */
    if (key == KEY_STAR && held) {
        g.diag ^= 1u;
        g.dirty = true;
        return true;
    }
    if (key == KEY_2 && pressed && !held) {
        g.irqCount   = 0u;
        g.fskCount   = 0u;
        g.otherCount = 0u;
        g.finCount   = 0u;
        g.fifoWords  = 0u;
        g.ovCount    = 0u;
        g.frames     = 0u;
        lbj_stat_sync = 0u;
        lbj_stat_ok   = 0u;
        lbj_stat_bad  = 0u;
        g.dirty = true;
        return true;
    }
#endif

    if (!pressed || held)
        return false;

    if (key == KEY_UP || key == KEY_DOWN) {
        if (key == KEY_UP)
            g.cfg = (uint8_t)((g.cfg + LBJ_CFG_COUNT - 1u) % LBJ_CFG_COUNT);
        else
            g.cfg = (uint8_t)((g.cfg + 1u) % LBJ_CFG_COUNT);
        LBJ_StopSampler();
        memset(&g.rx, 0, sizeof(g.rx));
        g.dirty = true;
        return true;
    }

    if (key == KEY_STAR) {
        g.order = (uint8_t)((g.order + 1u) & 3u);
        memset(&g.rx, 0, sizeof(g.rx));
        g.dirty = true;
        return true;
    }

    return false;
}

/* ---- interrupts -------------------------------------------------------- */

/* One line per batch on the 2.5 mm K-head UART (38400 8N1).  The 32 FIFO
 * words are dumped raw, before rev16/inversion, so the PC can replay every
 * shift, bit order and polarity offline. */
static void lbj_dump_batch(void)
{
    static uint16_t lastSec = 0xFFFFu;
    char line[48];
    char *o;

    /* One dump per second is enough for a capture and keeps the blocking
     * UART send from stalling the main loop on noise batches. */
    if (lastSec == g.sec)
        return;
    lastSec = g.sec;

    o = line;
    o = put(o, "LBJ C");
    o = putu(o, g.cfg);
    *o++ = ' ';
    *o++ = "MLIJ"[g.order & 3u];
    o = put(o, " S");
    o = putu(o, g.shift);
    o = put(o, " D");
    o = putu(o, g.addrDist);
    o = put(o, " IRQ");
    o = puthex4(o, g.lastIrq);
    o = put(o, " ST");
    o = puthex4(o, g.status);
    *o++ = '\r';
    *o++ = '\n';
    UART_Send(line, (uint32_t)(o - line));

    for (uint8_t row = 0; row < 4u; row++) {
        o = line;
        *o++ = 'W';
        *o++ = (char)('0' + row);
        *o++ = ':';
        *o++ = ' ';
        for (uint8_t i = 0; i < 8u; i++) {
            o = puthex4(o, g.raw[row * 8u + i]);
            *o++ = ' ';
        }
        *o++ = '\r';
        *o++ = '\n';
        UART_Send(line, (uint32_t)(o - line));
    }
}

#ifdef ENABLE_LBJ_HWSYNC
/* Decode the words collected in g.raw.  `complete` means the engine reported
 * the end of the batch: the raw words are shown, the batch counter advances,
 * the engine is re-armed and the capture is dumped.  A stop-time flush passes
 * complete=false so it only decodes what already arrived. */
static uint8_t lbj_decode_batch(uint8_t nwords, bool complete)
{
    if (nwords < 2u)
        return 0u;

    const bool lsb = (g.order & 1u) != 0u;
    const bool inv = (g.order & 2u) != 0u;

    /* Rebuild the batch in air order, then let the BCH score all 32 bit
     * offsets.  The hardware sync can be one or more bits away from the real
     * codeword boundary; without this search a one-bit offset turns every
     * codeword into garbage. */
    uint16_t words[LBJ_BATCH_WORDS];
    for (uint8_t n = 0; n < nwords; n++)
        words[n] = lsb ? rev16(g.raw[n]) : g.raw[n];
    for (uint8_t n = nwords; n < LBJ_BATCH_WORDS; n++)
        words[n] = 0u;      /* do not score the previous batch's tail */

    uint8_t measuredShift = 0u;
    const uint8_t addrDist = lbj_measure_addr(words, inv, &measuredShift);

    g.addrDist = addrDist;
    g.shift    = measuredShift;

    /* Decode at the shift the address measurement chose, so the S<shift> on
     * screen is what the decoder actually used.  A noise-only batch is
     * rejected by the BCH either way. */
    const uint8_t shift = measuredShift;

    /* Every batch starts with its own sync: reset the decoder so a previous
     * frame's half-finished state cannot leak into this one. */
    memset(&g.rx, 0, sizeof(g.rx));

    /* Prepend the POCSAG sync.  For an inverted signal, prepend the inverted
     * sync: lbj_dec then sets pol=0 and inverts every queued codeword back to
     * normal.  (Prepending the normal sync would pin the decoder to pol=1 and
     * make inverted signals undecodable.) */
    const uint32_t sync = inv ? ~POCSAG_SYNC : POCSAG_SYNC;
    for (int b = 31; b >= 0; b--)
        lbj_rx_bit(&g.rx, (uint8_t)((sync >> b) & 1u));
    lbj_process(&g.rx);

    const uint8_t ncodewords = (uint8_t)(nwords / 2u);
    for (uint8_t j = 0; j < ncodewords; j++) {
        const uint16_t start = (uint16_t)(shift + j * 32u);

        for (uint8_t b = 0; b < 32u; b++) {
            const uint16_t i = (uint16_t)(start + b);
            const uint8_t bit =
                i < LBJ_BATCH_BITS ? lbj_payload_bit(words, i) : 0u;
            lbj_rx_bit(&g.rx, bit);
        }
        lbj_process(&g.rx);
    }

    g.fill = 0;

    if (complete) {
        g.rawCount = LBJ_BATCH_WORDS;
        g.frames++;

        lbj_fsk_rearm();
        g.dirty = true;
        lbj_dump_batch();
    }

    return ncodewords;
}
#endif /* ENABLE_LBJ_HWSYNC */

#ifndef ENABLE_LBJ_HWSYNC
/* Push one sliced word into the decoder in air order.  g.order keeps the
 * MSB/LSB and normal/inverted variants the batch path had, so a wrong FIFO bit
 * order or a wrong polarity is still one STAR press away. */
static void lbj_raw_word(uint16_t w)
{
    /* Keep the last 32 words for the raw row and the UART dump. */
    for (uint8_t i = 0; i + 1u < LBJ_BATCH_WORDS; i++)
        g.raw[i] = g.raw[i + 1u];
    g.raw[LBJ_BATCH_WORDS - 1u] = w;
    if (g.rawCount < LBJ_BATCH_WORDS)
        g.rawCount++;

    if (g.order & 1u)
        w = rev16(w);
    if (g.order & 2u)
        w = (uint16_t)~w;

    for (int b = 15; b >= 0; b--)
        lbj_rx_bit(&g.rx, (uint8_t)((w >> b) & 1u));

    g.fifoWords++;
}
#endif /* !ENABLE_LBJ_HWSYNC */

void LBJ_HandleInterrupts(uint16_t bits)
{
    if (!g.on)
        return;

    g.lastIrq = bits;

#ifdef ENABLE_LBJ_HWSYNC
    /* Drain the FIFO as words arrive: the engine's FIFO is much smaller than
     * one 68-byte batch, so waiting for the finished interrupt would lose
     * most of it.  REG_5E is programmed to a one-word threshold. */
    if (bits & LBJ_IRQ_FIFO) {
        const uint16_t count = BK4819_ReadRegister(BK4819_REG_5E) & 7u;
        for (uint16_t i = 0; i < count && g.fill < LBJ_BATCH_WORDS; i++)
            g.raw[g.fill++] = BK4819_ReadRegister(BK4819_REG_5F);
    }

    if (bits & LBJ_IRQ_DONE) {
        g.status = BK4819_ReadRegister(BK4819_REG_0B);

        while (g.fill < LBJ_BATCH_WORDS)
            g.raw[g.fill++] = BK4819_ReadRegister(BK4819_REG_5F);

        lbj_decode_batch(LBJ_BATCH_WORDS, true);
    }
#else
    g.irqCount++;

    if (bits & LBJ_IRQ_FIFO) {
        g.fskCount++;

        /* Drain everything the slicer handed over.  REG_5E bits 2:0 is the
         * current FIFO level: with a one-word threshold it is normally 1, but
         * a late main loop can leave more behind.  The guard stops a stuck
         * level from spinning here forever. */
        for (uint8_t guard = 0; guard < 32u; guard++) {
            uint16_t n = BK4819_ReadRegister(BK4819_REG_5E) & 7u;

            if (!n)
                break;
            if (n >= 7u)          /* FIFO full: a word may have been lost */
                g.ovCount++;
            while (n--)
                lbj_raw_word(BK4819_ReadRegister(BK4819_REG_5F));
        }

        /* Main-loop context, same as the TDCS decoder: run the BCH and the
         * word state machine right after the words were queued. */
        lbj_process(&g.rx);
    } else if (bits & (LBJ_IRQ_SYNC | LBJ_IRQ_DONE)) {
        g.finCount++;
    } else {
        g.otherCount++;
    }
#endif
}

/* ---- tick -------------------------------------------------------------- */

bool LBJ_RxVfoIsLbj(void)
{
    return gRxVfo && gRxVfo->Modulation == MODULATION_LBJ;
}

bool LBJ_ScreenActive(void)
{
    return gScreenToDisplay == DISPLAY_MAIN && LBJ_RxVfoIsLbj();
}

void LBJ_Tick(void)
{
    if (++g.subSec >= 100u) {
        g.subSec = 0u;
        g.sec++;
    }

    if (gCurrentFunction == FUNCTION_TRANSMIT || gPttIsPressed) {
        LBJ_StopSampler();
        return;
    }
    if (!LBJ_RxVfoIsLbj()) {
        LBJ_StopSampler();
        return;
    }

    /* Like the K1 LBJ sampler: only run while the squelch is open.  If the
     * engine kept running through power save, CheckRadioInterrupts() was
     * skipped and the FSK FIFO filled with stale words; the next report then
     * read garbage until the radio was restarted.
     *
     * A short carrier gap between the short block and its detail block must
     * not cut the batch: keep the engine up for 2 s (200 ticks), the same
     * debounce the K1 LBJRadio uses.  Power save starts after 10 s, so the
     * debounce never overlaps it. */
    if (g_SquelchLost) {
        if (++g.lostTicks >= 200u)
            LBJ_StopSampler();
        return;
    }
    g.lostTicks = 0u;

    /* A retune resets REG_43/REG_3D; restart the FSK engine so the filter
     * and the batch setup are applied again. */
    if (g.on && g.lastFreq != gRxVfo->pRX->Frequency) {
        LBJ_StopSampler();
    }

    if (!g.on)
        lbj_start();

    if (g.txState) {
        if (++g.txTick >= 300u) {      /* 3 s */
            g.txState      = 0u;
            g.txTick       = 0u;
            gUpdateDisplay = true;
        }
    }

#ifndef ENABLE_LBJ_HWSYNC
    /* One raw-word dump per second on the K-head UART, the same format the
     * batch build used: a capture can be replayed on the PC to check bit
     * order and polarity offline.  lbj_dump_batch() self-limits to 1/s. */
    if (g.on)
        lbj_dump_batch();
#endif

    if (g.dirty) {
        g.dirty        = false;
        gUpdateDisplay = true;
    }
}

/* ---- screen ------------------------------------------------------------ */

#ifndef ENABLE_LBJ_HWSYNC
/* The reference DIAG page: the same registers and counters, so a capture can
 * be compared field by field with the build that decodes. */
static void lbj_draw_diag(void)
{
    char s[24];
    char *o;

    o = s;
    o = put(o, "R58:");
    o = puthex4(o, BK4819_ReadRegister(BK4819_REG_58));
    o = put(o, " R59:");
    o = puthex4(o, BK4819_ReadRegister(BK4819_REG_59));
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, 0);

    const int16_t dbm = BK4819_GetRSSI_dBm();

    o = s;
    o = put(o, "R30:");
    o = puthex4(o, BK4819_ReadRegister(BK4819_REG_30));
    o = put(o, " R");
    o = putu(o, BK4819_GetRSSI());
    *o++ = ' ';
    if (dbm < 0) {
        *o++ = '-';
        o = putu(o, (uint32_t)(-dbm));
    } else {
        o = putu(o, (uint32_t)dbm);
    }
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, 1);

    o = s;
    o = put(o, "IRQ ");
    o = putu(o, g.irqCount);
    o = put(o, " FSK ");
    o = putu(o, g.fskCount);
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, 2);

    o = s;
    o = put(o, "FIF ");
    o = putu(o, g.fifoWords);
    o = put(o, " OV ");
    o = putu(o, g.ovCount);
    o = put(o, " SYNC ");
    o = putu(o, lbj_stat_sync);
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, 3);

    o = s;
    o = put(o, "OK ");
    o = putu(o, lbj_stat_ok);
    o = put(o, " BAD ");
    o = putu(o, lbj_stat_bad);
    o = put(o, " FIN ");
    o = putu(o, g.finCount);
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, 4);

    o = s;
    o = put(o, "B");
    o = putu(o, (uint32_t)gRxVfo->Band + 1u);
    o = put(o, " C");
    o = putu(o, g.cfg);
    o = put(o, " M");
    *o++ = "MLIJ"[g.order & 3u];
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, 5);

    UI_PrintStringSmallNormal("STAR:DIAG 2:CLR", 0, 0, 6);

    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}
#endif /* !ENABLE_LBJ_HWSYNC */

void LBJ_Draw(void)
{
    char s[40];
    char *o;

    UI_DisplayClear();

#ifndef ENABLE_LBJ_HWSYNC
    if (g.diag) {
        lbj_draw_diag();
        return;
    }
#endif

    o = s;
    o = put(o, "A");
    if (g.shortAddr)
        o = putu(o, g.shortAddr);
    else
        o = put(o, "-------");
    o = put(o, " S");
    o = putu(o, g.shift);
    o = put(o, " D");
    o = putu(o, g.addrDist);
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, 0);

    o = s;
    o = put(o, "IRQ ");
    o = puthex4(o, g.lastIrq);
    o = put(o, " ST ");
    o = puthex4(o, g.status);
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, 1);

    /* Three 16-bit words per row: four of them (19 chars, 133 px) run off
     * the 128 px display and merge the last two words. */
    for (uint8_t row = 0; row < 2u; row++) {
        o = s;
        for (uint8_t i = 0; i < 3u; i++) {
            const uint8_t k = (uint8_t)(row * 3u + i);
            if (k >= g.rawCount)
                break;
            o = puthex4(o, g.raw[k]);
            *o++ = ' ';
        }
        *o = '\0';
        UI_PrintStringSmallNormal(s, 0, 0, (uint8_t)(2u + row));
    }

    const char *bcd  = g.bcdShortLen ? g.bcdShort : g.bcd;
    const uint8_t blen = g.bcdShortLen ? g.bcdShortLen : g.bcdLen;

    if (blen) {
        char line[20];
        uint8_t n = (blen > 18u) ? 18u : blen;

        memcpy(line, bcd, n);
        line[n] = '\0';
        UI_PrintStringSmallNormal(line, 0, 0, 4);

        if (blen > 18u) {
            n = (uint8_t)(blen - 18u);
            if (n > 18u)
                n = 18u;
            memcpy(line, bcd + 18u, n);
            line[n] = '\0';
            UI_PrintStringSmallNormal(line, 0, 0, 5);
        }
    } else {
        UI_PrintStringSmallNormal("WAIT SHORT...", 0, 0, 4);
    }

    o = s;
    o = put(o, "F");
    o = putu(o, g.frames);
    o = put(o, " C");
    o = putu(o, g.cfg);
    o = put(o, " 5C=");
    o = puthex2(o, (uint8_t)LBJ_CFG[g.cfg % LBJ_CFG_COUNT].reg5c);
    static const char ORDER_CHAR[] = "MLIJ";
    *o++ = ' ';
    *o++ = ORDER_CHAR[g.order & 3u];
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, 6);

    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}
