/* LBJ / POCSAG receive for the classic UV-K5 - see lbj.h.
 *
 * The K1 LBJRadio samples the BK4819 AF output on PA4 at 19.2 kHz.  The
 * classic UV-K5 has no audio-to-ADC path, so this port uses the BK4819 FSK
 * engine in direct-FSK 1.2K mode (the AirCopy engine) and feeds the FIFO
 * codewords to the K1 POCSAG decoder (lbj_dec.c).
 *
 * This is the first, diagnostic build: it prints the raw FIFO words and the
 * decoded BCD string, so the bit order and the sync can be checked on air
 * before the report parser and the Chinese popup are added.
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

#define POCSAG_SYNC      0x7CD215D8u

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
    char     bcd[72];
    uint8_t  bcdLen;
    uint32_t addr;
    uint32_t lastFreq;
    uint8_t  txState;    /* 0 idle, 1 sending, 2 sent, 3 denied */
    uint16_t txTick;
} g;

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

static char *putfreq(char *o, uint32_t f)
{
    /* f is in 10 Hz units: 40000000 -> 400.0000 */
    o = putu(o, f / 100000u);
    *o++ = '.';
    const uint32_t r = f % 100000u;
    *o++ = (char)('0' + (r / 10000u) % 10u);
    *o++ = (char)('0' + (r /  1000u) % 10u);
    *o++ = (char)('0' + (r /   100u) % 10u);
    *o++ = (char)('0' + (r /    10u) % 10u);
    return o;
}

/* ---- FSK engine -------------------------------------------------------- */

static void lbj_fsk_rearm(void)
{
    /* Full AirCopy-style reset between batches.  The light 0x4068/0x3068
     * re-arm left the FIFO and the bit clock in whatever state the previous
     * frame ended in, so after a couple of reports the codeword alignment
     * drifted and only a radio restart recovered.  PrepareFSKReceive()
     * resets the engine, clears the FIFO and re-enables RX. */
    BK4819_PrepareFSKReceive();

    BK4819_WriteRegister(BK4819_REG_43, 0x3628u);
    BK4819_WriteRegister(BK4819_REG_3D, 0x2AABu);

    const uint16_t mask = BK4819_ReadRegister(BK4819_REG_3F);
    BK4819_WriteRegister(BK4819_REG_3F, (uint16_t)(mask | LBJ_IRQ_MASK));
}

static void lbj_start(void)
{
    if (g.on)
        return;

    memset(&g.rx, 0, sizeof(g.rx));

    BK4819_ResetFSK();

    /* Direct FSK 1.2K, 1200 baud, POCSAG sync, no CRC, 64-byte payload. */
    BK4819_WriteRegister(BK4819_REG_70, 0x00E0u);   /* tone2 on, gain 48     */
    BK4819_WriteRegister(BK4819_REG_72, 0x3065u);   /* tone2 1200 baud       */
    BK4819_WriteRegister(BK4819_REG_58, 0x00C1u);   /* FSK 1.2K RX           */
    BK4819_WriteRegister(BK4819_REG_5A, 0x7CD2u);   /* POCSAG sync 7CD215D8  */
    BK4819_WriteRegister(BK4819_REG_5B, 0x15D8u);
    BK4819_WriteRegister(BK4819_REG_5C, 0xAA30u);   /* no CRC                */
    BK4819_WriteRegister(BK4819_REG_5D, 0x3F00u);   /* 64 bytes = 16 words   */
    BK4819_WriteRegister(BK4819_REG_5E, 0x0201u);   /* FIFO threshold 1 word */

    BK4819_PrepareFSKReceive();

    /* 25 kHz IF and the wide weak-signal filter, as in the BYP/RAW mode. */
    BK4819_WriteRegister(BK4819_REG_43, 0x3628u);
    BK4819_WriteRegister(BK4819_REG_3D, 0x2AABu);

    const uint16_t mask = BK4819_ReadRegister(BK4819_REG_3F);
    BK4819_WriteRegister(BK4819_REG_3F, (uint16_t)(mask | LBJ_IRQ_MASK));

    g.on       = true;
    g.rawCount = 0;
    g.fill     = 0;
    g.lastFreq = gRxVfo->pRX->Frequency;
}

void LBJ_StopSampler(void)
{
    if (!g.on)
        return;

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
#ifdef ENABLE_LBJ_TX
    if (key == KEY_PTT && LBJ_RxVfoIsLbj()) {
        if (pressed && !held)
            lbj_tx_send();
        return true;
    }
#else
    (void)key;
    (void)pressed;
    (void)held;
#endif
    return false;
}

/* ---- interrupts -------------------------------------------------------- */

void LBJ_HandleInterrupts(uint16_t bits)
{
    if (!g.on)
        return;

    g.lastIrq = bits;

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

        /* Every batch starts with its own sync: reset the decoder so a
         * previous frame's half-finished state cannot leak into this one. */
        memset(&g.rx, 0, sizeof(g.rx));

        /* Prepend the POCSAG sync.  If the engine's FIFO starts with the
         * sync as well, the decoder simply re-syncs on the real one. */
        for (int b = 31; b >= 0; b--)
            lbj_rx_bit(&g.rx, (uint8_t)((POCSAG_SYNC >> b) & 1u));
        lbj_process(&g.rx);

        for (uint8_t n = 0; n + 1u < LBJ_BATCH_WORDS; n = (uint8_t)(n + 2u)) {
            const uint32_t cw =
                ((uint32_t)g.raw[n] << 16) | (uint32_t)g.raw[n + 1u];

            for (int b = 31; b >= 0; b--)
                lbj_rx_bit(&g.rx, (uint8_t)((cw >> b) & 1u));
            lbj_process(&g.rx);
        }

        g.rawCount = LBJ_BATCH_WORDS;
        g.fill     = 0;
        g.frames++;

        lbj_fsk_rearm();
        g.dirty = true;
    }
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
     * read garbage until the radio was restarted. */
    if (g_SquelchLost) {
        LBJ_StopSampler();
        return;
    }

    /* A retune resets REG_43/REG_3D; restart the FSK engine so the 25 kHz
     * filter and the batch setup are applied again. */
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

    if (g.dirty) {
        g.dirty        = false;
        gUpdateDisplay = true;
    }
}

/* ---- screen ------------------------------------------------------------ */

void LBJ_Draw(void)
{
    char s[40];
    char *o;

    UI_DisplayClear();

    o = s;
    o = put(o, "LBJ ");
    o = putfreq(o, gRxVfo->pRX->Frequency);
    o = put(o, " F");
    o = putu(o, g.frames);
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

    if (g.bcdLen) {
        char line[20];
        uint8_t n = (g.bcdLen > 18u) ? 18u : g.bcdLen;

        memcpy(line, g.bcd, n);
        line[n] = '\0';
        UI_PrintStringSmallNormal(line, 0, 0, 4);

        if (g.bcdLen > 18u) {
            n = (uint8_t)(g.bcdLen - 18u);
            if (n > 18u)
                n = 18u;
            memcpy(line, g.bcd + 18u, n);
            line[n] = '\0';
            UI_PrintStringSmallNormal(line, 0, 0, 5);
        }
    } else {
        UI_PrintStringSmallNormal("WAIT POCSAG...", 0, 0, 4);
    }

    if (g.txState == 1u)
        UI_PrintStringSmallNormal("TX...", 0, 0, 6);
    else if (g.txState == 2u)
        UI_PrintStringSmallNormal("TX SENT", 0, 0, 6);
    else if (g.txState == 3u)
        UI_PrintStringSmallNormal("TX DENIED", 0, 0, 6);

    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}
