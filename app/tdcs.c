/* TDCS receive mode - see tdcs.h.
 *
 * The BK4819 FSK engine is configured with losehu's proven MDC1200/Messenger
 * RX recipe (driver/bk4819.c::enable_msg_rx): REG_58 = 0x3F03 (RX mode 111 =
 * FFSK 1200/1800), REG_5C = 0x5625, REG_5E = 0x0201 (FIFO almost full at one
 * word), REG_59 = 0xC008/0x1008 (4-byte sync, 1-byte preamble).  Only the
 * sync word and packet length differ for TDCS.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "tdcs.h"
#include "tdcs_dec.h"

#include "audio.h"
#include "driver/backlight.h"
#include "driver/bk4819.h"
#include "driver/keyboard.h"
#include "driver/st7565.h"
#include "frequencies.h"
#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "radio.h"
#include "settings.h"
#include "ui/helper.h"
#include "ui/main.h"
#include "ui/ui.h"

/* ---- receiver ---------------------------------------------------------- */

#define TDCS_HISTORY        8u
#define TDCS_PAGES          4u
#define TDCS_ROWS           6u
#define TDCS_FOOTER         6u
#define TDCS_FIELDS         20u
#define TDCS_MERGE_SECS     30u
#define TDCS_POPUP_SECS     10u   /* fixed: the K1 TCDPOP setting is skipped */
#define TDCS_STALL_TICKS    150u  /* 1.5 s without FIFO/sync = carrier gone  */

/* BK4819 FSK engine: losehu's MDC1200/Messenger RX recipe (driver/bk4819.c
 * enable_msg_rx), the proven FFSK 1200/1800 path. */
#define TDCS_FSK_TONE2_GAIN 0x00E0u  /* REG_70: tone2 on, gain 96             */
#define TDCS_FSK_TONE2_1200 0x3065u  /* REG_72: 1200 Hz / bit clock           */
#define TDCS_FSK_5C_NOCRC   0x5625u  /* losehu MDC1200 RX value, CRC off      */
#define TDCS_FSK_5D_75BYTE  0x4A00u  /* 75 bytes = 23 codewords               */
#define TDCS_FSK_5E_1WORD   0x0201u  /* RX FIFO almost full at 1 word         */
#define TDCS_IRQ_SYNC       0x0002u
#define TDCS_IRQ_FIFO       0x1000u
#define TDCS_IRQ_DONE       0x2000u
#define TDCS_IRQ_MASK       (TDCS_IRQ_SYNC | TDCS_IRQ_FIFO | TDCS_IRQ_DONE)

/* ---- test transmitter (same physical layer as the tdcstx app) ---------- */
#define TDCS_SYNC_TX        0x0DD4259Fu
#define TDCS_BCH_GEN        0x05B9u
#define TDCS_PREAMBLE       160u
#define TDCS_TX_LEVEL       66u
#define TDCS_CYC_PER_BIT    (48000000u / 1200u)
#define TDCS_SYST_LOAD      (*(volatile uint32_t *)0xE000E014u)
#define TDCS_SYST_VAL       (*(volatile uint32_t *)0xE000E018u)
#define TDCS_F_MARK         12389u
#define TDCS_F_SPACE        18584u

typedef struct {
    tdcs_msg_t m;
    uint16_t   sec;
    int16_t    rssi;
} tdcs_rec_t;

static struct {
    tdcs_rec_t rec[TDCS_HISTORY];
    uint8_t    count, cur, page;
    bool       popup, dismissed, pinned, dirty, on, capturing;
    uint8_t    idle, ledPhase, ledTick;
    uint16_t   sec, subSec, popSec;
} g;

/* ---- tiny helpers ------------------------------------------------------ */

static char *put(char *o, const char *s)
{
    while (*s)
        *o++ = *s++;
    return o;
}

static char *putu(char *o, uint32_t v)
{
    char t[10];
    uint8_t n = 0;

    if (!v) {
        *o++ = '0';
        return o;
    }
    while (v) {
        t[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (n)
        *o++ = t[--n];
    return o;
}

static char *puti(char *o, int32_t v)
{
    if (v < 0) {
        *o++ = '-';
        v = -v;
    }
    return putu(o, (uint32_t)v);
}

static char *puthex2(char *o, uint8_t v)
{
    static const char H[] = "0123456789ABCDEF";
    *o++ = H[v >> 4];
    *o++ = H[v & 0x0Fu];
    return o;
}

static char *putpad(char *o, uint32_t v, uint8_t width)
{
    char t[10];
    uint8_t n = 0;

    do {
        t[n++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v && n < 10u);
    while (n < width)
        t[n++] = '0';
    while (n)
        *o++ = t[--n];
    return o;
}

static char *putage(char *o, uint16_t sec)
{
    if (sec < 60u) {
        o = putu(o, sec);
        *o++ = 's';
        return o;
    }

    uint32_t v = sec;
    const unsigned m = (unsigned)(v / 60u);
    v %= 60u;
    if (m > 99u)
        return put(o, "99m+");
    o = putu(o, m);
    *o++ = 'm';
    if (v < 10u)
        *o++ = '0';
    o = putu(o, v);
    *o++ = 's';
    return o;
}

/* ---- field names ------------------------------------------------------- */

static const char *sig_name(uint8_t v)
{
    switch (v) {
        case 0: return "始发";
        case 1: return "调车";
        case 2: return "发车";
        case 3: return "到达";
        case 4: return "通过";
        case 5: return "预告";
        case 6: return "许可";
        default: return NULL;
    }
}

static const char *sig_type_name(uint8_t v)
{
    switch (v) {
        case 2: return "发车";
        case 3: return "到达";
        case 4: return "通过";
        case 5: return "预告";
        case 6: return "许可";
        default: return NULL;
    }
}

/* Short labels: an 8x8 Chinese glyph advances 9 px, so a four-character label
 * still leaves most of the 128 px row for the value.  Index 14/18 (STNEXT and
 * DRVEXT) are merged into 13/17 and never drawn. */
static const char *const TDCS_LABEL[TDCS_FIELDS] = {
    "信号机", "信号种类", "本补客货", "车次种类", "车次",
    "机车类型", "类型扩充", "机车号", "公里标", "速度",
    "总重", "计长", "辆数", "车站号", "",
    "区段号", "实际交路", "司机号", "",
    "状态"
};

/* Display order: the order in the frame is not the order an operator reads. */
static const uint8_t TDCS_PAGE_MAP[TDCS_PAGES][TDCS_ROWS] = {
    {  3,  4,  5,  6,  7,  8 },   /* CLASS TRAIN LOCO TYPE LOCO# KM   */
    {  9, 10, 11, 12, 13, 0xFF }, /* SPEED WEIGHT LENGTH CARS STN+EXT      */
    { 15, 16, 17, 0xFF,  0,  1 }, /* SECT ROUTE DRIVER+EXT SIGNAL SIGTYP   */
    {  2, 19, 0xFF, 0xFF, 0xFF, 0xFF }   /* CREW STATUS */
};

static void field_row(uint8_t row, const char *label, const char *value)
{
    UI_PrintStringSmallCN(label, 0, row);
    UI_PrintStringSmallCN(value, 52, row);
}

static void draw_field(uint8_t row, const tdcs_rec_t *r, uint8_t idx)
{
    const tdcs_msg_t *m = &r->m;
    char v[24];
    char *o;

    switch (idx) {
        case 0: {
            const char *n = sig_name(m->sig);
            if (!n) {
                o = v;
                o = put(o, "未知 ");
                o = putu(o, m->sig);
                *o = '\0';
                n = v;
            }
            field_row(row, TDCS_LABEL[idx], n);
            break;
        }
        case 1: {
            const char *n = sig_type_name(m->sig_type);
            if (!n) {
                o = v;
                o = put(o, "未知 ");
                o = putu(o, m->sig_type);
                *o = '\0';
                n = v;
            }
            field_row(row, TDCS_LABEL[idx], n);
            break;
        }
        case 2: {
            static const char *const CREW[4] = {
                "主货", "主客", "补货", "补客"
            };
            field_row(row, TDCS_LABEL[idx], CREW[m->crew & 3u]);
            break;
        }
        case 3: {
            bool any = false;
            o = v;
            for (uint8_t i = 0; i < 4u; i++) {
                const uint8_t c = (uint8_t)m->cls[i];
                if (c == ' ')
                    continue;
                *o++ = (c >= 0x20u && c < 0x7Fu) ? (char)c : '.';
                any = true;
            }
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], any ? v : "--");
            break;
        }
        case 4:
            if (m->train_no > 9999999u) {
                field_row(row, TDCS_LABEL[idx], "-------");
            } else {
                o = putpad(v, m->train_no, 7u);
                *o = '\0';
                field_row(row, TDCS_LABEL[idx], v);
            }
            break;
        case 5:
            o = v;
            o = putu(o, m->loco_type);
            o = put(o, " (");
            o = puthex2(o, m->loco_type);
            *o++ = ')';
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 6:
            o = putu(v, m->loco_ext);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 7:
            o = putu(v, m->loco_no);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 8:
            if (!m->km_valid) {
                field_row(row, TDCS_LABEL[idx], "----");
            } else {
                o = v;
                if (m->km_neg)
                    *o++ = '-';
                *o++ = 'K';
                o = putu(o, m->km_abs / 1000u);
                *o++ = '+';
                o = putpad(o, m->km_abs % 1000u, 3u);
                *o++ = m->km_inc ? '^' : 'v';
                *o = '\0';
                field_row(row, TDCS_LABEL[idx], v);
            }
            break;
        case 9:
            o = putu(v, m->speed);
            o = put(o, "km/h");
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 10:
            o = putu(v, m->weight);
            *o++ = 't';
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 11:
            o = putu(v, m->length10 / 10u);
            *o++ = '.';
            o = putu(o, m->length10 % 10u);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 12:
            o = putu(v, m->cars);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 13:
            o = putu(v, m->station);
            o = put(o, " 扩 ");
            o = putu(o, m->station_ext);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 14:
            o = putu(v, m->station_ext);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 15:
            o = putu(v, m->section);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 16:
            o = putu(v, m->route);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 17:
            o = putu(v, m->driver);
            *o++ = ' ';
            o = putu(o, m->driver_ext);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 18:
            o = putu(v, m->driver_ext);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 19:
            o = v;
            o = put(o, m->crc_ok ? "正常 " : "错误 ");
            if (m->test)
                o = put(o, "测试 ");
            o = puti(o, r->rssi);
            o = put(o, "dB");
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        default:
            break;
    }
}

void TDCS_DrawPopup(void)
{
    if (!g.count)
        return;

    const tdcs_rec_t *r = &g.rec[g.cur];
    char s[32];
    char *o;

    UI_DisplayClear();

    for (uint8_t i = 0; i < TDCS_ROWS; i++) {
        const uint8_t idx = TDCS_PAGE_MAP[g.page][i];
        if (idx >= TDCS_FIELDS)
            continue;
        draw_field(i, r, idx);
    }

    /* page number, top right of the first field row */
    o = s;
    *o++ = 'P';
    o = putu(o, (uint32_t)g.page + 1u);
    *o++ = '/';
    o = putu(o, TDCS_PAGES);
    *o = '\0';
    UI_PrintStringSmallNormal(s, 100, 0, 0);

    /* footer: history index, age */
    o = s;
    o = putu(o, (uint32_t)g.cur + 1u);
    *o++ = '/';
    o = putu(o, g.count);
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, TDCS_FOOTER);

    if (r->m.test)
        UI_PrintStringSmallNormal("TEST", 24, 0, TDCS_FOOTER);

    o = putage(s, (uint16_t)(g.sec - r->sec));
    *o = '\0';
    UI_PrintStringSmallNormal(s, 52, 0, TDCS_FOOTER);

    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}

/* ---- LED --------------------------------------------------------------- */

#define TDCS_LED_PHASES 16u

static const uint8_t TDCS_LED_TICKS[TDCS_LED_PHASES] = {
    10u, 10u, 10u, 50u,
    10u, 10u, 10u, 50u,
    10u, 10u, 10u, 50u,
    10u, 10u, 10u, 50u
};

static void ledService(void)
{
    if (!TDCS_RxVfoIsTdcs() || gCurrentFunction == FUNCTION_TRANSMIT || gPttIsPressed) {
        if (g.ledPhase) {
            g.ledPhase = 0u;
            g.ledTick  = 0u;
            BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
        }
        return;
    }

    if (!g.ledPhase) {
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
        return;
    }

    if (g.ledTick) {
        g.ledTick--;
        return;
    }

    const bool on = (g.ledPhase & 1u) == 0u;
    BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, on);
    g.ledTick = TDCS_LED_TICKS[TDCS_LED_PHASES - g.ledPhase];
    g.ledPhase--;
}

/* ---- history ----------------------------------------------------------- */

static void tdcs_store(const tdcs_msg_t *m)
{
    int found = -1;

    for (uint8_t i = 0; i < g.count; i++) {
        if (g.rec[i].m.train_no == m->train_no &&
            g.rec[i].m.loco_no  == m->loco_no &&
            (uint16_t)(g.sec - g.rec[i].sec) <= TDCS_MERGE_SECS) {
            found = (int)i;
            break;
        }
    }

    if (found >= 0) {
        g.rec[found].m    = *m;
        g.rec[found].sec  = g.sec;
        g.rec[found].rssi = (int16_t)(BK4819_GetRSSI_dBm() + dBmCorrTable[gRxVfo->Band]);
        g.cur  = (uint8_t)found;
        g.page = 0u;
    } else {
        if (g.count < TDCS_HISTORY)
            g.count++;
        for (uint8_t i = (uint8_t)(g.count - 1u); i > 0u; i--)
            g.rec[i] = g.rec[i - 1u];
        g.rec[0].m    = *m;
        g.rec[0].sec  = g.sec;
        g.rec[0].rssi = (int16_t)(BK4819_GetRSSI_dBm() + dBmCorrTable[gRxVfo->Band]);
        g.cur  = 0u;
        g.page = 0u;
    }

    g.dirty = true;

    if (!g.ledPhase) {
        g.ledPhase = TDCS_LED_PHASES;
        g.ledTick  = 0u;
    }

    if (!g.dismissed) {
        if (!g.popup)
            g.pinned = false;      /* opened by a report: the timeout applies */
        g.popup = true;
    }
    g.popSec = g.sec;
    BACKLIGHT_TurnOn();
}

/* ---- FSK engine -------------------------------------------------------- */

static void tdcs_fsk_start(void)
{
    /* losehu MDC1200/Messenger RX recipe: 0x3F03 (RX mode 111, auto
     * preamble), 4-byte sync, 1-byte preamble. */
    BK4819_WriteRegister(BK4819_REG_43, 0x3628u);
    BK4819_WriteRegister(BK4819_REG_3D, 0x2AABu);

    BK4819_WriteRegister(BK4819_REG_70, TDCS_FSK_TONE2_GAIN);
    BK4819_WriteRegister(BK4819_REG_72, TDCS_FSK_TONE2_1200);
    BK4819_WriteRegister(BK4819_REG_58, 0x3F03u);
    BK4819_WriteRegister(BK4819_REG_5A, 0x0DD4u);
    BK4819_WriteRegister(BK4819_REG_5B, 0x259Fu);
    BK4819_WriteRegister(BK4819_REG_5C, TDCS_FSK_5C_NOCRC);
    BK4819_WriteRegister(BK4819_REG_5D, TDCS_FSK_5D_75BYTE);
    BK4819_WriteRegister(BK4819_REG_5E, TDCS_FSK_5E_1WORD);
    BK4819_WriteRegister(BK4819_REG_59, 0xC008u);
    BK4819_WriteRegister(BK4819_REG_59, 0x1008u);
    BK4819_WriteRegister(BK4819_REG_02, 0x0000u);

    const uint16_t mask = BK4819_ReadRegister(BK4819_REG_3F);
    BK4819_WriteRegister(BK4819_REG_3F, (uint16_t)(mask | TDCS_IRQ_MASK));

    g.on        = true;
    g.capturing = false;
    g.idle      = 0;
}

void TDCS_StopSampler(void)
{
    if (!g.on)
        return;

    g.on        = false;
    g.capturing = false;

    BK4819_WriteRegister(BK4819_REG_59, 0x0000u);
    BK4819_WriteRegister(BK4819_REG_58, 0x0000u);
    BK4819_WriteRegister(BK4819_REG_70, 0x0000u);

    const uint16_t mask = BK4819_ReadRegister(BK4819_REG_3F);
    BK4819_WriteRegister(BK4819_REG_3F, (uint16_t)(mask & ~TDCS_IRQ_MASK));
}

bool TDCS_Active(void)
{
    return g.on;
}

bool TDCS_RxVfoIsTdcs(void)
{
    return gRxVfo && gRxVfo->Modulation == MODULATION_TDCS;
}

/* Either VFO is TDCS: dual watch switching the RX VFO must not close the
 * window the operator is reading. */
static bool tdcs_any_vfo(void)
{
    return gEeprom.VfoInfo[0].Modulation == MODULATION_TDCS ||
           gEeprom.VfoInfo[1].Modulation == MODULATION_TDCS;
}

static void tdcs_take(void)
{
    const uint16_t w = BK4819_ReadRegister(BK4819_REG_5F);
    tdcs_msg_t m;

    if (TDCS_DecWord16(w) && TDCS_DecTake(&m))
        tdcs_store(&m);
}

static void tdcs_finish(void)
{
    tdcs_msg_t m;

    for (uint8_t i = 0; i < 8u; i++)
        tdcs_take();
    if (TDCS_DecEnd() && TDCS_DecTake(&m))
        tdcs_store(&m);
}

void TDCS_HandleInterrupts(uint16_t bits)
{
    if (!g.on)
        return;
    if (gCurrentFunction == FUNCTION_TRANSMIT || gPttIsPressed)
        return;

    if ((bits & TDCS_IRQ_SYNC) && !g.capturing) {
        TDCS_DecSync();
        g.capturing = true;
        g.idle      = 0;
    }

    if (bits & TDCS_IRQ_FIFO) {
        /* losehu reads the almost-full threshold and takes exactly that many
         * words; REG_5E is programmed to 1 word for the RX path. */
        const uint16_t count = BK4819_ReadRegister(BK4819_REG_5E) & 7u;
        for (uint16_t i = 0; i < count; i++)
            tdcs_take();
        g.idle = 0;
    }

    if (bits & TDCS_IRQ_DONE) {
        tdcs_finish();
        tdcs_fsk_start();          /* clear FIFO and wait for the next frame */
        return;
    }
}

/* ---- decode window ----------------------------------------------------- */

bool TDCS_PopupActive(void)
{
    return g.popup;
}

void TDCS_ClosePopup(void)
{
    if (g.popup) {
        g.popup     = false;
        g.dismissed = true;
        gUpdateDisplay = true;
        gUpdateStatus  = true;
    }
}

bool TDCS_OpenPopup(void)
{
    if (!g.count)
        return false;

    g.cur       = 0u;
    g.page      = 0u;
    g.popup     = true;
    g.pinned    = true;       /* opened by hand: it never times out */
    g.dismissed = false;
    g.dirty     = true;
    return true;
}

/* ---- test transmitter (same physical layer as the tdcstx app) ---------- */

#ifdef ENABLE_TDCS_TX

static uint16_t tdcs_crc16(const uint8_t *p, uint16_t n)
{
    uint16_t crc = 0u;
    while (n--) {
        crc ^= (uint16_t)(*p++) << 8;
        for (uint8_t i = 0; i < 8u; i++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint32_t tdcs_bch_encode(uint16_t data)
{
    uint32_t r = (uint32_t)data << 10;
    for (int i = 25; i >= 10; i--)
        if (r & (1u << i))
            r ^= (uint32_t)TDCS_BCH_GEN << (i - 10);
    return ((uint32_t)data << 10) | (r & 0x3FFu);
}

static void tdcs_build_test_frame(uint8_t *f)
{
    uint8_t *p;

    f[0]  = 0x0Cu;
    f[1]  = 43u;
    f[2]  = 0x00u;
    f[3]  = 0x3Fu;
    f[4]  = 0x1Fu;
    f[5]  = 0x00u;
    f[6]  = 0x00u;
    f[7]  = 0x1Fu;
    f[8]  = 0x8Cu;
    f[9]  = 0x30u;
    f[10] = 32u;

    p = f + 11u;
    p[0]  = 4u;                 /* signal aspect: PASS                 */
    p[1]  = 4u;                 /* signal type: PASS                   */
    p[2]  = 0u;                 /* main freight                        */
    p[3]  = 'K';                /* train class                         */
    p[4]  = ' ';
    p[5]  = ' ';
    p[6]  = ' ';
    p[7]  = 0xD2u;              /* train 1234, little-endian           */
    p[8]  = 0x04u;
    p[9]  = 0x00u;
    p[10] = 240u;               /* HXD1D                               */
    p[11] = 0u;
    p[12] = 0xD2u;              /* loco 1234                           */
    p[13] = 0x04u;
    p[14] = 0x39u;              /* km 12345, increasing                */
    p[15] = 0x30u;
    p[16] = 0x40u;
    p[17] = 80u;                /* speed 80 km/h                       */
    p[18] = 0u;
    p[19] = 0u;
    p[20] = 0xE8u;              /* weight 1000 t                       */
    p[21] = 0x03u;
    p[22] = 0xF4u;              /* length 500 (50.0 m)                 */
    p[23] = 0x01u;
    p[24] = 10u;                /* cars                                */
    p[25] = 20u;                /* station 20                          */
    p[26] = 0u;
    p[27] = 1u;
    p[28] = 1u;
    p[29] = 0x0Fu;              /* driver 30223 = 0x760F               */
    p[30] = 0x76u;
    p[31] = 83u;                /* driver extension                    */

    const uint16_t crc = tdcs_crc16(f, TDCS_FRAME_BYTES - 2u);
    f[43] = (uint8_t)(crc >> 8);
    f[44] = (uint8_t)(crc);
    f[45] = 0x00u;              /* pad after the CRC                   */
}

static uint32_t s_tPrev, s_tCyc, s_next;
static uint8_t  s_tone;

static void tdcs_clk_start(void)
{
    s_tPrev = TDCS_SYST_VAL;
    s_tCyc  = 0u;
}

static uint32_t tdcs_clk_cyc(void)
{
    const uint32_t v = TDCS_SYST_VAL;
    s_tCyc += (v <= s_tPrev) ? s_tPrev - v : s_tPrev + (TDCS_SYST_LOAD + 1u - v);
    s_tPrev = v;
    return s_tCyc;
}

static void tdcs_send_bit(uint8_t bit)
{
    while ((int32_t)(tdcs_clk_cyc() - s_next) < 0) { }

    if (bit != s_tone) {
        s_tone = bit;
        BK4819_WriteRegister(BK4819_REG_71, bit ? TDCS_F_MARK : TDCS_F_SPACE);
    }
    s_next += TDCS_CYC_PER_BIT;
}

static void tdcs_send_test(void)
{
    uint8_t f[TDCS_FRAME_PAD];

    if (TX_freq_check(gTxVfo->pTX->Frequency) != 0 ||
        gBatteryDisplayLevel == 0 || gBatteryDisplayLevel > 6) {
        gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
        return;
    }

    TDCS_StopSampler();

    RADIO_SetTxParameters();
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
    BK4819_WriteRegister(BK4819_REG_51, 0u);
    BK4819_TransmitTone(false, 1200u);
    BK4819_WriteRegister(BK4819_REG_70,
        (uint16_t)(0x8000u | ((uint16_t)TDCS_TX_LEVEL << 8)));

    tdcs_build_test_frame(f);

    s_tone = 0u;
    BK4819_WriteRegister(BK4819_REG_71, TDCS_F_SPACE);
    tdcs_clk_start();
    s_next = TDCS_CYC_PER_BIT;

    for (uint8_t i = 0; i < TDCS_PREAMBLE; i++)
        tdcs_send_bit((uint8_t)(i & 1u));

    for (int i = 31; i >= 0; i--)
        tdcs_send_bit((uint8_t)((TDCS_SYNC_TX >> i) & 1u));

    for (uint8_t i = 0; i < TDCS_CW_COUNT; i++) {
        const uint16_t w = (uint16_t)(((uint16_t)f[2u * i] << 8) | f[2u * i + 1u]);
        const uint32_t cw = tdcs_bch_encode(w);
        for (int b = 25; b >= 0; b--)
            tdcs_send_bit((uint8_t)((cw >> b) & 1u));
    }

    while ((int32_t)(tdcs_clk_cyc() - s_next) < 0) { }

    BK4819_EnterTxMute();
    BK4819_ToggleGpioOut(BK4819_GPIO1_PIN29_PA_ENABLE, false);
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
    RADIO_SetupRegisters(true);

    gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
}

#endif /* ENABLE_TDCS_TX */

/* ---- keys -------------------------------------------------------------- */

bool TDCS_HandleKey(uint8_t key, bool pressed, bool held)
{
    if (key == KEY_PTT) {
#ifdef ENABLE_TDCS_TX
        if (pressed && !held && TDCS_RxVfoIsTdcs()) {
            tdcs_send_test();
            return true;
        }
#endif
        TDCS_ClosePopup();
        TDCS_StopSampler();
        return false;
    }
    if (gScreenToDisplay != DISPLAY_MAIN)
        return false;

    if (!g.popup)
        return false;

    /* The window swallows the key, so clear any pending F+key state or the
     * next press would still be treated as a function-key combination. */
    gWasFKeyPressed = false;

    if (gLowBattery && !gLowBatteryConfirmed) {
        TDCS_ClosePopup();
        return false;
    }

    if (!pressed || held)
        return true;

    g.popSec = g.sec;

    if (key == KEY_EXIT) {
        TDCS_ClosePopup();
        return true;
    }
    if (key == KEY_UP || key == KEY_DOWN) {
        if (g.count) {
            int t = (int)g.cur + ((key == KEY_UP) ? -1 : 1);
            if (t < 0)
                t = 0;
            if (t > (int)g.count - 1)
                t = (int)g.count - 1;
            if ((uint8_t)t != g.cur) {
                g.cur  = (uint8_t)t;
                g.page = 0u;
                g.dirty = true;
            }
        }
        return true;
    }
    if (key == KEY_6) {
        if (g.page > 0u) {
            g.page--;
            g.dirty = true;
        }
        return true;
    }
    if (key == KEY_9) {
        if ((uint8_t)(g.page + 1u) < TDCS_PAGES) {
            g.page++;
            g.dirty = true;
        }
        return true;
    }
    if (key == KEY_2) {
        g.count = 0u;
        g.cur   = 0u;
        g.page  = 0u;
        TDCS_ClosePopup();
        return true;
    }
    return true;
}

/* ---- 10 ms tick -------------------------------------------------------- */

void TDCS_Tick(void)
{
    if (++g.subSec >= 100u) {
        g.subSec = 0u;
        g.sec++;
    }

    ledService();

    if (g.dismissed && !g_SquelchLost)
        g.dismissed = false;

    if (gCurrentFunction == FUNCTION_TRANSMIT || gPttIsPressed) {
        TDCS_ClosePopup();
        TDCS_StopSampler();
        return;
    }

    /* Keep the receiver awake while the TDCS VFO is selected: the FSK engine
     * must already be armed when the 160-bit preamble starts, otherwise it can
     * miss the sync after a power-save wake-up. */
    if (g.popup || TDCS_RxVfoIsTdcs()) {
        gBatterySaveCountdown_10ms = battery_save_count_10ms;
        if (gCurrentFunction == FUNCTION_POWER_SAVE)
            FUNCTION_Select(FUNCTION_FOREGROUND);
    }

    if (gCurrentFunction == FUNCTION_POWER_SAVE) {
        TDCS_StopSampler();
        return;
    }

    if (!tdcs_any_vfo()) {
        TDCS_ClosePopup();
        TDCS_StopSampler();
        if (!g_SquelchLost)
            g.dismissed = false;
        return;
    }

    /* Dual watch has the other VFO up: the window stays (the operator is
     * reading a report, not watching a channel), only the engine stops. */
    if (!TDCS_RxVfoIsTdcs()) {
        TDCS_StopSampler();
        if (g.dirty) {
            g.dirty = false;
            g.popSec = g.sec;
            BACKLIGHT_TurnOn();
            gUpdateDisplay = true;
        }
        return;
    }

    if (!g.on) {
        tdcs_fsk_start();
    } else {
        /* RADIO_SetupRegisters() rewrites REG_3F on every channel change;
         * put the FSK interrupt bits back if they were lost. */
        const uint16_t mask = BK4819_ReadRegister(BK4819_REG_3F);
        if ((mask & TDCS_IRQ_MASK) != TDCS_IRQ_MASK)
            BK4819_WriteRegister(BK4819_REG_3F, (uint16_t)(mask | TDCS_IRQ_MASK));

        /* The engine's byte counter stalls when the carrier drops, so
         * RX_FINISHED often never arrives for a real burst. */
        if (g.capturing && ++g.idle > TDCS_STALL_TICKS) {
            tdcs_finish();
            tdcs_fsk_start();
        }
    }

    if (g.popup && !g.dismissed && !g.pinned) {
        if ((uint16_t)(g.sec - g.popSec) >= TDCS_POPUP_SECS)
            TDCS_ClosePopup();
    }

    if (g.dirty) {
        g.dirty = false;
        g.popSec = g.sec;
        BACKLIGHT_TurnOn();
        gUpdateDisplay = true;
    }
}
