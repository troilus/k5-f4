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
#ifdef ENABLE_TDCS_ACK
    #include "qr.h"
#endif

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
#ifdef ENABLE_TDCS_ACK
#define ACK_HISTORY         8u
#define MERGED_MAX          (TDCS_HISTORY + ACK_HISTORY)
#endif
#define TDCS_PAGES          3u
#define TDCS_ROWS           6u
#define TDCS_FOOTER         6u
#define TDCS_FIELDS         18u
#define TDCS_MERGE_SECS     30u
#define TDCS_POPUP_SECS     10u   /* fixed: the K1 TCDPOP setting is skipped */
#define TDCS_STALL_TICKS    150u  /* 1.5 s without FIFO/sync = carrier gone  */

/* BK4819 FSK engine: losehu's MDC1200/Messenger RX recipe (driver/bk4819.c
 * enable_msg_rx), the proven FFSK 1200/1800 path. */
#define TDCS_FSK_TONE2_GAIN 0x00E0u  /* REG_70: tone2 on, gain 96             */
#define TDCS_FSK_TONE2_1200 0x3065u  /* REG_72: 1200 Hz / bit clock           */
#define TDCS_FSK_5C_NOCRC   0x5625u  /* losehu MDC1200 RX value, CRC off      */
#ifdef ENABLE_TDCS_ACK
/* A table 7 reply is 31 codewords = 806 bits = 101 bytes. */
#define TDCS_FSK_5D_BYTES   0x6400u
#else
#define TDCS_FSK_5D_BYTES   0x4A00u  /* 75 bytes = 23 codewords               */
#endif
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

#ifdef ENABLE_TDCS_ACK
typedef struct {
    ack_msg_t a;
    uint16_t  sec;
    int16_t   rssi;
} ack_rec_t;

/* The merged history: kind 0 points into rec[], kind 1 into ack[]. */
typedef struct {
    uint8_t kind;
    uint8_t idx;
} merge_ent_t;
#endif

static struct {
    tdcs_rec_t rec[TDCS_HISTORY];
    uint8_t    count, cur, page;
#ifdef ENABLE_TDCS_ACK
    ack_rec_t  ack[ACK_HISTORY];
    merge_ent_t mlist[MERGED_MAX];
    uint8_t    acount, mcount;
    bool       ackQr;
    uint64_t   qrM[QR_N];
#endif
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

/* Locomotive model code -> ASCII name.  Same table as the K1 TDCS/LBJ
 * decoder (App/app/lbj_dual.c).  TDCS carries the type in one byte, so the
 * K1 entries above 255 (CRH/CR registrations) can never appear here. */
static const char *loco_type_name(uint8_t c)
{
    switch (c) {
        case   1: return "JF";
        case   3: return "QJ";
        case   5: return "JS";
        case   6: return "KD7";
        case  55: return "LJK";
        case  81: return "DF21";
        case 101: return "DF";
        case 102: return "DF2";
        case 103: return "DF3";
        case 104: return "DF4";
        case 105: return "DF4K";
        case 106: return "DF4C";
        case 107: return "DF5";
        case 108: return "DF5W";
        case 109: return "DF6";
        case 110: return "DF7";
        case 111: return "DF8";
        case 112: return "DF9";
        case 113: return "DF10";
        case 114: return "DFH1";
        case 115: return "DFH2";
        case 116: return "DFH3";
        case 117: return "DFH5";
        case 118: return "BJ";
        case 119: return "BJW";
        case 120: return "ND2";
        case 121: return "ND3";
        case 122: return "ND4";
        case 123: return "ND5";
        case 124: return "NY5";
        case 125: return "NY6";
        case 126: return "NY7";
        case 127: return "QY";
        case 128: return "DFH21";
        case 129: return "DF7B";
        case 130: return "DF5S";
        case 131: return "DF7C";
        case 132: return "DF7S";
        case 133: return "GK1";
        case 134: return "GK1F";
        case 135: return "DF4E";
        case 136: return "DF7D";
        case 137: return "GK1A";
        case 138: return "DF11";
        case 139: return "TA";
        case 140: return "DF10F";
        case 141: return "DF4D";
        case 142: return "DF8B";
        case 143: return "DF12";
        case 144: return "DF7E";
        case 145: return "NYJ1";
        case 146: return "NZJ1";
        case 147: return "NZJ2";
        case 148: return "DF4DJ";
        case 149: return "XSG";
        case 150: return "SZ";
        case 151: return "NJ2";
        case 152: return "DF7G";
        case 153: return "NDJ3";
        case 156: return "DF11Z";
        case 157: return "FXN3D";
        case 158: return "DF11G";
        case 160: return "HXN3";
        case 161: return "HXN5";
        case 162: return "HXN3B";
        case 163: return "HXN5B";
        case 167: return "FXN3B";
        case 169: return "FXN3C";
        case 170: return "FXN5C";
        case 171: return "FXN3-J";
        case 201: return "8G";
        case 202: return "8K";
        case 203: return "6G";
        case 204: return "6K";
        case 205: return "SS1";
        case 206: return "SS3";
        case 207: return "SS4";
        case 208: return "SS5";
        case 209: return "SS6";
        case 210: return "SS3B";
        case 211: return "SS7";
        case 212: return "SS8";
        case 213: return "SS7B";
        case 214: return "SS7C";
        case 215: return "SS6B";
        case 216: return "SS9";
        case 217: return "SS7D";
        case 218: return "DJXM";
        case 219: return "DJ1";
        case 220: return "DJ2";
        case 221: return "DJF";
        case 222: return "LJD";
        case 223: return "XFH";
        case 224: return "SS7E";
        case 225: return "SS4G";
        case 226: return "SS3C";
        case 228: return "TS";
        case 229: return "DJ4HX";
        case 230: return "KTT";
        case 231: return "HXD1";
        case 232: return "HXD2";
        case 233: return "HXD3";
        case 234: return "HXD1B";
        case 235: return "HXD2B";
        case 236: return "HXD3B";
        case 237: return "HXD1C";
        case 238: return "HXD2C";
        case 239: return "HXD3C";
        case 240: return "HXD1D";
        case 241: return "HXD2D";
        case 242: return "HXD3D";
        case 243: return "FXD1B";
        case 244: return "FXD2B";
        case 245: return "FXD1";
        case 246: return "FXD3";
        case 247: return "FXD1-J";
        case 248: return "FXD3-J";
        case 249: return "KZ25TA";
        case 251: return "KZ25TB";
        case 252: return "HXD1D-J";
        case 254: return "FXD1H";
        default:  return 0;
    }
}

/* Short labels: an 8x8 Chinese glyph advances 9 px, so a four-character label
 * with ": " still leaves most of the 128 px row for the value. */
static const char *const TDCS_LABEL[TDCS_FIELDS] = {
    "信号机", "信号种类", "本补客货", "车次种类", "车次",
    "机车类型", "类型扩充", "机车号", "公里标", "速度",
    "总重", "计长", "辆数", "车站号",
    "区段号", "实际交路", "司机号", "状态"
};

/* Display order: the order in the frame is not the order an operator reads.
 * Exactly 18 fields over three pages of six rows, the UV-K1 arrangement. */
static const uint8_t TDCS_PAGE_MAP[TDCS_PAGES][TDCS_ROWS] = {
    {  3,  4,  5,  6,  7,  8 },   /* CLASS TRAIN LOCO TYPE LOCO# KM   */
    {  9, 10, 11, 12, 13, 14 },   /* SPEED WEIGHT LENGTH CARS STN SECT */
    { 15, 16, 17,  0,  1,  2 }    /* ROUTE DRIVER STATUS SIG SIGTYP CREW */
};

static void field_row(uint8_t row, const char *label, const char *value)
{
    char s[48];
    char *o = s;

    while (*label)
        *o++ = *label++;
    *o++ = ':';
    *o++ = ' ';
    while (*value)
        *o++ = *value++;
    *o = '\0';
    UI_PrintStringSmallCN(s, 0, row);
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
        case 2:
            o = put(v, (m->crew & 2u) ? "补机 " : "本务 ");
            o = put(o, (m->crew & 1u) ? "客车" : "货车");
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
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
        case 5: {
            /* The type byte uses the LBJ model code space (240 is HXD1D).
             * Show the name with the raw code in hex; a seven-character
             * name drops the hex so the row stays inside 128 px. */
            const char *n = loco_type_name(m->loco_type);
            o = v;
            if (n) {
                o = put(o, n);
                if (o - v <= 6) {
                    *o++ = '(';
                    o = puthex2(o, m->loco_type);
                    *o++ = ')';
                }
            } else {
                o = putu(o, m->loco_type);
                o = put(o, " (");
                o = puthex2(o, m->loco_type);
                *o++ = ')';
            }
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        }
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
            o = put(o, " 扩充 ");
            o = putu(o, m->station_ext);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 14:
            o = putu(v, m->section);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 15:
            o = putu(v, m->route);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 16:
            o = putu(v, m->driver);
            o = put(o, " 扩充 ");
            o = putu(o, m->driver_ext);
            *o = '\0';
            field_row(row, TDCS_LABEL[idx], v);
            break;
        case 17:
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

#ifdef ENABLE_TDCS_ACK
/* ---- TDCS ACK window: TB/T 3231 table 7 -------------------------------- */

static char *putbcd2(char *o, uint8_t v)
{
    const uint8_t hi = (uint8_t)(v >> 4);
    const uint8_t lo = (uint8_t)(v & 0x0Fu);

    *o++ = (hi <= 9u) ? (char)('0' + hi) : '?';
    *o++ = (lo <= 9u) ? (char)('0' + lo) : '?';
    return o;
}

static const char *ack_name(uint8_t t)
{
    switch (t) {
        case 0x81: return "自动确认";
        case 0x82: return "手动签收";
        case 0x91: return "调车请求";
        default:   return "回执";
    }
}

/* The map QR page: 33 x 33 modules scaled into 70 x 56 px, the K1 layout. */
#define QR_PX_W 70u
#define QR_PX_H 56u
#define QR_X ((LCD_WIDTH  - QR_PX_W) / 2u)
#define QR_Y ((LCD_HEIGHT - QR_PX_H) / 2u)

static void qr_pixel(uint8_t x, uint8_t y)
{
    if (y < 8u)
        PutPixelStatus(x, y, true);
    else
        PutPixel(x, (uint8_t)(y - 8u), true);
}

static void draw_qr(void)
{
    UI_DisplayClear();
    memset(gStatusLine, 0, sizeof(gStatusLine));

    for (uint8_t my = 0; my < QR_N; my++) {
        const uint64_t row = g.qrM[my];
        if (!row)
            continue;

        const unsigned y0 = QR_Y + ((unsigned)my * QR_PX_H + QR_N / 2u) / QR_N;
        const unsigned y1 = QR_Y + ((unsigned)(my + 1u) * QR_PX_H + QR_N / 2u) / QR_N;

        for (uint8_t mx = 0; mx < QR_N; mx++) {
            if (!(row & (1ull << mx)))
                continue;

            const unsigned x0 = QR_X + ((unsigned)mx * QR_PX_W + QR_N / 2u) / QR_N;
            const unsigned x1 = QR_X + ((unsigned)(mx + 1u) * QR_PX_W + QR_N / 2u) / QR_N;

            for (unsigned y = y0; y < y1; y++)
                for (unsigned x = x0; x < x1; x++)
                    qr_pixel((uint8_t)x, (uint8_t)y);
        }
    }

    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}

static void draw_ack(const ack_rec_t *r)
{
    const ack_msg_t *a = &r->a;
    char s[32];
    char v[32];
    char *o;

    if (g.ackQr) {
        draw_qr();
        return;
    }

    UI_DisplayClear();

    /* row 0: train number, date on the right */
    o = s;
    o = put(o, a->train[0] ? a->train : "----");
    *o = '\0';
    UI_PrintStringSmallBold(s, 0, 0, 0);

    if (a->date[0] || a->date[1] || a->date[2]) {
        o = s;
        o = putbcd2(o, a->date[0]);
        *o++ = '-';
        o = putbcd2(o, a->date[1]);
        *o++ = '-';
        o = putbcd2(o, a->date[2]);
        *o = '\0';
        UI_PrintStringSmallNormal(s, (uint8_t)(128u - (uint8_t)(o - s) * 7u), 0, 0);
    }

    /* row 1: reply type and time */
    o = v;
    o = put(o, ack_name(a->type));
    *o++ = ' ';
    o = putbcd2(o, a->time[0]);
    *o++ = ':';
    o = putbcd2(o, a->time[1]);
    *o++ = ':';
    o = putbcd2(o, a->time[2]);
    *o = '\0';
    field_row(1, "回执", v);

    field_row(2, "机车", a->loco[0] ? a->loco : "--");
    field_row(3, "编号", a->num[0] ? a->num : "--");

    if (a->km_valid) {
        o = v;
        if (a->km_neg)
            *o++ = '-';
        *o++ = 'K';
        o = putu(o, a->km_abs / 1000u);
        *o++ = '+';
        o = putpad(o, a->km_abs % 1000u, 3u);
        *o++ = a->km_inc ? '^' : 'v';
        *o = '\0';
    } else {
        o = put(v, "--");
        *o = '\0';
    }
    field_row(4, "公里", v);

    o = v;
    o = putu(o, (uint32_t)(((uint16_t)a->place_hi << 8) | a->place_lo));
    o = put(o, " 包");
    o = putu(o, a->pkt);
    *o = '\0';
    field_row(5, "处所", v);

    /* footer: merged index, age, the QR hint */
    o = s;
    o = putu(o, (uint32_t)g.cur + 1u);
    *o++ = '/';
    o = putu(o, g.mcount);
    *o = '\0';
    UI_PrintStringSmallNormal(s, 0, 0, TDCS_FOOTER);

    o = putage(s, (uint16_t)(g.sec - r->sec));
    *o = '\0';
    UI_PrintStringSmallNormal(s, 52, 0, TDCS_FOOTER);

    if (a->fix)
        UI_PrintStringSmallNormal("*地图", 84, 0, TDCS_FOOTER);

    ST7565_BlitStatusLine();
    ST7565_BlitFullScreen();
}
#endif /* ENABLE_TDCS_ACK */

void TDCS_DrawPopup(void)
{
#ifdef ENABLE_TDCS_ACK
    if (!g.mcount)
        return;

    const merge_ent_t *e = &g.mlist[g.cur];
    if (e->kind == 1u) {
        draw_ack(&g.ack[e->idx]);
        return;
    }
    const tdcs_rec_t *r = &g.rec[e->idx];
#else
    if (!g.count)
        return;
    const tdcs_rec_t *r = &g.rec[g.cur];
#endif
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
#ifdef ENABLE_TDCS_ACK
    o = putu(o, g.mcount);
#else
    o = putu(o, g.count);
#endif
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

#ifdef ENABLE_TDCS_ACK
static uint16_t merge_sec(const merge_ent_t *e)
{
    return (e->kind == 0u) ? g.rec[e->idx].sec : g.ack[e->idx].sec;
}

/* Rebuild the merged TDCS + ACK list, newest first.  Both arrays are already
 * newest-first, so the insertion sort is over at most 16 entries. */
static void merged_rebuild(void)
{
    uint8_t n = 0u;

    for (uint8_t i = 0; i < g.count; i++) {
        g.mlist[n].kind = 0u;
        g.mlist[n].idx  = i;
        n++;
    }
    for (uint8_t i = 0; i < g.acount; i++) {
        g.mlist[n].kind = 1u;
        g.mlist[n].idx  = i;
        n++;
    }

    for (uint8_t i = 1u; i < n; i++) {
        const merge_ent_t e = g.mlist[i];
        const uint16_t s = merge_sec(&e);
        int j = (int)i - 1;
        while (j >= 0 && merge_sec(&g.mlist[j]) < s) {
            g.mlist[j + 1] = g.mlist[j];
            j--;
        }
        g.mlist[j + 1] = e;
    }
    g.mcount = n;
}
#endif

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

#ifdef ENABLE_TDCS_ACK
    merged_rebuild();
    g.cur   = 0u;
    g.ackQr = false;
#endif

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

#ifdef ENABLE_TDCS_ACK
static void ack_store(const ack_msg_t *a)
{
    int found = -1;

    for (uint8_t i = 0; i < g.acount; i++) {
        if (g.ack[i].a.type == a->type &&
            strcmp(g.ack[i].a.train, a->train) == 0 &&
            strcmp(g.ack[i].a.loco, a->loco) == 0 &&
            strcmp(g.ack[i].a.num, a->num) == 0 &&
            (uint16_t)(g.sec - g.ack[i].sec) <= TDCS_MERGE_SECS) {
            found = (int)i;
            break;
        }
    }

    if (found >= 0) {
        g.ack[found].a    = *a;
        g.ack[found].sec  = g.sec;
        g.ack[found].rssi = (int16_t)(BK4819_GetRSSI_dBm() + dBmCorrTable[gRxVfo->Band]);
    } else {
        if (g.acount < ACK_HISTORY)
            g.acount++;
        for (uint8_t i = (uint8_t)(g.acount - 1u); i > 0u; i--)
            g.ack[i] = g.ack[i - 1u];
        g.ack[0].a    = *a;
        g.ack[0].sec  = g.sec;
        g.ack[0].rssi = (int16_t)(BK4819_GetRSSI_dBm() + dBmCorrTable[gRxVfo->Band]);
    }

    merged_rebuild();
    g.cur   = 0u;
    g.page  = 0u;
    g.ackQr = false;
    g.dirty = true;

    if (!g.ledPhase) {
        g.ledPhase = TDCS_LED_PHASES;
        g.ledTick  = 0u;
    }

    if (!g.dismissed) {
        if (!g.popup)
            g.pinned = false;
        g.popup = true;
    }
    g.popSec = g.sec;
    BACKLIGHT_TurnOn();
}
#endif /* ENABLE_TDCS_ACK */

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
    BK4819_WriteRegister(BK4819_REG_5D, TDCS_FSK_5D_BYTES);
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
#ifdef ENABLE_TDCS_ACK
    ack_msg_t a;
#endif

    if (!TDCS_DecWord16(w))
        return;
#ifdef ENABLE_TDCS_ACK
    if (TDCS_DecTakeAck(&a)) {
        ack_store(&a);
        return;
    }
#endif
    if (TDCS_DecTake(&m))
        tdcs_store(&m);
}

static void tdcs_finish(void)
{
    tdcs_msg_t m;
#ifdef ENABLE_TDCS_ACK
    ack_msg_t a;
#endif

    for (uint8_t i = 0; i < 8u; i++)
        tdcs_take();
    if (!TDCS_DecEnd())
        return;
#ifdef ENABLE_TDCS_ACK
    if (TDCS_DecTakeAck(&a)) {
        ack_store(&a);
        return;
    }
#endif
    if (TDCS_DecTake(&m))
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
#ifdef ENABLE_TDCS_ACK
        g.ackQr     = false;
#endif
        gUpdateDisplay = true;
        gUpdateStatus  = true;
    }
}

bool TDCS_OpenPopup(void)
{
#ifdef ENABLE_TDCS_ACK
    if (!g.mcount)
        return false;
    g.ackQr = false;
#else
    if (!g.count)
        return false;
#endif

    g.cur       = 0u;
    g.page      = 0u;
    g.popup     = true;
    g.pinned    = true;       /* opened by hand: it never times out */
    g.dismissed = false;
    g.dirty     = true;
    gUpdateDisplay = true;
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
#ifdef ENABLE_TDCS_ACK
    /* The QR page: star or EXIT closes only the code, back to the receipt. */
    if (g.ackQr) {
        if (key == KEY_STAR || key == KEY_EXIT) {
            g.ackQr = false;
            g.dirty = true;
        }
        return true;
    }
    if (key == KEY_STAR) {
        if (g.mcount && g.mlist[g.cur].kind == 1u) {
            const ack_msg_t *a = &g.ack[g.mlist[g.cur].idx].a;
            if (a->fix && QR_Build(a->lat6, a->lon6, g.qrM)) {
                g.ackQr = true;
                g.dirty = true;
            }
        }
        return true;
    }
#endif
    if (key == KEY_UP || key == KEY_DOWN) {
#ifdef ENABLE_TDCS_ACK
        const int n = (int)g.mcount;
#else
        const int n = (int)g.count;
#endif
        if (n) {
            int t = (int)g.cur + ((key == KEY_UP) ? -1 : 1);
            if (t < 0)
                t = 0;
            if (t > n - 1)
                t = n - 1;
            if ((uint8_t)t != g.cur) {
                g.cur  = (uint8_t)t;
                g.page = 0u;
                g.dirty = true;
            }
        }
        return true;
    }
    if (key == KEY_6 || key == KEY_9) {
#ifdef ENABLE_TDCS_ACK
        if (g.mcount && g.mlist[g.cur].kind == 1u)
            return true;      /* a receipt is one page */
#endif
        if (key == KEY_6) {
            if (g.page > 0u) {
                g.page--;
                g.dirty = true;
            }
        } else if ((uint8_t)(g.page + 1u) < TDCS_PAGES) {
            g.page++;
            g.dirty = true;
        }
        return true;
    }
    if (key == KEY_2) {
        g.count = 0u;
#ifdef ENABLE_TDCS_ACK
        g.acount = 0u;
        g.mcount = 0u;
#endif
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
        /* A window opened by hand stays up on any mode/frequency; only a
         * report-triggered window follows the TDCS VFO. */
        if (!g.pinned)
            TDCS_ClosePopup();
        TDCS_StopSampler();
        if (!g_SquelchLost)
            g.dismissed = false;
        if (g.dirty) {
            g.dirty = false;
            g.popSec = g.sec;
            BACKLIGHT_TurnOn();
            gUpdateDisplay = true;
        }
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
