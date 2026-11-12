/* TDCS decoder core - see tdcs_dec.h.
 *
 * Frame (45 bytes):
 *   0     0C   frame start
 *   1     43   frame length excluding CRC; total = f[1] + 2 = 45
 *   2     --   station address
 *   3..6  --   locomotive address
 *   7     1F   signature (FF in a test frame)
 *   8     8C   signature (FF in a test frame)
 *   9     30   information function code (FF in a test frame)
 *   10    32   information length; total = f[10] + 13 = 45
 *   11..42 payload (32 bytes)
 *   43..44 CRC-16/XMODEM, poly 1021, init 0, big-endian, over f[0..42]
 *
 * On air: 160-bit alternating preamble, 32-bit sync 0x0DD4259F, then 23
 * BCH(26,16) codewords (16 data bits MSB first + 10 parity bits).
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "tdcs_dec.h"

#define TDCS_BCH_GEN 0x05B9u

typedef struct {
    uint32_t cw;
    uint8_t  cw_bits, cw_index, total_cw, nbytes;
    uint8_t  frame[TDCS_FRAME_PAD];
} tdcs_col_t;

typedef struct {
    uint8_t    state;      /* 0 idle, 1 collecting                         */
    uint8_t    inv;        /* invert the data bits (deviation polarity)     */
    tdcs_col_t col;
} tdcs_fr_t;

static tdcs_fr_t  s_fr[2];
static tdcs_msg_t s_msg;
static bool       s_have;
#ifdef ENABLE_TDCS_ACK
static ack_msg_t  s_ack;
static bool       s_have_ack;
#endif

/* ---- BCH(26,16), generator 0x5B9 --------------------------------------- */

static uint16_t bch_syn(uint32_t cw)
{
    uint32_t r = cw & 0x3FFFFFFu;
    for (int i = 25; i >= 10; i--) {
        if ((r >> i) & 1u)
            r ^= (uint32_t)TDCS_BCH_GEN << (i - 10);
    }
    return (uint16_t)(r & 0x3FFu);
}

static uint16_t bch_decode(uint32_t cw)
{
    const uint16_t s = bch_syn(cw);
    if (s == 0u)
        return (uint16_t)((cw >> 10) & 0xFFFFu);

    uint16_t syn1[TDCS_CW_BITS];
    for (uint8_t p = 0; p < TDCS_CW_BITS; p++)
        syn1[p] = bch_syn(1u << p);

    for (uint8_t p = 0; p < TDCS_CW_BITS; p++) {
        if (syn1[p] == s)
            return (uint16_t)(((cw ^ (1u << p)) >> 10) & 0xFFFFu);
    }

    for (uint8_t p = 0; p < TDCS_CW_BITS; p++) {
        for (uint8_t q = (uint8_t)(p + 1u); q < TDCS_CW_BITS; q++) {
            if ((uint16_t)(syn1[p] ^ syn1[q]) == s)
                return (uint16_t)(((cw ^ (1u << p) ^ (1u << q)) >> 10) & 0xFFFFu);
        }
    }

    /* Uncorrectable: keep the raw data, the CRC has the last word. */
    return (uint16_t)((cw >> 10) & 0xFFFFu);
}

/* ---- frame ------------------------------------------------------------- */

static uint16_t crc16(const uint8_t *p, uint16_t n)
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

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t le24(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

/* Parse one 45-byte frame.  Returns false when it is not an A-class TDCS
 * frame or the CRC does not match. */
static bool tdcs_parse(const uint8_t *f, uint8_t len, tdcs_msg_t *m)
{
    if (len < TDCS_FRAME_BYTES)
        return false;
    if (f[0] != 0x0Cu || f[1] != 43u || f[10] != 32u)
        return false;
    /* 3F 1F is the fixed part of the 40-bit address (TB/T 3325 table 9:
     * 003F1F0000H).  The station and locomotive parts around it vary, so only
     * this magic pair is checked; it keeps a frame of another protocol from
     * being parsed as TDCS. */
    if (f[3] != 0x3Fu || f[4] != 0x1Fu)
        return false;

    const bool test = (f[7] == 0xFFu && f[8] == 0xFFu && f[9] == 0xFFu);
    if (!test && (f[7] != 0x1Fu || f[8] != 0x8Cu || f[9] != 0x30u))
        return false;
    if ((uint16_t)f[1] + 2u != (uint16_t)f[10] + 13u)
        return false;

    const uint16_t rx = (uint16_t)(((uint16_t)f[43] << 8) | f[44]);
    if (crc16(f, TDCS_FRAME_BYTES - 2u) != rx)
        return false;

    const uint8_t *p = f + 11u;
    m->sig       = (uint8_t)(p[0] & 0x0Fu);
    m->sig_type  = (uint8_t)(p[1] & 0x07u);
    m->crew      = (uint8_t)(p[2] & 0x03u);
    for (uint8_t i = 0; i < 4u; i++)
        m->cls[i] = p[3u + i];
    m->train_no  = le24(p + 7u);
    m->loco_type = p[10];
    m->loco_ext  = p[11];
    m->loco_no   = le16(p + 12u);

    /* 9999888 / 8888888 are the test-frame kilometre sentinels and 9999999 is
     * the "no value" one.  The raw 24-bit value is compared before the 18-bit
     * mask: after masking, a sentinel turns into a plausible-looking post
     * (9999999 used to print as K1611+391). */
    const uint32_t km_raw  = le24(p + 14u);
    const bool     km_test = (km_raw == 9999888u || km_raw == 8888888u);

    m->km_valid  = ((km_raw & 0x3FFFFFu) <= 999999u && !km_test) ? 1u : 0u;
    m->km_neg    = (p[16] & 0x80u) ? 1u : 0u;
    m->km_inc    = (p[16] & 0x40u) ? 1u : 0u;
    m->km_abs    = km_raw & 0x3FFFFFu;

    m->speed       = (uint16_t)(le24(p + 17u) & 0x3FFu);
    m->weight      = le16(p + 20u);
    m->length10    = le16(p + 22u);
    m->cars        = p[24];
    m->station     = p[25];
    m->station_ext = p[26];
    m->section     = p[27];
    m->route       = p[28];
    m->driver      = le16(p + 29u);
    m->driver_ext  = p[31];
    m->test        = (test || km_test) ? 1u : 0u;
    m->crc_ok      = 1u;
    return true;
}

#ifdef ENABLE_TDCS_ACK
/* ---- TDCS ACK: TB/T 3231 table 7 --------------------------------------- */

/* BCD bytes -> number, 0xFFFFFFFF when a nibble is not a digit. */
static uint32_t ack_bcd(const uint8_t *p, uint8_t n)
{
    uint32_t v = 0u;

    for (uint8_t i = 0; i < n; i++) {
        const uint8_t hi = (uint8_t)(p[i] >> 4);
        const uint8_t lo = (uint8_t)(p[i] & 0x0Fu);
        if (hi > 9u || lo > 9u)
            return 0xFFFFFFFFu;
        v = v * 100u + (uint32_t)hi * 10u + lo;
    }
    return v;
}

/* Printable ASCII only; trailing spaces dropped. */
static void ack_str(char *dst, const uint8_t *src, uint8_t n)
{
    uint8_t k = 0u;

    for (uint8_t i = 0; i < n; i++) {
        const uint8_t c = src[i];
        if (c >= 0x20u && c < 0x7Fu)
            dst[k++] = (char)c;
    }
    while (k && dst[k - 1u] == ' ')
        k--;
    dst[k] = '\0';
}

/* Table 7 signing position: longitude 2 BCD degrees + 3 BCD minutes,
 * latitude 1 BCD degree + 3 BCD minutes.  Minutes are MM.mmmm (six digits). */
static void ack_fix(const uint8_t *p, ack_msg_t *a)
{
    const uint32_t lon_deg = ack_bcd(p + 0u, 2u);
    const uint32_t lon_min = ack_bcd(p + 2u, 3u);
    const uint32_t lat_deg = ack_bcd(p + 5u, 1u);
    const uint32_t lat_min = ack_bcd(p + 6u, 3u);

    if (lon_deg == 0xFFFFFFFFu || lon_min == 0xFFFFFFFFu ||
        lat_deg == 0xFFFFFFFFu || lat_min == 0xFFFFFFFFu)
        return;
    if (lon_deg > 180u || lat_deg > 90u ||
        lon_min >= 600000u || lat_min >= 600000u)
        return;

    a->lon6 = lon_deg * 1000000u + (lon_min * 5u) / 3u;
    a->lat6 = lat_deg * 1000000u + (lat_min * 5u) / 3u;
    a->fix  = 1u;
}

/* Parse one 0x88 table 7 frame. */
static bool ack_parse(const uint8_t *f, uint8_t len, ack_msg_t *a)
{
    if (len < 61u)
        return false;
    if (f[0] != 0x0Cu || f[9] != 0x88u)
        return false;

    const uint8_t m = f[10];
    if (m < 48u)
        return false;
    if (f[1] != (uint8_t)(m + 11u))
        return false;
    if ((uint16_t)f[1] + 2u > len)
        return false;

    const uint16_t rx = (uint16_t)(((uint16_t)f[f[1]] << 8) | f[f[1] + 1u]);
    if (crc16(f, f[1]) != rx)
        return false;

    const uint8_t *p = f + 11u;
    if (p[0] != 0x81u && p[0] != 0x82u && p[0] != 0x91u)
        return false;

    memset(a, 0, sizeof(*a));
    a->type = p[0];
    a->func = p[1];
    memcpy(a->date, p + 2u, 3u);
    memcpy(a->time, p + 5u, 3u);
    ack_str(a->train, p + 8u, 7u);
    ack_str(a->loco, p + 15u, 8u);
    a->place_lo = p[23];
    ack_str(a->num, p + 24u, 6u);

    const uint32_t km = le24(p + 30u);
    if (km != 0xFFFFFFu) {
        a->km_valid = 1u;
        a->km_abs   = km & 0x3FFFFFu;
        a->km_neg   = (km & 0x800000u) ? 1u : 0u;
        a->km_inc   = (km & 0x400000u) ? 1u : 0u;
    }

    ack_fix(p + 33u, a);
    a->place_hi = p[42];
    a->pkt      = p[47];
    return true;
}
#endif /* ENABLE_TDCS_ACK */

/* ---- 26-bit codewords -------------------------------------------------- */

static bool framer_bit(tdcs_fr_t *f, uint8_t bit)
{
    if (!f->state)
        return false;

    tdcs_col_t *c = &f->col;
    const uint8_t b = f->inv ? (uint8_t)!bit : bit;

    c->cw = (c->cw << 1) | b;
    if (++c->cw_bits < TDCS_CW_BITS)
        return false;
    c->cw_bits = 0u;

    const uint16_t data = bch_decode(c->cw);
    c->cw = 0u;

    if ((uint16_t)c->nbytes + 2u <= TDCS_FRAME_PAD) {
        c->frame[c->nbytes++] = (uint8_t)(data >> 8);
        c->frame[c->nbytes++] = (uint8_t)(data & 0xFFu);
    }
    c->cw_index++;

    if (c->cw_index == 1u) {
        if (c->frame[0] != 0x0Cu) {
            f->state = 0u;
            return false;
        }
#ifdef ENABLE_TDCS_ACK
        /* 43 is the 45-byte train report; a table 7 reply is m + 11 with
         * m >= 48, i.e. 59 bytes or more. */
        const uint8_t flen = c->frame[1];
        if (flen != 43u && flen < 59u) {
            f->state = 0u;
            return false;
        }
        c->total_cw = (uint8_t)(((uint16_t)flen + 3u) / 2u);
#else
        if (c->frame[1] != 43u) {
            f->state = 0u;
            return false;
        }
        c->total_cw = (uint8_t)(((uint16_t)c->frame[1] + 3u) / 2u);
#endif
        if (c->total_cw < 2u || c->total_cw > TDCS_CW_COUNT) {
            f->state = 0u;
            return false;
        }
    }

    if (c->cw_index >= c->total_cw) {
        f->state = 0u;
#ifdef ENABLE_TDCS_ACK
        /* A table 7 reply travels on the TDCS channel in a 0x88 frame. */
        if (c->frame[9] == 0x88u) {
            ack_msg_t a;
            if (ack_parse(c->frame, (uint8_t)(c->frame[1] + 2u), &a)) {
                s_ack      = a;
                s_have_ack = true;
                s_fr[0].state = 0u;
                s_fr[1].state = 0u;
                return true;
            }
            return false;
        }
#endif
        tdcs_msg_t m;
        /* The codeword stream is padded to an even byte count, so an odd
         * frame carries one byte outside the CRC.  Hand the decoder the
         * declared length, not the padded one. */
        if (tdcs_parse(c->frame, (uint8_t)(c->frame[1] + 2u), &m)) {
            s_msg  = m;
            s_have = true;
            s_fr[0].state = 0u;
            s_fr[1].state = 0u;
            return true;
        }
    }
    return false;
}

/* ---- public ------------------------------------------------------------ */

void TDCS_DecReset(void)
{
    memset(s_fr, 0, sizeof(s_fr));
    memset(&s_msg, 0, sizeof(s_msg));
    s_have = false;
#ifdef ENABLE_TDCS_ACK
    memset(&s_ack, 0, sizeof(s_ack));
    s_have_ack = false;
#endif
}

void TDCS_DecSync(void)
{
    memset(s_fr, 0, sizeof(s_fr));
    s_fr[0].state = 1u; s_fr[0].inv = 0u;
    s_fr[1].state = 1u; s_fr[1].inv = 1u;
}

bool TDCS_DecWord16(uint16_t w)
{
    bool got = false;

    for (uint8_t i = 0; i < 8u; i++) {              /* low byte first,  */
        const uint8_t b = (uint8_t)((w >> (7u - i)) & 1u);
        got |= framer_bit(&s_fr[0], b);
        got |= framer_bit(&s_fr[1], b);
    }
    for (uint8_t i = 0; i < 8u; i++) {              /* then high byte   */
        const uint8_t b = (uint8_t)((w >> (15u - i)) & 1u);
        got |= framer_bit(&s_fr[0], b);
        got |= framer_bit(&s_fr[1], b);
    }
    return got;
}

bool TDCS_DecEnd(void)
{
    bool got = false;

    for (uint8_t i = 0; i < 2u; i++) {
        if (s_fr[i].state) {
            got |= framer_bit(&s_fr[i], 0u);
            got |= framer_bit(&s_fr[i], 0u);
        }
    }
    return got;
}

bool TDCS_DecTake(tdcs_msg_t *out)
{
    if (!s_have)
        return false;
    *out = s_msg;
    s_have = false;
    return true;
}

#ifdef ENABLE_TDCS_ACK
bool TDCS_DecTakeAck(ack_msg_t *out)
{
    if (!s_have_ack)
        return false;
    *out = s_ack;
    s_have_ack = false;
    return true;
}
#endif
