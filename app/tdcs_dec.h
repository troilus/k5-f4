/* TDCS decoder core - no hardware dependencies.
 *
 * 450 MHz train-number check system (TB/T 3325-2013, A-class frame).
 * The BK4819 FSK engine has already recovered the bit clock and stripped the
 * 32-bit frame sync, so this core only has to collect 26-bit BCH(26,16)
 * codewords, check the CRC and parse the 45-byte frame.
 */
#ifndef APP_TDCS_DEC_H
#define APP_TDCS_DEC_H

#include <stdint.h>
#include <stdbool.h>

#define TDCS_FRAME_BYTES  45u
#ifdef ENABLE_TDCS_ACK
/* A table 7 reply is 48 + 11 = 59 bytes, i.e. 31 codewords; the codeword
 * stream carries two data bytes per codeword, so the pad is 62 bytes. */
#define TDCS_FRAME_PAD    62u
#define TDCS_CW_COUNT     31u
#else
#define TDCS_FRAME_PAD    46u
#define TDCS_CW_COUNT     23u
#endif
#define TDCS_CW_BITS      26u

/* One decoded A-class frame. */
typedef struct {
    uint8_t  cls[4];
    uint32_t train_no;
    uint32_t km_abs;
    uint16_t loco_no, weight, length10, driver, speed;
    uint8_t  loco_type, loco_ext, crew;
    uint8_t  km_valid, km_neg, km_inc;
    uint8_t  cars, station, station_ext, section, route, driver_ext;
    uint8_t  sig, sig_type, test, crc_ok;
} tdcs_msg_t;

#ifdef ENABLE_TDCS_ACK
/* One TB/T 3231 table 7 confirmation/sign-off/shunting-request frame (0x88).
 * The strings are the printable ASCII the frame carries, trailing spaces
 * dropped.  lon6/lat6 are 1e-6 degrees, built from the BCD signing position. */
typedef struct {
    uint8_t  type, func;
    uint8_t  date[3], time[3];
    uint8_t  place_lo, place_hi;
    uint8_t  km_valid, km_neg, km_inc, fix;
    uint8_t  pkt;
    char     train[8], loco[9], num[7];
    uint32_t km_abs;
    uint32_t lon6, lat6;
} ack_msg_t;
#endif

void TDCS_DecReset(void);

/* The BK4819 matched the frame sync and stripped it: the next FIFO bits are
 * codeword 0.  Two framers run in parallel, one normal and one inverted, so
 * the deviation polarity does not have to be known in advance. */
void TDCS_DecSync(void);

/* One FIFO word as REG_5F hands it over: low byte first, MSB first inside
 * each byte.  Returns true when a valid frame completed. */
bool TDCS_DecWord16(uint16_t w);

/* Carrier went away: complete a codeword that is one or two bits short.
 * Returns true when a valid frame completed. */
bool TDCS_DecEnd(void);

/* Take the last decoded frame (clears the pending flag). */
bool TDCS_DecTake(tdcs_msg_t *out);

#ifdef ENABLE_TDCS_ACK
/* Take the last decoded table 7 reply (clears its pending flag). */
bool TDCS_DecTakeAck(ack_msg_t *out);
#endif

#endif /* APP_TDCS_DEC_H */
