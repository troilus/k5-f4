/* LBJ / POCSAG receive for the classic UV-K5.
 *
 * The K1 LBJRadio samples the BK4819 AF output on PA4 at 19.2 kHz; the classic
 * UV-K5 has no audio-to-ADC path (PA9 is the battery divider), so this port
 * uses the BK4819 FSK engine in direct-FSK 1.2K mode instead, the same engine
 * the AirCopy path and TDCS use.  The POCSAG core (lbj_dec.c) is the K1 file.
 */
#ifndef APP_LBJ_H
#define APP_LBJ_H

#include <stdbool.h>
#include <stdint.h>

/* The selected RX VFO is in LBJ modulation. */
bool LBJ_RxVfoIsLbj(void);

/* The LBJ decode screen owns the main display. */
bool LBJ_ScreenActive(void);

/* 10 ms tick: starts/stops the FSK engine, repaints on a new report. */
void LBJ_Tick(void);

/* Hand the already-read BK4819 interrupt bits over (app.c). */
void LBJ_HandleInterrupts(uint16_t bits);

/* Full-screen decode/debug page. */
void LBJ_Draw(void);

/* Stop the FSK engine now (mode change, TX). */
void LBJ_StopSampler(void);

/* Key hook: PTT sends the fixed LBJ test frame when ENABLE_LBJ_TX is set. */
bool LBJ_HandleKey(uint8_t key, bool pressed, bool held);

#endif /* APP_LBJ_H */
