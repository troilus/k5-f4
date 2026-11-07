/* TDCS receive mode (resident firmware feature).
 *
 * A channel whose modulation is set to TDCS is a receive-only mode for the
 * 450 MHz train-number check system (TB/T 3325-2013, A-class frame).
 *
 * The BK4819 FSK engine is configured with losehu's proven MDC1200/Messenger
 * RX recipe (FFSK 1200/1800) and hands the raw codeword bits over through
 * REG_5F.  The portable BCH/CRC/parse core lives in tdcs_dec.c.
 */
#ifndef APP_TDCS_H
#define APP_TDCS_H

#include <stdbool.h>
#include <stdint.h>

/* True when the selected RX VFO is in TDCS mode. */
bool TDCS_RxVfoIsTdcs(void);

/* Called from the 10 ms tick: ages reports, services the LED, starts/stops the
 * FSK engine from the VFO state and refreshes the decode window. */
void TDCS_Tick(void);

/* True while the FSK engine is armed (dual watch is held off then). */
bool TDCS_Active(void);

/* Stop the engine now (PTT, mode change, overlay app launch). */
void TDCS_StopSampler(void);

/* The BK4819 interrupt bits, handed over by CheckRadioInterrupts() so this
 * module never steals the radio's squelch/CTCSS interrupts. */
void TDCS_HandleInterrupts(uint16_t bits);

/* Decode window state. */
bool TDCS_PopupActive(void);
void TDCS_ClosePopup(void);
void TDCS_DrawPopup(void);

/* Open the decode window by hand (F+7): shows the newest report and stays up
 * until EXIT or PTT.  Returns false when there is nothing decoded yet. */
bool TDCS_OpenPopup(void);

/* Key hook: while the window is up every key event is swallowed.  Only a fresh
 * press acts - UP/DOWN step through the history, 6/9 page the field list,
 * 2 clears the history and EXIT closes the window.  PTT closes the window but
 * is left to the normal transmit path.  Returns true when the key was used. */
bool TDCS_HandleKey(uint8_t key, bool pressed, bool held);

#endif /* APP_TDCS_H */
