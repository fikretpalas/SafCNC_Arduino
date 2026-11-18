/*
 * ============================================================================
 *  SafCNC - Arduino Uno / Nano motion controller firmware for Mach3
 * ----------------------------------------------------------------------------
 *  File      : SafCNC_Config.h - user settings
 *  Developer : Fikret Palas
 *  E-mail    : safcnc@tutamail.com
 *  Web       : https://github.com/fikretpalas
 *  Plugin    : https://github.com/fikretpalas/SafCNC_Mach3Plugin_en  (English)
 *              https://github.com/fikretpalas/SafCNC_Mach3Plugin_tr  (Turkish)
 *  License   : MIT
 *  Cards     : this firmware is for the Arduino Uno / Nano (about 30 kHz); the same
 *              protocol can run on an STM32 at up to 100 kHz (not tested)
 * ============================================================================
 *  Edit the values below to match your machine, then upload again.
 */
#ifndef SAFCNC_CONFIG_H
#define SAFCNC_CONFIG_H

/* ---------------- Machine I/O polarity ---------------- */

/* Invert STEP pulses. Bit 0 = X, bit 1 = Y, bit 2 = Z. */
#define STEP_INVERT 0x00

/* Invert DIRECTION signals. Bit 0 = X, bit 1 = Y, bit 2 = Z. */
#define DIR_INVERT 0x00

/* 1 = stepper ENABLE pin is active LOW (A4988, DRV8825, CNC Shield V3). */
#define ENABLE_ACTIVE_LOW 1

/* Invert inputs. Bits: 0x01 X limit, 0x02 Y limit, 0x04 Z limit,
   0x08 Feed Hold, 0x10 Cycle Start, 0x20 E-Stop, 0x40 Probe.
   By default all inputs are active LOW: the switch connects the pin to GND
   and the internal pull-up resistors are enabled. */
#define INPUT_INVERT 0x00

/* ---------------- Safety ---------------- */

/* 1 = the card's own E-Stop input (A0) stops the machine, even if Mach3 does not. */
#define LOCAL_ESTOP 1

/* E-Stop input debounce time in milliseconds. */
#define ESTOP_DEBOUNCE_MS 5

/* 1 = keep stepper drivers enabled (holding torque) after a non-E-Stop fault. */
#define HOLD_TORQUE_ON_FAULT 1

/* Number of consecutive step ticks a limit / probe input must be active. */
#define LIMIT_FILTER 3
#define PROBE_FILTER 2

/* A slice with more steps than this is "fast". If the buffer runs empty
   right after a fast slice, the card reports a buffer underrun. */
#define UNDERRUN_STEPS 32

/* ---------------- Spindle ---------------- */

/* Spindle PWM on D11 (Timer2):
   0 = Arduino default (~490 Hz), 1 = fast PWM (~980 Hz), 2 = fast PWM (~7.8 kHz). */
#define SPINDLE_PWM_MODE 1

/* ---------------- Timing - change only if you know the protocol ---------------- */

#define STEP_PULSE_US 4       /* step pulse width in microseconds              */
#define BAUD 115200           /* must match the Mach3 plugin                   */
#define SUBTICKS 160          /* step ticks per 5 ms slice -> 32 kHz tick rate */
#define STEP_LIMIT 160        /* max steps per axis per 5 ms slice (= 32 kHz). */
                              /* Sent to the Mach3 plugin in the K_INFO packet. */
#define QSIZE 200             /* motion buffer: 200 slices x 5 ms = 1 second   */
#define STATUS_B2_FIFO 1
#define STATUS_PERIOD_MS 50   /* status period while the PC is talking to the card  */
#define STATUS_IDLE_MS 500    /* status period at power-up and after the link is lost */
#define LINK_ALIVE_MS 300     /* PC counts as connected if a valid packet came this recently */
#define STATUS_PHASE_MS 25
#define CONN_TIMEOUT_MS 200
#define BYTE_GAP_US 3000

/* Debug: 1 = pulse A4 high during the step interrupt (disables the Mist output). */
#define ISR_TIMING_PIN 0

#endif
