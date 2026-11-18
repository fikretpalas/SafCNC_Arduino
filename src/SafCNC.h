/*
 * ============================================================================
 *  SafCNC - Arduino Uno / Nano motion controller firmware for Mach3
 * ----------------------------------------------------------------------------
 *  File      : SafCNC.h - public interface of the library
 *  Developer : Fikret Palas
 *  E-mail    : safcnc@tutamail.com
 *  Web       : https://github.com/fikretpalas
 *  Plugin    : https://github.com/fikretpalas/SafCNC_Mach3Plugin_en  (English)
 *              https://github.com/fikretpalas/SafCNC_Mach3Plugin_tr  (Turkish)
 *  License   : MIT
 *  Cards     : this firmware is for the Arduino Uno / Nano (about 30 kHz); the same
 *              protocol can run on an STM32 at up to 100 kHz (not tested)
 * ============================================================================
 *  Turns an ATmega328P board (Arduino Uno, Nano, Pro Mini 16 MHz) into a
 *  USB motion controller for Mach3, used together with the SafCNC Mach3 plugin.
 *  The pinout is compatible with GRBL 1.1 and the Arduino CNC Shield V3.51.
 *
 *  Usage:
 *    #include <SafCNC.h>
 *    void setup() { SafCNC.begin(); }
 *    void loop()  { SafCNC.update(); }
 *
 *  Machine settings are in SafCNC_Config.h.
 *  Do not use delay() or Serial in your sketch: the card owns the serial
 *  port, and update() must be called as often as possible.
 */
#ifndef SAFCNC_H
#define SAFCNC_H

#include <Arduino.h>

#define SAFCNC_VERSION "1.0.1"

class SafCNCClass {
public:
  /* Configure pins, step timer and serial port. Call once from setup(). */
  void begin();
  /* Handle serial packets, link timeout and E-Stop. Call from loop(). */
  void update();
};

extern SafCNCClass SafCNC;

#endif
