/*
 * SafCNC - Arduino Uno / Nano motion controller firmware for Mach3
 * Example   : SafCNC_Basic.ino - the complete firmware sketch
 * Developer : Fikret Palas <safcnc@tutamail.com>
 * Web       : https://github.com/fikretpalas
 * Plugin    : https://github.com/fikretpalas/SafCNC_Mach3Plugin_en  (English)
 *             https://github.com/fikretpalas/SafCNC_Mach3Plugin_tr  (Turkish)
 * License   : MIT
 * Cards     : Arduino Uno / Nano (about 30 kHz); the same protocol can run on
 *             an STM32 at up to 100 kHz (not tested)
 *
 * Select "Arduino Uno" or "Arduino Nano" (ATmega328P, 16 MHz) and upload.
 * Machine settings are in src/SafCNC_Config.h inside the library folder.
 */
#include <SafCNC.h>

void setup() {
  SafCNC.begin();
}

void loop() {
  SafCNC.update();
}
