/*
 * ============================================================================
 *  SafCNC - Arduino Uno / Nano motion controller firmware for Mach3
 * ----------------------------------------------------------------------------
 *  File      : SafCNC.cpp - step generator, packet protocol and safety logic
 *  Developer : Fikret Palas
 *  E-mail    : safcnc@tutamail.com
 *  Web       : https://github.com/fikretpalas
 *  Plugin    : https://github.com/fikretpalas/SafCNC_Mach3Plugin_en  (English)
 *              https://github.com/fikretpalas/SafCNC_Mach3Plugin_tr  (Turkish)
 *  License   : MIT
 *  Cards     : this firmware is for the Arduino Uno / Nano (about 30 kHz); the same
 *              protocol can run on an STM32 at up to 100 kHz (not tested)
 *  Version   : 1.0.1
 * ============================================================================
 *  Works together with the SafCNC Mach3 plugin (Lazarus / Free Pascal source
 *  in the repositories above).
 *
 *  BOARDS
 *    Arduino Uno, Nano, Pro Mini - ATmega328P at 16 MHz only.
 *
 *  PERFORMANCE
 *    The PC sends one move packet per 5 ms slice (max 160 steps per axis).
 *    A Timer1 interrupt at 32 kHz spreads the steps evenly with a DDA.
 *    Up to ~30 kHz step rate per axis in normal use (32 kHz theoretical).
 *    Motion buffer: 200 slices = 1 second.
 *
 *  PINOUT - GRBL 1.1 / Arduino CNC Shield V3.51 compatible
 *    (V3.00 clones: Z limit -> SpnEn header, spindle PWM -> Z+ header)
 *    D2  X step        D5  X dir        D9  X limit
 *    D3  Y step        D6  Y dir        D10 Y limit
 *    D4  Z step        D7  Z dir        D12 Z limit
 *    D8  stepper enable (active LOW)
 *    D11 spindle PWM   D13 spindle direction
 *    A0  E-Stop        A1  Feed Hold    A2  Cycle Start
 *    A3  Flood coolant A4  Mist coolant A5  Probe
 *    Inputs are active LOW with internal pull-ups (switch to GND).
 *
 *  CONNECTION TIME
 *    With the Arduino bootloader the card connects in about 7 seconds,
 *    because the bootloader runs first every time the port is opened.
 *    Upload with an ISP programmer ("Sketch > Upload Using Programmer")
 *    to remove the bootloader - the card then connects almost instantly.
 *    Use "Tools > Burn Bootloader" to restore normal USB uploads.
 *
 *  SAFETY
 *    200 ms link watchdog, checksum and sequence counter on every packet,
 *    local E-Stop input, limit and probe inputs checked in the step interrupt.
 * ============================================================================
 */
#include "SafCNC.h"
#include "SafCNC_Config.h"

#if !defined(__AVR_ATmega328P__)
#error "SafCNC: please select an ATmega328P board (Arduino Uno, Nano or Pro Mini 16 MHz)"
#endif

SafCNCClass SafCNC;


/* ---------------- Timing and compile-time checks ----------------
 * One 5 ms slice (200 per second) is split into SUBTICKS step ticks.
 * At 16 MHz: 16,000,000 / 200 / 160 = 500 CPU cycles per tick = 32 kHz. */
#define TICK_CYCLES (F_CPU / 200UL / SUBTICKS)
#define STEP_PULSE_CYCLES (F_CPU / 1000000UL * STEP_PULSE_US)
static_assert((F_CPU / 200UL) % SUBTICKS == 0, "SafCNC: F_CPU/200 must be divisible by SUBTICKS");
static_assert(SUBTICKS <= 255, "SafCNC: SUBTICKS must fit in 8 bits");
static_assert(STEP_LIMIT >= 1 && STEP_LIMIT <= SUBTICKS, "SafCNC: STEP_LIMIT must be 1..SUBTICKS");
static_assert(STEP_LIMIT <= 500, "SafCNC: STEP_LIMIT above 500 (100 kHz) is not allowed by the protocol");
static_assert(QSIZE >= 2 && QSIZE <= 255, "SafCNC: QSIZE must be 2..255");
static_assert(TICK_CYCLES >= 300, "SafCNC: step tick too short - a 16 MHz board is required");
static_assert(LIMIT_FILTER >= 1 && PROBE_FILTER >= 1, "SafCNC: LIMIT_FILTER and PROBE_FILTER must be at least 1");
static_assert((INPUT_INVERT & 0x80) == 0, "SafCNC: INPUT_INVERT bit 7 must be 0");
static_assert(STATUS_IDLE_MS >= STATUS_PERIOD_MS, "SafCNC: STATUS_IDLE_MS must be >= STATUS_PERIOD_MS");
static_assert(LINK_ALIVE_MS > STATUS_PERIOD_MS, "SafCNC: LINK_ALIVE_MS must be > STATUS_PERIOD_MS");

/* ---------------- Protocol (must match uCardProtocol.pas / uKartProtokol.pas in the plugin) ----------------
 * Packet commands. PC -> card: K_RESET/K_READY/K_ACTIVE (8 bytes), K_MOVE (10 bytes).
 * Card -> PC: status packet (8 bytes) with the current state as command,
 *             and K_INFO (8 bytes) with the card's speed:
 *               b2..b3 = STEP_LIMIT, 16-bit big-endian (max steps per axis per
 *                        5 ms slice; step rate = STEP_LIMIT x 200 Hz)
 *               b4..b5 = SUBTICKS, 16-bit big-endian (step ticks per slice)
 *             Arduino Uno / Nano: 160 / 160 = 32 kHz. The protocol allows at
 *             most 500 steps per slice (100 kHz, e.g. an STM32 card).
 *             The sub-tick in the RESET status packet is one byte: a card
 *             with more than 255 sub-ticks sends it scaled to 0..255
 *             (sub * 256 / SUBTICKS).
 *             K_INFO is sent at power-up, after a link timeout (CONN_TIMEOUT_MS)
 *             and on the first K_RESET of every new connection, just before
 *             the RESET status packet. */
#define K_RESET 65
#define K_READY 66
#define K_ACTIVE 67
#define K_MOVE 68
#define K_INFO 64
#define LEN_STATUS 8
#define LEN_MOVE 10

/* Reset reason codes (H). Sent to the PC and shown in Mach3. */
#define H_POWERON 0
#define H_ESTOP 1
#define H_SEQ 2
#define H_CHECKSUM 3
#define H_BADCMD 4
#define H_BADLEN 5
#define H_TIMEOUT 6
#define H_OVERFLOW 7
#define H_STEPLIMIT 8
#define H_LOCALESTOP 9
#define H_BADSTATE 10
#define H_LIMIT 11
#define H_PROBE 12
#define H_UNDERRUN 13
#define H_MAX 13

/* Output byte from the PC: spindle, coolant and stop options. */
#define O_SPEN 0x01
#define O_SPDIR 0x02
#define O_FLOOD 0x04
#define O_MIST 0x08
#define O_LIMSTOP 0x10
#define O_PRBSTOP 0x20
#define O_UNDERRUN 0x40
#define O_CTRL (O_LIMSTOP | O_PRBSTOP | O_UNDERRUN)

/* Input byte sent to the PC: limits, buttons, E-Stop, probe. */
#define I_LIMX 0x01
#define I_LIMY 0x02
#define I_LIMZ 0x04
#define I_HOLD 0x08
#define I_START 0x10
#define I_ESTOP 0x20
#define I_PROBE 0x40

/* Card states:
 *   RESET - stopped, drivers may be off, waiting for the PC
 *   READY - linked, outputs active, no motion
 *   ACTIVE - accepts move packets, buffer empty
 *   MOVE  - executing slices from the buffer */
#define ST_RESET 0
#define ST_READY 1
#define ST_ACTIVE 2
#define ST_MOVE 3

/* ---------------- Pins (GRBL 1.1 layout) ---------------- */
#define PIN_ENABLE 8
#define PIN_SPINDLE 11
#define PIN_SPDIR 13
#define PIN_FLOOD A3
#define PIN_MIST A4
#define PD_STEP 0x1C
#define PD_DIR 0xE0
#define PB_LIMX _BV(1)
#define PB_LIMY _BV(2)
#define PB_LIMZ _BV(4)
#define PB_LIMITS (PB_LIMX | PB_LIMY | PB_LIMZ)
#define PC_ESTOP _BV(0)
#define PC_HOLD _BV(1)
#define PC_START _BV(2)
#define PC_PROBE _BV(5)
#define PC_INPUTS (PC_ESTOP | PC_HOLD | PC_START | PC_PROBE)
#define STEP_IDLE ((STEP_INVERT & 0x07) << 2)
#define DIR_FLIP ((DIR_INVERT & 0x07) << 5)
#define PB_INV (((INPUT_INVERT & I_LIMX) ? PB_LIMX : 0) | \
               ((INPUT_INVERT & I_LIMY) ? PB_LIMY : 0) | \
               ((INPUT_INVERT & I_LIMZ) ? PB_LIMZ : 0))
#define PC_PRB_INV ((INPUT_INVERT & I_PROBE) ? PC_PROBE : 0)

/* ---------------- Motion buffer ----------------
 * One slice = steps for X, Y, Z during 5 ms (0..160 each) + direction bits.
 * Ring buffer: handleMove() writes at head, the timer interrupt reads at tail. */
struct Slice {
  uint8_t ax, ay, az;
  uint8_t flags;
};
#define SF_FAST 0x01
static Slice q[QSIZE];
static volatile uint8_t head = 0;
static volatile uint8_t tail = 0;

/* Shared between the interrupt and the main loop. */
static volatile uint8_t state = ST_RESET;
static volatile uint8_t tripCode = 0;
static volatile uint8_t ctrl = 0;
static volatile uint8_t sub = 0;
static volatile uint16_t sliceCnt = 0;
static uint8_t hCode = H_POWERON;
static bool resetAck = false;
static uint8_t spindleVal = 0;
static uint8_t outBits = 0;
static uint8_t txN = 0;
static uint8_t rxExpectN = 0;
static uint16_t stopL = 0;
static uint8_t stopSub = 0;
static uint32_t lastRx = 0;
static bool linkSeen = false;   /* a valid packet has been received from the PC */
static uint32_t nextStatusAt = 0;
static uint32_t estopSince = 0;

/* DDA (digital differential analyzer) state of the current slice. */
static uint8_t absX, absY, absZ;
static uint8_t remX, remY, remZ;
static uint8_t accX, accY, accZ;
static uint8_t fastSlice;
static uint8_t limCnt, prbCnt;

/* ---------------- Step generator (runs in the timer interrupt, 32 kHz) ----------------
 * 1. Checks limit and probe inputs (filtered) and trips a stop if needed.
 * 2. At the start of each slice takes the next entry from the buffer.
 *    An empty buffer after a fast slice means the PC was too late (underrun).
 * 3. A DDA spreads the slice's steps evenly over the 160 ticks
 *    and all three axes are pulsed together on PORTD. */
static inline void stepTick() {
  uint8_t st = state;
  if (st < ST_ACTIVE || tripCode) return;
  if (st == ST_MOVE) {
    uint8_t c = ctrl;
    if (c & O_LIMSTOP) {
      if ((uint8_t)(~PINB ^ PB_INV) & PB_LIMITS) {
        if (++limCnt >= LIMIT_FILTER) { tripCode = H_LIMIT; return; }
      } else limCnt = 0;
    }
    if (c & O_PRBSTOP) {
      if ((uint8_t)(~PINC ^ PC_PRB_INV) & PC_PROBE) {
        if (++prbCnt >= PROBE_FILTER) { tripCode = H_PROBE; return; }
      } else prbCnt = 0;
    }
  }
  if (sub == 0) {
    uint8_t t = tail;
    if (t == head) {
      if (st == ST_MOVE) {
        if (fastSlice && (ctrl & O_UNDERRUN)) tripCode = H_UNDERRUN;
        fastSlice = 0;
        state = ST_ACTIVE;
      }
      return;
    }
    const Slice &s = q[t];
    absX = s.ax; remX = SUBTICKS - s.ax; accX = 0;
    absY = s.ay; remY = SUBTICKS - s.ay; accY = 0;
    absZ = s.az; remZ = SUBTICKS - s.az; accZ = 0;
    fastSlice = s.flags & SF_FAST;
    PORTD = (PORTD & ~PD_DIR) | (s.flags & PD_DIR);
    tail = (t + 1 >= QSIZE) ? 0 : t + 1;
    sliceCnt++;
    state = ST_MOVE;
  }
  uint8_t m = 0;
  if (accX >= remX) { accX -= remX; m |= _BV(2); } else accX += absX;
  if (accY >= remY) { accY -= remY; m |= _BV(3); } else accY += absY;
  if (accZ >= remZ) { accZ -= remZ; m |= _BV(4); } else accZ += absZ;
  if (m) {
    PORTD ^= m;
    __builtin_avr_delay_cycles(STEP_PULSE_CYCLES);
    PORTD ^= m;
  }
  if (++sub >= SUBTICKS) sub = 0;
}

/* Timer1 compare interrupt, every TICK_CYCLES CPU cycles. */
ISR(TIMER1_COMPA_vect) {
#if ISR_TIMING_PIN
  PORTC |= _BV(4);
#endif
  stepTick();
#if ISR_TIMING_PIN
  PORTC &= ~_BV(4);
#endif
}

/* Reads all inputs (active LOW) into the I_ bit format, with polarity applied. */
static uint8_t readInputs() {
  uint8_t b = ~PINB, c = ~PINC, i = 0;
  if (b & PB_LIMX) i |= I_LIMX;
  if (b & PB_LIMY) i |= I_LIMY;
  if (b & PB_LIMZ) i |= I_LIMZ;
  if (c & PC_HOLD) i |= I_HOLD;
  if (c & PC_START) i |= I_START;
  if (c & PC_ESTOP) i |= I_ESTOP;
  if (c & PC_PROBE) i |= I_PROBE;
  return i ^ INPUT_INVERT;
}
static inline bool estopActive() { return readInputs() & I_ESTOP; }
static inline void driversEnable(bool on) {
  digitalWrite(PIN_ENABLE, (on == (ENABLE_ACTIVE_LOW != 0)) ? LOW : HIGH);
}

/* Number of slices waiting in the buffer. */
static uint8_t fifoFill() {
  uint8_t t = tail, h = head;
  return (h >= t) ? (uint8_t)(h - t) : (uint8_t)(h + QSIZE - t);
}

/* Sends an 8-byte packet: command, counter, 4 data bytes, 16-bit checksum.
 * The checksum makes the sum of all bytes zero. */
static void sendPacket8(uint8_t k, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5) {
  uint8_t p[LEN_STATUS];
  p[0] = k; p[1] = txN++; p[2] = b2; p[3] = b3; p[4] = b4; p[5] = b5;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 6; i++) sum += p[i];
  uint16_t mm = (uint16_t)(0 - sum);
  p[6] = (uint8_t)(mm >> 8);
  p[7] = (uint8_t)mm;
  Serial.write(p, LEN_STATUS);
}

/* Info packet: tells the PC the fastest the card can step, so the plugin can
 * limit Mach3's velocity instead of tripping H_STEPLIMIT. */
static void sendInfo() {
  sendPacket8(K_INFO, (uint8_t)((uint16_t)STEP_LIMIT >> 8), (uint8_t)STEP_LIMIT,
              (uint8_t)((uint16_t)SUBTICKS >> 8), (uint8_t)SUBTICKS);
}

/* Status period: every 50 ms while the PC is talking to the card,
 * every 500 ms at power-up and after the link is lost (no valid packet
 * for LINK_ALIVE_MS). */
static uint16_t statusPeriod() {
  if (linkSeen && (uint32_t)(millis() - lastRx) <= LINK_ALIVE_MS) return STATUS_PERIOD_MS;
  return STATUS_IDLE_MS;
}

/* Status packet. In RESET it reports the reason and where motion stopped
 * (slice number + sub-tick) so the PC can correct its position. */
static void sendStatus() {
  uint8_t st = state;
  if (st == ST_RESET) {
    sendPacket8(K_RESET, hCode, (uint8_t)(stopL >> 8), (uint8_t)stopL, stopSub);
  } else {
    uint8_t k = (st == ST_READY) ? K_READY : (st == ST_ACTIVE) ? K_ACTIVE : K_MOVE;
    sendPacket8(k, STATUS_B2_FIFO ? fifoFill() : 0, spindleVal, outBits, readInputs());
  }
  nextStatusAt = millis() + statusPeriod();
}

/* Spindle PWM, spindle direction and coolant outputs. */
static void applyOutputs() {
  analogWrite(PIN_SPINDLE, (outBits & O_SPEN) ? spindleVal : 0);
  digitalWrite(PIN_SPDIR, (outBits & O_SPDIR) ? HIGH : LOW);
  digitalWrite(PIN_FLOOD, (outBits & O_FLOOD) ? HIGH : LOW);
#if !ISR_TIMING_PIN
  digitalWrite(PIN_MIST, (outBits & O_MIST) ? HIGH : LOW);
#endif
  ctrl = outBits & O_CTRL;
}

/* Stops everything immediately: clears the buffer, records the stop position,
 * turns off outputs and (for E-Stop) disables the stepper drivers. */
static void enterReset(uint8_t h, bool notify) {
  noInterrupts();
  uint8_t t = tripCode;
  tripCode = 0;
  stopL = sliceCnt;
  stopSub = sub;
  state = ST_RESET;
  head = 0; tail = 0; sub = 0;
  limCnt = 0; prbCnt = 0;
  interrupts();
  bool cut = !HOLD_TORQUE_ON_FAULT || h == H_POWERON || h == H_ESTOP || h == H_LOCALESTOP;
  if (t && h != H_ESTOP && h != H_LOCALESTOP) h = t;
  PORTD = (PORTD & ~(PD_STEP | PD_DIR)) | STEP_IDLE;
  if (cut) driversEnable(false);
  spindleVal = 0; outBits = 0;
  applyOutputs();
  hCode = h;
  resetAck = false;
  if (notify) sendStatus();
}
static void protoError(uint8_t h) {
  if (state != ST_RESET) enterReset(h, true);
}

/* Expected packet length for a command byte, 0 = unknown command. */
static uint8_t rxLength(uint8_t k) {
  switch (k) {
    case K_RESET: case K_READY: case K_ACTIVE: return LEN_STATUS;
    case K_MOVE: return LEN_MOVE;
    default: return 0;
  }
}
static inline int16_t rd16(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* K_READY / K_ACTIVE from the PC: leaves RESET (if allowed) and updates outputs. */
static void handleStateCmd(uint8_t k, uint8_t s, uint8_t o) {
  uint8_t st = state;
  o &= 0x7F;
  if (st == ST_RESET) {
    if (!resetAck) return;
    if (LOCAL_ESTOP && estopActive()) { hCode = H_LOCALESTOP; return; }
    spindleVal = s; outBits = o; applyOutputs();
    driversEnable(true);
    sliceCnt = 0;
    state = (k == K_ACTIVE) ? ST_ACTIVE : ST_READY;
    return;
  }
  if (k == K_READY && st == ST_MOVE) { protoError(H_BADSTATE); return; }
  spindleVal = s; outBits = o; applyOutputs();
  if (k == K_ACTIVE && st == ST_READY) {
    state = ST_ACTIVE;
  } else if (k == K_READY && st == ST_ACTIVE) {
    noInterrupts();
    bool idle = (state == ST_ACTIVE && head == tail);
    if (idle) state = ST_READY;
    interrupts();
    if (!idle) protoError(H_BADSTATE);
  }
}

/* K_MOVE: checks the step counts and puts one slice into the buffer.
 * The memory barrier makes sure the slice is complete before head moves. */
static void handleMove(const uint8_t *p) {
  uint8_t st = state;
  if (st != ST_ACTIVE && st != ST_MOVE) { protoError(H_BADSTATE); return; }
  int16_t v[3] = { rd16(p + 2), rd16(p + 4), rd16(p + 6) };
  uint8_t a[3], flags = 0, peak = 0;
  for (uint8_t i = 0; i < 3; i++) {
    if (v[i] < -STEP_LIMIT || v[i] > STEP_LIMIT) { protoError(H_STEPLIMIT); return; }
    a[i] = (uint8_t)(v[i] < 0 ? -v[i] : v[i]);
    if (v[i] < 0) flags |= (uint8_t)(0x20 << i);
    if (a[i] > peak) peak = a[i];
  }
  flags ^= DIR_FLIP;
  if (peak > UNDERRUN_STEPS) flags |= SF_FAST;
  uint8_t h = head;
  uint8_t n = (h + 1 >= QSIZE) ? 0 : h + 1;
  if (n == tail) { protoError(H_OVERFLOW); return; }
  q[h].ax = a[0]; q[h].ay = a[1]; q[h].az = a[2]; q[h].flags = flags;
  asm volatile("" ::: "memory");
  head = n;
}

/* Checks checksum and sequence counter, then runs the command. */
static void handlePacket(const uint8_t *p, uint8_t n) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < n - 2; i++) sum += p[i];
  sum += ((uint16_t)p[n - 2] << 8) | p[n - 1];
  if (sum != 0) { protoError(H_CHECKSUM); return; }
  if (state != ST_RESET && p[0] != K_RESET && p[1] != rxExpectN) {
    protoError(H_SEQ);
    return;
  }
  rxExpectN = p[1] + 1;
  uint32_t now = millis();
  /* New connection: first packet ever, or the PC was silent long enough to
   * count as disconnected (e.g. plugin restarted without resetting the card). */
  bool freshLink = !linkSeen || (uint32_t)(now - lastRx) > LINK_ALIVE_MS;
  lastRx = now;
  linkSeen = true;
  switch (p[0]) {
    case K_RESET:
      if (state != ST_RESET) enterReset(p[2] <= H_MAX ? p[2] : H_POWERON, false);
      if (freshLink || !resetAck) sendInfo();
      resetAck = true;
      nextStatusAt = now + STATUS_PHASE_MS;
      break;
    case K_READY:
    case K_ACTIVE:
      nextStatusAt = now + STATUS_PHASE_MS;
      handleStateCmd(p[0], p[3], p[4]);
      break;
    case K_MOVE:
      handleMove(p);
      break;
  }
}

/* ---------------- Public API ---------------- */

/* Pin setup, spindle PWM frequency, serial port and the 32 kHz step timer. */
void SafCNCClass::begin() {
  DDRD |= PD_STEP | PD_DIR;
  PORTD = (PORTD & ~(PD_STEP | PD_DIR)) | STEP_IDLE;
  driversEnable(false);
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_SPINDLE, OUTPUT);
  pinMode(PIN_SPDIR, OUTPUT);
  pinMode(PIN_FLOOD, OUTPUT);
  pinMode(PIN_MIST, OUTPUT);
  DDRB &= ~PB_LIMITS; PORTB |= PB_LIMITS;
  DDRC &= ~PC_INPUTS; PORTC |= PC_INPUTS;
#if SPINDLE_PWM_MODE == 1
  TCCR2A |= _BV(WGM21);
#elif SPINDLE_PWM_MODE == 2
  TCCR2A |= _BV(WGM21);
  TCCR2B = (TCCR2B & ~(_BV(CS22) | _BV(CS21) | _BV(CS20))) | _BV(CS21);
#endif
  Serial.begin(BAUD);
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A = TICK_CYCLES - 1;
  TCNT1 = 0;
  TIMSK1 = _BV(OCIE1A);
  interrupts();
  sendInfo();
  enterReset(H_POWERON, true);
  lastRx = millis();
}

/* Main loop service: builds packets from serial bytes, handles stops
 * from the interrupt, the 200 ms link watchdog, local E-Stop and status. */
void SafCNCClass::update() {
  static uint8_t buf[LEN_MOVE];
  static uint8_t len = 0, need = 0;
  static uint32_t lastByteUs = 0;
  if (tripCode) enterReset(tripCode, true);
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    uint32_t nowUs = micros();
    if (len > 0 && (nowUs - lastByteUs) > BYTE_GAP_US) {
      len = 0;
      protoError(H_BADLEN);
    }
    lastByteUs = nowUs;
    if (len == 0) {
      need = rxLength(b);
      if (need == 0) { protoError(H_BADCMD); continue; }
    }
    buf[len++] = b;
    if (len == need) { len = 0; handlePacket(buf, need); }
  }
  if (len > 0 && (uint32_t)(micros() - lastByteUs) > BYTE_GAP_US) {
    len = 0;
    protoError(H_BADLEN);
  }
  uint32_t now = millis();
  if (state != ST_RESET) {
    if (now - lastRx > CONN_TIMEOUT_MS) {
      /* Stop first, then report: K_INFO + RESET status, same order as power-up. */
      enterReset(H_TIMEOUT, false);
      sendInfo();
      sendStatus();
    } else if (LOCAL_ESTOP) {
      if (estopActive()) {
        if (estopSince == 0) estopSince = now | 1;
        else if (now - estopSince >= ESTOP_DEBOUNCE_MS) {
          estopSince = 0;
          enterReset(H_LOCALESTOP, true);
        }
      } else {
        estopSince = 0;
      }
    }
  }
  if ((int32_t)(now - nextStatusAt) >= 0 && Serial.availableForWrite() >= LEN_STATUS) {
    sendStatus();
  }
}
