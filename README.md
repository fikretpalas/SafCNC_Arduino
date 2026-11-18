# SafCNC Arduino Firmware

Firmware that turns an **Arduino Uno / Nano** (ATmega328P, 16 MHz) into a 3-axis USB motion
controller for **Mach3**. The PC sends the motion in 5 ms slices; the card buffers one second of them
and generates step and direction signals from a 32 kHz timer interrupt.

**[Türkçe açıklama için tıklayın](README.tr.md)**

| | |
|---|---|
| Mach3 plugin (English source) | [SafCNC_Mach3Plugin_en](https://github.com/fikretpalas/SafCNC_Mach3Plugin_en) |
| Mach3 plugin (Turkish source) | [SafCNC_Mach3Plugin_tr](https://github.com/fikretpalas/SafCNC_Mach3Plugin_tr) |

## Features

* 3 axes, up to 160 steps per 5 ms slice = 32 kHz per axis; steps are spread evenly by a DDA
* All three axes are pulsed together with direct port access
* Ring buffer of 200 slices (1 second of motion)
* Pinout compatible with GRBL 1.1 and the Arduino CNC Shield V3.51
* Limit and probe inputs are filtered and checked inside the step interrupt
* Local E-Stop input that stops the machine even if the PC does not
* 200 ms link watchdog, sequence counter and 16-bit checksum on every packet
* After a stop the card reports the exact slice and sub-tick, so the PC can restore the true position
* Reports its speed limit (`STEP_LIMIT`, 160 steps per slice = 32 kHz) to the plugin at power-up and on
  every new connection; the plugin plans the motion for that speed
* Spindle PWM pin, spindle direction, flood and mist outputs
* All settings are compile-time constants, checked with `static_assert`

## Pinout

| Pin | Function | Pin | Function | Pin | Function |
|---|---|---|---|---|---|
| D2 | X step | D5 | X direction | D9 | X limit |
| D3 | Y step | D6 | Y direction | D10 | Y limit |
| D4 | Z step | D7 | Z direction | D12 | Z limit |
| D8 | Stepper enable (active LOW) | D11 | Spindle PWM | D13 | Spindle direction |
| A0 | E-Stop | A1 | Feed Hold | A2 | Cycle Start |
| A3 | Flood coolant | A4 | Mist coolant | A5 | Probe |

Inputs are active LOW with the internal pull-ups enabled: wire each switch between the pin and GND.
On CNC Shield V3.00 clones the Z limit is on the *SpnEn* header and the spindle PWM on *Z+*.

## Install

1. Download this repository as a ZIP (*Code > Download ZIP*) and add it in the Arduino IDE with
   *Sketch > Include Library > Add .ZIP Library*.
2. Open *File > Examples > SafCNC > SafCNC_Basic*.
3. Select *Arduino Uno* or *Arduino Nano* and upload.
4. Install the Mach3 plugin from one of the repositories above.

The whole sketch is:

```cpp
#include <SafCNC.h>

void setup() { SafCNC.begin(); }
void loop()  { SafCNC.update(); }
```

Do not use `delay()` or `Serial` in the sketch: the firmware owns the serial port and `update()` must
run as often as possible.

## Settings

Edit `src/SafCNC_Config.h` in the library folder and upload again.

| Setting | Default | Meaning |
|---|---|---|
| `STEP_INVERT`, `DIR_INVERT` | `0x00` | Invert step / direction per axis (bit 0 = X, 1 = Y, 2 = Z) |
| `ENABLE_ACTIVE_LOW` | `1` | Stepper enable polarity (A4988, DRV8825, CNC Shield) |
| `INPUT_INVERT` | `0x00` | Invert individual inputs |
| `LOCAL_ESTOP` | `1` | The E-Stop input on A0 stops the card directly |
| `ESTOP_DEBOUNCE_MS` | `5` | E-Stop debounce time |
| `HOLD_TORQUE_ON_FAULT` | `1` | Keep the drivers enabled after a fault other than E-Stop |
| `LIMIT_FILTER`, `PROBE_FILTER` | `3`, `2` | Consecutive step ticks an input must be active |
| `SPINDLE_PWM_MODE` | `1` | Spindle PWM frequency: ~490 Hz, ~980 Hz or ~7.8 kHz |

The timing constants at the end of the file define the protocol; change them only together with the plugin.

## Connection time

With the Arduino bootloader the card needs about 7 seconds to connect, because the bootloader runs
every time the port is opened. Uploading with an ISP programmer (*Sketch > Upload Using Programmer*)
removes the bootloader and the card connects almost at once. *Tools > Burn Bootloader* restores normal
USB uploads.

## Protocol in brief

| Direction | Packet | Size | Content |
|---|---|---|---|
| PC -> card | state (`K_RESET`, `K_READY`, `K_ACTIVE`) | 8 bytes | command, counter, reset reason or spindle + outputs, checksum |
| PC -> card | move (`K_MOVE`) | 10 bytes | command, counter, signed X / Y / Z steps for the next 5 ms, checksum |
| card -> PC | status, every 50 ms | 8 bytes | state, counter, buffer level, spindle, outputs, inputs, checksum |
| card -> PC | info (`K_INFO`), at power-up and on a new connection | 8 bytes | command, counter, `STEP_LIMIT` (16 bit), `SUBTICKS` (16 bit), checksum |

Card states: `RESET` (stopped, waiting for the PC), `READY` (linked, no motion), `ACTIVE` (accepts
moves), `MOVE` (executing). Any protocol error, timeout, limit, probe or E-Stop event returns the card
to `RESET` and the reason code is sent to the PC.

The protocol is not specific to this board. The Mach3 plugin can also drive other cards that
implement it; an STM32 card can reach about 100 kHz (not tested), while the Arduino tops out at about 30 kHz.
Each card reports its own speed in the `K_INFO` packet (step rate = `STEP_LIMIT` x 200 Hz); the
protocol allows at most 500 steps per 5 ms slice (100 kHz).

> A CNC machine can injure people and destroy tools. Always fit a hardware E-Stop that cuts motor and
> spindle power. This software is provided without any warranty.

## Developer

**Fikret Palas** - <safcnc@tutamail.com> - <https://github.com/fikretpalas>

## License

[MIT](LICENSE)
