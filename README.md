# DRV2605L driver v2.2.1

Driver for the DRV2605L haptic breakout (EN tied to 3.3 V, IN/TRIG on J1 pin 2, ERM motor on TP1/TP2).
It is two files, `drv2605l.h` and `drv2605l.c`, in plain C that also compiles as C++.

- **Operations return a status:** 0 = OK; other statuses are listed below. Setup helpers and accessors have their declared return types.
- **It checks its own work:** chip ID at start-up, every setting read back after writing,
  timeouts on effects, over-temperature and overcurrent checks after every action and during
  real-time vibration.
- **It never guesses:** it tracks separately whether the motor output is confirmed stopped and
  whether the chip's settings are confirmed correct. If an I2C error leaves either uncertain, it
  rolls back to a known state or marks itself FAULT (error 12). `drv_stop()` clears FAULT only when
  the output is stopped *and* the settings are intact; a half-configured chip always needs `drv_recover()`.
- **Playback is asynchronous:** `drv_play()` starts an effect; call `drv_update()` in your
  loop to finish it. I2C port functions are synchronous and must have bounded timeouts.
  `drv_wait()`, `drv_self_test()` and `drv_recover()` deliberately block.
- **It works on any board:** the driver only talks to hardware through a few small functions you
  give it (the "port"). Copy-paste ports for Arduino and STM32 are below.

## Contents

1. [Error codes](#error-codes)
2. [Functions](#functions)
3. [Arduino: demo sketch](#arduino-demo-sketch)
4. [Arduino: debug console](#arduino-debug-console)
5. [STM32CubeIDE (HAL)](#stm32cubeide-hal)
6. [Host register map (FPGA link)](#host-register-map-fpga-link)
7. [Debugging](#debugging)
8. [Rules](#rules)

## Error codes

| N | Name | What to check first |
| --- | --- | --- |
| 0 | `DRV_OK` | Nothing, it worked |
| 1 | `DRV_ERR_NACK` | No answer at 0x5A: 3.3 V on J1 pin 1, GND, SDA/SCL swapped, pull-ups |
| 2 | `DRV_ERR_WRONG_ID` | Wrong chip, or another device at 0x5A |
| 3 | `DRV_ERR_VERIFY` | A setting didn't stick: long or noisy wires, try I2C at 100 kHz |
| 4 | `DRV_ERR_TIMEOUT` | Effect never finished: chip lost power, or `play_timeout_ms` too short |
| 5 | `DRV_ERR_OVER_TEMP` | Chip too hot: `od_clamp` too high or motor resistance too low |
| 6 | `DRV_ERR_OVERCURRENT` | Short across TP1/TP2 or a damaged motor |
| 7 | `DRV_ERR_BUSY` | Not a fault: something is still playing. Wait or call `drv_stop()` |
| 8 | `DRV_ERR_CAL_FAIL` | Calibration failed: motor not connected, or voltages far off |
| 9 | `DRV_ERR_DIAG_FAIL` | Motor check failed: not soldered, open wire, or not spinning |
| 10 | `DRV_ERR_BAD_ARG` | Value out of range (effect 1–123, strength 0–127, library 1–7) or unknown command |
| 11 | `DRV_ERR_NOT_INIT` | `drv_init()` hasn't succeeded yet |
| 12 | `DRV_ERR_NEEDS_RECOVER` | An I2C error left the chip state unknown: call `drv_recover()` (or `drv_stop()`) |
| 13 | `DRV_ERR_CANCELLED` | Sent to your callback when `drv_stop()` or `drv_recover()` cancels an action |

**Faults** (1–6, 8, 9, 12) are latched: `drv_last_error()` keeps the most recent fault until
`drv_clear_error()`, `drv_init()` or `drv_recover()` succeeds. **Rejections** (7, 10, 11, 13) are only
returned, never latched. If `port.log` is set, it receives every error with the register involved
(`FF` = an action ended badly, `FE` = a request was rejected). More detail is in `drv2605l.h`.

## Functions

| Call | Waits? | What it does |
| --- | --- | --- |
| `drv_default_config(&cfg)` | No | Fills safe defaults: ERM, open loop, `od_clamp` 0x8B (~3.0 V), library 1, verification on |
| `drv_init(&drv, &port, &cfg)` | ~2 ms | Finds the chip, checks its ID, writes and verifies settings, restores saved calibration |
| `drv_play(&drv, effect)` | No | Plays effect 1–123 (names like `DRV_EFFECT_DOUBLE_CLICK_100` are in the header) |
| `drv_play_sequence(&drv, steps, n)` | No | Up to 8 steps; a step of `0x80 \| n` is a pause of n × 10 ms |
| `drv_set_strength(&drv, level)` | No | Live vibration strength 0–127; 0 turns it off |
| `drv_stop(&drv)` | No | Tries every way to stop the motor, then confirms it. Clears FAULT only if the settings are intact |
| `drv_update(&drv)` | No | Call in your loop. 7 while busy, the result once when an action ends, 0 when idle, 12 every call while in FAULT |
| `drv_wait(&drv)` | Yes | Calls `drv_update()` until the current action ends (returns 12 at once in FAULT) |
| `drv_start_calibration(&drv)` | No | Auto-calibration (~1.2 s, motor buzzes); then `drv_get_calibration()` |
| `drv_self_test(&drv)` | ~1 s | Chip ID, register read-back test, motor connection check |
| `drv_standby(&drv, true/false)` | No | Sleep or wake the chip (the only power saving, since EN is tied high) |
| `drv_recover(&drv)` | ~5–15 ms | Frees a stuck bus, resets the chip (confirmed), re-applies settings and calibration |
| `drv_last_error` / `drv_clear_error` | No | Read / clear the latched fault |
| `drv_set_callback(&drv, fn, user)` | No | `fn(user, result)` runs exactly once when each action ends; set it after `drv_init()`. 13 (cancelled) only if the stop was confirmed |
| `drv_reg_read` / `drv_reg_write` | No | Raw register access for debugging |
| `drv_host_write` / `drv_host_read` | Varies | Host register map for the FPGA link (see below) |

To skip calibration on later boots, save the three values that `drv_get_calibration()` returns and
put them in `cfg.cal` (with `cfg.cal.valid = true`) before `drv_init()`. Without saved values,
`drv_init()` writes the datasheet defaults, so the chip never keeps leftovers.

Calibration results are restored only after output is confirmed stopped. A failed reset
transfer invalidates configuration even if the bus reports an error, because the chip may
have accepted the reset. If stopping cannot be confirmed during calibration cleanup, no
calibration registers are written and successful recovery is required before playback.

Any calibration that doesn't succeed (failure, timeout, I2C error or `drv_stop()`) puts the previous
values back on the chip. `drv_load_calibration()` stops real-time vibration first and does the same if
it fails. If the previous values can't be put back, the driver goes to FAULT until `drv_recover()`.

`drv_init()` checks the whole config: the library must match the motor (ERM: 1–5 or 7, LRA: 6),
`brake_factor` 0–7, `loop_gain` 0–3, `drive_time` 0–31, `lra_ol_period` 0–127 (× 98.46 µs), with 0xFF
meaning "keep the chip default" for the last two. Anything else returns error 10.

## Arduino: demo sketch

Make a sketch folder containing `demo.ino` (below), `drv2605l.h` and `drv2605l.c`.
It runs the self-test, then plays effects 1 to 123, one every 1.5 s.
Serial Monitor at 115200 shows only `error N`, and `error N reg RR` (the register that failed).

```cpp
// demo.ino - DRV2605L breakout demo. Put drv2605l.h and drv2605l.c in the same folder.
// Wiring (J1): 1 = 3.3V, 2 = IN/TRIG -> TRIG_PIN, 3 = SDA, 4 = SCL, 5 = GND. ERM on TP1/TP2.
// Serial (115200) prints only "error N" / "error N reg RR". Meanings: README or drv2605l.h.
#include <Wire.h>
#include "drv2605l.h"

const int TRIG_PIN = 7;          // pin wired to IN/TRIG
const uint32_t I2C_HZ = 400000;  // use 100000 if you see error 3 on long wires

// ---------- Arduino port: connects the driver to Wire (same block in every sketch) ----------
static int portWrite(void *, uint8_t addr, const uint8_t *data, uint16_t len) {
  Wire.beginTransmission(addr);
  Wire.write(data, len);
  return Wire.endTransmission();  // 0 = chip ACKed
}
static int portRead(void *, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;  // false = repeated start
  if (Wire.requestFrom(addr, (uint8_t)len) != len) return -1;
  for (uint16_t i = 0; i < len; i++) data[i] = Wire.read();
  return 0;
}
static uint32_t portMillis(void *) { return millis(); }
static void portDelay(void *, uint32_t ms) { delay(ms); }
static void portTrig(void *, bool high) { digitalWrite(TRIG_PIN, high ? HIGH : LOW); }
static void lineOD(int pin, bool high) {  // open-drain: release = pull-up raises it
  if (high) pinMode(pin, INPUT);
  else { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
}
static int portRecover(void *) {  // frees a chip holding SDA low (9 clocks + STOP)
  Wire.end();
  lineOD(SDA, true);
  lineOD(SCL, true);
  delayMicroseconds(5);
  bool ok = digitalRead(SCL) == HIGH;  // SCL stuck low = short or missing pull-up: can't fix
  for (int i = 0; ok && i < 9 && digitalRead(SDA) == LOW; i++) {
    lineOD(SCL, false); delayMicroseconds(5);
    lineOD(SCL, true);  delayMicroseconds(5);
    ok = digitalRead(SCL) == HIGH;     // released SCL must actually rise
  }
  if (ok) {                            // STOP: SDA rises while SCL is high
    lineOD(SDA, false); delayMicroseconds(5);
    lineOD(SDA, true);  delayMicroseconds(5);
    ok = digitalRead(SDA) == HIGH && digitalRead(SCL) == HIGH;
  }
  Wire.begin();                        // check the lines before this: Wire takes the pins back
  Wire.setClock(I2C_HZ);               // begin() resets the speed
  return ok ? 0 : -1;
}
static void portLog(void *, drv_status_t code, uint8_t reg) {  // every failure + register
  Serial.print("error "); Serial.print((int)code);
  Serial.print(" reg "); Serial.println(reg, HEX);
}
static void makePort(drv_port_t *p) {
  memset(p, 0, sizeof(*p));
  p->i2c_write = portWrite;
  p->i2c_read = portRead;
  p->millis = portMillis;
  p->delay_ms = portDelay;
  p->set_trig = portTrig;       // nullptr if IN/TRIG is jumpered to GND
  p->bus_recover = portRecover; // nullptr if SDA/SCL names don't compile on your board
  p->log = portLog;             // nullptr for quiet operation
}
// ---------- end of port ----------

drv_port_t port;
drv_t drv;
uint8_t effect = DRV_EFFECT_STRONG_CLICK_100;
uint32_t lastStart = 0;

void report(drv_status_t s) {
  if (s == DRV_OK || s == DRV_ERR_BUSY) return;  // 7 just means "still playing"
  Serial.print("error "); Serial.println((int)s);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  pinMode(TRIG_PIN, OUTPUT);
  Wire.begin();
  Wire.setClock(I2C_HZ);

  makePort(&port);
  drv_config_t cfg;
  drv_default_config(&cfg);  // ERM, open loop, OD_CLAMP 0x8B (~3.0 V), library 1
  // cfg.od_clamp = 0x5D;    // ~2.0 V if your motor is rated lower

  drv_status_t s = drv_init(&drv, &port, &cfg);
  if (s) { report(s); while (true) delay(1000); }  // no chip: stop here
  report(drv_self_test(&drv));  // ID + read-back + motor check (motor buzzes briefly)
}

void loop() {
  drv_status_t u = drv_update(&drv);  // call often: finishes effects, reports faults
  bool due = millis() - lastStart >= 1500;

  if (u == DRV_ERR_NEEDS_RECOVER) {  // FAULT: chip state unknown, nothing will play
    if (due) {                       // retry recovery at most every 1.5 s
      lastStart = millis();
      report(u);
      report(drv_recover(&drv));     // unstick bus + reset + re-init
    }
    return;
  }
  report(u);

  if (!drv_is_busy(&drv) && due) {  // next effect every 1.5 s
    lastStart = millis();
    drv_status_t s = drv_play(&drv, effect);
    report(s);
    if (s == DRV_ERR_NACK) report(drv_recover(&drv));  // bus problem: recover right away
    if (s == DRV_OK) effect = effect >= 123 ? 1 : effect + 1;
  }
}
```

## Arduino: debug console

Same folder layout, with `console.ino` instead of the demo. Set the Serial Monitor to **115200** and
line ending **Newline**, then type commands:

| Command | Does | Reply |
| --- | --- | --- |
| `e 47` | Play effect 47 | `ok` |
| `r 100` / `r 0` | Live strength 100 / off | `ok` |
| `cal` | Calibrate (motor buzzes ~1 s) | `cal 0D 75 01` (save these) |
| `test` | Self-test | `ok` |
| `rec` | Recover (reset chip + re-init); the way out of `error 12` | `ok` |
| `s` / `w` | Sleep / wake | `ok` |
| `rd 17` | Read register 0x17 | `0x8B` |
| `wr 17 5D` | Write 0x5D to register 0x17 | `ok` |
| `help` | List commands | command list |

Registers are 0x00–0x22. Writing bit 0 of 0x1E is refused (`error 10`): it permanently burns the
chip's one-time memory. A line over 63 characters or containing a NUL byte also gets `error 10`. `rd`/`wr` bypass the driver, so the driver won't know about changes you make
to MODE (0x01) this way.

```cpp
// console.ino - type commands in the Serial Monitor (115200, line ending: Newline).
// Replies: "ok", "error N", or a value. Put drv2605l.h and drv2605l.c in the same folder.
#include <Wire.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "drv2605l.h"

const int TRIG_PIN = 7;

// ---------- Arduino port (same block as the demo, without the optional hooks) ----------
static int portWrite(void *, uint8_t addr, const uint8_t *data, uint16_t len) {
  Wire.beginTransmission(addr);
  Wire.write(data, len);
  return Wire.endTransmission();
}
static int portRead(void *, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(addr, (uint8_t)len) != len) return -1;
  for (uint16_t i = 0; i < len; i++) data[i] = Wire.read();
  return 0;
}
static uint32_t portMillis(void *) { return millis(); }
static void portDelay(void *, uint32_t ms) { delay(ms); }
static void portTrig(void *, bool high) { digitalWrite(TRIG_PIN, high ? HIGH : LOW); }
static void makePort(drv_port_t *p) {
  memset(p, 0, sizeof(*p));
  p->i2c_write = portWrite;
  p->i2c_read = portRead;
  p->millis = portMillis;
  p->delay_ms = portDelay;
  p->set_trig = portTrig;
}
// ---------- end of port ----------

drv_port_t port;
drv_t drv;
char line[64];
uint8_t lineLen = 0;
bool dropLine = false;

static void hex2(uint8_t v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}
static void reply(drv_status_t s) {  // 0 -> "ok", else "error N"
  if (s == DRV_OK) { Serial.println("ok"); return; }
  Serial.print("error "); Serial.println((int)s);
}
// Reads one number (base 10 or 16) from *p. Rejects signs, junk and out-of-range values.
static bool num(char *&p, int base, long lo, long hi, long &out) {
  while (*p && isspace((unsigned char)*p)) p++;
  if (base == 16 ? !isxdigit((unsigned char)*p) : !isdigit((unsigned char)*p)) return false;
  char *end;
  long v = strtol(p, &end, base);
  if (end == p || v < lo || v > hi || (*end && !isspace((unsigned char)*end))) return false;
  p = end;
  out = v;
  return true;
}
static bool done(char *p) {  // nothing left on the line
  while (*p && isspace((unsigned char)*p)) p++;
  return *p == 0;
}

static void run(char *p) {
  while (*p && isspace((unsigned char)*p)) p++;
  if (!*p) return;
  char *cmd = p;
  while (*p && !isspace((unsigned char)*p)) p++;
  if (*p) *p++ = 0;
  long a = 0, b = 0;

  if (!strcmp(cmd, "help") && done(p)) {
    Serial.println("e <1-123> | r <0-127> | cal | test | rec | s | w | rd <hex> | wr <hex> <hex>");
  } else if (!strcmp(cmd, "e") && num(p, 10, 1, 123, a) && done(p)) {
    reply(drv_play(&drv, (uint8_t)a));  // finishes in loop(); a fault there prints "error N"
  } else if (!strcmp(cmd, "r") && num(p, 10, 0, 127, a) && done(p)) {
    reply(drv_set_strength(&drv, (uint8_t)a));
  } else if (!strcmp(cmd, "cal") && done(p)) {
    drv_status_t s = drv_start_calibration(&drv);
    if (!s) s = drv_wait(&drv);  // ~1.2 s, motor buzzes
    drv_cal_t c;
    if (s || (s = drv_get_calibration(&drv, &c))) { reply(s); return; }
    Serial.print("cal "); hex2(c.comp); Serial.print(' ');
    hex2(c.bemf); Serial.print(' '); hex2(c.bemf_gain); Serial.println();
  } else if (!strcmp(cmd, "test") && done(p)) {
    reply(drv_self_test(&drv));
  } else if (!strcmp(cmd, "rec") && done(p)) {
    reply(drv_recover(&drv));  // the way out of error 12: reset chip + re-init
  } else if (!strcmp(cmd, "s") && done(p)) {
    drv_status_t stopped = drv_stop(&drv);
    reply(stopped ? stopped : drv_standby(&drv, true));
  } else if (!strcmp(cmd, "w") && done(p)) {
    reply(drv_standby(&drv, false));
  } else if (!strcmp(cmd, "rd") && num(p, 16, 0, 0x22, a) && done(p)) {
    uint8_t v;
    drv_status_t s = drv_reg_read(&drv, (uint8_t)a, &v);
    if (s) { reply(s); return; }
    Serial.print("0x"); hex2(v); Serial.println();
  } else if (!strcmp(cmd, "wr") && num(p, 16, 0, 0x22, a) && num(p, 16, 0, 0xFF, b) && done(p)) {
    if (a == 0x1E && (b & 1)) { reply(DRV_ERR_BAD_ARG); return; }  // OTP bit: one-time only
    reply(drv_reg_write(&drv, (uint8_t)a, (uint8_t)b));
  } else {
    reply(DRV_ERR_BAD_ARG);  // unknown command or bad number -> error 10
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  pinMode(TRIG_PIN, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  makePort(&port);
  drv_config_t cfg;
  drv_default_config(&cfg);
  reply(drv_init(&drv, &port, &cfg));  // "ok" = chip found and configured
}

drv_status_t lastUpdate = DRV_OK;

void loop() {
  drv_status_t s = drv_update(&drv);  // how a playing effect ended; 12 = needs "recover"
  if (s != DRV_OK && s != DRV_ERR_BUSY && s != lastUpdate) reply(s);  // print once, not every loop
  lastUpdate = s;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      bool ran = !dropLine;
      if (ran) { line[lineLen] = 0; run(line); }
      lineLen = 0;
      dropLine = false;
      if (ran) return;  // one command per loop(): drv_update() runs between pasted lines
    } else if (dropLine) {
      continue;
    } else if (c != 0 && lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;  // '\r' is treated as a space, so CRLF works too
    } else {
      reply(DRV_ERR_BAD_ARG);  // line too long or NUL byte: "error 10", rest of line ignored
      dropLine = true;
    }
  }
}
```

## STM32CubeIDE (HAL)

1. In CubeMX, enable **I2C1** (Standard 100 kHz or Fast 400 kHz) with internal pull-ups off (the board
   has 4.7 kΩ), set **PA7** as a GPIO output for IN/TRIG, and keep **PA5** as the LED output (LD2 on
   Nucleo boards). Change the pins in the code if yours differ.
2. Copy `drv2605l.h` into `Core/Inc` and `drv2605l.c` into `Core/Src`.
3. Paste each block below into `main.c` between the matching `USER CODE BEGIN` / `END` comments.

There's no serial output: on any error the LED blinks N times, pauses, and repeats.

```c
/* USER CODE BEGIN Includes */
#include <string.h>
#include "drv2605l.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* ---------- STM32 HAL port: connects the driver to hi2c1 ---------- */
static int portWrite(void *c, uint8_t addr, const uint8_t *data, uint16_t len) {
  (void)c; /* HAL wants the 8-bit address: 0x5A << 1 = 0xB4 */
  return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr << 1), (uint8_t *)data, len, 10) == HAL_OK ? 0 : -1;
}
static int portRead(void *c, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len) {
  (void)c; /* register write + repeated start + read in one call */
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, data, len, 10) == HAL_OK ? 0 : -1;
}
static uint32_t portMillis(void *c) { (void)c; return HAL_GetTick(); }
static void portDelay(void *c, uint32_t ms) { (void)c; HAL_Delay(ms); }
static void portTrig(void *c, bool high) { /* IN/TRIG on PA7: change to your pin */
  (void)c;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static drv_port_t port;
static drv_t drv;

/* No serial port needed: blinks the LED N times for "error N", forever. */
static void haltWithError(drv_status_t s) {
  for (;;) {
    for (int i = 0; i < (int)s; i++) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); HAL_Delay(200);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); HAL_Delay(200);
    }
    HAL_Delay(1500);
  }
}
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
  memset(&port, 0, sizeof(port));
  port.i2c_write = portWrite;
  port.i2c_read = portRead;
  port.millis = portMillis;
  port.delay_ms = portDelay;
  port.set_trig = portTrig;   /* NULL if IN/TRIG is jumpered to GND */
  drv_config_t cfg;
  drv_default_config(&cfg);   /* ERM, open loop, ~3.0 V, library 1 */
  drv_status_t st = drv_init(&drv, &port, &cfg);
  if (st != DRV_OK) haltWithError(st);
  st = drv_self_test(&drv);   /* ID + read-back + motor check */
  if (st != DRV_OK) haltWithError(st);
  uint8_t effect = DRV_EFFECT_STRONG_CLICK_100;
  uint32_t lastStart = 0;
/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
  while (1)
  {
    drv_status_t s = drv_update(&drv);         /* finishes effects, reports faults */
    if (s == DRV_ERR_NACK || s == DRV_ERR_NEEDS_RECOVER)
      s = drv_recover(&drv);                      /* bus lost / state unknown: reset + re-init */
    if (s != DRV_OK && s != DRV_ERR_BUSY) haltWithError(s);

    if (!drv_is_busy(&drv) && HAL_GetTick() - lastStart >= 1500) {
      lastStart = HAL_GetTick();
      s = drv_play(&drv, effect);
      if (s == DRV_ERR_NACK || s == DRV_ERR_NEEDS_RECOVER)
        s = drv_recover(&drv);                      /* chip state unknown: reset + re-init */
      if (s != DRV_OK) haltWithError(s);
      effect = effect >= 123 ? 1 : effect + 1;
    }
    /* USER CODE END WHILE */
```

## Host register map (FPGA link)

For the UART, SPI or I2C link from the FPGA, the code receiving bytes only has to call
`drv_host_write(&drv, addr, value)` and `drv_host_read(&drv, addr, &value)`.

| Addr | Name | Access | Meaning |
| --- | --- | --- | --- |
| 0x00 | STATUS | R | bit 0 busy · bit 1 fault latched · bit 2 calibrated · bit 3 chip found · bit 4 needs recovery |
| 0x01 | COMMAND | W | 1 play · 2 stop · 3 calibrate · 4 sleep · 5 wake · 6 play sequence · 7 self-test · 8 recover · 9 clear fault |
| 0x02 | EFFECT | R/W | Effect 1–123 used by command 1 |
| 0x03 | RTP_LEVEL | R/W | 0 off, 1–127 live strength |
| 0x04 | LIBRARY | R/W | 1–7 |
| 0x05 | FAULT_CODE | R | Latched fault number (table above), 0 = none |
| 0x06 | LAST_RESULT | R | Result of the host's last write, including rejections like 7 (busy) or 10 (bad value) |
| 0x10–0x17 | SEQ1–SEQ8 | R/W | Playlist for command 6; 0 ends it |
| 0x20 | RAW_ADDR | R/W | DRV2605L register 0x00–0x22 |
| 0x21 | RAW_DATA | R/W | Reads or writes that register. Writes to MODE, RTP and GO are refused (use COMMAND), as is the OTP bit |

If the FPGA side seems to do nothing, read 0x06 (LAST_RESULT): e.g. 10 = bad address or value,
7 = busy. Real chip or bus problems show in 0x05 (FAULT_CODE) until command 9 clears them.

## Debugging

- **Turn on the log hook** (`port.log`, already set in the demo). It prints `error N reg RR`, and
  the register tells you which step failed. `reg FF` means an effect, calibration or test ended badly.
- **A healthy chip after `drv_init()`** reads: STATUS (0x00) `0xE0`, MODE (0x01) `0x00`,
  LIBRARY (0x03) `0x01`, CONTROL3 (0x1D) with bit 5 set. Check these with the console's `rd`.
- **No vibration but no errors:** the chip is asleep (MODE `0x40`), `od_clamp` is too low, the motor
  isn't soldered across TP1/TP2, or an effect number is 0 (0 means "end of list").
- **Error 1 or 12 after it was working:** the I2C bus may be stuck after a reset mid-transfer, or the
  chip's state couldn't be confirmed. `drv_recover()` fixes both (the demo retries it every 1.5 s;
  in the console type `rec`). If it keeps failing, check wiring and power.
- **Reading STATUS (0x00) yourself clears two fault flags** (DIAG_RESULT, OVER_TEMP), per the datasheet,
  so a manual `rd 0` can hide a fault from the driver. Use it for the ID check only.
- **Error 3 on long wires:** lower I2C to 100 kHz.
- **Error 7 everywhere:** your loop isn't calling `drv_update()`, so the driver never notices effects finishing.

## Rules

- Call the driver from one place only (your main loop or one RTOS task), never from an interrupt.
- Call `drv_init()` before anything else. It sets up the whole `drv_t`, so an uninitialised local
  is fine for it; other calls on a `drv_t` that never went through `drv_init()` must at least be
  zeroed (`drv_t drv = {0};` or a global).
- The driver uses only I2C address 0x5A and the IN/TRIG pin. It prints nothing and allocates no memory.
- Settings: guide section 5 explains how to turn your motor's rated voltage into `od_clamp`
  (volts ÷ 0.02159). The default 0x8B suits a typical 3 V ERM.


## Regression tests (v2.2.1)

The `tests/test_driver.c` simulator injects failed transfers, transfers accepted by the
chip but reported as failed, ignored writes, calibration timeouts, recovery failures,
thermal/overcurrent status, and timer wraparound. Run from this directory:

```sh
sh tests/run.sh
```

The simulator checks software behavior; it does not model actuator mechanics or prove
electrical timing. Compile the application for your selected board and test on hardware
before relying on calibration or voltage settings. Recompile all application code with
the supplied header; the driver structure now tracks pending calibration cleanup.

Raw register writes deliberately bypass configuration tracking. After changing managed
settings through raw access, call `drv_recover()` to restore the configured state before
using normal playback APIs.
