/*
 * drv2605l.h - portable driver for the TI DRV2605L haptic driver (datasheet SLOS854D)
 *
 * Works in C and C++. The driver never touches hardware directly: you give it a
 * drv_port_t with I2C and timing functions (copy-paste ports for Arduino and STM32 HAL
 * are in README.md).
 *
 * Rules:
 *  - Not thread-safe and not for interrupts: call from one task / the main loop.
 *  - Calls marked NON-BLOCKING start an action and return at once; call
 *    drv_update() regularly (or drv_wait()) to finish it.
 *  - Every call returns a drv_status_t. 0 = OK; anything else is "error N".
 *  - drv_init() fully initialises the drv_t (any previous contents are ignored).
 *    Set the callback with drv_set_callback() AFTER drv_init().
 *
 * Two kinds of error:
 *  - FAULTS (1-6, 8, 9, 12): something went wrong on the chip or bus. They are latched in
 *    drv_last_error() until drv_clear_error(), drv_init() or drv_recover() succeeds.
 *  - REJECTIONS (7, 10, 11, 13): the request was refused or cancelled; nothing is latched.
 *  port.log (if set) receives every non-OK result except drv_update()'s normal "busy".
 *
 * Driver state you can trust: the driver tracks two facts separately.
 *  - Output: is the chip confirmed stopped (GO = 0, RTP = 0, internal-trigger mode)?
 *  - Configuration: do the chip's settings and calibration confirmed-match cfg?
 * If either can't be confirmed after an I2C error, the driver enters DRV_STATE_FAULT and
 * refuses work (error 12). drv_stop() leaves FAULT only when the output is confirmed stopped
 * AND the configuration is still valid; a half-configured chip always needs drv_recover().
 * drv_update() and drv_wait() keep returning 12 while in FAULT (11 before drv_init()).
 */
#ifndef DRV2605L_H
#define DRV2605L_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DRV2605L_VERSION "2.2.1"
#define DRV2605L_I2C_ADDR 0x5A /* 7-bit, fixed */

/* ---- Status codes ("error N") ----
 * DEBUGGING: each code says what failed and what to check first.
 * port.log gets (code, register): the register shows which step failed.
 * Special register values: 0xFF = an action ended badly (see drv_update()),
 * 0xFE = your request was rejected (bad argument, busy, not initialised, ...).
 */
typedef enum {
  DRV_OK = 0,
  /* 1: nobody answered at 0x5A. Check 3.3 V on J1 pin 1, GND, SDA/SCL not swapped,
   *    pull-ups present, I2C peripheral enabled. Run an I2C scanner: expect one device at 0x5A. */
  DRV_ERR_NACK = 1,
  /* 2: something answered but STATUS[7:5] != 7. Wrong chip (DRV2604L, DRV2605 without L)
   *    or another device sitting at 0x5A on the bus. */
  DRV_ERR_WRONG_ID = 2,
  /* 3: a write was ACKed but read back different. Noisy/long wires, I2C too fast (try 100 kHz),
   *    weak pull-ups, or a brown-out reset mid-sequence. port.log's reg says which register. */
  DRV_ERR_VERIFY = 3,
  /* 4: GO never cleared, or a reset never completed. Effect longer than play_timeout_ms,
   *    chip lost power, or MODE got changed behind the driver's back. */
  DRV_ERR_TIMEOUT = 4,
  /* 5: chip got too hot and shut down. Motor resistance too low or OD_CLAMP too high. */
  DRV_ERR_OVER_TEMP = 5,
  /* 6: overcurrent on OUT+/OUT-. Short across TP1/TP2, solder bridge, or a damaged motor. */
  DRV_ERR_OVERCURRENT = 6,
  /* 7: not a fault. Something is still playing/calibrating; wait for drv_update() or drv_stop(). */
  DRV_ERR_BUSY = 7,
  /* 8: auto-calibration failed. Motor missing on TP1/TP2, wrong ERM/LRA setting, or
   *    rated_voltage / od_clamp far from the motor's real rating. Previous calibration kept. */
  DRV_ERR_CAL_FAIL = 8,
  /* 9: diagnostics failed. Motor not soldered, open wire, or motor not spinning. */
  DRV_ERR_DIAG_FAIL = 9,
  /* 10: argument out of range (effect not 1-123, level > 127, library wrong for the motor type,
   *     unknown host register/command, NULL pointer, or an attempt to set the OTP bit). */
  DRV_ERR_BAD_ARG = 10,
  /* 11: call drv_init() first (and check its return value). */
  DRV_ERR_NOT_INIT = 11,
  /* 12: the driver lost track of the chip after an I2C error (state FAULT). Call drv_recover()
   *     (or drv_stop()) to get back to a known state. Check wiring if it keeps happening. */
  DRV_ERR_NEEDS_RECOVER = 12,
  /* 13: an action was cancelled by drv_stop() or drv_recover() (sent to the callback). */
  DRV_ERR_CANCELLED = 13
} drv_status_t;

/* ---- Platform layer: fill this in (README has Arduino and STM32 versions) ---- */
typedef struct {
  /* Write len bytes to the 7-bit address; data[0] is the register. Return 0 on ACK. */
  int (*i2c_write)(void *ctx, uint8_t addr7, const uint8_t *data, uint16_t len);
  /* Write reg, repeated start, read len bytes. Return 0 on success. */
  int (*i2c_read)(void *ctx, uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);
  uint32_t (*millis)(void *ctx);            /* milliseconds, free-running */
  void (*delay_ms)(void *ctx, uint32_t ms); /* blocking wait */
  void (*set_trig)(void *ctx, bool high);   /* optional: drives IN/TRIG; NULL if tied to GND */
  int (*bus_recover)(void *ctx);            /* optional: free a stuck bus; return 0 if freed */
  void (*log)(void *ctx, drv_status_t code, uint8_t reg); /* optional: every error */
  void *ctx;                                /* passed back to every function above */
} drv_port_t;

/* ---- Configuration ---- */
typedef enum { DRV_ERM = 0, DRV_LRA = 1 } drv_actuator_t;

typedef struct {
  uint8_t comp;      /* A_CAL_COMP  (0x18) */
  uint8_t bemf;      /* A_CAL_BEMF  (0x19) */
  uint8_t bemf_gain; /* BEMF_GAIN   (0x1A bits 1:0) */
  bool valid;        /* true = apply these at init */
} drv_cal_t;

typedef struct {
  drv_actuator_t actuator;  /* DRV_ERM or DRV_LRA */
  uint8_t rated_voltage;    /* 0x16, closed-loop level (guide section 5) */
  uint8_t od_clamp;         /* 0x17, ERM open loop: full-strength level (V / 0.02159) */
  uint8_t library;          /* ERM: 1-5 or 7 (libraries A-F). LRA: 6 */
  bool open_loop;           /* ERM: true recommended. LRA: false recommended */
  uint8_t brake_factor;     /* 0x1A bits 6:4, 0-7 (default 2). Used at init AND calibration */
  uint8_t loop_gain;        /* 0x1A bits 3:2, 0-3 (default 2). Used at init AND calibration */
  uint8_t drive_time;       /* LRA only: 0-31, or 0xFF = keep chip default */
  uint8_t lra_ol_period;    /* LRA open loop only: 0-127 (x 98.46 us), or 0xFF = keep default */
  bool verify_writes;       /* read back every config write (slower, safer) */
  uint32_t play_timeout_ms; /* max time one effect/sequence may run */
  drv_cal_t cal;            /* saved calibration, applied at init if valid */
} drv_config_t;

/* ---- Driver state ---- */
typedef enum {
  DRV_STATE_UNINIT = 0,
  DRV_STATE_IDLE,
  DRV_STATE_PLAYING,
  DRV_STATE_RTP,
  DRV_STATE_CALIBRATING,
  DRV_STATE_DIAG,
  DRV_STATE_STANDBY,
  DRV_STATE_FAULT /* chip state unknown after an I2C error: drv_stop() or drv_recover() */
} drv_state_t;

/* Called exactly once for every action that started (play, sequence, calibration,
 * diagnostics): with its result, or the error that ended it. DRV_ERR_CANCELLED means
 * drv_stop()/drv_recover() ended it AND the chip is confirmed stopped; if stopping could
 * not be confirmed you get the real error (e.g. 12) instead. */
typedef void (*drv_done_cb)(void *user, drv_status_t result);

typedef struct {
  drv_port_t port;
  drv_config_t cfg;
  drv_state_t state;
  drv_status_t last_error;  /* latched fault, see top of file */
  bool cal_pending;        /* calibration may have changed hardware results; restore before reuse */
  bool config_ok;           /* chip settings + calibration confirmed to match cfg */
  uint32_t op_start_ms;
  uint32_t op_timeout_ms;
  uint32_t last_poll_ms;    /* RTP fault polling */
  drv_done_cb on_done;
  void *cb_user;
  /* host register map shadow */
  uint8_t host_effect, host_rtp, host_raw_addr, host_seq[8];
  drv_status_t host_last_result;
} drv_t;

/* ---- Setup ---- */
void drv_default_config(drv_config_t *cfg); /* ERM, ~3.0 V open loop, library 1, verify on */
drv_status_t drv_init(drv_t *d, const drv_port_t *port, const drv_config_t *cfg); /* ~2 ms */
void drv_set_callback(drv_t *d, drv_done_cb cb, void *user); /* after drv_init() */

/* ---- Playback (NON-BLOCKING) ---- */
drv_status_t drv_play(drv_t *d, uint8_t effect); /* effect 1-123 */
/* up to 8 steps: effect 1-123, or 0x80 | n = pause of n x 10 ms */
drv_status_t drv_play_sequence(drv_t *d, const uint8_t *steps, uint8_t n);
drv_status_t drv_set_strength(drv_t *d, uint8_t level); /* real-time 0-127; 0 = off */
drv_status_t drv_stop(drv_t *d); /* also the way out of DRV_STATE_FAULT */
drv_status_t drv_select_library(drv_t *d, uint8_t lib);

/* ---- Progress ---- */
/* Returns DRV_ERR_BUSY while an action runs, its result once when it ends, DRV_OK when idle,
 * DRV_ERR_NEEDS_RECOVER (12) every call while in FAULT, DRV_ERR_NOT_INIT (11) before init.
 * In real-time mode it also checks the fault flags every 50 ms and stops on a fault. */
drv_status_t drv_update(drv_t *d);
drv_status_t drv_wait(drv_t *d); /* BLOCKING: drv_update() until not busy */
bool drv_is_busy(const drv_t *d);
drv_state_t drv_state(const drv_t *d);
drv_status_t drv_last_error(const drv_t *d);
void drv_clear_error(drv_t *d);

/* ---- Calibration, health, recovery ---- */
drv_status_t drv_start_calibration(drv_t *d); /* NON-BLOCKING, ~1.2 s, motor buzzes */
drv_status_t drv_get_calibration(const drv_t *d, drv_cal_t *out);
drv_status_t drv_load_calibration(drv_t *d, const drv_cal_t *cal);
drv_status_t drv_start_diagnostics(drv_t *d); /* NON-BLOCKING, motor buzzes */
drv_status_t drv_self_test(drv_t *d);         /* BLOCKING: ID, read-back, diagnostics */
drv_status_t drv_standby(drv_t *d, bool on);  /* EN is tied high: this is the sleep control */
drv_status_t drv_recover(drv_t *d);           /* bus recovery + confirmed chip reset + re-init */

/* ---- Raw register access (debugging; the driver doesn't track changes you make) ---- */
drv_status_t drv_reg_read(drv_t *d, uint8_t reg, uint8_t *value);
drv_status_t drv_reg_write(drv_t *d, uint8_t reg, uint8_t value);

/* ---- Host register map (same on UART / SPI / I2C, see README) ---- */
enum {
  DRV_HOST_STATUS = 0x00,      /* R: b0 BUSY, b1 FAULT, b2 CALIBRATED, b3 DRV_FOUND, b4 NEEDS_RECOVER */
  DRV_HOST_COMMAND = 0x01,     /* W: see DRV_CMD_* */
  DRV_HOST_EFFECT = 0x02,      /* RW: effect 1-123 for DRV_CMD_PLAY */
  DRV_HOST_RTP_LEVEL = 0x03,   /* RW: 0 off, 1-127 live strength */
  DRV_HOST_LIBRARY = 0x04,     /* RW: library (must match the motor type) */
  DRV_HOST_FAULT_CODE = 0x05,  /* R: latched fault ("error N"), 0 = none */
  DRV_HOST_LAST_RESULT = 0x06, /* R: result of the last host write (incl. 7, 10 rejections) */
  DRV_HOST_SEQ1 = 0x10,        /* RW: 0x10-0x17 playlist, 0 ends it */
  DRV_HOST_RAW_ADDR = 0x20,    /* RW: DRV2605L register 0x00-0x22 */
  DRV_HOST_RAW_DATA = 0x21     /* RW: read/write that register (OTP bit refused) */
};
enum {
  DRV_CMD_PLAY = 0x01, DRV_CMD_STOP = 0x02, DRV_CMD_CALIBRATE = 0x03,
  DRV_CMD_STANDBY = 0x04, DRV_CMD_WAKE = 0x05, DRV_CMD_PLAY_SEQ = 0x06,
  DRV_CMD_SELF_TEST = 0x07, DRV_CMD_RECOVER = 0x08, DRV_CMD_CLEAR_FAULT = 0x09
};
drv_status_t drv_host_write(drv_t *d, uint8_t addr, uint8_t value);
drv_status_t drv_host_read(drv_t *d, uint8_t addr, uint8_t *value);

/* ---- Named ROM effects (TI datasheet 12.1.2), e.g. drv_play(&d, DRV_EFFECT_DOUBLE_CLICK_100) ---- */
enum drv_effect {
  DRV_EFFECT_STRONG_CLICK_100 = 1,
  DRV_EFFECT_STRONG_CLICK_60 = 2,
  DRV_EFFECT_STRONG_CLICK_30 = 3,
  DRV_EFFECT_SHARP_CLICK_100 = 4,
  DRV_EFFECT_SHARP_CLICK_60 = 5,
  DRV_EFFECT_SHARP_CLICK_30 = 6,
  DRV_EFFECT_SOFT_BUMP_100 = 7,
  DRV_EFFECT_SOFT_BUMP_60 = 8,
  DRV_EFFECT_SOFT_BUMP_30 = 9,
  DRV_EFFECT_DOUBLE_CLICK_100 = 10,
  DRV_EFFECT_DOUBLE_CLICK_60 = 11,
  DRV_EFFECT_TRIPLE_CLICK_100 = 12,
  DRV_EFFECT_SOFT_FUZZ_60 = 13,
  DRV_EFFECT_STRONG_BUZZ_100 = 14,
  DRV_EFFECT_750_MS_ALERT_100 = 15,
  DRV_EFFECT_1000_MS_ALERT_100 = 16,
  DRV_EFFECT_STRONG_CLICK_1_100 = 17,
  DRV_EFFECT_STRONG_CLICK_2_80 = 18,
  DRV_EFFECT_STRONG_CLICK_3_60 = 19,
  DRV_EFFECT_STRONG_CLICK_4_30 = 20,
  DRV_EFFECT_MEDIUM_CLICK_1_100 = 21,
  DRV_EFFECT_MEDIUM_CLICK_2_80 = 22,
  DRV_EFFECT_MEDIUM_CLICK_3_60 = 23,
  DRV_EFFECT_SHARP_TICK_1_100 = 24,
  DRV_EFFECT_SHARP_TICK_2_80 = 25,
  DRV_EFFECT_SHARP_TICK_3_60 = 26,
  DRV_EFFECT_SHORT_DOUBLE_CLICK_STRONG_1_100 = 27,
  DRV_EFFECT_SHORT_DOUBLE_CLICK_STRONG_2_80 = 28,
  DRV_EFFECT_SHORT_DOUBLE_CLICK_STRONG_3_60 = 29,
  DRV_EFFECT_SHORT_DOUBLE_CLICK_STRONG_4_30 = 30,
  DRV_EFFECT_SHORT_DOUBLE_CLICK_MEDIUM_1_100 = 31,
  DRV_EFFECT_SHORT_DOUBLE_CLICK_MEDIUM_2_80 = 32,
  DRV_EFFECT_SHORT_DOUBLE_CLICK_MEDIUM_3_60 = 33,
  DRV_EFFECT_SHORT_DOUBLE_SHARP_TICK_1_100 = 34,
  DRV_EFFECT_SHORT_DOUBLE_SHARP_TICK_2_80 = 35,
  DRV_EFFECT_SHORT_DOUBLE_SHARP_TICK_3_60 = 36,
  DRV_EFFECT_LONG_DOUBLE_SHARP_CLICK_STRONG_1_100 = 37,
  DRV_EFFECT_LONG_DOUBLE_SHARP_CLICK_STRONG_2_80 = 38,
  DRV_EFFECT_LONG_DOUBLE_SHARP_CLICK_STRONG_3_60 = 39,
  DRV_EFFECT_LONG_DOUBLE_SHARP_CLICK_STRONG_4_30 = 40,
  DRV_EFFECT_LONG_DOUBLE_SHARP_CLICK_MEDIUM_1_100 = 41,
  DRV_EFFECT_LONG_DOUBLE_SHARP_CLICK_MEDIUM_2_80 = 42,
  DRV_EFFECT_LONG_DOUBLE_SHARP_CLICK_MEDIUM_3_60 = 43,
  DRV_EFFECT_LONG_DOUBLE_SHARP_TICK_1_100 = 44,
  DRV_EFFECT_LONG_DOUBLE_SHARP_TICK_2_80 = 45,
  DRV_EFFECT_LONG_DOUBLE_SHARP_TICK_3_60 = 46,
  DRV_EFFECT_BUZZ_1_100 = 47,
  DRV_EFFECT_BUZZ_2_80 = 48,
  DRV_EFFECT_BUZZ_3_60 = 49,
  DRV_EFFECT_BUZZ_4_40 = 50,
  DRV_EFFECT_BUZZ_5_20 = 51,
  DRV_EFFECT_PULSING_STRONG_1_100 = 52,
  DRV_EFFECT_PULSING_STRONG_2_60 = 53,
  DRV_EFFECT_PULSING_MEDIUM_1_100 = 54,
  DRV_EFFECT_PULSING_MEDIUM_2_60 = 55,
  DRV_EFFECT_PULSING_SHARP_1_100 = 56,
  DRV_EFFECT_PULSING_SHARP_2_60 = 57,
  DRV_EFFECT_TRANSITION_CLICK_1_100 = 58,
  DRV_EFFECT_TRANSITION_CLICK_2_80 = 59,
  DRV_EFFECT_TRANSITION_CLICK_3_60 = 60,
  DRV_EFFECT_TRANSITION_CLICK_4_40 = 61,
  DRV_EFFECT_TRANSITION_CLICK_5_20 = 62,
  DRV_EFFECT_TRANSITION_CLICK_6_10 = 63,
  DRV_EFFECT_TRANSITION_HUM_1_100 = 64,
  DRV_EFFECT_TRANSITION_HUM_2_80 = 65,
  DRV_EFFECT_TRANSITION_HUM_3_60 = 66,
  DRV_EFFECT_TRANSITION_HUM_4_40 = 67,
  DRV_EFFECT_TRANSITION_HUM_5_20 = 68,
  DRV_EFFECT_TRANSITION_HUM_6_10 = 69,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_LONG_SMOOTH_1_100_TO_0 = 70,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_LONG_SMOOTH_2_100_TO_0 = 71,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_MEDIUM_SMOOTH_1_100_TO_0 = 72,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_MEDIUM_SMOOTH_2_100_TO_0 = 73,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_SHORT_SMOOTH_1_100_TO_0 = 74,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_SHORT_SMOOTH_2_100_TO_0 = 75,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_LONG_SHARP_1_100_TO_0 = 76,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_LONG_SHARP_2_100_TO_0 = 77,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_MEDIUM_SHARP_1_100_TO_0 = 78,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_MEDIUM_SHARP_2_100_TO_0 = 79,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_SHORT_SHARP_1_100_TO_0 = 80,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_SHORT_SHARP_2_100_TO_0 = 81,
  DRV_EFFECT_TRANSITION_RAMP_UP_LONG_SMOOTH_1_0_TO_100 = 82,
  DRV_EFFECT_TRANSITION_RAMP_UP_LONG_SMOOTH_2_0_TO_100 = 83,
  DRV_EFFECT_TRANSITION_RAMP_UP_MEDIUM_SMOOTH_1_0_TO_100 = 84,
  DRV_EFFECT_TRANSITION_RAMP_UP_MEDIUM_SMOOTH_2_0_TO_100 = 85,
  DRV_EFFECT_TRANSITION_RAMP_UP_SHORT_SMOOTH_1_0_TO_100 = 86,
  DRV_EFFECT_TRANSITION_RAMP_UP_SHORT_SMOOTH_2_0_TO_100 = 87,
  DRV_EFFECT_TRANSITION_RAMP_UP_LONG_SHARP_1_0_TO_100 = 88,
  DRV_EFFECT_TRANSITION_RAMP_UP_LONG_SHARP_2_0_TO_100 = 89,
  DRV_EFFECT_TRANSITION_RAMP_UP_MEDIUM_SHARP_1_0_TO_100 = 90,
  DRV_EFFECT_TRANSITION_RAMP_UP_MEDIUM_SHARP_2_0_TO_100 = 91,
  DRV_EFFECT_TRANSITION_RAMP_UP_SHORT_SHARP_1_0_TO_100 = 92,
  DRV_EFFECT_TRANSITION_RAMP_UP_SHORT_SHARP_2_0_TO_100 = 93,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_LONG_SMOOTH_1_50_TO_0 = 94,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_LONG_SMOOTH_2_50_TO_0 = 95,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_MEDIUM_SMOOTH_1_50_TO_0 = 96,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_MEDIUM_SMOOTH_2_50_TO_0 = 97,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_SHORT_SMOOTH_1_50_TO_0 = 98,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_SHORT_SMOOTH_2_50_TO_0 = 99,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_LONG_SHARP_1_50_TO_0 = 100,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_LONG_SHARP_2_50_TO_0 = 101,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_MEDIUM_SHARP_1_50_TO_0 = 102,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_MEDIUM_SHARP_2_50_TO_0 = 103,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_SHORT_SHARP_1_50_TO_0 = 104,
  DRV_EFFECT_TRANSITION_RAMP_DOWN_SHORT_SHARP_2_50_TO_0 = 105,
  DRV_EFFECT_TRANSITION_RAMP_UP_LONG_SMOOTH_1_0_TO_50 = 106,
  DRV_EFFECT_TRANSITION_RAMP_UP_LONG_SMOOTH_2_0_TO_50 = 107,
  DRV_EFFECT_TRANSITION_RAMP_UP_MEDIUM_SMOOTH_1_0_TO_50 = 108,
  DRV_EFFECT_TRANSITION_RAMP_UP_MEDIUM_SMOOTH_2_0_TO_50 = 109,
  DRV_EFFECT_TRANSITION_RAMP_UP_SHORT_SMOOTH_1_0_TO_50 = 110,
  DRV_EFFECT_TRANSITION_RAMP_UP_SHORT_SMOOTH_2_0_TO_50 = 111,
  DRV_EFFECT_TRANSITION_RAMP_UP_LONG_SHARP_1_0_TO_50 = 112,
  DRV_EFFECT_TRANSITION_RAMP_UP_LONG_SHARP_2_0_TO_50 = 113,
  DRV_EFFECT_TRANSITION_RAMP_UP_MEDIUM_SHARP_1_0_TO_50 = 114,
  DRV_EFFECT_TRANSITION_RAMP_UP_MEDIUM_SHARP_2_0_TO_50 = 115,
  DRV_EFFECT_TRANSITION_RAMP_UP_SHORT_SHARP_1_0_TO_50 = 116,
  DRV_EFFECT_TRANSITION_RAMP_UP_SHORT_SHARP_2_0_TO_50 = 117,
  DRV_EFFECT_LONG_BUZZ_FOR_PROGRAMMATIC_STOPPING_100 = 118,
  DRV_EFFECT_SMOOTH_HUM_1_NO_KICK_OR_BRAKE_PULSE_50 = 119,
  DRV_EFFECT_SMOOTH_HUM_2_NO_KICK_OR_BRAKE_PULSE_40 = 120,
  DRV_EFFECT_SMOOTH_HUM_3_NO_KICK_OR_BRAKE_PULSE_30 = 121,
  DRV_EFFECT_SMOOTH_HUM_4_NO_KICK_OR_BRAKE_PULSE_20 = 122,
  DRV_EFFECT_SMOOTH_HUM_5_NO_KICK_OR_BRAKE_PULSE_10 = 123,
};

#ifdef __cplusplus
}
#endif
#endif /* DRV2605L_H */
