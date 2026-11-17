/* drv2605l.c - see drv2605l.h. Register numbers: datasheet SLOS854D section 8.6.
 *
 * DEBUGGING QUICK START
 *  1. Set port.log to print "error <code> reg <reg>": every failure reports where it happened
 *     (reg 0xFF = an action ended badly, 0xFE = a request was rejected).
 *  2. Healthy chip after drv_init(), read with drv_reg_read():
 *       0x00 STATUS  = 0xE0 (top 3 bits = 7). NOTE: reading STATUS clears DIAG_RESULT and
 *                    OVER_TEMP (datasheet Table 4), so reading it yourself can hide a fault.
 *       0x01 MODE    = 0x00 (0x40 = asleep, 0x05 = real-time, 0x07 = calibrating)
 *       0x03 LIBRARY = 0x01 for the ERM default
 *       0x1A FEEDBACK = 0x28 | BEMF_GAIN for ERM (brake 2, loop gain 2); 0xA8 | gain for LRA
 *       0x1D CONTROL3 bit 5 = 1 (ERM open loop)
 *  3. Motor never moves but no errors: MODE stuck at 0x40 (standby), OD_CLAMP too low,
 *     motor not soldered across TP1/TP2, or effect 0 in the sequence (0 = "end").
 *  4. Error 12 / state FAULT: an I2C error happened mid-change and the driver could not confirm
 *     what the chip is doing. drv_recover() clocks a stuck bus free, resets the chip, re-inits.
 *  5. The driver keeps its own idea of the chip state in d->state. If you write MODE, GO or
 *     RTP yourself with drv_reg_write(), the driver will not know: prefer the API calls.
 */
#include "drv2605l.h"

#include <string.h>

enum {
  R_STATUS = 0x00, R_MODE = 0x01, R_RTP = 0x02, R_LIBRARY = 0x03, R_SEQ1 = 0x04,
  R_GO = 0x0C, R_RATED = 0x16, R_ODCLAMP = 0x17, R_ACAL_COMP = 0x18, R_ACAL_BEMF = 0x19,
  R_FEEDBACK = 0x1A, R_CTRL1 = 0x1B, R_CTRL2 = 0x1C, R_CTRL3 = 0x1D, R_CTRL4 = 0x1E,
  R_OL_PERIOD = 0x20, R_LAST = 0x22
};
enum { MODE_INTTRIG = 0, MODE_RTP = 5, MODE_DIAG = 6, MODE_AUTOCAL = 7 };

#define MODE_MASK 0x47       /* STANDBY + MODE[2:0] */
#define ST_DIAG_RESULT 0x08  /* clears on read */
#define ST_OVER_TEMP 0x02    /* latching, clears on read */
#define ST_OC_DETECT 0x01    /* latching */
#define CAL_TIMEOUT_MS 2500  /* AUTO_CAL_TIME = 3 takes ~1-1.2 s */
#define DIAG_TIMEOUT_MS 2500
#define RTP_POLL_MS 50       /* how often drv_update() checks faults in real-time mode */
#define REG_ACTION 0xFF      /* log reg: an action ended badly */
#define REG_REJECT 0xFE      /* log reg: request rejected */

/* ---------- error helpers ---------- */

static void log_err(drv_t *d, drv_status_t s, uint8_t reg) {
  if (d && d->port.log) d->port.log(d->port.ctx, s, reg);
}

/* A fault on the chip/bus: latched until cleared, and logged with the register involved. */
static drv_status_t fault(drv_t *d, drv_status_t s, uint8_t reg) {
  d->last_error = s;
  log_err(d, s, reg);
  return s;
}

/* A refused request: logged, not latched. */
static drv_status_t reject(drv_t *d, drv_status_t s) {
  log_err(d, s, REG_REJECT);
  return s;
}

static uint32_t now(drv_t *d) { return d->port.millis(d->port.ctx); }

static bool busy(const drv_t *d) {
  return d && (d->state == DRV_STATE_PLAYING || d->state == DRV_STATE_CALIBRATING ||
               d->state == DRV_STATE_DIAG);
}

/* May new work start? */
static drv_status_t gate(drv_t *d) {
  if (!d) return DRV_ERR_BAD_ARG;
  if (d->state == DRV_STATE_UNINIT) return reject(d, DRV_ERR_NOT_INIT);
  if (d->state == DRV_STATE_FAULT) return reject(d, DRV_ERR_NEEDS_RECOVER);
  if (busy(d)) return reject(d, DRV_ERR_BUSY);
  return DRV_OK;
}

static bool valid_effect(uint8_t e) { return e >= 1 && e <= 123; }

static bool library_fits(drv_actuator_t a, uint8_t lib) {
  return a == DRV_LRA ? lib == 6 : (lib >= 1 && lib <= 7 && lib != 6);
}

/* ---------- register access ---------- */

static drv_status_t rd(drv_t *d, uint8_t reg, uint8_t *v) {
  if (d->port.i2c_read(d->port.ctx, DRV2605L_I2C_ADDR, reg, v, 1) != 0)
    return fault(d, DRV_ERR_NACK, reg);
  return DRV_OK;
}

static drv_status_t wr(drv_t *d, uint8_t reg, uint8_t v) {
  uint8_t buf[2] = {reg, v};
  if (d->port.i2c_write(d->port.ctx, DRV2605L_I2C_ADDR, buf, 2) != 0)
    return fault(d, DRV_ERR_NACK, reg);
  return DRV_OK;
}

/* Write a whole register, then read it back if verification is on. */
static drv_status_t write_checked(drv_t *d, uint8_t reg, uint8_t value) {
  drv_status_t s = wr(d, reg, value);
  if (s || !d->cfg.verify_writes) return s;
  uint8_t chk;
  if ((s = rd(d, reg, &chk))) return s;
  return chk == value ? DRV_OK : fault(d, DRV_ERR_VERIFY, reg);
}

/* Change only the bits in mask (read-modify-write), then verify those bits. */
static drv_status_t write_bits(drv_t *d, uint8_t reg, uint8_t mask, uint8_t value) {
  uint8_t cur, chk;
  drv_status_t s = rd(d, reg, &cur);
  if (s) return s;
  if ((s = wr(d, reg, (uint8_t)((cur & ~mask) | (value & mask))))) return s;
  if (!d->cfg.verify_writes) return DRV_OK;
  if ((s = rd(d, reg, &chk))) return s;
  return (chk & mask) == (value & mask) ? DRV_OK : fault(d, DRV_ERR_VERIFY, reg);
}

static drv_status_t set_mode(drv_t *d, uint8_t mode) {
  return write_bits(d, R_MODE, MODE_MASK, mode); /* also clears STANDBY */
}

/* Emergency stop after a failure in the middle of a change. Every stop write is attempted
 * even if an earlier one failed (any one of them may be enough to silence the motor), then
 * GO, RTP and MODE are read back.
 * Output confirmed stopped AND configuration valid -> IDLE. Otherwise -> FAULT.
 * Never claims IDLE on a guess. Returns s unchanged. */
static bool stop_output(drv_t *d) {
  void *c = d->port.ctx;
  const uint8_t go0[2] = {R_GO, 0}, rtp0[2] = {R_RTP, 0}, mode0[2] = {R_MODE, 0};
  uint8_t go = 1, rtp = 1, mode = 0xFF;
  int failures = 0;
  failures += d->port.i2c_write(c, DRV2605L_I2C_ADDR, go0, 2) != 0;   /* cancel action */
  failures += d->port.i2c_write(c, DRV2605L_I2C_ADDR, rtp0, 2) != 0;  /* zero strength */
  failures += d->port.i2c_write(c, DRV2605L_I2C_ADDR, mode0, 2) != 0; /* leave RTP/cal/diag */
  failures += d->port.i2c_read(c, DRV2605L_I2C_ADDR, R_GO, &go, 1) != 0;
  failures += d->port.i2c_read(c, DRV2605L_I2C_ADDR, R_RTP, &rtp, 1) != 0;
  failures += d->port.i2c_read(c, DRV2605L_I2C_ADDR, R_MODE, &mode, 1) != 0;
  bool stopped = failures == 0 && !(go & 1) && rtp == 0 && (mode & MODE_MASK) == 0;
  d->host_rtp = 0;
  return stopped;
}

static drv_status_t abort_to_idle(drv_t *d, drv_status_t s) {
  bool stopped = stop_output(d);
  d->state = (stopped && d->config_ok && !d->cal_pending) ? DRV_STATE_IDLE : DRV_STATE_FAULT;
  if (d->state == DRV_STATE_FAULT) log_err(d, DRV_ERR_NEEDS_RECOVER, REG_ACTION);
  return s;
}

/* The chip's settings no longer match cfg: stop output and require drv_recover(). */
static drv_status_t config_broken(drv_t *d, drv_status_t s) {
  d->config_ok = false;
  return abort_to_idle(d, s); /* always ends in FAULT now */
}

/* What the calibration registers should hold: the saved calibration, or the datasheet
 * power-on defaults (Table 3: A_CAL_COMP 0x0C, A_CAL_BEMF 0x6C, BEMF_GAIN 2) if none. */
static drv_cal_t baseline_cal(const drv_t *d) {
  drv_cal_t c = {0x0C, 0x6C, 2, false};
  return d->cfg.cal.valid ? d->cfg.cal : c;
}

/* Every started action ends here exactly once (state already set by the caller). */
static drv_status_t end_action(drv_t *d, drv_status_t r) {
  if (r != DRV_OK && r != DRV_ERR_CANCELLED) {
    d->last_error = r;
    log_err(d, r, REG_ACTION);
  }
  if (d->on_done) d->on_done(d->cb_user, r);
  return r;
}

static drv_status_t start_op(drv_t *d, drv_state_t st, uint32_t timeout_ms) {
  /* A failed transaction may still have reached the chip. Track calibration
   * before attempting GO, so stop/recovery cannot overlook partial results. */
  if (st == DRV_STATE_CALIBRATING) d->cal_pending = true;
  drv_status_t s = wr(d, R_GO, 0x01);
  if (s) return abort_to_idle(d, s);
  d->state = st;
  d->op_start_ms = now(d);
  d->op_timeout_ms = timeout_ms;
  return DRV_OK;
}

static drv_status_t load_cal_hw(drv_t *d, const drv_cal_t *cal) {
  drv_status_t s;
  if ((s = write_checked(d, R_ACAL_COMP, cal->comp))) return s;
  if ((s = write_checked(d, R_ACAL_BEMF, cal->bemf))) return s;
  return write_bits(d, R_FEEDBACK, 0x03, cal->bemf_gain);
}

/* After any unsuccessful calibration or calibration load, put the baseline values back
 * (saved calibration, or defaults). If even that fails, the chip holds a mix of values:
 * mark the configuration broken so nothing plays until drv_recover(). */
static bool restore_cal(drv_t *d) {
  /* Never write results while an unconfirmed calibration may still change them. */
  if (!stop_output(d)) {
    d->config_ok = false;
    d->state = DRV_STATE_FAULT;
    fault(d, DRV_ERR_NEEDS_RECOVER, R_MODE);
    return false;
  }
  drv_cal_t base = baseline_cal(d);
  bool saved_verify = d->cfg.verify_writes;
  d->cfg.verify_writes = true; /* always confirm a restore */
  drv_status_t s = load_cal_hw(d, &base);
  d->cfg.verify_writes = saved_verify;
  if (s == DRV_OK) {
    d->cal_pending = false;
    d->state = d->config_ok ? DRV_STATE_IDLE : DRV_STATE_FAULT;
    return true;
  }
  config_broken(d, s);
  return false;
}

static bool valid_config(const drv_config_t *c) {
  if (c->actuator != DRV_ERM && c->actuator != DRV_LRA) return false;
  if (!library_fits(c->actuator, c->library) || c->od_clamp == 0) return false;
  if (c->brake_factor > 7 || c->loop_gain > 3) return false;
  if (c->drive_time > 0x1F && c->drive_time != 0xFF) return false;
  if (c->lra_ol_period > 0x7F && c->lra_ol_period != 0xFF) return false;
  if (c->cal.valid && c->cal.bemf_gain > 3) return false;
  return true;
}

/* ---------- setup ---------- */

void drv_default_config(drv_config_t *cfg) {
  if (!cfg) return;
  memset(cfg, 0, sizeof(*cfg));
  cfg->actuator = DRV_ERM;
  cfg->rated_voltage = 0x8E; /* ~3.0 V closed loop (21.18 mV/step) */
  cfg->od_clamp = 0x8B;      /* ~3.0 V open loop (21.59 mV/step) */
  cfg->library = 1;          /* ERM library A */
  cfg->open_loop = true;
  cfg->brake_factor = 2;     /* TI: "valid for most actuators" */
  cfg->loop_gain = 2;
  cfg->drive_time = 0xFF;
  cfg->lra_ol_period = 0xFF;
  cfg->verify_writes = true;
  cfg->play_timeout_ms = 5000;
}

drv_status_t drv_init(drv_t *d, const drv_port_t *port, const drv_config_t *cfg) {
  if (!d) return DRV_ERR_BAD_ARG;
  /* Copy first (port/cfg may point into *d), then wipe everything: old contents of *d,
   * including any callback pointer, are never used. */
  if (!port || !cfg) {
    memset(d, 0, sizeof(*d));
    return DRV_ERR_BAD_ARG;
  }
  drv_port_t p = *port;
  drv_config_t c = *cfg;
  memset(d, 0, sizeof(*d));
  d->port = p;
  d->cfg = c;
  d->host_effect = 1;
  d->state = DRV_STATE_UNINIT;
  if (!p.i2c_write || !p.i2c_read || !p.millis || !p.delay_ms) return reject(d, DRV_ERR_BAD_ARG);
  if (!valid_config(&c)) return reject(d, DRV_ERR_BAD_ARG);

  if (p.set_trig) p.set_trig(p.ctx, false); /* IN/TRIG must not float */
  p.delay_ms(p.ctx, 1);                      /* >= 250 us after power-up */

  /* Step 1: is the chip there? error 1 here = wiring/power/pull-ups (see drv2605l.h).
   * This read also clears stale DIAG_RESULT / OVER_TEMP flags. */
  uint8_t status;
  drv_status_t s = rd(d, R_STATUS, &status);
  if (s) return s;
  /* Step 2: is it the right chip? A DRV2605L reports 7 in STATUS[7:5] (default 0xE0). */
  if ((status >> 5) != 7) return fault(d, DRV_ERR_WRONG_ID, R_STATUS);

  /* Step 3: stop anything still running (the MCU may have reset while the chip stayed powered). */
  if ((s = wr(d, R_GO, 0x00))) return s;
  if ((s = wr(d, R_RTP, 0x00))) return s;

  /* Step 4: configure. With verify_writes on, an error 3 here names the exact register. */
  if ((s = set_mode(d, MODE_INTTRIG))) return s; /* wake (chip powers up asleep) */
  if ((s = write_bits(d, R_FEEDBACK, 0xFC,
                      (uint8_t)((c.actuator == DRV_LRA ? 0x80 : 0) | (c.brake_factor << 4) |
                                (c.loop_gain << 2)))))
    return s;
  if ((s = write_checked(d, R_RATED, c.rated_voltage))) return s;
  if ((s = write_checked(d, R_ODCLAMP, c.od_clamp))) return s;
  if (c.actuator == DRV_ERM)
    s = write_bits(d, R_CTRL3, 0x20, c.open_loop ? 0x20 : 0x00); /* ERM_OPEN_LOOP */
  else
    s = write_bits(d, R_CTRL3, 0x01, c.open_loop ? 0x01 : 0x00); /* LRA_OPEN_LOOP */
  if (s) return s;
  if (c.actuator == DRV_LRA && c.drive_time != 0xFF &&
      (s = write_bits(d, R_CTRL1, 0x1F, c.drive_time)))
    return s;
  if (c.actuator == DRV_LRA && c.lra_ol_period != 0xFF &&
      (s = write_bits(d, R_OL_PERIOD, 0x7F, c.lra_ol_period)))
    return s;
  if ((s = write_bits(d, R_LIBRARY, 0x07, c.library))) return s;
  /* Calibration registers: saved values, or datasheet defaults, so nothing is left over
   * from an earlier session or an interrupted calibration. */
  drv_cal_t base = baseline_cal(d);
  if ((s = load_cal_hw(d, &base))) return s;

  d->config_ok = true; /* every setting above was written (and verified if enabled) */
  d->state = DRV_STATE_IDLE;
  d->last_error = DRV_OK;
  return DRV_OK;
}

void drv_set_callback(drv_t *d, drv_done_cb cb, void *user) {
  if (!d) return;
  d->on_done = cb;
  d->cb_user = user;
}

/* ---------- playback ---------- */

drv_status_t drv_play_sequence(drv_t *d, const uint8_t *steps, uint8_t n) {
  drv_status_t s = gate(d);
  if (s) return s;
  if (!steps || n == 0 || n > 8) return reject(d, DRV_ERR_BAD_ARG);
  for (uint8_t i = 0; i < n; i++)
    if (!(steps[i] & 0x80) && !valid_effect(steps[i])) return reject(d, DRV_ERR_BAD_ARG);

  if (d->state == DRV_STATE_RTP) {
    if ((s = wr(d, R_RTP, 0))) return abort_to_idle(d, s);
    d->host_rtp = 0;
  }
  if ((s = set_mode(d, MODE_INTTRIG))) return abort_to_idle(d, s);
  for (uint8_t i = 0; i < 8; i++)
    if ((s = write_checked(d, (uint8_t)(R_SEQ1 + i), i < n ? steps[i] : 0)))
      return abort_to_idle(d, s);
  return start_op(d, DRV_STATE_PLAYING, d->cfg.play_timeout_ms);
}

drv_status_t drv_play(drv_t *d, uint8_t effect) {
  if (!valid_effect(effect)) return reject(d, DRV_ERR_BAD_ARG);
  return drv_play_sequence(d, &effect, 1);
}

drv_status_t drv_set_strength(drv_t *d, uint8_t level) {
  drv_status_t s = gate(d);
  if (s) return s;
  if (level > 127) return reject(d, DRV_ERR_BAD_ARG);
  if (level == 0) {
    if (d->state != DRV_STATE_RTP) { /* not vibrating: nothing to change on the chip */
      d->host_rtp = 0;
      return DRV_OK;
    }
    if ((s = wr(d, R_RTP, 0)) || (s = set_mode(d, MODE_INTTRIG))) return abort_to_idle(d, s);
    d->host_rtp = 0;
    d->state = DRV_STATE_IDLE;
    return DRV_OK;
  }
  if (d->state != DRV_STATE_RTP && (s = set_mode(d, MODE_RTP))) return abort_to_idle(d, s);
  if ((s = write_checked(d, R_RTP, level))) return abort_to_idle(d, s);
  if (d->state != DRV_STATE_RTP) d->last_poll_ms = now(d);
  d->state = DRV_STATE_RTP;
  d->host_rtp = level;
  return DRV_OK;
}

/* Stops output and confirms it. Leaves FAULT only if the configuration is still valid:
 * a half-configured chip stays in FAULT and needs drv_recover(). */
drv_status_t drv_stop(drv_t *d) {
  if (!d) return DRV_ERR_BAD_ARG;
  if (d->state == DRV_STATE_UNINIT) return reject(d, DRV_ERR_NOT_INIT);
  if (d->state == DRV_STATE_STANDBY) return DRV_OK; /* nothing can be running */
  bool was_action = busy(d);
  abort_to_idle(d, DRV_OK); /* GO = 0, RTP = 0, internal trigger, all confirmed */
  if (d->cal_pending) restore_cal(d); /* undo half-written calibration */
  drv_status_t s = d->state == DRV_STATE_IDLE ? DRV_OK : fault(d, DRV_ERR_NEEDS_RECOVER, R_MODE);
  /* "cancelled" only when the stop is confirmed; otherwise the owner gets the real error */
  if (was_action) end_action(d, s == DRV_OK ? DRV_ERR_CANCELLED : s);
  return s;
}

drv_status_t drv_select_library(drv_t *d, uint8_t lib) {
  drv_status_t s = gate(d);
  if (s) return s;
  if (!library_fits(d->cfg.actuator, lib)) return reject(d, DRV_ERR_BAD_ARG);
  if ((s = write_bits(d, R_LIBRARY, 0x07, lib))) {
    /* the chip may now hold the new value: put the configured one back, or require recovery */
    if (write_bits(d, R_LIBRARY, 0x07, d->cfg.library)) return config_broken(d, s);
    return s;
  }
  d->cfg.library = lib;
  return DRV_OK;
}

/* ---------- progress ---------- */

/* Judge STATUS once: DIAG_RESULT and OVER_TEMP clear when read, so every flag
 * must be evaluated from the same read. Faults outrank a calibration/diagnostic result. */
static drv_status_t judge(uint8_t st, drv_state_t what) {
  if (st & ST_OVER_TEMP) return DRV_ERR_OVER_TEMP;
  if (st & ST_OC_DETECT) return DRV_ERR_OVERCURRENT;
  if (st & ST_DIAG_RESULT) {
    if (what == DRV_STATE_CALIBRATING) return DRV_ERR_CAL_FAIL;
    if (what == DRV_STATE_DIAG) return DRV_ERR_DIAG_FAIL;
  }
  return DRV_OK;
}

/* How a non-blocking action ends:
 *   start call (play / calibrate / diagnostics) writes GO = 1 and sets d->state
 *   drv_update() reads GO: 1 = still running (returns BUSY), 0 = chip finished
 *   then it reads STATUS once to judge the result and returns to IDLE (or FAULT if the
 *   chip's state can't be confirmed).
 * DEBUG: BUSY forever = drv_update() not called often or play_timeout_ms huge;
 * 4 (TIMEOUT) = GO really stayed 1. */
/* Ends a calibration or diagnostic run (any outcome) and an action of any kind.
 * Calibration: success commits the fresh results; every other exit restores the baseline
 * calibration (or marks the configuration broken). Output is always returned to internal
 * trigger, and that step's exact error is reported if nothing failed earlier. */
static drv_status_t finish_action(drv_t *d, drv_state_t what, drv_status_t r) {
  if (what == DRV_STATE_CALIBRATING) {
    drv_cal_t fresh = {0, 0, 0, false};
    if (r == DRV_OK) { /* commit only when every result was read */
      uint8_t fb = 0;
      drv_status_t e;
      if ((e = rd(d, R_ACAL_COMP, &fresh.comp)) || (e = rd(d, R_ACAL_BEMF, &fresh.bemf)) ||
          (e = rd(d, R_FEEDBACK, &fb)))
        r = e;
      fresh.bemf_gain = fb & 0x03;
      fresh.valid = true;
    }
    if (r == DRV_OK) {
      d->cfg.cal = fresh;
      d->cal_pending = false;
    } else if (!restore_cal(d)) {
      return end_action(d, r); /* config broken: state is FAULT */
    }
  }
  if (what != DRV_STATE_PLAYING) {
    drv_status_t m = set_mode(d, MODE_INTTRIG);
    if (m) {
      abort_to_idle(d, m);
      return end_action(d, r ? r : m); /* e.g. 3 (VERIFY), not a generic NACK */
    }
  }
  d->state = DRV_STATE_IDLE;
  return end_action(d, r);
}

drv_status_t drv_update(drv_t *d) {
  uint8_t go, st;
  drv_status_t r;

  /* not "OK" while the driver can't work: polling loops must see this */
  if (!d) return DRV_ERR_BAD_ARG;
  if (d->state == DRV_STATE_UNINIT) return DRV_ERR_NOT_INIT;
  if (d->state == DRV_STATE_FAULT) return DRV_ERR_NEEDS_RECOVER;

  if (d->state == DRV_STATE_RTP) { /* continuous vibration: watch for faults */
    if ((uint32_t)(now(d) - d->last_poll_ms) < RTP_POLL_MS) return DRV_OK;
    d->last_poll_ms = now(d);
    if ((r = rd(d, R_STATUS, &st))) return abort_to_idle(d, r);
    if ((r = judge(st, DRV_STATE_RTP))) return abort_to_idle(d, fault(d, r, R_STATUS));
    return DRV_OK;
  }
  if (!busy(d)) return DRV_OK;

  drv_state_t what = d->state;
  if ((r = rd(d, R_GO, &go))) { /* can't tell if it's still running: stop it */
    abort_to_idle(d, r);
    if (what == DRV_STATE_CALIBRATING) restore_cal(d);
    return end_action(d, r);
  }
  if (go & 0x01) {
    if ((uint32_t)(now(d) - d->op_start_ms) <= d->op_timeout_ms) return DRV_ERR_BUSY;
    abort_to_idle(d, DRV_ERR_TIMEOUT); /* cancel; FAULT if the cancel can't be confirmed */
    if (what == DRV_STATE_CALIBRATING) restore_cal(d);
    return end_action(d, fault(d, DRV_ERR_TIMEOUT, R_GO));
  }
  if ((r = rd(d, R_STATUS, &st))) {
    abort_to_idle(d, r);
    if (what == DRV_STATE_CALIBRATING) restore_cal(d);
    return end_action(d, r);
  }
  return finish_action(d, what, judge(st, what));
}

drv_status_t drv_wait(drv_t *d) {
  drv_status_t s;
  while ((s = drv_update(d)) == DRV_ERR_BUSY) d->port.delay_ms(d->port.ctx, 1);
  return s;
}

bool drv_is_busy(const drv_t *d) { return busy(d); }
drv_state_t drv_state(const drv_t *d) { return d ? d->state : DRV_STATE_UNINIT; }
drv_status_t drv_last_error(const drv_t *d) { return d ? d->last_error : DRV_ERR_BAD_ARG; }
void drv_clear_error(drv_t *d) { if (d) d->last_error = DRV_OK; }

/* ---------- calibration, health, recovery ---------- */

drv_status_t drv_start_calibration(drv_t *d) {
  drv_status_t s = gate(d);
  if (s) return s;
  if (d->state == DRV_STATE_RTP) {
    if ((s = wr(d, R_RTP, 0))) return abort_to_idle(d, s);
    d->host_rtp = 0;
  }
  if ((s = set_mode(d, MODE_AUTOCAL))) return abort_to_idle(d, s);
  /* same brake factor / loop gain as drv_init() so results match the running config */
  if ((s = write_bits(d, R_FEEDBACK, 0x7C,
                      (uint8_t)((d->cfg.brake_factor << 4) | (d->cfg.loop_gain << 2)))))
    return abort_to_idle(d, s);
  if ((s = write_bits(d, R_CTRL4, 0xF0, 3 << 4))) return abort_to_idle(d, s); /* AUTO_CAL_TIME 3 */
  if (d->cfg.actuator == DRV_LRA && d->cfg.drive_time != 0xFF &&
      (s = write_bits(d, R_CTRL1, 0x1F, d->cfg.drive_time)))
    return abort_to_idle(d, s);
  if ((s = write_bits(d, R_CTRL2, 0x3F, (3 << 4) | (1 << 2) | 1))) /* sample 3, blank 1, idiss 1 */
    return abort_to_idle(d, s);
  s = start_op(d, DRV_STATE_CALIBRATING, CAL_TIMEOUT_MS);
  if (s) restore_cal(d);
  return s;
}

drv_status_t drv_get_calibration(const drv_t *d, drv_cal_t *out) {
  if (!d || !out) return DRV_ERR_BAD_ARG;
  *out = d->cfg.cal;
  return d->cfg.cal.valid ? DRV_OK : DRV_ERR_NOT_INIT;
}

drv_status_t drv_load_calibration(drv_t *d, const drv_cal_t *cal) {
  drv_status_t s = gate(d); /* rejects an uninitialised driver before any I2C call */
  if (s) return s;
  if (!cal || cal->bemf_gain > 3) return reject(d, DRV_ERR_BAD_ARG);
  if (!stop_output(d)) {
    d->state = DRV_STATE_FAULT;
    return fault(d, DRV_ERR_NEEDS_RECOVER, R_MODE);
  }
  d->state = DRV_STATE_IDLE;
  if ((s = load_cal_hw(d, cal))) {
    restore_cal(d); /* previous values back on the chip, or FAULT until drv_recover() */
    return s;
  }
  d->cfg.cal = *cal;
  d->cfg.cal.valid = true;
  return DRV_OK;
}

drv_status_t drv_start_diagnostics(drv_t *d) {
  drv_status_t s = gate(d);
  if (s) return s;
  if (d->state == DRV_STATE_RTP) {
    if ((s = wr(d, R_RTP, 0))) return abort_to_idle(d, s);
    d->host_rtp = 0;
  }
  if ((s = set_mode(d, MODE_DIAG))) return abort_to_idle(d, s);
  return start_op(d, DRV_STATE_DIAG, DIAG_TIMEOUT_MS);
}

drv_status_t drv_self_test(drv_t *d) {
  drv_status_t s = gate(d);
  if (s) return s;
  if (d->state == DRV_STATE_RTP && (s = drv_stop(d))) return s;

  uint8_t v;
  if ((s = rd(d, R_STATUS, &v))) return s; /* also clears stale flags before diagnostics */
  if ((v >> 5) != 7) return fault(d, DRV_ERR_WRONG_ID, R_STATUS);

  /* Read-back test on RTP_INPUT (no effect outside RTP mode), both bit patterns. */
  static const uint8_t patterns[2] = {0x55, 0xAA};
  for (int i = 0; i < 2; i++) {
    if ((s = wr(d, R_RTP, patterns[i])) || (s = rd(d, R_RTP, &v))) return abort_to_idle(d, s);
    if (v != patterns[i]) return abort_to_idle(d, fault(d, DRV_ERR_VERIFY, R_RTP));
  }
  if ((s = wr(d, R_RTP, 0))) return abort_to_idle(d, s);

  if ((s = drv_start_diagnostics(d))) return s; /* checks the motor is connected */
  return drv_wait(d);
}

drv_status_t drv_standby(drv_t *d, bool on) {
  drv_status_t s = gate(d);
  if (s) return s;
  if (on) {
    if (d->state == DRV_STATE_RTP && (s = wr(d, R_RTP, 0))) return abort_to_idle(d, s);
    if ((s = write_bits(d, R_MODE, 0x40, 0x40))) return abort_to_idle(d, s);
    d->state = DRV_STATE_STANDBY;
  } else {
    if ((s = set_mode(d, MODE_INTTRIG))) return abort_to_idle(d, s);
    d->state = DRV_STATE_IDLE;
  }
  d->host_rtp = 0;
  return DRV_OK;
}

drv_status_t drv_recover(drv_t *d) {
  if (!d) return DRV_ERR_BAD_ARG;
  drv_port_t p = d->port;
  drv_config_t c = d->cfg;
  drv_done_cb cb = d->on_done;
  void *user = d->cb_user;
  if (!p.i2c_write || !p.i2c_read || !p.millis || !p.delay_ms) return DRV_ERR_NOT_INIT;
  bool was_action = busy(d); /* its owner is told how recovery ended (below) */
  drv_status_t s = DRV_OK;

  /* Stage 1: free the bus (if the port can). */
  if (p.bus_recover && p.bus_recover(p.ctx) != 0) {
    s = fault(d, DRV_ERR_NACK, REG_ACTION);
  } else {
    /* Stage 2: DEV_RESET (same as a power cycle). The write must be ACKed... */
    uint8_t buf[2] = {R_MODE, 0x80};
    d->config_ok = false; /* even a failed reset transfer may have taken effect */
    if (p.i2c_write(p.ctx, DRV2605L_I2C_ADDR, buf, 2) != 0) {
      s = fault(d, DRV_ERR_NACK, R_MODE);
    } else {
      d->config_ok = false; /* reset sent: settings are gone from here on */
      /* ...and the reset must be seen to finish (DEV_RESET self-clears). */
      bool reset_done = false;
      for (int i = 0; i < 10 && !reset_done; i++) {
        uint8_t m;
        p.delay_ms(p.ctx, 1);
        reset_done = p.i2c_read(p.ctx, DRV2605L_I2C_ADDR, R_MODE, &m, 1) == 0 && !(m & 0x80);
      }
      /* Stage 3: full re-init with the same config and saved calibration. */
      s = reset_done ? drv_init(d, &p, &c) : fault(d, DRV_ERR_TIMEOUT, R_MODE);
      d->on_done = cb; /* drv_init() wiped them */
      d->cb_user = user;
    }
  }
  if (s) {
    d->state = DRV_STATE_FAULT; /* bus, reset or re-init not confirmed */
    /* A cancelled calibration may have changed results even if bus recovery
     * failed before a reset was attempted. Only full recovery may unlock it. */
    if (d->cal_pending) d->config_ok = false;
  }
  /* "cancelled" only if the chip was really reset and re-initialised */
  if (was_action && cb) cb(user, s == DRV_OK ? DRV_ERR_CANCELLED : s);
  return s;
}

/* ---------- raw access (public) ---------- */

drv_status_t drv_reg_read(drv_t *d, uint8_t reg, uint8_t *value) {
  if (!d) return DRV_ERR_BAD_ARG;
  if (!value || reg > R_LAST) return reject(d, DRV_ERR_BAD_ARG);
  if (d->state == DRV_STATE_UNINIT) return reject(d, DRV_ERR_NOT_INIT);
  return rd(d, reg, value);
}

drv_status_t drv_reg_write(drv_t *d, uint8_t reg, uint8_t value) {
  if (!d) return DRV_ERR_BAD_ARG;
  if (reg > R_LAST) return reject(d, DRV_ERR_BAD_ARG);
  if (d->state == DRV_STATE_UNINIT) return reject(d, DRV_ERR_NOT_INIT);
  return wr(d, reg, value);
}

/* ---------- host register map ----------
 * The UART / SPI / I2C handler for the FPGA link only needs to call drv_host_write() and
 * drv_host_read(). DEBUG: if the FPGA side "does nothing", read LAST_RESULT (0x06) for the
 * result of its last write (e.g. 10 = bad address/value, 7 = busy) and FAULT_CODE (0x05)
 * for real chip/bus faults. */

static drv_status_t play_host_seq(drv_t *d) {
  uint8_t n = 0;
  while (n < 8 && d->host_seq[n] != 0) n++;
  return n ? drv_play_sequence(d, d->host_seq, n) : reject(d, DRV_ERR_BAD_ARG);
}

static drv_status_t host_write(drv_t *d, uint8_t addr, uint8_t value) {
  if (addr >= DRV_HOST_SEQ1 && addr < DRV_HOST_SEQ1 + 8) {
    if (value && !(value & 0x80) && !valid_effect(value)) return reject(d, DRV_ERR_BAD_ARG);
    d->host_seq[addr - DRV_HOST_SEQ1] = value;
    return DRV_OK;
  }
  switch (addr) {
    case DRV_HOST_COMMAND:
      switch (value) {
        case DRV_CMD_PLAY: return drv_play(d, d->host_effect);
        case DRV_CMD_STOP: return drv_stop(d);
        case DRV_CMD_CALIBRATE: return drv_start_calibration(d);
        case DRV_CMD_STANDBY: return drv_standby(d, true);
        case DRV_CMD_WAKE: return drv_standby(d, false);
        case DRV_CMD_PLAY_SEQ: return play_host_seq(d);
        case DRV_CMD_SELF_TEST: return drv_self_test(d);
        case DRV_CMD_RECOVER: return drv_recover(d);
        case DRV_CMD_CLEAR_FAULT: drv_clear_error(d); return DRV_OK;
        default: return reject(d, DRV_ERR_BAD_ARG);
      }
    case DRV_HOST_EFFECT:
      if (!valid_effect(value)) return reject(d, DRV_ERR_BAD_ARG);
      d->host_effect = value;
      return DRV_OK;
    case DRV_HOST_RTP_LEVEL: return drv_set_strength(d, value);
    case DRV_HOST_LIBRARY: return drv_select_library(d, value);
    case DRV_HOST_RAW_ADDR:
      if (value > R_LAST) return reject(d, DRV_ERR_BAD_ARG);
      d->host_raw_addr = value;
      return DRV_OK;
    case DRV_HOST_RAW_DATA:
      /* MODE, RTP and GO are owned by the driver (use COMMAND); OTP burning is one-time only */
      if (d->host_raw_addr == R_MODE || d->host_raw_addr == R_RTP || d->host_raw_addr == R_GO ||
          (d->host_raw_addr == R_CTRL4 && (value & 0x01)))
        return reject(d, DRV_ERR_BAD_ARG);
      return drv_reg_write(d, d->host_raw_addr, value);
    default: return reject(d, DRV_ERR_BAD_ARG); /* read-only or unknown */
  }
}

drv_status_t drv_host_write(drv_t *d, uint8_t addr, uint8_t value) {
  if (!d) return DRV_ERR_BAD_ARG;
  drv_status_t r = host_write(d, addr, value);
  d->host_last_result = r; /* rejections show here, not in FAULT_CODE */
  return r;
}

drv_status_t drv_host_read(drv_t *d, uint8_t addr, uint8_t *value) {
  if (!d) return DRV_ERR_BAD_ARG;
  if (!value) return reject(d, DRV_ERR_BAD_ARG);
  if (addr >= DRV_HOST_SEQ1 && addr < DRV_HOST_SEQ1 + 8) {
    *value = d->host_seq[addr - DRV_HOST_SEQ1];
    return DRV_OK;
  }
  switch (addr) {
    case DRV_HOST_STATUS:
      *value = (uint8_t)((busy(d) ? 0x01 : 0) | (d->last_error ? 0x02 : 0) |
                         (d->cfg.cal.valid ? 0x04 : 0) |
                         (d->state != DRV_STATE_UNINIT ? 0x08 : 0) |
                         (d->state == DRV_STATE_FAULT ? 0x10 : 0));
      return DRV_OK;
    case DRV_HOST_COMMAND: *value = 0; return DRV_OK; /* write-only */
    case DRV_HOST_EFFECT: *value = d->host_effect; return DRV_OK;
    case DRV_HOST_RTP_LEVEL: *value = d->host_rtp; return DRV_OK;
    case DRV_HOST_LIBRARY: *value = d->cfg.library; return DRV_OK;
    case DRV_HOST_FAULT_CODE: *value = (uint8_t)d->last_error; return DRV_OK;
    case DRV_HOST_LAST_RESULT: *value = (uint8_t)d->host_last_result; return DRV_OK;
    case DRV_HOST_RAW_ADDR: *value = d->host_raw_addr; return DRV_OK;
    case DRV_HOST_RAW_DATA: return drv_reg_read(d, d->host_raw_addr, value);
    default: *value = 0; return reject(d, DRV_ERR_BAD_ARG);
  }
}
