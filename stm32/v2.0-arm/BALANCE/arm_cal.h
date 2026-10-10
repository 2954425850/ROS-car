#ifndef __ARM_CAL_H
#define __ARM_CAL_H

/* ------------------------------------------------------------------
 * Base joint calibration. Every measured number comes from the bring-up
 * session of 2026-09-27 (M1..M7 - see docs/plans/2026-09-26-arm-
 * bringup-record.md); the derived ones say so and show their formula.
 *
 * M1 verdict: the pot WRAPS. Period 1388 raw = 320.3 deg at the M2
 * scale; boundary pair raw -194/+1194, symmetric +-694 raw about the
 * centre; 4/4 identical wrap events in a 120 s hand rotation. A 320 deg
 * period is shorter than the +-225 deg envelope, so one reading maps to
 * two legal angles - defect #14. The fix (reviewed by the design
 * session, "approve with changes") is the boot-anchor scheme: at power
 * on, only a reading inside the central band - whose alias twin lies
 * beyond the physical envelope - is accepted as absolute; anything else
 * unloads the servo and asks the human to turn the base to a known
 * direction (car front). At run time the tracker has NO blind zone: the
 * wrapping pot is valid everywhere, each fresh reading is snapped onto
 * the branch the estimate already sits on, and the old search/homing
 * machinery is gone.
 * ------------------------------------------------------------------ */

#define ARM_CAL_MEASURED     1    /* M1..M7 measured 2026-09-27 */

/* ---- M1: the pot wraps - one reading per 320.3 deg of base travel ---- */
#define ARM_POT_WRAPS        1

/* ---- M2: the mapping, from car-front / car-left 90 deg references ----
 * Car front (0 deg) reads raw 499; car left (90 deg) reads raw 889, so
 * 390 raw per 90 deg. DEG_AT_RAW0 is set from 500 - the exact centre -
 * so that raw 500 maps to exactly 0.0 deg; the 1-count offset to the
 * measured 499 is 0.23 deg, below the pot noise. */
#define ARM_POT_DEG_AT_RAW0  (-115.4f)  /* mechanical deg when raw == 0     */
#define ARM_POT_DEG_PER_RAW  (0.2308f)  /* 390 raw per 90 deg, M2 measured  */
#define ARM_POT_PERIOD_DEG   (ARM_POT_DEG_PER_RAW * 1388.0f)  /* M1: 320.3 */

/* ---- boot anchor (#14 decision, revised): NO band, full period ----
 * The anchoring accepts any valid principal reading. The wrap ambiguity
 * (which lap the boot position is in) is handled by PROCEDURE: the task
 * layer returns the base to car front (0 deg) before power-off, so the
 * boot reading is the true front. Abnormal endings can park the base one
 * lap off (up to the mechanical ~450 deg) - documented, accepted, manual
 * recovery (record §8). The flash fast-path is the future complete fix. */
#define ARM_ANCHOR_CONFIRM_TICKS 150  /* 3 s: also lets the 2 s reset pose
                                       * finish before control starts (the
                                       * STOP flood of defect #13 must not
                                       * freeze it at 20 ms)                */

/* ---- boot centering: after anchoring, drive to car front (0 deg) ----
 * Tolerance 6 deg: above the measured friction stall residual (-5.4 deg
 * on the - side at KP 18) so the loop can finish; the residual stays
 * inside the +-30 deg parking requirement. Timeout 12 s = give up and
 * release control anyway (the host can fine-command afterwards). */
#define ARM_CENTER_TOL_DEG        6.0f
#define ARM_CENTER_TIMEOUT_TICKS  600

/* ---- boot centering speed ramp: counts of speed field per tick ----
 * Without it the centering command steps 0 -> VEL_MAX in one tick and
 * the base jerks into motion. 40 counts/tick ramps 0 -> 600 in 300 ms. */
#define ARM_CENTER_RAMP_STEP      40

/* ---- M3: speed calibration, measured at the real pack voltage ----
 * Fine 20 Hz traces: 141 raw/s @ +200, 270..330 raw/s @ +-300 on clean
 * stretches - nonlinear and direction-asymmetric (the + side drags, see
 * the record). 162..254 true deg/s at speed 1000; 200 sits mid-band and
 * keeps the spread inside the 0.75x..1.5x tolerance. */
#define ARM_SPEED_1000_DPS   200.0f

/* ---- M5: speed 0 does NOT hold the axis (pushes like unpowered) ----
 * Level ground removes gravity drift; cable torsion and chassis
 * acceleration remain. KP was halved to 10 for the first bring-up
 * (oscillation caution), then raised to 18 after the D2/D3 acceptance:
 * at 10 the loop stalls ~11 deg short of large step targets (friction
 * deadband); 18 keeps the command above the stall threshold with no
 * visible oscillation in D4-style streaming. Link-loss coasting is a
 * documented, accepted behaviour (manual recovery, see the record). */
#define ARM_VEL_KP           18.0f
#define ARM_VEL_MAX          600        /* speed field clamp                */

/* ---- base travel ------------------------------------------------------
 * 360 for the follow feature: the tracker (every-tick unwrap) and the
 * velocity loop have no range limit of their own, the target field
 * (+-1000) simply scales to +-360 deg = 0.36 deg/count. The physical
 * boundaries stay the screws (~450) and the cable (720) - the operator
 * owns the excursion risk (record §8). Note this also means the base can
 * now be PARKED beyond +-260 by a follow task ending abnormally - the
 * boot band's alias exclusion is then best-effort (see record §8, the
 * flash fast-path is the future complete fix). */
#define ARM_SOFT_LIMIT_DEG   360        /* follow feature requirement      */

/* ---- health limits (section 5 of the runtime block) ------------------- */
#define ARM_TEMP_MAX_C       70
#define ARM_VIN_MIN_MV       6400

#endif
