#ifndef __ARM_H
#define __ARM_H

#include "system.h"
#include "lx_servo.h"
#include "arm_cal.h"

/* ---- task parameters ---- */
#define ARM_TASK_PRIO     3
#define ARM_STK_SIZE      512
#define ARM_RATE_HZ       50

/* ---- joints: 0xAC field i drives servo ARM_JOINT_ID[i] ---- */
#define ARM_JOINTS        6
extern const u8  ARM_JOINT_ID[ARM_JOINTS];    /* {1,2,3,4,5,6} */
extern const u16 ARM_RESET_POSE[ARM_JOINTS];  /* spec 5.5.1    */
#define ARM_BASE_JOINT    5                   /* index of servo 6 */
#define ARM_GRIPPER_FIELD 0
#define ARM_GRIPPER_MAX   600                 /* stall protection; M7 measured
                                               * the mechanism pins at ~625 */

/* ---- base hold-loop anti-dither (2026-09-27, field-report: the turret
 * micro-hunted around the held target with no host traffic at all) ----
 * Root cause: after streaming stops, ArmTime stays at the last frame's
 * small t_ms, so arm_base_vel()'s feed-forward err/ts keeps running at the
 * streaming gain with nobody asking it to hurry any more. Fix: detect
 * "hold" (no fresh 0xAC for N ticks) and run the loop with a hold t_ms.
 * The streaming path is bit-for-bit unchanged.
 *
 * The angle is NOT low-passed in either state. A filter was tried and
 * dropped: the pot's measured noise floor is one count (0.23 deg), so
 * there is nothing to filter, and the lag it adds is phase margin the
 * loop needs. Hold t_ms is the lever. */
#define ARM_HOLD_T_MS        500u  /* hold-state travel time (same scale as
                                    * the intake's t_ms==0 default)        */
#define ARM_STREAM_AGE_TICKS 5u    /* ticks (50Hz) since the last fresh
                                    * 0xAC above which we are "holding"   */

/* ---- ArmHold stop (2026-10-02, field report: the loaded shoulder crept
 * 594 -> 629 counts across a 2-3 s host restart, against ~5 in a normal
 * hold) ----
 * Under ArmHold the five position servos are sent MOVE_STOP for this many
 * ticks after the hold begins - enough that one lost bus frame cannot leave
 * a joint moving - and then left alone. Sent every tick, as it used to be,
 * each stop re-targets the servo to where it is NOW, so a joint that gravity
 * has let sag a count gets a lower target fifty times a second and creeps
 * down for as long as the hold lasts: an e-stop or a link loss with a
 * payload in the gripper slowly lowers it. */
#define ARM_HOLD_STOP_TICKS  3u

/* ---- setpoint interpolator (2026-10-01, field report: the arm is "not
 * silky" while streaming) ----
 * Root cause: the host streams at 25Hz with t_ms=40, i.e. t_ms == the frame
 * period, while the firmware samples on its own 50Hz clock. The two clocks
 * beat, so the gap the servo sees between targets wanders over 20/40/60 ms
 * against a fixed 40 ms move: it either arrives and stops for a tick, or is
 * cut off mid-profile. Stop-go, every frame.
 *
 * Fix: a frame only sets a TARGET and a rate (cover the distance in the
 * frame's t_ms, or in the measured frame gap if that is longer). Every tick
 * the setpoint walks toward the target by that rate, and the setpoint - not
 * the frame - goes to the servos, with a move time of ARM_INTERP_T_MS. That
 * is two ticks, so consecutive setpoints overlap and the servo is never
 * allowed to arrive and stop between them; a late tick is absorbed too. */
#define ARM_INTERP_T_MS      40u   /* move time given to the servo for each
                                    * per-tick setpoint (2 ticks overlap)  */
#define ARM_SERVO_T_MAX      120u  /* per-joint move time ceiling: a slow
                                    * joint gets its step interval + 1 tick */
#define ARM_STREAM_GAP_MAX   10u   /* a frame gap up to this many ticks is
                                    * "streaming" and stretches the move   */
#define ARM_JOINT_VMAX       1000.0f /* position joints, counts/s ceiling  */
#define ARM_TRACK_T_MS       40u   /* base loop t_ms while the setpoint is
                                    * moving: the acceptance-validated
                                    * streaming value                      */

/* ---- base tracker states (also reported in the 0x56 frame) ----
 *
 * Only UNKNOWN and OK are ever assigned, so on the wire this field is
 * effectively a boolean. The numbers are left as they are so that 1/3/4 can
 * be given a meaning later without a protocol flag day.
 *
 * HOMING(1), BLIND(3) and FAULT(4) used to be defined here. Nothing ever
 * assigned any of them, and BLIND contradicted the tracker outright: the
 * wrapping pot is valid at every angle, so there is no blind zone to be in.
 *
 * FAULT deserves its own line, because it reads like the flag that says "the
 * base is unusable" and it is not that flag. That news travels in the STATUS
 * byte as ARM_ST_BASE_FAULT; base_st never carries 4. It never fed ArmHold
 * either - a base fault deliberately does NOT stop the five position joints,
 * which are healthy either way. Reaching for base_st == 4 as a fault test
 * costs a bench session and finds nothing, which is exactly what happened. */
#define ARM_BASE_UNKNOWN  0
#define ARM_BASE_OK       2

/* ---- base tracker (pure data, see the ARM_LOGIC block in arm.c) ----
 *
 * _ros_tooling/_gen_arm_test.py lifts these two definitions out of THIS
 * file, between the start of the first one and the closing line of the
 * second, so the host test compiles the real layout instead of a copy.
 * Three rules follow from that and are not style preferences:
 *   1. the ARM_BASE definition has to stay first and has to keep its
 *      opening brace on a line of its own, with the first member indented
 *      exactly four spaces - the slice is a literal substring search for
 *      that opening;
 *   2. the two definitions stay adjacent, with no other declaration
 *      between them;
 *   3. the last member of ARM_CAL must keep ARM_CAL as its closing name.
 * Word the comments below so that none of them reproduces those literal
 * anchors: a match inside a comment would be found FIRST, and the slice
 * would come out empty (that has already happened once while writing
 * this file - the anchor text was quoted in this very comment).
 */
typedef struct
{
    float deg;             /* absolute angle estimate, degrees            */
    s16   pot_raw;         /* last raw reading, for the 0x56 frame        */
    u8    state;
    u8    in_valid;
} ARM_BASE;

typedef struct
{
    float pot_deg_at_raw0;
    float pot_deg_per_raw;
    float pot_period_deg;      /* the wrap period, P (M1: 320.3 deg)     */
    s16   speed_1000_dps;
    s16   soft_limit_deg;
    float vel_kp;
    s16   vel_max;
} ARM_CAL;

/* ---- pure logic (host testable, ARM_LOGIC block) ---- */
float arm_pot_deg(const ARM_CAL *c, s16 raw);
void  arm_base_init(ARM_BASE *b);
void  arm_track_step(ARM_BASE *b, const ARM_CAL *c, u8 have_raw, s16 raw);
s16   arm_base_vel(const ARM_CAL *c, float err_deg, u16 t_ms);
float arm_field_to_deg(const ARM_CAL *c, s16 field);
s16   arm_deg_to_field(const ARM_CAL *c, float deg);
u16   arm_interp_ticks(u16 t_ms, u16 gap_ticks, u16 gap_max, u16 tick_ms);
float arm_interp_rate(float cur, float tgt, u16 n_ticks, float rate_max);
float arm_interp_step(float cur, float tgt, float rate);
u16   arm_servo_t(u16 gap_ticks, u16 tick_ms, u16 t_min, u16 t_max);
float arm_base_rate_max(const ARM_CAL *c, u16 rate_hz);
s16   arm_base_ff(const ARM_CAL *c, float field_per_tick, u16 rate_hz);

/* ---- task ---- */
extern ARM_BASE ArmBase;
extern ARM_CAL  ArmCal;
void arm_cal_load(void);
void arm_task(void *pvParameters);

#endif
