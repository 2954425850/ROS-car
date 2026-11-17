#include "arm.h"

/* ------------------------------------------------------------------
 * Joint tables. The order is the 0xAC field order: field i drives
 * servo ARM_JOINT_ID[i], and the fifth field (index 5) is the base,
 * which is the only joint that runs in motor mode.
 * ------------------------------------------------------------------ */
const u8  ARM_JOINT_ID[ARM_JOINTS]   = { 1, 2, 3, 4, 5, 6 };
const u16 ARM_RESET_POSE[ARM_JOINTS] = { 226, 500, 177, 129, 408, 500 };

ARM_BASE ArmBase;
ARM_CAL  ArmCal;

/* Copy arm_cal.h into the runtime struct once at task start. The logic
 * takes the whole calibration as an argument, so this is the only place
 * the macros are read - which is what lets the host test build the same
 * struct from the same header. */
void arm_cal_load(void)
{
    ArmCal.pot_deg_at_raw0    = ARM_POT_DEG_AT_RAW0;
    ArmCal.pot_deg_per_raw    = ARM_POT_DEG_PER_RAW;
    ArmCal.pot_period_deg     = ARM_POT_PERIOD_DEG;
    ArmCal.speed_1000_dps     = (s16)ARM_SPEED_1000_DPS;
    ArmCal.soft_limit_deg     = ARM_SOFT_LIMIT_DEG;
    ArmCal.vel_kp             = ARM_VEL_KP;
    ArmCal.vel_max            = ARM_VEL_MAX;
}

/* ==== ARM_LOGIC_BEGIN ==== */
/*
 * Base joint tracking. Deliberately pure: no registers, no globals, no
 * clock, every input an argument. The whole reason this is host testable
 * is that its failure modes (bad direction hint, sitting exactly on the
 * window edge, a noisy reading at the boundary) take tens of minutes to
 * reproduce on the bench and about a microsecond here.
 */

float arm_pot_deg(const ARM_CAL *c, s16 raw)
{
    return c->pot_deg_at_raw0 + c->pot_deg_per_raw * (float)raw;
}

/* Convert the 0xAC base field (-1000..+1000) to degrees and back. */
float arm_field_to_deg(const ARM_CAL *c, s16 field)
{
    return (float)field * (float)c->soft_limit_deg / 1000.0f;
}

s16 arm_deg_to_field(const ARM_CAL *c, float deg)
{
    float f = deg * 1000.0f / (float)c->soft_limit_deg;
    if(f >  1000.0f) f =  1000.0f;
    if(f < -1000.0f) f = -1000.0f;
    return (s16)f;
}

/* Reset the base tracker. The angle stays unknown until the boot anchor
 * accepts a reading inside the safe band; UNKNOWN keeps the control loop
 * out of the base until then. */
void arm_base_init(ARM_BASE *b)
{
    b->deg      = 0.0f;
    b->pot_raw  = 0;
    b->state    = ARM_BASE_UNKNOWN;
    b->in_valid = 0;
}


/* Boot anchor (#14 decision, revised): the anchoring accepts ANY valid
 * principal reading. The wrap ambiguity (a reading maps to true angles
 * one period apart) is handled by PROCEDURE - the task layer returns the
 * base to car front before power-off, so the boot position is the true
 * front - and is a documented, accepted residual for abnormal endings
 * (manual recovery). No band, no human prompt at boot: the state machine
 * anchors on the first good read, then boot centering drives to 0 deg.
 * The anchoring accepts any principal reading: the wrap ambiguity (which
 * lap the boot position is in) is handled by PROCEDURE, not by the band
 * - see the INIT_HOME block. */

/* Running tracker. The wrapping pot is valid at every angle (one reading
 * per period), so there is no blind zone and no integration: every fresh
 * reading is snapped onto the branch the estimate already sits on by
 * picking the congruent candidate nearest to it. The estimate can only
 * lose the branch by drifting further than half a period (160 deg) from
 * the truth between two ticks - 20 ms at the speed limit is 4 deg, a
 * ~40x margin. A bus miss simply holds the estimate; the next reading
 * re-snaps it. */
void arm_track_step(ARM_BASE *b, const ARM_CAL *c, u8 have_raw, s16 raw)
{
    float tau, d, half;

    if(b->state != ARM_BASE_OK) return;
    if(!have_raw) return;

    tau  = arm_pot_deg(c, raw);
    d    = tau - b->deg;
    half = 0.5f * c->pot_period_deg;
    while(d >  half) d -= c->pot_period_deg;
    while(d < -half) d += c->pot_period_deg;

    b->deg     += d;
    b->pot_raw  = raw;
    b->in_valid = 1;
}

/* Velocity loop. Point-to-point and streaming share it: a large t_ms
 * makes the feed-forward dominate and arrive on time, a small one clamps
 * it and leaves a plain proportional loop, which is what following wants. */
s16 arm_base_vel(const ARM_CAL *c, float err_deg, u16 t_ms)
{
    float v, ts;

    if(err_deg > -0.5f && err_deg < 0.5f) return 0;

    ts = (t_ms == 0) ? 0.5f : ((float)t_ms / 1000.0f);
    v  = err_deg / ts + c->vel_kp * err_deg;

    if(v >  (float)c->vel_max) v =  (float)c->vel_max;
    if(v < -(float)c->vel_max) v = -(float)c->vel_max;
    return (s16)v;
}

/* Setpoint interpolator (see arm.h). How many ticks a fresh frame's move
 * is spread over: its own t_ms (0 = the 500 ms default), stretched to the
 * measured gap since the previous frame when frames are streaming, plus
 * ONE tick of slack.
 *
 * The slack is the part that matters. Covering a streamed step in exactly
 * the gap lands the setpoint on the target just as the next frame is due,
 * so a frame one tick late (Python timers on the Pi do that all the time)
 * still means a one-tick stop. With n = gap + 1 the walk settles where one
 * tick's worth of distance is always left over - its fixed point is "rate
 * == the host's own speed" - and a late frame is absorbed with no stop, at
 * the price of one tick (20 ms) of extra lag. A lone point-to-point frame
 * (gap above gap_max) keeps exactly its own t_ms. */
u16 arm_interp_ticks(u16 t_ms, u16 gap_ticks, u16 gap_max, u16 tick_ms)
{
    u16 t = (t_ms == 0) ? 500u : t_ms;
    u16 n = (u16)((t + tick_ms / 2u) / tick_ms);
    if(gap_ticks <= gap_max)
    {
        if(gap_ticks > n) n = gap_ticks;
        n++;
    }
    if(n < 1u) n = 1u;
    return n;
}

/* Per-tick rate (units per tick, >= 0) that covers cur -> tgt in n ticks,
 * capped at rate_max. */
float arm_interp_rate(float cur, float tgt, u16 n_ticks, float rate_max)
{
    float d = tgt - cur;
    if(d < 0.0f) d = -d;
    if(n_ticks < 1u) n_ticks = 1u;
    d = d / (float)n_ticks;
    if(d > rate_max) d = rate_max;
    return d;
}

/* Move time for one per-joint send: the joint's own step interval (ticks
 * since its previous send) plus one tick of overlap, so a slow joint glides
 * from count to count instead of jumping and waiting. Clamped: never below
 * the two-tick overlap, and never so long that a joint waking from rest
 * answers sluggishly (the next send supersedes it anyway). */
u16 arm_servo_t(u16 gap_ticks, u16 tick_ms, u16 t_min, u16 t_max)
{
    u32 t = ((u32)gap_ticks + 1u) * (u32)tick_ms;
    if(t < t_min) t = t_min;
    if(t > t_max) t = t_max;
    return (u16)t;
}

/* One tick of the walk: move by at most rate, land exactly on tgt. */
float arm_interp_step(float cur, float tgt, float rate)
{
    float d = tgt - cur;
    if(d >  rate) return cur + rate;
    if(d < -rate) return cur - rate;
    return tgt;
}

/* The base setpoint may not outrun the speed the motor is allowed to
 * deliver (vel_max), in base-field units per tick. */
float arm_base_rate_max(const ARM_CAL *c, u16 rate_hz)
{
    float dps = (float)c->vel_max * (float)c->speed_1000_dps / 1000.0f;
    return dps * 1000.0f / (float)c->soft_limit_deg / (float)rate_hz;
}

/* Velocity feed-forward: the speed field that moves the base exactly as
 * fast as its setpoint moved this tick. With it the P loop only has to
 * correct the residual, instead of building up a following error of
 * rate/K degrees before the base keeps pace with the stick. */
s16 arm_base_ff(const ARM_CAL *c, float field_per_tick, u16 rate_hz)
{
    float dps = field_per_tick * (float)c->soft_limit_deg / 1000.0f * (float)rate_hz;
    float v   = dps * 1000.0f / (float)c->speed_1000_dps;
    if(v >  (float)c->vel_max) v =  (float)c->vel_max;
    if(v < -(float)c->vel_max) v = -(float)c->vel_max;
    return (s16)v;
}
/* ==== ARM_LOGIC_END ==== */

/* ---- bring-up state machine ---------------------------------------- */
#define INIT_WAIT       0
#define INIT_TORQUE     1
#define INIT_PROBE      2
#define INIT_CONFIG     3
#define INIT_POSE       4
#define INIT_HOME       5
#define INIT_READY      6
#define INIT_ABSENT     7

/* Latest command taken from the 0xAC frame, and the derived hold flags.
 * File scope rather than locals because the frame is published by an ISR
 * and copied under a critical section. Task 7 declared the next two; the
 * runtime intake block fills the whole set from RosArm inside ONE critical
 * section, so every one of them describes the SAME frame. */
static s16 ArmPos[ARM_JOINTS];   /* 0xAC fields: [0..4] joints, [5] base    */
static u16 ArmTime;              /* travel time of the newest frame, ms     */
static u8  ArmFlags;             /* flags byte of the newest frame          */
static u8  ArmFlagsPrev;         /* flags of the frame BEFORE it - the re-home
                                  * request is an EDGE, not a level: see the
                                  * runtime block. ArmFlags is a snapshot, so
                                  * it stays valid until the next 0xAC lands.  */
static s16 ArmTargetPos;      /* base field of the newest 0xAC frame, raw  */
static u8  ArmHold = 1;       /* derived hold flag - see the note below    */
static u8  ArmCanRehome;      /* may run a SEARCH - deliberately a WIDER
                               * set than ArmHold. A base fault holds the
                               * axis but must stay re-homeable: folding it
                               * in here would deadlock the one mechanism
                               * that can clear the fault, and the base
                               * would stay dead until a power cycle.    */
static u8  ArmChanged;        /* the 5-axis preset ran this tick, so 3c must
                               * trigger. Set in 3a and consumed in 3c,
                               * because the base block sits between them. */

/* ArmHold starts at 1 on purpose. Its initial value cannot survive into the
 * finished firmware - the runtime block recomputes it from the estop / link
 * state on every tick, before the base block runs - but "hold" is the only
 * safe reading for the interval between the task starting and the stage
 * machine reaching INIT_READY. With it at 0, a base that had just homed
 * would close the loop on the all-zero ArmTargetPos and slew hard to
 * field 0 before any 0xAC frame had ever arrived. */

/* ------------------------------------------------------------------
 * The one task that owns the servo bus. Every lx_* call in the firmware
 * originates here, from a single thread, so the half-duplex line is never
 * driven by two callers at once.
 *
 * Power-up order (design doc section 6): wait for the servo rail to come
 * up, bring the joints up one at a time, probe the bus, put servo 6 into
 * motor mode at speed 0 (motor mode is NOT saved across power cycles, so
 * this has to run every boot), walk the five position joints to the
 * factory pose, and only then home the base.
 * ------------------------------------------------------------------ */
void arm_task(void *pvParameters)
{
    u32 last = getSysTickCnt();
    u8  stage = INIT_WAIT;
    u8  retry = 0;
    u8  j     = 0;
#if ARM_CAL_MEASURED
    /* Only the boot-anchor path reads these, and that path is compiled out
     * when the calibration is unmeasured. Left unconditional they are
     * "set but not used" in an uncalibrated build - the same dead code
     * that ARMCC's #185 was complaining about, just one level down. */
    u8  anchor_ok   = 0;         /* in-band reads while waiting           */
    u16 anchor_miss = 0;         /* consecutive bus misses (dead servo)   */
    u16 center_ticks = 0;        /* ticks spent driving to car front      */
#endif
    u8  prev_stage     = 0xFF;   /* the stage at the top of the last tick */
    u32 t0    = getSysTickCnt();

    /* ---- runtime state (only touched once the stage machine is done) ---- */
    u16 tick       = 0;          /* ticks spent in INIT_READY              */
    u16 arm_fb_div = 0;          /* divides the 50Hz tick down to the uplink */
    u8  health_j   = 0;          /* round robin: one servo per second      */
    u8  probe_j    = 0;          /* round robin: two position joints/tick   */
    s16 last_p[ARM_JOINTS];      /* last PRESET target, i.e. what the servo
                                  * was actually told to walk to - not what
                                  * the newest frame asked for            */
    /* Setpoint interpolator (arm.h). [0..4] in servo counts, [5] in base
     * field units. tgt is the newest frame's (clamped) target, sp the
     * setpoint walking toward it, rate its per-tick step. INIT_POSE and
     * INIT_HOME seed all three wherever they leave the arm. */
    float sp[ARM_JOINTS];
    float tgt[ARM_JOINTS];
    float rate[ARM_JOINTS];
    float base_rate_max;
    u8    quiet[ARM_JOINTS - 1]; /* ticks since each joint was last sent  */
    s16   v_ff = 0;              /* base feed-forward, set in 3a, used in 3b */
    u8  cmd_seen    = 0;         /* a valid 0xAC has EVER been taken. Until it
                                  * has, ArmPos is still the zero-initialised
                                  * static, and "never commanded" must not be
                                  * read as "commanded to all zeros". The note
                                  * on ArmHold's initial value protects the
                                  * window BEFORE this loop; the runtime block
                                  * used to drop that protection the moment the
                                  * link came alive, which is exactly when the
                                  * host starts sending its 0xAA keepalive.  */
    u16 arm_age     = 999;       /* ticks since the last fresh 0xAC (hold detect) */
    u8  hold_stops  = 0;         /* MOVE_STOP ticks sent in this ArmHold      */
    u8  i;

    /* the signature is FreeRTOS's; this task reads no argument */
    (void)pvParameters;

    arm_cal_load();
    ArmBase.state = ARM_BASE_UNKNOWN;
    RosArmFb.status = 0;

    /* -1 is not a legal target (0..1000), so the first frame always counts
     * as a change even before any 0xAC has been seen. */
    for(i = 0; i < ARM_JOINTS; i++)
    {
        last_p[i] = -1;
        sp[i] = tgt[i] = (i < ARM_BASE_JOINT) ? (float)ARM_RESET_POSE[i] : 0.0f;
        rate[i] = 0.0f;
        if(i < ARM_BASE_JOINT) quiet[i] = 255u;
    }
    base_rate_max = arm_base_rate_max(&ArmCal, ARM_RATE_HZ);

    while(1)
    {
        vTaskDelayUntil(&last, F2T(ARM_RATE_HZ));

        /* Every entry into INIT_HOME re-arms the anchor bookkeeping.
         * anchor_ok/anchor_miss are task locals, so without this a second
         * attempt - the runtime block sets stage back to INIT_HOME on a
         * ROS_ARM_FLAG_REHOME - would still be carrying the counts from
         * the first one. The "known" bit goes too, but ONLY on the anchor
         * path: while the base is unanchored, ArmBase.deg is not an
         * absolute angle and must not be reported as one. A re-home over a
         * live tracker keeps both the estimate and the bit - see the state
         * test below. ArmBase itself is re-initialised by whoever sets the
         * stage, and then only when it is not already tracking.
         *
         * prev_stage is sampled BEFORE the switch, not after: sampling it
         * after would record the stage the switch just moved to, and this
         * entry would then never fire on the tick after the move. */
        if(stage == INIT_HOME && prev_stage != INIT_HOME)
        {
#if ARM_CAL_MEASURED
            anchor_ok   = 0;
            anchor_miss = 0;
            center_ticks = 0;
#endif
            RosArmFb.status |=  ARM_ST_HOMING;
            /* Only the anchor path may say "the angle is unknown". A
             * re-home over a live tracker keeps its accumulated estimate,
             * so the reported angle stays absolute for the whole return. */
            if(ArmBase.state != ARM_BASE_OK)
                RosArmFb.status &= (u8)~(ARM_ST_BASE_KNOWN | ARM_ST_BASE_FAULT);
        }
        /* An entry hook on READY used to live here, copying RosArmFb.pos[0..4]
         * into ArmPos[] so that the zero-initialised ArmPos could not be
         * preset. It is removed, for two independent reasons - and it was
         * never built, so nothing on the bench ever ran it.
         *
         * Its input was never the joint readback. Section 4, the only writer
         * of a real readback, sits behind the READY gate and has not run yet
         * on this tick, while the INIT_HOME path overwrites pos[0..4] with its
         * own instrumentation on every one of its ticks: bus frame count, bus
         * error count, the reply length, the replying servo's ID, and the base
         * pot raw. The seed therefore commanded the five joints to counter
         * values.
         *
         * And it re-armed on EVERY entry to READY, so a host-requested re-home
         * brought it back long after the first 0xAC had landed - outside the
         * reach of the cmd_seen gate, which by construction only covers the
         * pre-command window.
         *
         * The gate is what actually closes the hole. Before the first 0xAC,
         * 3a and 3b act on nothing, so there is nothing for a seed to protect.
         * Obsolete, not relocated. */
        prev_stage = stage;

        switch(stage)
        {
            case INIT_WAIT:
                if((u32)(getSysTickCnt() - t0) > 300u) { stage = INIT_TORQUE; j = 0; }
                break;

            case INIT_TORQUE:
                /* servo power comes up before the MCU; bring each joint up
                 * one at a time so a shorted winding cannot brown out the
                 * whole rail at once */
                if(j < ARM_JOINTS)
                {
                    lx_torque(ARM_JOINT_ID[j], 1);
                    j++;
                }
                else { stage = INIT_PROBE; retry = 0; }
                break;

            case INIT_PROBE:
            {
                /* reading ID1 is the presence test, same idea as the
                 * vendor firmware's bus-vs-PWM detection (spec 7.6) */
                u8 id = 0;
                if(lx_read_id(1, &id) == 0)
                {
                    RosArmFb.status |= ARM_ST_ONLINE;
                    stage = INIT_CONFIG;
                }
                else if(++retry >= 3)
                {
                    stage = INIT_ABSENT;
                }
                break;
            }

            case INIT_CONFIG:
                /* mode is NOT power-down saved - this must run every boot,
                 * and it must be speed 0 or the base takes off */
                lx_set_mode(6, LX_MODE_MOTOR, 0);
                stage = INIT_POSE;
                j = 0;
                break;

            case INIT_POSE:
                /* the five position joints walk to the factory pose
                 * together; the base is deliberately left alone. The move
                 * time scales with the ACTUAL travel: the joints are
                 * usually already near the reset pose, and a handful of
                 * counts spread over a fixed 2 s makes the servo step one
                 * count at a time - the visible "not silky" reset. A read
                 * failure assumes close but forces >= 1 s (conservative). */
                if(j == 0)
                {
                    u8 i;
                    u16 dmax = 0, tmove;
                    u8 rd_fail = 0;
                    for(i = 0; i < (ARM_JOINTS - 1); i++)
                    {
                        s16 cur = (s16)ARM_RESET_POSE[i];
                        s16 d;
                        if(lx_read_pos(ARM_JOINT_ID[i], &cur) != 0) rd_fail = 1;
                        d = (s16)ARM_RESET_POSE[i] - cur;
                        if(d < 0) d = (s16)-d;
                        if(d > dmax) dmax = (u16)d;
                    }
                    tmove = (u16)(dmax * 7u);        /* ~143 counts/s */
                    if(rd_fail != 0 && tmove < 1000u) tmove = 1000u;
                    if(tmove < 300u) tmove = 300u;
                    if(tmove > 2000u) tmove = 2000u;
                    for(i = 0; i < (ARM_JOINTS - 1); i++)
                        lx_move_wait(ARM_JOINT_ID[i], ARM_RESET_POSE[i], tmove);
                    lx_move_start_all();
                    j = 1;
                }
                else
                {
                    /* The preset above has just commanded the five joints to
                     * ARM_RESET_POSE, so that is where they are - and it has
                     * to become the shadow the runtime block compares against.
                     * The frame that REQUESTED this homing carries the pose
                     * from BEFORE it and is still sitting in ArmPos; left
                     * alone, the first READY tick finds ArmPos != last_p
                     * (they differ by the readback/command undershoot even in
                     * steady state, so this is not a corner case), presets,
                     * and drives all five joints back to the pre-homing pose
                     * in 500 ms. The homing undoes itself and looks like it
                     * never ran - the field report is exactly that. */
                    for(i = 0; i < (ARM_JOINTS - 1); i++)
                    {
                        ArmPos[i] = last_p[i] = (s16)ARM_RESET_POSE[i];
                        sp[i] = tgt[i] = (float)ARM_RESET_POSE[i];
                        rate[i] = 0.0f;
                        quiet[i] = 255u;
                    }
                    /* A re-home over a live tracker keeps the accumulated
                     * angle. INIT_HOME's anchor block is already skipped by
                     * its own state test, and the centring below then runs
                     * from this estimate instead of a fresh principal
                     * reading. Only a cold boot (UNKNOWN) wipes it. */
                    if(ArmBase.state != ARM_BASE_OK)
                        arm_base_init(&ArmBase);
                    RosArmFb.status |= ARM_ST_HOMING;
                    stage = INIT_HOME;
                }
                break;

            case INIT_HOME:
                /* The calibration is unmeasured until Task 10 replaces the
                 * placeholders in arm_cal.h. Closing the loop on them would
                 * produce a confident, WRONG angle - the arm would go where
                 * it thinks it is, which is worse than not going. Hold the
                 * base at zero speed and say so instead; Task 10 flips the
                 * macro after M1..M8.
                 *
                 * ArmBase.state goes to UNKNOWN and not to FAULT on purpose:
                 * a fault would set ArmHold and stop the five position joints
                 * as well, and those work perfectly well on an unmeasured
                 * calibration. The status BIT carries the "base unusable"
                 * news to the host instead. */
#if !ARM_CAL_MEASURED
                /* Compile time, not run time - ARM_CAL_MEASURED is a
                 * constant. Written as `if(!ARM_CAL_MEASURED)` this made
                 * every line below it dead code, and ARMCC said so:
                 *
                 *   arm.c(521): warning: #185-D: dynamic initialization in
                 *   unreachable code
                 *
                 * The warning was correct. #if removes the dead code
                 * rather than hiding the warning, and it also keeps the
                 * whole homing path out of the uncalibrated image. */
                lx_set_mode(6, LX_MODE_MOTOR, 0);
                ArmBase.state = ARM_BASE_UNKNOWN;
                RosArmFb.status &= (u8)~ARM_ST_HOMING;
                RosArmFb.status |=  ARM_ST_BASE_FAULT;
                stage = INIT_READY;
                break;
#else
            {
                /* Boot anchor (#14 decision, revised): the anchoring accepts
                 * ANY valid principal reading - the wrap ambiguity (a
                 * reading maps to true angles one period apart) is handled
                 * by PROCEDURE: the task layer returns the base to car
                 * front before power-off, so the boot reading is the true
                 * front. Abnormal endings can park the base one lap off -
                 * documented, accepted, manual recovery (record §8).
                 * The 150-tick in-band confirmation also lets the 2 s reset
                 * pose finish before the control loop starts (defect #13).
                 * After anchoring, BOOT CENTERING drives the base to car
                 * front (0 deg, up to half a period of travel) and only
                 * then releases control. */
                s16 raw = 0;
                u8  rd;
                float err;

                rd = lx_read_pos(6, &raw);

                if(rd != 0)
                {
                    /* ---- bus miss (~22% at 50 Hz, periodic - correlated
                     * with the 50 Hz chassis uplink TX): NOT evidence
                     * about where the base is. Hold the anchor count and
                     * the miss count is CONSECUTIVE misses only - after
                     * 10 s of pure failure the servo is dead: give up the
                     * base (five joints keep working). ---- */
                    if(++anchor_miss > 500)
                    {
                        lx_set_mode(6, LX_MODE_MOTOR, 0);
                        ArmBase.state = ARM_BASE_UNKNOWN;
                        RosArmFb.status |=  ARM_ST_BASE_FAULT;
                        RosArmFb.status &= (u8)~ARM_ST_BASE_KNOWN;
                        stage = INIT_READY;
                    }
                    /* pos[0..4] IS the 0x56 frame's five-joint field, and the
                     * host reads it as joint angles. Section 4 is the only
                     * writer of real ones and it sits behind the READY gate,
                     * so whatever is put here is published as "the joints"
                     * for the whole of INIT_HOME. It used to be the LxRx bus
                     * counters, and the host copies what it sees straight
                     * into its command targets - so a host holding an input
                     * across the homing would command the five joints to
                     * counter values the moment the gate opened.
                     *
                     * Report the documented "no reading" sentinel instead.
                     * It is also the honest answer: nothing is reading the
                     * joints during INIT_HOME. The bus counters were bring-up
                     * instrumentation for M1/M6; that calibration is done, and
                     * the 0xAD/0x57 passthrough is the live window for bus
                     * health. Nothing in the firmware ever READS this array -
                     * usartx.c only packs it into the frame. */
                    for(i = 0; i < (ARM_JOINTS - 1); i++)
                        RosArmFb.pos[i] = 0;
                    RosArmFb.pot_raw = -1;
                    break;
                }
                anchor_miss = 0;

                /* Same reason as the block above: for the duration of
                 * INIT_HOME this field is published as the five joint
                 * angles, so report the "no reading" sentinel rather than
                 * the LxRx bus counters. */
                for(i = 0; i < (ARM_JOINTS - 1); i++)
                    RosArmFb.pos[i] = 0;

                if(ArmBase.state != ARM_BASE_OK)
                {
                    /* ---- anchor confirmation: 150 in-band reads, misses
                     * hold the count. 3.8 s at the measured 78% success
                     * rate; also lets the 2 s reset pose finish before
                     * the control loop starts (defect #13). ---- */
                    if(++anchor_ok < ARM_ANCHOR_CONFIRM_TICKS) break;

                    ArmBase.deg      = arm_pot_deg(&ArmCal, raw);
                    ArmBase.pot_raw  = raw;
                    ArmBase.state    = ARM_BASE_OK;
                    ArmBase.in_valid = 1;
                    lx_torque(6, 1);
                    RosArmFb.anchor  = 0;
                    RosArmFb.pot_raw    = raw;
                    RosArmFb.base_state = ARM_BASE_OK;
                    RosArmFb.status &= (u8)~ARM_ST_HOMING;
                    RosArmFb.status &= (u8)~ARM_ST_BASE_FAULT;
                    RosArmFb.status |=  ARM_ST_BASE_KNOWN;
                    center_ticks = 0;
                    break;                       /* centering starts next tick */
                }

                if(ArmBase.state != ARM_BASE_OK)
                {
                    /* ---- anchor confirmation: 150 in-band reads. 3 s,
                     * which also lets the 2 s reset pose finish before the
                     * control loop starts (defect #13). ---- */
                    if(++anchor_ok < ARM_ANCHOR_CONFIRM_TICKS) break;

                    ArmBase.deg      = arm_pot_deg(&ArmCal, raw);
                    ArmBase.pot_raw  = raw;
                    ArmBase.state    = ARM_BASE_OK;
                    ArmBase.in_valid = 1;
                    lx_torque(6, 1);
                    RosArmFb.anchor  = 0;
                    RosArmFb.pot_raw    = raw;
                    RosArmFb.base_state = ARM_BASE_OK;
                    RosArmFb.status &= (u8)~ARM_ST_HOMING;
                    RosArmFb.status &= (u8)~ARM_ST_BASE_FAULT;
                    RosArmFb.status |=  ARM_ST_BASE_KNOWN;
                    center_ticks = 0;
                    break;                       /* centering starts next tick */
                }

                /* ---- boot centering: drive to car front (0 deg). The move
                 * is bounded by half a period (160.15 deg) and the tracker
                 * keeps the estimate continuous across any wrap crossings
                 * beyond it; the velocity loop is the same one the runtime
                 * uses. The commanded speed is RAMPED (a step here made the
                 * base jerk into motion); timeout: release control anyway -
                 * the residual (friction stall) is documented and the host
                 * can fine-command. ---- */
                arm_track_step(&ArmBase, &ArmCal, 1, raw);
                err = -ArmBase.deg;
                if(err < 0.0f) err = -err;

                center_ticks++;
                if(err <= ARM_CENTER_TOL_DEG || center_ticks > ARM_CENTER_TIMEOUT_TICKS)
                {
                    lx_set_mode(6, LX_MODE_MOTOR, 0);
                    /* The anchor path clears this when it anchors, well
                     * before centring; the keep path skips that block, so
                     * the return has to clear it here or it stays set. */
                    RosArmFb.status &= (u8)~ARM_ST_HOMING;
                    /* Same reasoning as the five joints in INIT_POSE: the
                     * shadow base target has to be where the homing LEFT the
                     * base, not the field the re-home frame asked for. That
                     * frame still reads ArmPos[base], and ArmTargetPos is
                     * reloaded from it on every runtime tick, so without this
                     * the base turns the whole way back to its pre-homing
                     * angle and the centring is cancelled - the base half of
                     * the same "homing undoes itself" report. */
                    ArmPos[ARM_BASE_JOINT] = arm_deg_to_field(&ArmCal, ArmBase.deg);
                    /* and the interpolator starts from there, standing */
                    sp[ARM_BASE_JOINT] = tgt[ARM_BASE_JOINT] =
                        (float)ArmPos[ARM_BASE_JOINT];
                    rate[ARM_BASE_JOINT] = 0.0f;
                    RosArmFb.pot_raw    = raw;
                    RosArmFb.base_state = ARM_BASE_OK;
                    stage = INIT_READY;
                    break;
                }
                {
                    s16 v_cmd = arm_base_vel(&ArmCal, -ArmBase.deg, 500);
                    s16 ramp  = (s16)(center_ticks * ARM_CENTER_RAMP_STEP);
                    if(v_cmd >  ramp) v_cmd =  ramp;
                    if(v_cmd < -ramp) v_cmd = -ramp;
                    lx_set_mode(6, LX_MODE_MOTOR, v_cmd);
                }
                RosArmFb.pot_raw    = raw;
                RosArmFb.base_state = ARM_BASE_OK;
                break;
            }
#endif

            case INIT_READY:
                /* Control is live - and this case has to exist, because
                 * without it READY falls through to the default below, which
                 * clears ONLINE on EVERY 20 ms tick. The bit INIT_PROBE set
                 * (correctly) is therefore wiped fifty times a second and the
                 * host never sees it. That is the "烧录版固件从不置 ONLINE
                 * 位" mystery written into the launch file, and it is why
                 * require_online had to be set False - which in turn disables
                 * the host's own "do not command the arm until it is ready"
                 * gate. Nothing in the firmware reads ARM_ST_ONLINE; it exists
                 * for the host, so it has to survive the runtime. */
                break;

            case INIT_ABSENT:
            default:
                RosArmFb.status &= (u8)~ARM_ST_ONLINE;
                break;
        }

        /* ----------------------------------------------------------------
         * Runtime. The stage machine is finished with the power-up
         * sequence; from here on every tick is the 50Hz control loop.
         *
         * The order below is protocol spec 7.9 and it is not free to change:
         *   3a preset the five position joints
         *   3b send the base speed        <- the ARM_BASE_TICK block
         *   3c broadcast MOVE_START
         * A motor-mode servo starts the instant its speed frame lands, while
         * the other five wait for the broadcast, so the base command has to
         * be on the wire before the trigger or the six axes start a frame
         * apart (~1.4ms). _gen_arm_test.py asserts this order on the real
         * bytes of this file.
         * ---------------------------------------------------------------- */
        /* ------------------------------------------------------------
         * Reporting and safety run from BOOT, not only from INIT_READY:
         * during a human-anchor wait the host must still see the arm
         * (0x56 with byte 19 set), and the passthrough is the bench's
         * window onto the pot. Only CONTROL (the sections after the
         * gate) waits for READY.
         *
         * 6. bring-up passthrough (Task 4), compiled out of shipped
         *    builds. It rides the 0xAD/0x57 frames, which is how the
         *    bench drives the servo bus by proxy.
         * ------------------------------------------------------------ */
        if(RosFault & ROS_FAULT_ESTOP) RosArmFb.status |=  ARM_ST_ESTOP;
        else                           RosArmFb.status &= (u8)~ARM_ST_ESTOP;

#if ARM_DEBUG_PASSTHROUGH
        if(RosRaw.rx_ready)
        {
            u8  raw[ROS_RAW_MAX_PAYLOAD];
            u8  n, tmo;
            u16 tmo_ms;
            u8  rx[ROS_RAWFB_SIZE - 8];

            taskENTER_CRITICAL();
            n       = RosRaw.n;
            tmo_ms  = RosRaw.timeout_ms;
            for(tmo = 0; tmo < n; tmo++) raw[tmo] = RosRaw.raw[tmo];
            RosRaw.rx_ready = 0;
            taskEXIT_CRITICAL();

            tmo = (u8)lx_raw_exchange(raw, n, rx, (u8)sizeof(rx), tmo_ms);
            ros_send_raw_fb(rx, tmo);
        }
#endif

        /* byte 19 of the 0x56 frame: "the control loop is live".
         *
         * It was a legacy boot-anchor flag that nothing ever set, so it went
         * out as a constant 0. The host wants exactly this bit and nothing
         * else. During INIT_POSE / INIT_HOME the firmware does not consume
         * 0xAC frames at all, so a frame the host sends then just sits in the
         * mailbox and is applied as "the newest command" the instant the
         * stage machine reaches READY - which is how a re-home could undo
         * itself. With this bit the host can hold its arm commands until the
         * loop is actually running, instead of guessing a fixed number of
         * seconds and hoping the homing finished inside it.
         *
         * Set here, before the send, and in the block that runs from BOOT
         * rather than from READY - running it only at READY would be the one
         * place the host can already tell. */
        RosArmFb.anchor = (u8)(stage == INIT_READY);

        /* Before READY this is the only uplink. At READY section 7 at the
         * bottom of the tick sends it instead (with this tick's readback),
         * and this one must stand aside: both used to run, each bumping
         * arm_fb_div, so READY sent 0x56 on EVERY tick (50Hz, not 25Hz)
         * and spent ~2 ms of the arm task's 20 ms on the duplicate. */
        if(stage != INIT_READY && ++arm_fb_div >= (ARM_RATE_HZ / 20))
        {
            arm_fb_div = 0;
            ros_send_arm_fb();
        }

        if(stage != INIT_READY) continue;      /* control waits for READY */
        tick++;

        /* ------------------------------------------------------------
         * 1. take the newest 0xAC frame
         *
         * One critical section, one snapshot. The USART1 ISR publishes a
         * whole frame under its own single publish flag, so copying all of
         * it between taskENTER_CRITICAL() and taskEXIT_CRITICAL() is what
         * keeps ArmPos / ArmTime / ArmFlags describing the SAME frame -
         * read field by field outside the section, a new frame landing
         * midway would have the base block close the loop on one frame's
         * target with another frame's travel time.
         * ---------------------------------------------------------- */
        {
            u8 fresh = 0;
            if(RosArm.rx_ready)
            {
                taskENTER_CRITICAL();
                for(i = 0; i < ARM_JOINTS; i++) ArmPos[i] = RosArm.pos[i];
                ArmTime  = RosArm.t_ms;
                ArmFlags = RosArm.flags;
                RosArm.rx_ready = 0;
                fresh = 1;
                taskEXIT_CRITICAL();
            }
            if(fresh)
            {
                /* arm_age is the ticks since the previous frame minus one,
                 * so the gap between the two frames is arm_age + 1 */
                u16 n = arm_interp_ticks(ArmTime, (u16)(arm_age + 1u),
                                         ARM_STREAM_GAP_MAX,
                                         (u16)(1000u / ARM_RATE_HZ));
                for(i = 0; i < ARM_JOINTS; i++)
                {
                    float t, rmax;
                    if(i < ARM_BASE_JOINT)
                    {
                        /* Two-sided clamp, protocol spec 11.4: the vendor
                         * firmware clipped the upper end only, so a negative
                         * target ran to the far stop. The test is done on
                         * the signed value on purpose - casting first turns
                         * -1 into 65535, i.e. the wrong end of the joint. */
                        s16 p = ArmPos[i];
                        if(p < 0)    p = 0;
                        if(p > 1000) p = 1000;
                        if(i == ARM_GRIPPER_FIELD && p > ARM_GRIPPER_MAX)
                            p = ARM_GRIPPER_MAX;          /* stall protection */
                        t    = (float)p;
                        rmax = ARM_JOINT_VMAX / (float)ARM_RATE_HZ;
                    }
                    else
                    {
                        s16 p = ArmPos[i];
                        if(p >  1000) p =  1000;
                        if(p < -1000) p = -1000;
                        t    = (float)p;
                        rmax = base_rate_max;
                    }
                    tgt[i]  = t;
                    rate[i] = arm_interp_rate(sp[i], t, n, rmax);
                }
                arm_age = 0;
                cmd_seen = 1;
            }
            else if(arm_age < 60000) arm_age++;
        }

        /* ------------------------------------------------------------
         * 2. TWO separate gates, and they are not the same set
         *
         * ArmHold       = stop moving. Set by estop, link loss, or arm not
         *                 enabled.
         * ArmCanRehome  = allowed to run a homing SEARCH. Set by the same
         *                 three.
         *
         * A base fault is in NEITHER, and that is deliberate on both counts.
         * It must not stop the five position joints - they work perfectly
         * well with a dead base - and it must not block the one mechanism
         * that can clear it, or a faulted base could never be re-homed and
         * would stay dead until a power cycle. So a fault stops only the
         * base, and it does that through the state gate in 3b (ArmBase.state
         * is UNKNOWN there, so the whole block is skipped), while the STATUS
         * byte carries ARM_ST_BASE_FAULT to the host.
         *
         * There used to be an `if(ArmBase.state == ARM_BASE_FAULT)` line
         * here. Nothing can produce that state value, so it was a do-nothing
         * that read like a safety interlock - see the note in arm.h.
         * ---------------------------------------------------------- */
        ArmHold      = 0;
        ArmCanRehome = 1;
        if(RosFault & ROS_FAULT_ESTOP)         { ArmHold = 1; ArmCanRehome = 0; }
        if(RosFault & ROS_FAULT_TIMEOUT)       { ArmHold = 1; ArmCanRehome = 0; }
        if(!(RosDown.flags & ROS_FLAG_ENABLE)) { ArmHold = 1; ArmCanRehome = 0; }

        if(ArmHold) RosArmFb.status |=  ARM_ST_ESTOP;
        else        RosArmFb.status &= (u8)~ARM_ST_ESTOP;

        /* re-home on request, gated on ArmCanRehome - NOT on ArmHold.
         *
         * EDGE triggered, deliberately. ArmFlags is a frame SNAPSHOT, so a
         * host that holds bit0 set would otherwise re-home again the
         * instant each search finished - an endless homing loop with the
         * base sweeping the whole budget every few seconds.
         *
         * The edge has to be consumed BEFORE the `continue`, not after the
         * if. `continue` jumps back to the top of while(1), so a consuming
         * assignment placed after the if is skipped on exactly the path
         * that triggers the search: ArmFlagsPrev stays clear, ArmFlags
         * still has bit0 set (nothing clears it - it is a snapshot), and
         * the same edge fires again the moment the search it started ends.
         * That ordering is the endless loop this whole block exists to
         * prevent, so it is not a style choice. */
        {
            u8 rehome = (u8)((ArmFlags & ROS_ARM_FLAG_REHOME) &&
                             !(ArmFlagsPrev & ROS_ARM_FLAG_REHOME));
            ArmFlagsPrev = ArmFlags;    /* consume the edge either way */

            if(rehome && ArmCanRehome)
            {
                /* A re-home runs the SAME stage machine as a power-on: enter
                 * INIT_POSE, which walks the five position joints to
                 * ARM_RESET_POSE and then hands over to INIT_HOME for the
                 * base anchor and the drive to car front. One homing routine,
                 * one meaning, whoever asks for it - the 0xAC re-home bit is
                 * the interface, and this is what it now does.
                 *
                 * j = 0 is the whole trick: INIT_POSE's first tick takes the
                 * readbacks and does the preset, its second hands over to
                 * INIT_HOME. Jumping straight to INIT_HOME - what this used
                 * to do - re-anchored the base only and left the five joints
                 * wherever they happened to be, so "home" meant two different
                 * things depending on who asked.
                 *
                 * arm_base_init is deliberately NOT called here any more:
                 * INIT_POSE calls it on its own path, and a third copy only
                 * gives them a way to drift apart. It is conditional there:
                 * a re-home over a live tracker keeps the accumulated angle,
                 * and INIT_HOME's own state test skips the re-anchor. */
                RosArmFb.status |= ARM_ST_HOMING;
                RosArmFb.status &= (u8)~ARM_ST_BASE_FAULT;
                stage = INIT_POSE;
                j = 0;
                continue;
            }
        }

        /* ------------------------------------------------------------
         * 3a. WALK the setpoints one tick, then PRESET the five position
         *     joints - only when a rounded setpoint actually moved, which
         *     is what keeps the bus free once the arm has arrived. The
         *     trigger is deliberately NOT here: see 3b.
         *
         * The frame is no longer sent as-is (see the interpolator note in
         * arm.h): what goes on the wire is the per-tick setpoint, with a
         * fixed two-tick move time, so consecutive moves overlap and the
         * servo never arrives-and-stops between frames.
         *
         * Under ArmHold the setpoints freeze where they are. The servos are
         * stopped by 3c and the base gets speed 0 in 3b; on release the
         * walk resumes from the frozen setpoint instead of jumping.
         *
         * cmd_seen: before the first 0xAC there is no target at all. The
         * setpoints sit on the reset pose INIT_POSE seeded, so nothing would
         * move anyway, but the gate says it in so many words: "never
         * commanded" is not "commanded to the reset pose".
         * ---------------------------------------------------------- */
        {
            u8  changed = 0;
            s16 q[ARM_JOINTS - 1];

            v_ff = 0;
            if(cmd_seen && !ArmHold)
            {
                for(i = 0; i < ARM_JOINTS; i++)
                {
                    float prev = sp[i];
                    sp[i] = arm_interp_step(sp[i], tgt[i], rate[i]);
                    if(i == ARM_BASE_JOINT)
                        v_ff = arm_base_ff(&ArmCal, sp[i] - prev, ARM_RATE_HZ);
                }
            }
            for(i = 0; i < (ARM_JOINTS - 1); i++)
            {
                q[i] = (s16)(sp[i] + 0.5f);   /* sp is 0..1000, never negative */
                if(q[i] != last_p[i]) changed = 1;
            }

            /* ArmChanged is gated on cmd_seen and !ArmHold here, not only at
             * the preset: 3c consumes it to broadcast MOVE_START and update
             * last_p[], so leaving it set with nothing preset would put a
             * trigger on the wire with no move behind it and poison last_p[]
             * with targets no servo was ever given. */
            ArmChanged = (u8)(changed && cmd_seen && !ArmHold);

            /* Per joint, and only the joints whose rounded setpoint moved
             * (2026-10-01, second field report: grainy, and the loaded
             * shoulder/elbow wobble). Two things changed here:
             *
             * - A joint that is standing still is no longer re-commanded
             *   every tick just because ANOTHER joint moved. Each new move
             *   restarts the servo's own profile from wherever gravity has
             *   let it sag, which is a 50Hz kick on exactly the joints that
             *   carry the load.
             * - The move time follows the joint's real step interval. A slow
             *   joint whose count only changes every few ticks used to get a
             *   40 ms move, then stand still until the next count: one
             *   0.24 deg tick-tock per step, the "grain". Now it gets the
             *   interval it actually has (+1 tick of overlap), so it glides. */
            if(ArmChanged)
            {
                for(i = 0; i < (ARM_JOINTS - 1); i++)
                {
                    if(q[i] == last_p[i])
                    {
                        if(quiet[i] < 255u) quiet[i]++;
                        continue;
                    }
                    lx_move_wait(ARM_JOINT_ID[i], (u16)q[i],
                                 arm_servo_t((u16)(quiet[i] + 1u),
                                             (u16)(1000u / ARM_RATE_HZ),
                                             ARM_INTERP_T_MS, ARM_SERVO_T_MAX));
                    last_p[i] = q[i];
                    quiet[i]  = 0;
                }
            }
            else
            {
                for(i = 0; i < (ARM_JOINTS - 1); i++)
                    if(quiet[i] < 255u) quiet[i]++;
            }
        }

        /* the base loop closes on the walking setpoint, not the raw frame */
        if(sp[ARM_BASE_JOINT] >= 0.0f)
            ArmTargetPos = (s16)(sp[ARM_BASE_JOINT] + 0.5f);
        else
            ArmTargetPos = (s16)(sp[ARM_BASE_JOINT] - 0.5f);

        /* ------------------------------------------------------------
         * 3b. THE BASE SPEED GOES EXACTLY HERE - between the preset and
         *     the trigger. Protocol spec 7.9: preset the five joints, then
         *     send the base speed, then broadcast the trigger.
         *
         * A motor-mode servo starts the instant its speed frame lands,
         * while the other five wait for MOVE_START. Sending the base speed
         * after the broadcast staggers the six axes by one mode frame plus
         * the broadcast - about 1.4ms at 115200, visible on a 450deg slew.
         *
         * This block is a 50Hz velocity loop: it runs EVERY tick and must
         * not be moved inside the "changed" gate of 3a. 3c consumes
         * ArmChanged, which is what remembers whether 3a preset anything.
         * ---------------------------------------------------------- */
        /* ==== ARM_BASE_TICK_BEGIN ==== */
        /* ---- base joint: track, then close the loop ----
         *
         * The two markers around this block are a slice boundary:
         * _gen_arm_test.py lifts the block out of this file and asserts the
         * call order below on the real bytes, because arm_task itself cannot
         * be host compiled. Keep them.
         *
         * ORDERING, protocol spec 7.9: preset the five joints, THEN send the
         * base speed, THEN broadcast MOVE_START. A motor-mode servo starts
         * the instant its speed command lands, the other five start on the
         * broadcast - so the base command has to be on the wire before the
         * broadcast, or the six axes start a frame apart. 3a presets, this
         * block is 3b, 3c broadcasts; _gen_arm_test.py asserts that order
         * on the bytes of this file.
         */
        if(ArmBase.state == ARM_BASE_OK)
        {
            u8   have_raw = 0;
            u8   in_hold;
            s16  raw      = 0;
            float err;
            s16  v_cmd;
            u16  t_eff;

            /* the wrapping pot is valid at every angle - read it every
             * tick, no probe gating (see arm_track_step) */
            if(lx_read_pos(6, &raw) == 0) have_raw = 1;
            else raw = 0;

            arm_track_step(&ArmBase, &ArmCal, have_raw, raw);

            /* ArmTime, not RosArm.t_ms: the intake took a snapshot of the
             * whole frame under one critical section, and reading the live
             * mailbox here could pair THIS frame's target with a NEWER
             * frame's travel time. The 5-axis preset uses the same
             * snapshot, so both paths drive one frame's worth of intent. */
            /* Hold-vs-stream: while 0xAC frames flow, run the loop exactly
             * as acceptance-validated (raw angle, frame t_ms). Once frames
             * stop, run it as a HOLD with a hold t_ms, so the feed-forward
             * err/ts stops running at the stale streaming t_ms. Invisible
             * while the operator is driving.
             *
             * The angle itself is NOT low-passed in either state. That was
             * tried on the bench and dropped: the base pot's measured noise
             * floor is one count (0.23 deg), with the reading often not
             * moving at all between ticks, so there is nothing to filter -
             * while a 57 ms lag in the feedback path spends phase margin the
             * loop does not have to spare, and shortening hold t_ms is what
             * actually damps the hunt. One lever at a time also keeps a
             * bench result attributable to the change that produced it. */
            /* With the interpolator the setpoint keeps walking after the
             * last frame (a point-to-point frame spreads over its whole
             * t_ms), so "hold" also needs the setpoint to have ARRIVED.
             * While it is walking, the loop runs at the validated streaming
             * t_ms: the timing is the interpolator's job now, the feed-
             * forward v_ff carries the setpoint's own speed, and the P part
             * only corrects what is left. */
            in_hold = (arm_age > ARM_STREAM_AGE_TICKS) &&
                      (sp[ARM_BASE_JOINT] == tgt[ARM_BASE_JOINT]);
            err   = arm_field_to_deg(&ArmCal, ArmTargetPos) - ArmBase.deg;
            t_eff = in_hold ? (u16)ARM_HOLD_T_MS : (u16)ARM_TRACK_T_MS;
            /* cmd_seen: same reason as 3a. Before the first 0xAC,
             * ArmTargetPos is the zero-initialised ArmPos[base], i.e. "drive
             * to car front" - a command nobody gave. Hold the axis at zero
             * speed instead; INIT_HOME has already parked the base. */
            v_cmd = (ArmHold || !cmd_seen) ? 0 : arm_base_vel(&ArmCal, err, t_eff);
            if(!ArmHold && cmd_seen)
            {
                s32 v = (s32)v_cmd + (s32)v_ff;
                if(v >  ArmCal.vel_max) v =  ArmCal.vel_max;
                if(v < -ArmCal.vel_max) v = -ArmCal.vel_max;
                v_cmd = (s16)v;
            }

            lx_set_mode(6, LX_MODE_MOTOR, v_cmd);

            RosArmFb.pot_raw    = have_raw ? raw : RosArmFb.pot_raw;
            RosArmFb.base_state = ArmBase.state;
        }
        /* ==== ARM_BASE_TICK_END ==== */

        /* ------------------------------------------------------------
         * 3c. TRIGGER the five position joints.
         *
         * ArmChanged is consumed rather than re-derived: it was computed
         * in 3a, and the base block above sits between the two, so it is
         * also what keeps the trigger and the preset describing the same
         * decision. last_p[] was already updated by 3a's preset.
         * ---------------------------------------------------------- */
        if(ArmChanged && !ArmHold)
        {
            lx_move_start_all();
        }
        else if(ArmHold)
        {
            /* Stop on entering the hold, then leave the servos alone - see
             * ARM_HOLD_STOP_TICKS: repeating the stop every tick walks a
             * loaded joint down. */
            if(hold_stops < ARM_HOLD_STOP_TICKS)
            {
                for(i = 0; i < (ARM_JOINTS - 1); i++)
                    lx_move_stop(ARM_JOINT_ID[i]);
                hold_stops++;
            }
            /* MOVE_STOP does nothing to a motor-mode servo - the base
             * needs an explicit speed 0, which 3b sends */
        }
        if(!ArmHold) hold_stops = 0;

        /* ------------------------------------------------------------
         * 4. readback: two position joints per tick, round robin
         *
         * A timeout is reported as 0 rather than as the previous value: 0
         * is outside the mechanical travel of every joint (125..875), so
         * the host can tell "no reading" from "at the low stop". The base
         * is not in the rotation - 3b already reported it.
         * ---------------------------------------------------------- */
        {
            u8 k;
            for(k = 0; k < 2; k++)
            {
                u8 idx = probe_j;
                s16 p  = 0;
                if(idx < (ARM_JOINTS - 1) && lx_read_pos(ARM_JOINT_ID[idx], &p) == 0)
                    RosArmFb.pos[idx] = p;
                else if(idx < (ARM_JOINTS - 1))
                    RosArmFb.pos[idx] = 0;      /* 0 = "no reading" */
                probe_j = (u8)((probe_j + 1) % (ARM_JOINTS - 1));
            }
        }
        RosArmFb.pos[ARM_BASE_JOINT] = arm_deg_to_field(&ArmCal, ArmBase.deg);

        /* ------------------------------------------------------------
         * 5. health: one servo per second, round robin
         *
         * Read and reported, not acted on. ARM_ST_OVERTEMP / ARM_ST_
         * UNDERVOLT tell the host what the servos think; the firmware
         * keeps the joints where they are. That is deliberate - the
         * design doc section 9 stops for an estop or a command timeout,
         * not for a servo's own temperature report.
         * ---------------------------------------------------------- */
        if((tick % ARM_RATE_HZ) == 0)
        {
            u16 mv = 0;
            u8  tc_ = 0;
            if(lx_read_vin(ARM_JOINT_ID[health_j], &mv) == 0)
            {
                if(mv < ARM_VIN_MIN_MV) RosArmFb.status |=  ARM_ST_UNDERVOLT;
                else                    RosArmFb.status &= (u8)~ARM_ST_UNDERVOLT;
            }
            if(lx_read_temp(ARM_JOINT_ID[health_j], &tc_) == 0)
            {
                if(tc_ > ARM_TEMP_MAX_C) RosArmFb.status |=  ARM_ST_OVERTEMP;
                else                     RosArmFb.status &= (u8)~ARM_ST_OVERTEMP;
            }
            health_j = (u8)((health_j + 1) % ARM_JOINTS);
        }

        /* ------------------------------------------------------------
         * 7. the 0x56 uplink
         *
         * The divisor is ARM_RATE_HZ/20, which is 2 at 50Hz - an integer
         * divisor cannot hit 20Hz exactly, so this reports every second
         * tick, i.e. 25Hz. The design doc asks for "20Hz" and erring
         * upward is the safe direction for the host's freshness check.
         * ArmBase.deg is reported here, so the frame is only as absolute
         * as the status bits say it is: ARM_ST_BASE_KNOWN is the flag
         * that makes it one.
         * ---------------------------------------------------------- */
        if(++arm_fb_div >= (ARM_RATE_HZ / 20))
        {
            arm_fb_div = 0;
            ros_send_arm_fb();
        }
    }
}
