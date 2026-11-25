#include "ws2811.h"
#include "ws_link.h"

/* ==== WS_LOGIC_BEGIN ==== */
/* Nothing below this marker may include an STM32 header or touch a global
 * owned by another module. The host test harness slices this region out
 * verbatim and compiles it against plain stubs, so keep it parametric. */

/**************************************************************************
Function: HSV -> RGB, integer only, no libm dependency.
Input   : h 0..359, s 0..255, v 0..255
Output  : *r *g *b 0..255
**************************************************************************/
void ws_hsv2rgb(u16 h, u8 s, u8 v, u8 *r, u8 *g, u8 *b)
{
	u8  region, rem;
	u16 p, q, t;

	if (h >= 360) h %= 360;
	region = (u8)(h / 60);          /* 0..5 */
	rem    = (u8)(h % 60);          /* 0..59 */

	p = (u16)((u32)v * (255 - s) / 255);
	q = (u16)((u32)v * (255 - (u32)s * rem / 60) / 255);
	t = (u16)((u32)v * (255 - (u32)s * (60 - rem) / 60) / 255);

	switch (region) {
	case 0:  *r = (u8)v; *g = (u8)t; *b = (u8)p; break;
	case 1:  *r = (u8)q; *g = (u8)v; *b = (u8)p; break;
	case 2:  *r = (u8)p; *g = (u8)v; *b = (u8)t; break;
	case 3:  *r = (u8)p; *g = (u8)q; *b = (u8)v; break;
	case 4:  *r = (u8)t; *g = (u8)p; *b = (u8)v; break;
	default: *r = (u8)v; *g = (u8)p; *b = (u8)q; break;
	}
}
/* Speed -> (hue, brightness) breakpoints. The first segment is deliberate:
 * below 0.10 m/s the hue is LOCKED at 160 deg and only brightness moves,
 * because hue alone is invisible in the cyan->green corner while the
 * operator is creeping off the line. See design doc section 4.1. */
static const float ws_v_tab[6] = {0.02f, 0.10f, 0.30f, 0.50f, 0.65f, 0.75f};
static const u16   ws_h_tab[6] = {160,   160,   120,   60,    30,    0    };
static const u8    ws_b_tab[6] = {89,    191,   230,   255,   255,   255  };
/* 89 = 0.35*255, 191 = 0.75*255, 230 = 0.90*255 */

WS_SpeedOut ws_speed_map(float mps)
{
	WS_SpeedOut o;
	u8 i;
	float t;

	if (mps < 0.0f) mps = -mps;

	if (mps <= ws_v_tab[0]) {
		o.hue = ws_h_tab[0]; o.bright = ws_b_tab[0]; return o;
	}
	if (mps >= ws_v_tab[5]) {
		o.hue = ws_h_tab[5]; o.bright = ws_b_tab[5]; return o;
	}
	for (i = 0; i < 5; i++)
		if (mps < ws_v_tab[i + 1]) break;

	t = (mps - ws_v_tab[i]) / (ws_v_tab[i + 1] - ws_v_tab[i]);
	o.hue    = (u16)((float)ws_h_tab[i] +
	                 t * ((float)ws_h_tab[i + 1] - (float)ws_h_tab[i]));
	o.bright = (u8)((float)ws_b_tab[i] +
	                t * ((float)ws_b_tab[i + 1] - (float)ws_b_tab[i]));
	return o;
}
#define WS_STEP_MS        40
#define WS_ENTER_MPS      0.03f
#define WS_EXIT_MPS       0.015f
#define WS_ENTER_HOLD_MS  100
#define WS_EXIT_HOLD_MS   500
#define WS_FADE_MS        300
/* 255 * 40 / 300 = 34 -> eight steps to cross, 320ms nominal */
#define WS_FADE_STEP      ((u8)(255 * WS_STEP_MS / WS_FADE_MS))
/* first-order LPF: alpha = dt / (tau + dt), tau = 100ms */
#define WS_LPF_ALPHA      (40.0f / 140.0f)

void ws_state_init(WS_State *st)
{
	st->filt     = 0.0f;
	st->moving   = 0;
	st->blend    = 0;
	st->start_ms = 0;
	st->stop_ms  = 0;
}

void ws_state_step(WS_State *st, float v_abs)
{
	if (v_abs < 0.0f) v_abs = -v_abs;
	st->filt += (v_abs - st->filt) * WS_LPF_ALPHA;

	if (!st->moving) {
		if (st->filt > WS_ENTER_MPS) {
			st->start_ms += WS_STEP_MS;
			if (st->start_ms >= WS_ENTER_HOLD_MS) {
				st->moving  = 1;
				st->stop_ms = 0;
			}
		} else {
			st->start_ms = 0;
		}
		st->blend = (st->blend > WS_FADE_STEP)
		          ? (u8)(st->blend - WS_FADE_STEP) : 0;
	} else {
		if (st->filt < WS_EXIT_MPS) {
			st->stop_ms += WS_STEP_MS;
			if (st->stop_ms >= WS_EXIT_HOLD_MS) {
				st->moving   = 0;
				st->start_ms = 0;
			}
		} else {
			st->stop_ms = 0;
		}
		if (st->blend < 255) {
			u16 b = (u16)st->blend + WS_FADE_STEP;
			st->blend = (b > 255) ? 255 : (u8)b;
		}
	}
}

u8 ws_frame[WS_PIXELS * 3];

void ws_clear(void)
{
	u16 i;
	for (i = 0; i < WS_PIXELS * 3; i++) ws_frame[i] = 0;
}

void ws_put(u8 phys, u8 r, u8 g, u8 b)
{
	u8 *p;
	if (phys >= WS_PIXELS) return;          /* bounds check, not an assert */
	p = &ws_frame[(u16)phys * 3];
#if WS_ORDER_RBG
	p[0] = r; p[1] = b; p[2] = g;
#elif WS_ORDER_GRB
	p[0] = g; p[1] = r; p[2] = b;
#else
	p[0] = r; p[1] = g; p[2] = b;
#endif
}

/* Logical position -> physical pixel. Only the position-aware effects
 * (calibration, nav progress, orbit direction) go through this; solid
 * fills ignore it and are therefore immune to how the strip was glued on. */
u8 ws_map(u8 logical)
{
	s16 v = (s16)(WS_DIR * (s16)logical) + WS_PHASE;
	while (v < 0) v += WS_PIXELS;
	return (u8)(v % WS_PIXELS);
}

void ws_fill_solid(u8 r, u8 g, u8 b)
{
	u8 i;
	for (i = 0; i < WS_PIXELS; i++) ws_put(i, r, g, b);
}

void ws_render_calib(u8 step)
{
	ws_clear();
	ws_put(ws_map(step), 255, 255, 255);
}

/**************************************************************************
Function: Channel-order probe. Drives the first third of the ring with
          pure red, the middle third with pure green and the last third
          with pure blue. Whatever order the strip actually SHOWS them in
          is its byte order -- read it off the strip and set WS_ORDER_* in
          ws2811.h to match. Immune to brightness and to the render
          pipeline; it is the ground truth for the wiring.
**************************************************************************/
void ws_render_rgb_probe(void)
{
	u8 i;

	ws_clear();
	for (i = 0; i < WS_PIXELS; i++) {
		if      (i < WS_PIXELS / 3)          ws_put(i, 255,   0,   0);
		else if (i < (WS_PIXELS * 2) / 3)    ws_put(i,   0, 255,   0);
		else                                 ws_put(i,   0,   0, 255);
	}
}

#define WS_IDLE_PERIOD_MS   2000    /* breathe period, slow on purpose  */
#define WS_IDLE_LO          13      /* ~5%  of 255                      */
#define WS_IDLE_HI          115     /* ~45% of 255                      */
#define WS_LOWBAT_PERIOD_MS 4000    /* low battery breathes slower      */
#define WS_LOWBAT_CAP       51      /* ~20% of 255, power saving        */
#define WS_LOST_PERIOD_MS   800     /* purple blink period              */
#define WS_ORBIT_PERIOD_MS  3000
/* Old 40-pixel strip used a 3-pixel tail. Keep the same PHYSICAL length
 * on the 80-pixel strip: 80 * 3 / 40 = 6. */
#define WS_ORBIT_TAIL       ((WS_PIXELS * 3 + 39) / 40)

u8 ws_breath(u32 ms, u16 period_ms, u8 lo, u8 hi)
{
	u32 ph   = ms % period_ms;
	u32 half = period_ms / 2;
	u32 k    = (ph < half) ? ph : (period_ms - ph);     /* 0..half */
	if (half == 0) return hi;
	return (u8)((u32)lo + ((u32)(hi - lo) * k) / half);
}

/* Idle base colour: status drives the hue, breathing drives brightness. */
static void ws_idle_color(const WS_Overlay *ov, u32 ms, u8 *r, u8 *g, u8 *b)
{
	u16 hue;
	u8  bright;

	if (ov->lowbat) {
		hue    = 30;                                  /* orange       */
		bright = ws_breath(ms, WS_LOWBAT_PERIOD_MS, WS_IDLE_LO, WS_LOWBAT_CAP);
	} else if (ov->ros_link == 2) {
		hue    = 285;                                 /* purple, blink */
		bright = ((ms % WS_LOST_PERIOD_MS) < WS_LOST_PERIOD_MS / 2)
		       ? WS_IDLE_HI : WS_IDLE_LO;
	} else if (ov->ros_link == 0) {
		hue    = 50;                                  /* yellow: wait  */
		bright = ws_breath(ms, WS_IDLE_PERIOD_MS, WS_IDLE_LO, WS_IDLE_HI);
	} else {
		hue    = 160;                                 /* cyan: normal  */
		bright = ws_breath(ms, WS_IDLE_PERIOD_MS, WS_IDLE_LO, WS_IDLE_HI);
	}
	ws_hsv2rgb(hue, 255, bright, r, g, b);
}

/* Comet running around the ring. Position goes through ws_map so a strip
 * glued on backwards only reverses the chase, not its shape. */
static void ws_idle_orbit(u32 ms, u8 r, u8 g, u8 b)
{
	u32 head = (ms % WS_ORBIT_PERIOD_MS) * WS_PIXELS / WS_ORBIT_PERIOD_MS;
	u8  i;

	ws_clear();
	for (i = 0; i < WS_ORBIT_TAIL; i++) {
		u8 atten = (u8)(255 / (i + 1));             /* 255, 127, 85 */
		u8 phys  = ws_map((u8)((head + WS_PIXELS - i) % WS_PIXELS));
		ws_put(phys,
		       (u8)((u32)r * atten / 255),
		       (u8)((u32)g * atten / 255),
		       (u8)((u32)b * atten / 255));
	}
}

void ws_render_idle(const WS_Overlay *ov, u32 ms)
{
	u8 r, g, b;

	ws_idle_color(ov, ms, &r, &g, &b);
	/* The comet is an ATTENTION cue, so reserve it for the one state that
	 * genuinely needs attention: a lost ROS link. Waiting for the host,
	 * and low battery, both light the WHOLE ring -- a 3-pixel comet is far
	 * too quiet for a safety indication, and on the bench it just read as
	 * "everything else is off / the strip looks broken". */
	if (ov->ros_link == 2) {
		ws_idle_orbit(ms, r, g, b);
	} else {
		ws_fill_solid(r, g, b);
	}
}

/* One full lap of the ring. Was 250ms, which is a 4Hz sweep -- at that
 * speed the arc crosses the whole ring in a quarter of a second and the
 * two directions are indistinguishable, so the left and right indicators
 * looked identical on the bench. 1500ms makes the direction obvious. */
#define WS_TURN_PERIOD_MS   1500
/* Old 40-pixel strip lit 10 pixels (1/4 of the strip). Keep the fraction. */
#define WS_TURN_SIDE        (WS_PIXELS / 4)
#define WS_TURN_HUE         40      /* amber                            */
#define WS_BRAKE_PERIOD_MS  150
#define WS_BRAKE_DIM        90      /* the dark phase of the flash      */
#define WS_ESTOP_PERIOD_MS  125     /* 8Hz strobe                       */
#define WS_NAV_DONE_HUE     210     /* deep blue: already covered       */
#define WS_NAV_HEAD_HUE     195     /* bright cyan: the moving comet    */

/* t = 0 -> a, t = 255 -> b */
static u8 ws_mix(u8 a, u8 b, u8 t)
{
	return (u8)(((u32)a * (255u - t) + (u32)b * (u32)t) / 255u);
}

/* Directional flow drawn ON TOP of the motion base colour, so the speed
 * reading survives while the turn direction is added. */
static void ws_ov_turn(s8 dir, u32 ms)
{
	u32 head = (ms % WS_TURN_PERIOD_MS) * WS_PIXELS / WS_TURN_PERIOD_MS;
	u8  i;

	for (i = 0; i < WS_TURN_SIDE; i++) {
		u32 logical = (dir > 0) ? (head + i) : (head + WS_PIXELS - i);
		u8  ar, ag, ab;
		ws_hsv2rgb(WS_TURN_HUE, 255, 255, &ar, &ag, &ab);
		ws_put(ws_map((u8)(logical % WS_PIXELS)), ar, ag, ab);
	}
}

static void ws_ov_reverse(u32 ms)
{
	u32 head = (ms % WS_ORBIT_PERIOD_MS) * WS_PIXELS / WS_ORBIT_PERIOD_MS;
	u8  i;

	ws_clear();
	for (i = 0; i < WS_ORBIT_TAIL; i++) {
		u8 atten = (u8)(255 / (i + 1));
		u8 phys  = ws_map((u8)((head + i) % WS_PIXELS));
		ws_put(phys, atten, atten, atten);
	}
}

static void ws_ov_nav(u16 permille, u32 ms)
{
	u8 lit, i, head;
	u8 dr, dg, db, hr, hg, hb;

	ws_clear();
	if (permille > 1000) permille = 1000;
	lit  = (u8)((u32)permille * WS_PIXELS / 1000);
	head = (u8)((ms / 100) % WS_PIXELS);
	ws_hsv2rgb(WS_NAV_DONE_HUE, 255, 255, &dr, &dg, &db);
	ws_hsv2rgb(WS_NAV_HEAD_HUE, 255, 255, &hr, &hg, &hb);

	for (i = 0; i < WS_PIXELS; i++) {
		u8 phys = ws_map(i);
		if (i < lit)        ws_put(phys, dr, dg, db);
		else if (i == head) ws_put(phys, hr, hg, hb);
	}
}

void ws_render(const WS_State *st, WS_SpeedOut sp, const WS_Overlay *ov, u32 ms)
{
	u8 mr, mg, mb, i, bl = st->blend;

	/* ---- L3 full-band overrides, highest priority first ---- */
	if (ov->estop) {
		u8 on = ((ms % WS_ESTOP_PERIOD_MS) < WS_ESTOP_PERIOD_MS / 2) ? 255 : 0;
		ws_fill_solid(on, 0, 0);
		return;
	}
	if (ov->lowbat) { ws_render_idle(ov, ms); return; }
	if (ov->brake) {
		u8 on = ((ms % WS_BRAKE_PERIOD_MS) < WS_BRAKE_PERIOD_MS * 2 / 3)
		      ? 255 : WS_BRAKE_DIM;
		ws_fill_solid(on, 0, 0);
		return;
	}
	if (ov->reverse) { ws_ov_reverse(ms); return; }
	if (ov->nav_permille != 0xFFFF) { ws_ov_nav(ov->nav_permille, ms); return; }

	/* ---- L2 motion colour, fully opaque: fast path ---- */
	ws_hsv2rgb(sp.hue, 255, sp.bright, &mr, &mg, &mb);
	if (bl >= 255) {
		ws_fill_solid(mr, mg, mb);
	} else {
		/* L1 idle layer paints FIRST, comet included, then the motion
		 * colour is crossfaded in per pixel.
		 *
		 * Delegating to ws_render_idle -- rather than filling a solid
		 * idle colour -- is what actually lets the comet reach the
		 * strip. An earlier revision called ws_idle_color() directly
		 * here, which made ws_render_idle() reachable only from the
		 * test suite: the orbit effect was dead code on the real car. */
		ws_render_idle(ov, ms);
		if (bl > 0) {
			for (i = 0; i < WS_PIXELS; i++) {
				u8 *p = &ws_frame[(u16)i * 3];
#if WS_ORDER_GRB
				p[0] = ws_mix(p[0], mg, bl);
				p[1] = ws_mix(p[1], mr, bl);
				p[2] = ws_mix(p[2], mb, bl);
#elif WS_ORDER_RBG
				/* RBG stores r,b,g; the old #else branch mixed r,g,b
				 * in the wrong slots, so the idle->motion crossfade
				 * rotated colours on this strip. */
				p[0] = ws_mix(p[0], mr, bl);
				p[1] = ws_mix(p[1], mb, bl);
				p[2] = ws_mix(p[2], mg, bl);
#else
				p[0] = ws_mix(p[0], mr, bl);
				p[1] = ws_mix(p[1], mg, bl);
				p[2] = ws_mix(p[2], mb, bl);
#endif
			}
		}
	}

	/* ---- L3 directional overlay ---- */
	if (ov->turn != 0) ws_ov_turn(ov->turn, ms);
}

/**************************************************************************
Function: Legacy 3-SPI-bit-per-WS-bit expansion, kept for the host suite.
          The F407 hardware no longer uses this: at 2.625MHz its T1L is
          0.381us, below the WS2811 data-sheet minimum of 0.58us.  The
          hardware layer now calls ws_pack_byte4() below.
             0 -> 100  = 0.381us high, 0.762us low
             1 -> 110  = 0.762us high, 0.381us low  <-- T1L too short
          MSB first: 8 source bits -> 24 SPI bits -> exactly 3 bytes.
**************************************************************************/
void ws_pack_byte(u8 src, u8 out[3])
{
	u32 acc = 0;
	u8  bit;

	for (bit = 0; bit < 8; bit++) {
		u32 code = ((src >> (7 - bit)) & 1u) ? 0x6u : 0x4u;   /* 110 : 100 */
		acc = (acc << 3) | code;
	}
	out[0] = (u8)(acc >> 16);
	out[1] = (u8)(acc >> 8);
	out[2] = (u8)(acc);
}

/**************************************************************************
Function: 4-SPI-bit-per-WS-bit expansion, used by the F407 SPI2 master at
          2.625MHz (APB1 42MHz / 16).  One SPI bit is 0.381us:
             0 -> 1000  = 0.381us high, 1.143us low
             1 -> 1100  = 0.762us high, 0.762us low
          High times (what WS2811 actually discriminates) are 0.38/0.76us.
          MSB first: 8 source bits -> 32 SPI bits -> exactly 4 bytes.
**************************************************************************/
void ws_pack_byte4(u8 src, u8 out[4])
{
	u32 acc = 0;
	u8  bit;

	for (bit = 0; bit < 8; bit++) {
		u32 code = ((src >> (7 - bit)) & 1u) ? 0xCu : 0x8u;  /* 1100 : 1000 */
		acc = (acc << 4) | code;
	}
	out[0] = (u8)(acc >> 24);
	out[1] = (u8)(acc >> 16);
	out[2] = (u8)(acc >> 8);
	out[3] = (u8)(acc);
}

/* ==== WS_DEMO_LOGIC_BEGIN ==== */
/* PE0 preview mode, split in two because the host suite only extracts
 * WS_LOGIC. This half is the pure part -- the stage table and the input
 * synthesis -- so the "the OLED name matches what you actually see"
 * contract is covered by assertions.
 *
 * TO REMOVE THE WHOLE FEATURE
 *   1. set WS_DEMO_ENABLE to 0 in ws2811.h  -> feature gone, code kept
 *   2. to delete outright, drop: this block, WS_DEMO_HW_BEGIN..END, the
 *      two marked hook blocks in ws2811_task, the marked lines in
 *      BALANCE/show.c, and the WS_DEMO section of ws2811.h
 * Nothing else in the project references any of it. */
#if WS_DEMO_ENABLE

static u8  s_demo_stage = 0;        /* 0 = off, running the real inputs */
static u32 s_demo_ms    = 0;        /* ms since entering the stage       */

/* Snapshots of what the current stage feeds the engine, so the OLED page
 * can report them without recomputing anything. */
static float s_demo_v     = 0.0f;
static u8    s_demo_estop = 0, s_demo_brake = 0;
static s8    s_demo_turn  = 0;
static u8    s_demo_ros   = 1;

/* Deliberately NOT static: the only reader is ws_demo_oled(), which lives in
 * WS_HW. The host suite compiles this half alone, so a static table would be
 * reported as "defined but not used" by -Wall. External linkage is how the
 * sibling table ws_frame is handled in this same region. */
const char *const ws_demo_name[WS_DEMO_STAGES + 1] = {
	"DEMO OFF",
	"IDLE CYAN",     /* 1  */
	"WAIT ROS",      /* 2  */
	"ROS LOST",      /* 3  */
	"LOW BAT",       /* 4  */
	"SPEED RAMP",    /* 5  */
	"CRUISE GRN",    /* 6  */
	"MAX RED",       /* 7  */
	"TURN LEFT",     /* 8  */
	"TURN RIGHT",    /* 9  */
	"BRAKE",         /* 10 */
	"E-STOP",        /* 11 */
	"REVERSE",       /* 12 */
	"NAV PROG",      /* 13 */
	"CALIBRATE",     /* 14 */
	"ALL WHITE",     /* 15 */
	"RGB PROBE",     /* 16 */
};

u8 ws_demo_active(void)   { return s_demo_stage != 0; }
u8 ws_demo_stage_no(void) { return s_demo_stage; }

/* The ONE place that changes stage. The PE0 poll in ws2811_task and the
 * test hooks both go through here: three copies of this increment would
 * mean a mutation test on one of them proves nothing about the others. */
void ws_demo_advance(void)
{
	s_demo_stage = (u8)(s_demo_stage + 1);
	if (s_demo_stage > WS_DEMO_STAGES) s_demo_stage = 0;
	s_demo_ms = 0;
}

/* Test hooks, so the host suite can drive the stage machine without a
 * button. Both go through ws_demo_advance() on purpose. */
void ws_demo_reset_for_test(void) { s_demo_stage = 0; s_demo_ms = 0; }
void ws_demo_press_for_test(void) { ws_demo_advance(); }

/* Pure: synthesises the engine inputs for the current stage. The button
 * poll happens in the task, NOT here -- click() is hardware and would
 * break the host build. */
u8 ws_demo_frame(float *v, WS_Overlay *ov, u32 ms)
{
	if (!s_demo_stage) return 0;            /* off: leave v/ov untouched */

	s_demo_ms += WS_STEP_MS;

	/* Stage 14/15 paint the frame themselves and bypass ws_render(). */
	if (s_demo_stage == 14) {
		ws_render_calib((u8)((ms / 250) % WS_PIXELS));
		return 1;
	}
	if (s_demo_stage == 15) {
		ws_fill_solid(255, 255, 255);
		return 1;
	}
	if (s_demo_stage == 16) {
		ws_render_rgb_probe();
		return 1;
	}

	/* Every other stage only fabricates the INPUTS. The effect code is
	 * untouched, so what you see is exactly what the car would show. */
	*v = 0.0f;
	memset(ov, 0, sizeof(*ov));
	ov->nav_permille = 0xFFFF;
	ov->ros_link = 1;

	switch (s_demo_stage) {
	case 1: break;                                    /* idle cyan        */
	case 2:  ov->ros_link = 0; break;                 /* waiting yellow   */
	case 3:  ov->ros_link = 2; break;                 /* lost purple      */
	case 4:  ov->lowbat = 1;   break;                 /* low battery      */
	case 5: {                                         /* 8s speed ramp    */
		u32 ph = s_demo_ms % 8000;
		float t = (ph < 4000u ? ph : 8000u - ph) / 4000.0f;
		*v = t * WS_VMAX_MPS;
		break;
	}
	case 6:  *v = 0.25f; break;
	case 7:  *v = WS_VMAX_MPS; break;
	case 8:  *v = 0.45f; ov->turn =  1; break;
	case 9:  *v = 0.45f; ov->turn = -1; break;
	case 10: *v = 0.30f; ov->brake = 1; break;
	case 11: ov->estop = 1; break;
	case 12: ov->reverse = 1; break;
	case 13: /* nav sweeps 0 -> 100% over 3s, then repeats */
		ov->nav_permille = (u16)((s_demo_ms % 3000u) * 1000u / 3000u);
		break;
	default: *v = 0.0f; break;
	}

	/* Snapshot for the OLED page. Stages 14/15 returned early above, so
	 * they keep the previous values -- they paint the frame directly and
	 * have no meaningful v/ov to report. */
	s_demo_v     = *v;
	s_demo_estop = ov->estop;
	s_demo_brake = ov->brake;
	s_demo_turn  = ov->turn;
	s_demo_ros   = ov->ros_link;
	return 0;
}
#endif  /* WS_DEMO_ENABLE -- end of the host-testable half */
/* ==== WS_DEMO_LOGIC_END ==== */

/* ==== WS_LOGIC_END ==== */

/* ==== WS_HW_BEGIN ==== */
#include "delay.h"
#include "adc.h"

/* The F407 does not generate WS2811 timing any more.  It publishes the
 * chassis state to the C542 LED controller over USART3:
 *
 *   PC10 / USART3_TX (AF7) -> C542 PA10 (USART1_RX, Arduino D2)
 *   PC11 / USART3_RX (AF7) <- C542 PA9  (USART1_TX, Arduino D8)
 *   GND                    -- GND
 *
 * 115200 8N1, one 13-byte STATE frame every 40 ms (~325 B/s).  The wire
 * format lives in ws_link.h so both sides share one definition.
 *
 * PB10/PB11 are NOT usable for USART3: bsp_siic.c/I2C.c drive them as the
 * IMU's software I2C.  PC10/PC11 are free in this firmware (PC6-PC9 are the
 * TIM8 motor PWM, PC12 is the lidar's UART5).  If the board does not bring
 * PC10/PC11 out, PD8/PD9 (also AF7) are the alternative. */

static u8 s_link_seq;

static void ws_link_init(void)
{
	GPIO_InitTypeDef  g;
	USART_InitTypeDef u;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

	g.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
	g.GPIO_Mode  = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_Speed = GPIO_Speed_25MHz;
	g.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &g);

	u.USART_BaudRate            = 115200;
	u.USART_WordLength          = USART_WordLength_8b;
	u.USART_StopBits            = USART_StopBits_1;
	u.USART_Parity              = USART_Parity_No;
	u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	u.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &u);
	USART_Cmd(USART3, ENABLE);
}

static u8 ws_crc8_update(u8 crc, u8 b)
{
	u8 i;

	crc ^= b;
	for (i = 0; i < 8u; i++)
		crc = (crc & 0x80u) ? (u8)((crc << 1) ^ 0x07u) : (u8)(crc << 1);
	return crc;
}

static void ws_link_put(u8 b)
{
	u32 spin = 0;

	while ((USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET) &&
	       (spin++ < 100000u)) {
		/* bounded: a disconnected controller must never wedge the task */
	}
	if (spin < 100000u) USART_SendData(USART3, b);
}

/* Drain whatever the controller sent (5 Hz STATUS frames).  They are not
 * interpreted yet; reading them keeps ORE from ever latching. */
static void ws_link_drain(void)
{
	while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET) {
		(void)USART_ReceiveData(USART3);
	}
	if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET) {
		(void)USART_ReceiveData(USART3);
	}
}

/* Speed calibration into the controller's colour table.
 *
 * The C542's ws_speed_map() breakpoints were tuned for a 0.75 m/s full
 * scale (the old 60 LED/m car), but this chassis tops out at ~0.50 m/s:
 * measured 2026-11-25 over /wheel_odom, max 0.503, p99 0.486.  With the
 * old mapping full throttle only ever reached hue 60 (orange), never red.
 *
 * Scaling the value sent makes every speed reach the warm colours earlier
 * without reflashing the controller.  2.0 -> 0.10 m/s is green-cyan,
 * 0.20 m/s is yellow-orange, 0.30 m/s is orange-red, 0.40+ is red.
 * Set back to 1.0f if the controller's table is retuned later. */
#define WS_LINK_SPEED_GAIN  2.0f

static void ws_link_send(float v, const WS_Overlay *ov)
{
	u8  out[6u + WS_LINK_STATE_LEN + 1u];
	u8  crc = 0;
	u8  f = 0;
	u8  i;
	s16 speed_cms;
	u16 nav = ov->nav_permille;

	if (ov->estop)   f |= WS_LINK_F_ESTOP;
	if (ov->lowbat)  f |= WS_LINK_F_LOWBAT;
	if (ov->brake)   f |= WS_LINK_F_BRAKE;
	if (ov->reverse) f |= WS_LINK_F_REVERSE;
	f |= (u8)(((u8)ov->ros_link & 3u) << WS_LINK_F_ROS_SHIFT);
	if      (ov->turn > 0) f |= WS_LINK_F_TURN_L;
	else if (ov->turn < 0) f |= WS_LINK_F_TURN_R;

	/* Clamp before the int16 conversion; an out-of-range float would be
	 * undefined behaviour.  The chassis never exceeds a few m/s. */
	v *= WS_LINK_SPEED_GAIN;
	if (v >  3.0f) v =  3.0f;
	if (v < -3.0f) v = -3.0f;
	speed_cms = (s16)(v * 100.0f + ((v >= 0.0f) ? 0.5f : -0.5f));

	out[0]  = WS_LINK_SOF0;
	out[1]  = WS_LINK_SOF1;
	out[2]  = WS_LINK_VER;
	out[3]  = WS_LINK_TYPE_STATE;
	out[4]  = WS_LINK_STATE_LEN;
	out[5]  = s_link_seq++;
	out[6]  = f;
	out[7]  = WS_BRIGHT_MAX;
	out[8]  = (u8)((u16)speed_cms & 0xFFu);
	out[9]  = (u8)((u16)speed_cms >> 8);
	out[10] = (u8)(nav & 0xFFu);
	out[11] = (u8)(nav >> 8);
	out[12] = WS_LINK_MODE_NORMAL;

	for (i = 2u; i < 6u + WS_LINK_STATE_LEN; i++) crc = ws_crc8_update(crc, out[i]);
	out[6u + WS_LINK_STATE_LEN] = crc;

	ws_link_drain();
	for (i = 0; i < 6u + WS_LINK_STATE_LEN + 1u; i++) ws_link_put(out[i]);
}

static float ws_wheel_speed(void)
{
	float a = MOTOR_A.Encoder < 0 ? -MOTOR_A.Encoder : MOTOR_A.Encoder;
	float b = MOTOR_B.Encoder < 0 ? -MOTOR_B.Encoder : MOTOR_B.Encoder;
	float c = MOTOR_C.Encoder < 0 ? -MOTOR_C.Encoder : MOTOR_C.Encoder;
	float d = MOTOR_D.Encoder < 0 ? -MOTOR_D.Encoder : MOTOR_D.Encoder;
	return (a + b + c + d) * 0.25f;
}

static void ws_collect_overlay(WS_Overlay *ov, u32 ms)
{
	static u32   brake_until = 0;
	static float prev = 0.0f;
	static u8    link_seen = 0;
	float v     = ws_wheel_speed();
	float decel = prev - v;
	prev = v;

	/* RosDown.rx_ready is a ONE-FRAME publish flag: the USART ISR sets it
	 * and Balance_task clears it again at 100 Hz.  Sampling it from this
	 * 25 Hz task caught it at random, so the ring alternated between
	 * "waiting" (yellow) and "connected" (cyan) even with a healthy link.
	 * The frame counter is monotonic, so use it as the "ever connected"
	 * latch; the existing ROS_RX_TIMEOUT_MS watchdog still supplies the
	 * real lost-link indication through ROS_FAULT_TIMEOUT. */
	if (RosDown.frames != 0u) link_seen = 1;

	ov->estop  = (Flag_Stop || Avoid_ON_Flag) ? 1 : 0;
	ov->lowbat = (Voltage < WS_LOWBAT_V) ? 1 : 0;
	ov->ros_link = (RosFault & ROS_FAULT_TIMEOUT) ? 2
	             : (link_seen ? 1 : 0);
	ov->reverse  = 0;              /* no reverse channel on this chassis */
	ov->nav_permille = 0xFFFF;     /* no odometry feed yet, see plan notes */
	ov->turn = 0;
	if (Move_Z > 0.20f)       ov->turn =  1;
	else if (Move_Z < -0.20f) ov->turn = -1;

	if (decel > 0.15f) brake_until = ms + 450;   /* three 150ms flashes */
	ov->brake = (ms < brake_until) ? 1 : 0;
}

void ws2811_task(void *pvParameters)
{
	WS_Overlay ov;
	u32 ms = 0;

	(void)pvParameters;
	ws_link_init();

	for (;;) {
		float v = ws_wheel_speed();

		memset(&ov, 0, sizeof(ov));
		ov.nav_permille = 0xFFFF;
		ws_collect_overlay(&ov, ms);

		/* Everything below the render split now happens on the C542:
		 * ws_state_step(), ws_speed_map(), ws_render() and the SPI bit
		 * packing all run there, fed by this frame. */
		ws_link_send(v, &ov);

		/* FIXED 40ms tick -- do not make this adaptive.
		 *
		 * The controller's ws_state_step() is hard-calibrated for a
		 * 40ms step: WS_FADE_STEP, the 100ms enter dwell and the 500ms
		 * exit dwell all assume it.  At a 100ms tick the 500ms exit
		 * hysteresis silently becomes ~1250ms, so the ring stays in
		 * motion mode for over a second after stopping. */
		vTaskDelay(WS_STEP_MS);
		ms += WS_STEP_MS;
	}
}
/* ==== WS_DEMO_HW_BEGIN ==== */
/* Second half: the OLED page. Lives in WS_HW because OLED_ShowString is
 * hardware; the host suite cannot compile it. */
#if WS_DEMO_ENABLE

/* oled_show() hands the whole screen over while a stage is active,
 * because all six of its lines are already in use by the normal page. */
void ws_demo_oled(void)
{
	/* line 0: DEMO nn/15 */
	OLED_ShowString(0, 0, "DEMO");
	OLED_ShowNumber(30, 0, s_demo_stage, 2, 12);
	OLED_ShowString(48, 0, "/16   ");

	/* line 1: stage name, padded so a shorter name does not leave tails */
	OLED_ShowString(0, 10, "                ");
	OLED_ShowString(0, 10, (const u8 *)ws_demo_name[s_demo_stage]);

	/* line 2: the synthetic wheel speed this stage feeds the engine */
	OLED_ShowString(0, 20, "v=");
	OLED_ShowNumber(18, 20, (u32)(s_demo_v * 1000.0f), 4, 12);

	/* line 3: what the overlay bits are doing */
	OLED_ShowString(0, 30, "E");
	OLED_ShowNumber(8,  30, (u32)s_demo_estop, 1, 12);
	OLED_ShowString(20, 30, "B");
	OLED_ShowNumber(28, 30, (u32)s_demo_brake, 1, 12);
	OLED_ShowString(40, 30, "T");
	OLED_ShowNumber(48, 30, (u32)(s_demo_turn + 1), 1, 12);
	OLED_ShowString(60, 30, "R");
	OLED_ShowNumber(68, 30, (u32)s_demo_ros, 1, 12);

	/* line 4: real battery voltage, still worth watching while parked */
	OLED_ShowString(0, 40, "BAT");
	OLED_ShowNumber(30, 40, (u32)(Voltage * 10), 4, 12);

	/* line 5: the only control */
	OLED_ShowString(0, 50, "PE0 = NEXT");
}
#endif  /* WS_DEMO_ENABLE */
/* ==== WS_DEMO_HW_END ==== */
/* ==== WS_HW_END ==== */
