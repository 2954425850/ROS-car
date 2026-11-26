/**
  ******************************************************************************
  * file           : ws2811.c
  * brief          : WS2811/WS2812 effect engine + STM32C542 SPI driver.
  *
  * PORTING NOTES (C542 -> F407)
  *   The block between WS_LOGIC_BEGIN and WS_LOGIC_END is the same code that
  *   runs on the F407 chassis.  It touches no STM32 header and no global
  *   owned by another module, so it can be copied verbatim.  Only
  *   ws2811_init() and ws_show() below are C542-specific: on the F407 they
  *   become the SPI1/DMA2_Stream5 pair from HARDWARE/ws2811.c.
  *
  *   Protocol: one WS bit is carried by 3 SPI bits at 2.25 MHz
  *     WS 0 -> 100  (0.444 us high, 0.889 us low ; spec 0.40 / 0.85)
  *     WS 1 -> 110  (0.889 us high, 0.444 us low ; spec 0.80 / 0.45)
  *   MSB first: 8 WS bits -> 24 SPI bits -> exactly 3 SPI bytes.
  *
  *   Hardware: NUCLEO-C542RC, 144 MHz PCLK2, SPI1 (PA7 / Arduino D11),
  *   prescaler /64 -> 2.25 MHz.  Blocking TX is used on the bench: one
  *   80-pixel frame is 820 bytes = 2.9 ms inside a 40 ms tick, which is
  *   invisible in the effect timing and avoids the C5 LPDMA bring-up.
  ******************************************************************************
  */
#include "ws2811.h"

#include <string.h>

#include "stm32_hal.h"

/* Needed both for the VCP log (USART2) and for the host link (USART1). */
#include "stm32c5xx_ll_usart.h"

#include "ws_link.h"

/* ==== WS_LOGIC_BEGIN ==== */
/* Nothing below this marker may include an STM32 header or touch a global
 * owned by another module.  Keep it parametric so the same block can be
 * host-tested and pasted into the F407 firmware. */

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
/* 255 * 40 / 300 = 34 -> eight steps to cross, 320 ms nominal */
#define WS_FADE_STEP      ((u8)(255 * WS_STEP_MS / WS_FADE_MS))
/* first-order LPF: alpha = dt / (tau + dt), tau = 100 ms */
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
#elif WS_ORDER_BGR
	p[0] = b; p[1] = g; p[2] = r;
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
Function: Channel-order probe. Drives the first third of the strip with
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
#define WS_IDLE_HI          115     /* ~45% idle max; global cap 255    */
#define WS_LOWBAT_PERIOD_MS 4000    /* low battery breathes slower      */
#define WS_LOWBAT_CAP       51      /* ~20% of 255, power saving        */
#define WS_LOST_PERIOD_MS   800     /* purple blink period              */
#define WS_ORBIT_PERIOD_MS  3000
/* The old 2 m strip had 40 pixels and a 3-pixel comet.  Keep the comet the
 * same PHYSICAL length on the 80-pixel strip. */
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
#if WS_IDLE_BREATHE
		bright = ws_breath(ms, WS_IDLE_PERIOD_MS, WS_IDLE_LO, WS_IDLE_HI);
#else
		bright = WS_IDLE_HI;                      /* steady test    */
#endif
	} else {
		hue    = 160;                                 /* cyan: normal  */
#if WS_IDLE_BREATHE
		bright = ws_breath(ms, WS_IDLE_PERIOD_MS, WS_IDLE_LO, WS_IDLE_HI);
#else
		bright = WS_IDLE_HI;                      /* steady test    */
#endif
	}
	ws_hsv2rgb(hue, 255, bright, r, g, b);
}

/* Comet running around the strip. Position goes through ws_map so a strip
 * glued on backwards only reverses the chase, not its shape. */
static void ws_idle_orbit(u32 ms, u8 r, u8 g, u8 b)
{
	u32 head = (ms % WS_ORBIT_PERIOD_MS) * WS_PIXELS / WS_ORBIT_PERIOD_MS;
	u8  i;

	ws_clear();
	for (i = 0; i < WS_ORBIT_TAIL; i++) {
		u8 atten = (u8)(255 / (i + 1));             /* 255, 127, 85, ... */
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
	 * and low battery, both light the WHOLE ring -- a small comet is far
	 * too quiet for a safety indication. */
	if (ov->ros_link == 2) {
		ws_idle_orbit(ms, r, g, b);
	} else {
		ws_fill_solid(r, g, b);
	}
}

/* One full lap of the strip. 1500 ms makes the turn direction obvious;
 * the old 40-pixel firmware used 10 lit pixels, so keep the same fraction
 * of the strip lit on the 80-pixel strip. */
#define WS_TURN_PERIOD_MS   1500
#define WS_TURN_SIDE        (WS_PIXELS / 4)
#define WS_TURN_HUE         40      /* amber                            */
#define WS_BRAKE_PERIOD_MS  150
#define WS_BRAKE_DIM        90      /* the dark phase of the flash      */
#define WS_ESTOP_PERIOD_MS  125     /* 8 Hz strobe                      */
#define WS_NAV_DONE_HUE     210     /* deep blue: already covered       */
#define WS_NAV_HEAD_HUE     195     /* bright cyan: the moving comet    */

/* t = 0 -> a, t = 255 -> b */
static u8 ws_mix(u8 a, u8 b, u8 t)
{
	return (u8)(((u32)a * (255u - t) + (u32)b * (u32)t) / 255u);
}

/* Directional flow drawn ON TOP of the motion base colour, so the speed
 * reading survives while the turn direction is added.
 *
 * Left and right MUST travel in opposite directions.  The old code always
 * advanced "head" with time and only flipped which side the tail trailed
 * on, so both turns looked like the same rotation. */
static void ws_ov_turn(s8 dir, u32 ms)
{
	u32 ph = (ms % WS_TURN_PERIOD_MS) * WS_PIXELS / WS_TURN_PERIOD_MS;
	u32 head;
	u8  i;

	/* dir = +1 for Move_Z > 0 (left), -1 for right.  This chassis's ring is
	 * indexed the other way round, so the equations are swapped relative
	 * to the first version (measured 2026-11-26: without the swap left and
	 * right were mirrored). */
	if (dir > 0) head = (WS_PIXELS - ph) % WS_PIXELS;
	else         head = ph;

	for (i = 0; i < WS_TURN_SIDE; i++) {
		/* The tail always trails BEHIND the head's direction of travel. */
		u32 logical = (dir > 0) ? ((head + i) % WS_PIXELS)
		                        : ((head + WS_PIXELS - i) % WS_PIXELS);
		u8  ar, ag, ab;
		ws_hsv2rgb(WS_TURN_HUE, 255, 255, &ar, &ag, &ab);
		ws_put(ws_map((u8)logical), ar, ag, ab);
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
		 * strip. */
		ws_render_idle(ov, ms);
		if (bl > 0) {
			for (i = 0; i < WS_PIXELS; i++) {
				u8 *p = &ws_frame[(u16)i * 3];
#if WS_ORDER_GRB
				p[0] = ws_mix(p[0], mg, bl);
				p[1] = ws_mix(p[1], mr, bl);
				p[2] = ws_mix(p[2], mb, bl);
#elif WS_ORDER_RBG
				p[0] = ws_mix(p[0], mr, bl);
				p[1] = ws_mix(p[1], mb, bl);
				p[2] = ws_mix(p[2], mg, bl);
#elif WS_ORDER_BGR
				p[0] = ws_mix(p[0], mb, bl);
				p[1] = ws_mix(p[1], mg, bl);
				p[2] = ws_mix(p[2], mr, bl);
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
Function: Expand one source byte into the three SPI bytes that carry it.
          One WS2811 bit -> three SPI bits at 2.25 MHz (0.444 us each):
             0 -> 100  = 0.444 us high, 0.889 us low  (spec 0.40/0.85)
             1 -> 110  = 0.889 us high, 0.444 us low  (spec 0.80/0.45)
          MSB first, so source bit7 becomes the most significant SPI group:
          8 source bits -> 24 SPI bits -> exactly 3 bytes.
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

/* ==== WS_DEMO_LOGIC_BEGIN ==== */
/* Pure half of the bench preview: stage table and input synthesis.  The
 * stage machine is identical to the F407 firmware, so what the bench shows
 * is exactly what the car would show for the same inputs. */
#if WS_DEMO_ENABLE

static u8  s_demo_stage = 0;        /* 0 = off, running the real inputs */
static u32 s_demo_ms    = 0;        /* ms since entering the stage      */

static const char *const ws_demo_name[WS_DEMO_STAGES + 1] = {
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

const char *ws_demo_stage_name(void)
{
	return ws_demo_name[s_demo_stage];
}

/* The ONE place that changes stage. */
void ws_demo_advance(void)
{
	s_demo_stage = (u8)(s_demo_stage + 1);
	if (s_demo_stage > WS_DEMO_STAGES) s_demo_stage = 0;
	s_demo_ms = 0;
}

/* Pure: synthesises the engine inputs for the current stage.  The button
 * poll happens in the task, NOT here -- that would break the host build. */
u8 ws_demo_frame(float *v, WS_Overlay *ov, u32 ms)
{
	if (!s_demo_stage) return 0;            /* off: leave v/ov untouched */

	s_demo_ms += WS_STEP_MS;

	/* Stage 14/15/16 paint the frame themselves and bypass ws_render(). */
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
	case 5: {                                         /* 8 s speed ramp   */
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
	case 13: /* nav sweeps 0 -> 100% over 3 s, then repeats */
		ov->nav_permille = (u16)((s_demo_ms % 3000u) * 1000u / 3000u);
		break;
	default: *v = 0.0f; break;
	}

	return 0;
}
#endif  /* WS_DEMO_ENABLE */
/* ==== WS_DEMO_LOGIC_END ==== */

/* ==== WS_LOGIC_END ==== */

/* ==== WS_HW_BEGIN ==== */

/* Each pixel is 3 channels x 8 bits = 24 WS2811 bits; every WS2811 bit
 * expands to 3 SPI bits, so one pixel costs 24*3/8 = 9 SPI BYTES.
 *
 * DO NOT write (WS_PIXELS * 24 * 3) here.  That is the SPI BIT count, not
 * the byte count; sizing the buffer to it makes every frame 8x too long. */
#define WS_SPI_BYTES    (WS_PIXELS * 9)

/* Trailing zero SPI bytes.  One 0x00 byte is 8 low bit-times = 3.56 us of
 * continuous low on MOSI at 2.25 MHz, so N bytes give 3.56*N us of low --
 * and that low period is what latches the strip (WS2811 wants >50 us).
 * 100 bytes = 356 us; it costs 100 bytes out of a 40 ms frame. */
#define WS_RESET_BYTES  100

#if (WS_SPI_BYTES == (WS_PIXELS * 24 * 3))
#error "WS_SPI_BYTES is the SPI BIT count, not the BYTE count (should be WS_PIXELS * 9)"
#endif

#if WS_ORDER_RBG
#define WS_ORDER_STR "RBG"
#elif WS_ORDER_GRB
#define WS_ORDER_STR "GRB"
#elif WS_ORDER_BGR
#define WS_ORDER_STR "BGR"
#else
#define WS_ORDER_STR "RGB"
#endif

#if WS_SPI_DIV == 64
#define WS_SPI_PRESC  HAL_SPI_BAUD_RATE_PRESCALER_64
#define WS_SPI_KHZ    2250U
#elif WS_SPI_DIV == 128
#define WS_SPI_PRESC  HAL_SPI_BAUD_RATE_PRESCALER_128
#define WS_SPI_KHZ    1125U
#else
#error "WS_SPI_DIV must be 64 (800 kHz class) or 128 (400 kHz class test)"
#endif

static u8 s_tx[WS_SPI_BYTES + WS_RESET_BYTES];

/* Runtime brightness cap.  WS_BRIGHT_MAX is the build's hard ceiling; the
 * host may ask for less (e.g. 80 in the car), never for more. */
static u8 s_bright = WS_BRIGHT_MAX;

static hal_spi_handle_t s_spi;

volatile u32 ws_tx_ok;
volatile u32 ws_tx_err;
volatile u32 ws_last_status;   /* last hal_status_t from HAL_SPI_Transmit */
volatile u32 ws_last_err;      /* s_spi.last_error_codes (needs GET_LAST_ERRORS) */
volatile u32 ws_last_sr;       /* SPI1->SR just after the call              */
volatile u32 ws_last_cfg1;
volatile u32 ws_last_cfg2;
volatile u32 ws_last_cr1;
volatile u32 ws_last_cr2;
volatile u32 ws_last_tx_ms;    /* how long the failing call blocked         */

/**************************************************************************
Function: Pack the whole frame into s_tx.
          The 256-entry LUT is built by CALLING ws_pack_byte, so the bit
          pattern has exactly one definition and a host test covers it.
**************************************************************************/
static void ws_pack(void)
{
	static u8 lut[256][3];
	static u8 lut_ready = 0;
	u16 i, o = 0;

	if (!lut_ready) {
		u16 v;
		for (v = 0; v < 256; v++) ws_pack_byte((u8)v, lut[v]);
		lut_ready = 1;
	}

	/* Global brightness ceiling, applied here at the last mile so every
	 * effect obeys it without each renderer having to know about the cap.
	 * THIS IS THE ONLY PLACE THE CAP TAKES EFFECT: s_bright is WS_BRIGHT_MAX
	 * on the bench and the host-requested value (capped) once linked. */
	for (i = 0; i < WS_PIXELS * 3; i++) {
		u8 v = (u8)(((u32)ws_frame[i] * s_bright) / 255u);
		s_tx[o++] = lut[v][0];
		s_tx[o++] = lut[v][1];
		s_tx[o++] = lut[v][2];
	}
	for (i = 0; i < WS_RESET_BYTES; i++) s_tx[o++] = 0x00;
}

static void ws_show(void)
{
	hal_status_t st;
	u32 t0 = HAL_GetTick();

	ws_pack();
	st = HAL_SPI_Transmit(&s_spi, s_tx, sizeof(s_tx), 50U);

	ws_last_status = (u32)st;
	ws_last_tx_ms  = HAL_GetTick() - t0;
	ws_last_sr     = SPI1->SR;
	ws_last_cfg1   = SPI1->CFG1;
	ws_last_cfg2   = SPI1->CFG2;
	ws_last_cr1    = SPI1->CR1;
	ws_last_cr2    = SPI1->CR2;
#if USE_HAL_SPI_GET_LAST_ERRORS
	ws_last_err    = s_spi.last_error_codes;
#endif

	if (st == HAL_OK) {
		ws_tx_ok++;
	} else {
		ws_tx_err++;
		/* Leave the peripheral in a state the next frame can use. */
		(void)HAL_SPI_Abort(&s_spi);
	}
}

/* ---------------- VCP logging (USART2 = ST-LINK VCP) ---------------- */
#if WS_LOG_ENABLE
static u8 s_uart_ok;

static void ws_uart_init(void)
{
	hal_gpio_config_t g;

	memset(&g, 0, sizeof(g));
	g.mode        = HAL_GPIO_MODE_ALTERNATE;
	g.pull        = HAL_GPIO_PULL_UP;
	g.speed       = HAL_GPIO_SPEED_FREQ_LOW;
	g.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
	g.alternate   = HAL_GPIO_AF_7;              /* USART2_TX/RX, AF7 */
	(void)HAL_GPIO_Init(HAL_GPIOA, HAL_GPIO_PIN_2 | HAL_GPIO_PIN_3, &g);

	(void)HAL_RCC_USART2_SetKernelClkSource(HAL_RCC_USART2_CLK_SRC_PCLK1);
	HAL_RCC_USART2_EnableClock();

	LL_USART_SetBaudRate(USART2, 144000000U, LL_USART_PRESCALER_DIV1,
	                     LL_USART_OVERSAMPLING_16, 115200U);
	LL_USART_ConfigXfer(USART2,
	                    LL_USART_DATAWIDTH_8_BIT | LL_USART_PARITY_NONE |
#if WS_LINK_ENABLE
	                    LL_USART_DIRECTION_TX_RX,
#else
	                    LL_USART_DIRECTION_TX,
#endif
	                    LL_USART_STOP_BIT_1);
	LL_USART_EnableDirectionTx(USART2);
#if WS_LINK_ENABLE
	/* Bench convenience: the VCP RX line also accepts STATE frames, so a PC
	 * can drive the strip with no F407 attached. */
	LL_USART_EnableDirectionRx(USART2);
#endif
	LL_USART_Enable(USART2);
#if WS_LINK_ENABLE
	LL_USART_EnableIT_RXNE_RXFNE(USART2);
	NVIC_SetPriority(USART2_IRQn, 5U);
	NVIC_EnableIRQ(USART2_IRQn);
#endif

	s_uart_ok = 1;
}

static void ws_uart_putc(char c)
{
	u32 spin = 0;
	while ((LL_USART_IsActiveFlag_TXE_TXFNF(USART2) == 0U) && (spin++ < 200000U)) {
		/* bounded: a broken VCP must never wedge the LED loop */
	}
	if (spin < 200000U) LL_USART_TransmitData8(USART2, (u8)c);
}

static void ws_uart_puts(const char *s)
{
	if (!s_uart_ok) return;
	while (*s) ws_uart_putc(*s++);
}

static void ws_uart_u32(u32 v)
{
	char buf[11];
	int  i = 0;

	if (v == 0U) { ws_uart_putc('0'); return; }
	while (v > 0U && i < 10) { buf[i++] = (char)('0' + (v % 10U)); v /= 10U; }
	while (i > 0) ws_uart_putc(buf[--i]);
}

static void ws_log_stage(u8 stage)
{
	ws_uart_puts("[DEMO ");
	ws_uart_u32(stage);
	ws_uart_puts("] ");
#if WS_DEMO_ENABLE
	ws_uart_puts(ws_demo_stage_name());
#else
	ws_uart_puts("?");
#endif
	ws_uart_puts("\r\n");
}
#else
#define ws_uart_init()   do { } while (0)
#define ws_uart_puts(s)  do { (void)(s); } while (0)
#define ws_uart_u32(v)   do { (void)(v); } while (0)
#define ws_log_stage(s)  do { (void)(s); } while (0)
#endif /* WS_LOG_ENABLE */

/* ---------------- host link -------------------------------------------
 * USART1 on PA9 (TX) / PA10 (RX), 115200 8N1, protocol in ws_link.h.
 * The VCP's RX line feeds the same parser, so the bench can be driven from
 * a PC when no F407 is attached.  Parsing happens in the 40 ms task; the
 * ISRs only push bytes into a ring.  At 13 bytes per STATE frame and 25 Hz
 * that is ~325 bytes/s, far below what 115200 or a 128-byte ring can take.
 * ---------------------------------------------------------------------- */
#if WS_LINK_ENABLE

volatile u32 ws_link_frames;
volatile u32 ws_link_crc_err;
volatile u32 ws_link_last_ms;

#define WS_RX_RING_SIZE   128u

static volatile u8  s_rx_ring[WS_RX_RING_SIZE];
static volatile u16 s_rx_head;
static volatile u16 s_rx_tail;

static void ws_rx_push(u8 b)
{
	u16 n = (u16)((s_rx_head + 1u) & (WS_RX_RING_SIZE - 1u));
	if (n != s_rx_tail) {
		s_rx_ring[s_rx_head] = b;
		s_rx_head = n;
	}
}

static u8 ws_rx_pop(u8 *b)
{
	if (s_rx_tail == s_rx_head) return 0;
	*b = s_rx_ring[s_rx_tail];
	s_rx_tail = (u16)((s_rx_tail + 1u) & (WS_RX_RING_SIZE - 1u));
	return 1;
}

void USART1_IRQHandler(void)
{
	if (LL_USART_IsActiveFlag_ORE(USART1)) LL_USART_ClearFlag_ORE(USART1);
	if (LL_USART_IsActiveFlag_RXNE_RXFNE(USART1))
		ws_rx_push(LL_USART_ReceiveData8(USART1));
	if (LL_USART_IsActiveFlag_FE(USART1)) LL_USART_ClearFlag_FE(USART1);
	if (LL_USART_IsActiveFlag_NE(USART1)) LL_USART_ClearFlag_NE(USART1);
}

void USART2_IRQHandler(void)
{
	if (LL_USART_IsActiveFlag_ORE(USART2)) LL_USART_ClearFlag_ORE(USART2);
	if (LL_USART_IsActiveFlag_RXNE_RXFNE(USART2))
		ws_rx_push(LL_USART_ReceiveData8(USART2));
	if (LL_USART_IsActiveFlag_FE(USART2)) LL_USART_ClearFlag_FE(USART2);
	if (LL_USART_IsActiveFlag_NE(USART2)) LL_USART_ClearFlag_NE(USART2);
}

static u8 ws_crc8_update(u8 crc, u8 b)
{
	u8 i;

	crc ^= b;
	for (i = 0; i < 8u; i++)
		crc = (crc & 0x80u) ? (u8)((crc << 1) ^ 0x07u) : (u8)(crc << 1);
	return crc;
}

static ws_link_state_t s_host;
static u8  s_host_seen;          /* a valid STATE frame has ever arrived */
static u8  s_link_ok_prev;       /* previous tick's link state, for logs  */
static u8  s_mode = WS_LINK_MODE_NORMAL;

enum {
	PS_SOF0 = 0, PS_SOF1, PS_VER, PS_TYPE, PS_LEN, PS_SEQ, PS_DATA, PS_CRC
};

static u8 s_ps = PS_SOF0;
static u8 s_p_type, s_p_len, s_p_idx, s_p_crc;
static u8 s_p_buf[WS_LINK_MAX_PAYLOAD];

static void ws_link_apply(const u8 *p)
{
	s_host.flags        = p[0];
	s_host.bright       = p[1];
	s_host.speed_cms    = (s16)((u16)p[2] | ((u16)p[3] << 8));
	s_host.nav_permille = (u16)((u16)p[4] | ((u16)p[5] << 8));
	s_host.mode         = p[6];
	s_host_seen         = 1;
	ws_link_last_ms     = HAL_GetTick();
	ws_link_frames++;
}

static void ws_link_feed(u8 b)
{
	switch (s_ps) {
	case PS_SOF0:
		if (b == WS_LINK_SOF0) s_ps = PS_SOF1;
		break;
	case PS_SOF1:
		if (b == WS_LINK_SOF1)      s_ps = PS_VER;
		else if (b != WS_LINK_SOF0) s_ps = PS_SOF0;
		break;
	case PS_VER:
		if (b == WS_LINK_VER) {
			s_p_crc = ws_crc8_update(0u, b);
			s_ps    = PS_TYPE;
		} else {
			s_ps = (b == WS_LINK_SOF0) ? PS_SOF1 : PS_SOF0;
		}
		break;
	case PS_TYPE:
		s_p_type = b;
		s_p_crc  = ws_crc8_update(s_p_crc, b);
		s_ps     = PS_LEN;
		break;
	case PS_LEN:
		if (b > WS_LINK_MAX_PAYLOAD) { s_ps = PS_SOF0; break; }
		s_p_len  = b;
		s_p_idx  = 0;
		s_p_crc  = ws_crc8_update(s_p_crc, b);
		s_ps     = PS_SEQ;
		break;
	case PS_SEQ:
		s_p_crc = ws_crc8_update(s_p_crc, b);
		s_ps    = (s_p_len == 0u) ? PS_CRC : PS_DATA;
		break;
	case PS_DATA:
		s_p_buf[s_p_idx++] = b;
		s_p_crc = ws_crc8_update(s_p_crc, b);
		if (s_p_idx >= s_p_len) s_ps = PS_CRC;
		break;
	case PS_CRC:
		if (b == s_p_crc) {
			if (s_p_type == WS_LINK_TYPE_STATE && s_p_len == WS_LINK_STATE_LEN)
				ws_link_apply(s_p_buf);
		} else {
			ws_link_crc_err++;
		}
		s_ps = PS_SOF0;
		break;
	default:
		s_ps = PS_SOF0;
		break;
	}
}

static void ws_link_drain(void)
{
	u8 b;
	while (ws_rx_pop(&b)) ws_link_feed(b);
}

static void ws_link_putc(u8 b)
{
	u32 spin = 0;
	while ((LL_USART_IsActiveFlag_TXE_TXFNF(USART1) == 0u) && (spin++ < 200000u)) {
		/* bounded: a disconnected host must never wedge the LED loop */
	}
	if (spin < 200000u) LL_USART_TransmitData8(USART1, b);
}

/* 5 Hz status frame so the host can tell the controller is alive. */
static void ws_link_status_tx(u8 link_ok)
{
	static u32 last_ms;
	u32 now = HAL_GetTick();
	u8 out[6u + WS_LINK_STATUS_LEN + 1u];
	u8 crc = 0;
	u8 i;

	if ((now - last_ms) < 200u) return;
	last_ms = now;

	out[0] = WS_LINK_SOF0;
	out[1] = WS_LINK_SOF1;
	out[2] = WS_LINK_VER;
	out[3] = WS_LINK_TYPE_STATUS;
	out[4] = WS_LINK_STATUS_LEN;
	out[5] = 0;                                  /* seq */
	out[6] = s_mode;
	out[7] = link_ok;
	out[8] = (ws_link_crc_err > 255u) ? 255u : (u8)ws_link_crc_err;
	out[9] = WS_LINK_VER;

	for (i = 2; i < 6u + WS_LINK_STATUS_LEN; i++) crc = ws_crc8_update(crc, out[i]);
	out[6u + WS_LINK_STATUS_LEN] = crc;

	for (i = 0; i < 6u + WS_LINK_STATUS_LEN + 1u; i++) ws_link_putc(out[i]);
}

/* USART1 on PA9/PA10 uses AF7 on STM32C5 (same AF as USART2 on PA2/PA3).
 * The HAL also exposes AF4 for USART1 (the PB14/PB15 route); if RX never
 * sees a byte from the F407, build with -DWS_LINK_USART1_AF=HAL_GPIO_AF_4. */
#ifndef WS_LINK_USART1_AF
#define WS_LINK_USART1_AF   HAL_GPIO_AF_7
#endif

static void ws_link_init(void)
{
	hal_gpio_config_t g;

	memset(&g, 0, sizeof(g));
	g.mode        = HAL_GPIO_MODE_ALTERNATE;
	g.pull        = HAL_GPIO_PULL_UP;
	g.speed       = HAL_GPIO_SPEED_FREQ_LOW;
	g.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
	g.alternate   = WS_LINK_USART1_AF;           /* USART1_TX/RX on PA9/PA10 */
	(void)HAL_GPIO_Init(HAL_GPIOA, HAL_GPIO_PIN_9 | HAL_GPIO_PIN_10, &g);

	(void)HAL_RCC_USART1_SetKernelClkSource(HAL_RCC_USART1_CLK_SRC_PCLK2);
	HAL_RCC_USART1_EnableClock();

	LL_USART_SetBaudRate(USART1, 144000000U, LL_USART_PRESCALER_DIV1,
	                     LL_USART_OVERSAMPLING_16, WS_LINK_BAUD);
	LL_USART_ConfigXfer(USART1,
	                    LL_USART_DATAWIDTH_8_BIT | LL_USART_PARITY_NONE |
	                    LL_USART_DIRECTION_TX_RX,
	                    LL_USART_STOP_BIT_1);
	LL_USART_EnableDirectionTx(USART1);
	LL_USART_EnableDirectionRx(USART1);
	LL_USART_EnableIT_RXNE_RXFNE(USART1);
	LL_USART_Enable(USART1);

	NVIC_SetPriority(USART1_IRQn, 5U);
	NVIC_EnableIRQ(USART1_IRQn);
}

#else  /* !WS_LINK_ENABLE */
#define ws_link_drain()          do { } while (0)
#define ws_link_status_tx(ok)    do { (void)(ok); } while (0)
#endif /* WS_LINK_ENABLE */

/* GPIOD effectively: PA7 = SPI1_MOSI (AF5), PB7 = SPI1_SCK (AF6),
 * PC13 = B1, PA5 = LD1, PA2/PA3 = USART2 VCP, PA9/PA10 = USART1 host. */
int ws2811_init(void)
{
	hal_gpio_config_t gpio;
	hal_spi_config_t  cfg;

	/* --- strip data pin: PA7, AF5, push-pull, idle low --------------- */
	memset(&gpio, 0, sizeof(gpio));
	gpio.mode        = HAL_GPIO_MODE_ALTERNATE;
	gpio.pull        = HAL_GPIO_PULL_DOWN;      /* DIN defined low while idle */
	/* Medium slew: 2.25 MHz does not need a fast edge, and a slower edge
	 * rings less on the strip's DIN wire. */
	gpio.speed       = HAL_GPIO_SPEED_FREQ_MEDIUM;
	gpio.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
	gpio.alternate   = HAL_GPIO_AF_5;
	gpio.init_state  = HAL_GPIO_PIN_RESET;
	if (HAL_GPIO_Init(HAL_GPIOA, HAL_GPIO_PIN_7, &gpio) != HAL_OK) return -1;

	/* --- SPI1_SCK on PB7 (AF6).  The strip does not use SCK, but the
	 * C5 SPI master needs a real clock pin routed before it will shift
	 * the FIFO out.  PB7 is on the ST morpho header and unused here. --- */
	gpio.alternate = HAL_GPIO_AF_6;
	if (HAL_GPIO_Init(HAL_GPIOB, HAL_GPIO_PIN_7, &gpio) != HAL_OK) return -7;

	/* --- user button B1: PC13, external 10k pull-down, pressed = high - */
	memset(&gpio, 0, sizeof(gpio));
	gpio.mode        = HAL_GPIO_MODE_INPUT;
	gpio.pull        = HAL_GPIO_PULL_DOWN;
	gpio.speed       = HAL_GPIO_SPEED_FREQ_LOW;
	gpio.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
	gpio.init_state  = HAL_GPIO_PIN_RESET;
	if (HAL_GPIO_Init(HAL_GPIOC, HAL_GPIO_PIN_13, &gpio) != HAL_OK) return -2;

	/* --- user LED LD1: PA5, active high (through a MOSFET) ------------ */
	memset(&gpio, 0, sizeof(gpio));
	gpio.mode        = HAL_GPIO_MODE_OUTPUT;
	gpio.pull        = HAL_GPIO_PULL_NO;
	gpio.speed       = HAL_GPIO_SPEED_FREQ_LOW;
	gpio.output_type = HAL_GPIO_OUTPUT_PUSHPULL;
	gpio.init_state  = HAL_GPIO_PIN_RESET;
	if (HAL_GPIO_Init(HAL_GPIOA, HAL_GPIO_PIN_5, &gpio) != HAL_OK) return -3;

	/* --- SPI1: master, TX only, 8-bit, mode 0, MSB, /64 = 2.25 MHz ---- */
	if (HAL_RCC_SPI1_SetKernelClkSource(HAL_RCC_SPI1_CLK_SRC_PCLK2) != HAL_OK) return -4;

	if (HAL_SPI_Init(&s_spi, HAL_SPI1) != HAL_OK) return -5;

	memset(&cfg, 0, sizeof(cfg));
	cfg.mode                 = HAL_SPI_MODE_MASTER;
	cfg.direction            = HAL_SPI_DIRECTION_SIMPLEX_TX;
	cfg.data_width           = HAL_SPI_DATA_WIDTH_8_BIT;
	cfg.clock_polarity       = HAL_SPI_CLOCK_POLARITY_LOW;
	cfg.clock_phase          = HAL_SPI_CLOCK_PHASE_1_EDGE;
	cfg.baud_rate_prescaler  = WS_SPI_PRESC;
	cfg.first_bit            = HAL_SPI_MSB_FIRST;
	cfg.nss_pin_management   = HAL_SPI_NSS_PIN_MGMT_INTERNAL;
	if (HAL_SPI_SetConfig(&s_spi, &cfg) != HAL_OK) return -6;

#if WS_LINK_ENABLE
	ws_link_init();                 /* USART1 PA9/PA10, 115200 8N1 */
#endif
#if WS_LOG_ENABLE
	ws_uart_init();
	ws_uart_puts("\r\nWS2811 C542 LED controller\r\n  pixels=");
	ws_uart_u32((u32)WS_PIXELS);
	ws_uart_puts(" (2m, 120 LED/m, ");
	ws_uart_u32((u32)WS_LEDS_PER_IC);
	ws_uart_puts(" LED/IC)\r\n  bright=");
	ws_uart_u32((u32)WS_BRIGHT_MAX);
	ws_uart_puts("/255  data=PA7/D11 SPI1 ");
	ws_uart_u32(WS_SPI_KHZ);
	ws_uart_puts("kHz order=");
	ws_uart_puts(WS_ORDER_STR);
	ws_uart_puts("\r\n  host=USART1 PA9(TX)/PA10(RX) 115200 8N1\r\n"
	             "  B1=demo stage (only when no host), LD1=heartbeat\r\n");
#endif

	return 0;
}

/* One-shot bench self-test.  Not part of the car firmware.
 *   red -> green -> blue -> three-arc probe -> idle
 * The three-arc probe is the ground truth for WS_ORDER_*: report the
 * colour of the FIRST / MIDDLE / LAST third as they actually light up. */
#if WS_BOOT_TEST
static void ws_boot_test(void)
{
	ws_fill_solid(255,   0,   0); ws_show(); HAL_Delay(600);
	ws_fill_solid(  0, 255,   0); ws_show(); HAL_Delay(600);
	ws_fill_solid(  0,   0, 255); ws_show(); HAL_Delay(600);
	ws_render_rgb_probe();        ws_show(); HAL_Delay(2000);
	ws_fill_solid(  0,   0,   0); ws_show(); HAL_Delay(200);
}
#endif

#if WS_LOAD_TEST
static void ws_load_test(void)
{
	static const u8 col[3][3] = {
		{255,   0,   0},   /* red:   1 channel  */
		{  0, 255, 255},   /* cyan:  2 channels */
		{255, 255, 255},   /* white: 3 channels */
	};
	static const char *const nm[3] = {"RED (1CH)", "CYAN (2CH)", "WHITE (3CH)"};
	u8  c;
	u16 i;

	ws_uart_puts("[LOAD] 30% steady, 5 s per colour\r\n");
	for (;;) {
		for (c = 0; c < 3; c++) {
			ws_fill_solid(col[c][0], col[c][1], col[c][2]);
			ws_uart_puts("[LOAD] ");
			ws_uart_puts(nm[c]);
			ws_uart_puts(" ON\r\n");
			for (i = 0; i < 125U; i++) { ws_show(); HAL_Delay(40); }

			ws_fill_solid(0, 0, 0);
			ws_uart_puts("[LOAD] ");
			ws_uart_puts(nm[c]);
			ws_uart_puts(" OFF\r\n");
			for (i = 0; i < 25U; i++) { ws_show(); HAL_Delay(40); }
		}
	}
}
#endif /* WS_LOAD_TEST */

#if WS_BRIGHT_LADDER
static void ws_bright_ladder(void)
{
	/* 5, 10, 15, 20, 25, 30 % of 255 */
	static const u8 lvl[6] = {13, 26, 38, 51, 64, 77};
	static const u8 pct[6] = { 5, 10, 15, 20, 25, 30};
	u8  i;
	u16 j;

	ws_uart_puts("[LADDER] 6 levels, white then cyan, 4 s each\r\n");
	for (;;) {
		for (i = 0; i < 6; i++) {
			ws_fill_solid(lvl[i], lvl[i], lvl[i]);        /* white */
			ws_uart_puts("[LADDER] ");
			ws_uart_u32((u32)pct[i]);
			ws_uart_puts("% WHITE (");
			ws_uart_u32((u32)lvl[i]);
			ws_uart_puts("/255)\r\n");
			for (j = 0; j < 100U; j++) { ws_show(); HAL_Delay(40); }

			ws_fill_solid(0, lvl[i], lvl[i]);             /* cyan  */
			ws_uart_puts("[LADDER] ");
			ws_uart_u32((u32)pct[i]);
			ws_uart_puts("% CYAN  (");
			ws_uart_u32((u32)lvl[i]);
			ws_uart_puts("/255)\r\n");
			for (j = 0; j < 100U; j++) { ws_show(); HAL_Delay(40); }
		}
	}
}
#endif /* WS_BRIGHT_LADDER */

#if WS_FIXED_TEST
static void ws_fixed_test(void)
{
	ws_uart_puts("[FIXED] 13/255 (5%) white, constant\r\n");
	ws_fill_solid(255, 255, 255);
	for (;;) {
		ws_show();
		HAL_Delay(40);
	}
}
#endif /* WS_FIXED_TEST */

#if WS_BRINGUP_STATIC
/**************************************************************************
Function: Low-brightness static wiring/decoder test.
          Every phase is sent for ~1.8 s, then held for 1.5 s with the SPI
          completely stopped -- a healthy WS2811 strip must hold its last
          colour.  The first phase drives ONLY pixels 0..79 white:
            - whole 2 m lights  -> 80-pixel strip (3 LEDs per IC)
            - only first ~0.67m -> 240-pixel strip (1 LED per IC)
**************************************************************************/
static void ws_static_send(u16 frames)
{
	u16 i;
	for (i = 0; i < frames; i++) {
		ws_show();
		HAL_Delay(40);
	}
}

static void ws_static_test(void)
{
	static const u8 col[4][3] = {
		{255,   0,   0},   /* logical red   */
		{  0, 255,   0},   /* logical green */
		{  0,   0, 255},   /* logical blue  */
		{255, 255, 255},   /* white         */
	};
	static const char *const nm[4] = {"RED", "GREEN", "BLUE", "WHITE"};
	u8  c;
	u16 i;

	ws_uart_puts("[STATIC] pixels=");
	ws_uart_u32((u32)WS_PIXELS);
	ws_uart_puts(" bright=");
	ws_uart_u32((u32)WS_BRIGHT_MAX);
	ws_uart_puts(" spi_khz=");
	ws_uart_u32((u32)WS_SPI_KHZ);
	ws_uart_puts("\r\n");

	for (;;) {
		/* --- pixel-count test: only first 80 pixels white --- */
		ws_clear();
		for (i = 0; i < 80U && i < WS_PIXELS; i++) ws_put((u8)i, 255, 255, 255);
		ws_uart_puts("[STATIC] FIRST80 WHITE send\r\n");
		ws_static_send(38);
		ws_uart_puts("[STATIC] FIRST80 WHITE hold (no SPI)\r\n");
		HAL_Delay(1500);

		for (c = 0; c < 4; c++) {
			ws_fill_solid(col[c][0], col[c][1], col[c][2]);
			ws_uart_puts("[STATIC] ");
			ws_uart_puts(nm[c]);
			ws_uart_puts(" send\r\n");
			ws_static_send(38);
			ws_uart_puts("[STATIC] ");
			ws_uart_puts(nm[c]);
			ws_uart_puts(" hold (no SPI)\r\n");
			HAL_Delay(1500);
		}
	}
}
#endif /* WS_BRINGUP_STATIC */

/* Button debounce.  B1 on this board is active-high with only an external
 * 10 k pull-down (no RC filter), and the strip's 12 V current pulses ride on
 * the same ground.  A single noisy rising edge used to advance a demo stage,
 * so the strip appeared to change effect / strobe on its own.  Require a
 * LEVEL that is stable for 200 ms, and require a stable release before the
 * next press is accepted.  A held or stuck-high pin can only advance once. */
#define WS_BTN_STABLE_MS  200

/* Bare-metal replacement for the F407 FreeRTOS ws2811_task().  The tick is
 * FIXED at 40 ms: ws_state_step() is hard-calibrated for it (WS_FADE_STEP,
 * the 100 ms enter dwell and the 500 ms exit dwell all assume 40 ms). */
void ws2811_task(void)
{
	WS_State   st;
	WS_Overlay ov;
	WS_SpeedOut sp;
	u32 ms = 0;
	u16 btn_high_ms = 0;
	u16 btn_low_ms  = 0;
	u8  btn_armed   = 1;
	u8  stage_logged = 0xFF;

#if WS_CALIB_MODE
	u8 calib = 0;
#endif

	if (ws2811_init() != 0) {
		/* Leave LD1 blinking fast so a wiring/init failure is visible
		 * even with no strip attached. */
		for (;;) {
			HAL_GPIO_WritePin(HAL_GPIOA, HAL_GPIO_PIN_5, HAL_GPIO_PIN_SET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(HAL_GPIOA, HAL_GPIO_PIN_5, HAL_GPIO_PIN_RESET);
			HAL_Delay(100);
		}
	}

#if WS_FIXED_TEST
	/* One fixed dim-white state, no percentage labels. */
	ws_fixed_test();
#elif WS_BRIGHT_LADDER
	/* Find the highest clean brightness level. */
	ws_bright_ladder();
#elif WS_LOAD_TEST
	/* Power/current load test: red -> cyan -> white at a fixed 30 %. */
	ws_load_test();
#elif WS_BRINGUP_STATIC
	/* Diagnostic build: no effect engine, no button, dim static colours. */
	ws_static_test();
#else
	ws_state_init(&st);

#if WS_BOOT_TEST
	ws_boot_test();
#endif

	for (;;) {
		u32 t0 = HAL_GetTick();
		u8  btn;
		float v;
#if WS_LINK_ENABLE
		u8  link_ok;

		/* Bytes the UART ISRs collected since the last tick. */
		ws_link_drain();
		link_ok = (u8)((s_host_seen != 0u) &&
		               ((t0 - ws_link_last_ms) <= WS_LINK_TIMEOUT_MS));
		if (link_ok != s_link_ok_prev) {
			s_link_ok_prev = link_ok;
			ws_uart_puts(link_ok ? "[LINK] up\r\n" : "[LINK] lost\r\n");
		}
#endif

		/* B1: active-high, but hold the new level stable for 200 ms
		 * before accepting it.  Ignore short spikes and require a
		 * stable release before the next press. */
		btn = (HAL_GPIO_ReadPin(HAL_GPIOC, HAL_GPIO_PIN_13) == HAL_GPIO_PIN_SET)
		    ? 1U : 0U;
		if (btn) {
			btn_low_ms = 0;
			if (btn_high_ms < WS_BTN_STABLE_MS) btn_high_ms += WS_STEP_MS;
		} else {
			btn_high_ms = 0;
			if (btn_low_ms < WS_BTN_STABLE_MS) btn_low_ms += WS_STEP_MS;
		}
		if (btn_armed && (btn_high_ms >= WS_BTN_STABLE_MS)) {
#if WS_LINK_ENABLE
			/* While a host is talking, B1 does nothing; when the link
			 * is down it takes the ring back from the host preview. */
			if (!link_ok) {
				ws_demo_advance();
				s_host_seen = 0;
			}
#else
			ws_demo_advance();
#endif
			btn_armed = 0;
		}
		if (!btn_armed && (btn_low_ms >= WS_BTN_STABLE_MS)) {
			btn_armed = 1;
		}

#if WS_DEMO_ENABLE
		/* Report every stage change on the ST-LINK VCP. */
		if (ws_demo_stage_no() != stage_logged) {
			stage_logged = ws_demo_stage_no();
			ws_log_stage(stage_logged);
		}
#endif

		/* Defaults, then whichever source is active paints over them. */
		memset(&ov, 0, sizeof(ov));
		ov.nav_permille = 0xFFFF;
		v = 0.0f;

#if WS_LINK_ENABLE
		if (link_ok) {
			/* Host drives: its overlay is authoritative. */
			u8 f = s_host.flags;

			ov.estop    = (f & WS_LINK_F_ESTOP)   ? 1u : 0u;
			ov.lowbat   = (f & WS_LINK_F_LOWBAT)  ? 1u : 0u;
			ov.brake    = (f & WS_LINK_F_BRAKE)   ? 1u : 0u;
			ov.reverse  = (f & WS_LINK_F_REVERSE) ? 1u : 0u;
			ov.ros_link = (u8)((f & WS_LINK_F_ROS_MASK) >> WS_LINK_F_ROS_SHIFT);
			if      (f & WS_LINK_F_TURN_L) ov.turn =  1;
			else if (f & WS_LINK_F_TURN_R) ov.turn = -1;
			ov.nav_permille = s_host.nav_permille;
			v = (float)s_host.speed_cms / 100.0f;

			s_bright = (s_host.bright > WS_BRIGHT_MAX)
			         ? WS_BRIGHT_MAX : s_host.bright;
			s_mode   = s_host.mode;

			if (s_mode == WS_LINK_MODE_BLANK) {
				ws_clear();
			} else {
				ws_state_step(&st, v);
				sp = ws_speed_map(v);
				ws_render(&st, sp, &ov, ms);
			}
		} else if (s_host_seen) {
			/* Host was alive and went quiet: never freeze on the
			 * last picture.  The engine's "ROS lost" cue takes over. */
			ov.ros_link = 2;
			s_bright    = WS_BRIGHT_MAX;
			ws_state_step(&st, 0.0f);
			sp = ws_speed_map(0.0f);
			ws_render(&st, sp, &ov, ms);
		} else
#endif
		{
			/* Bench preview: exactly the pre-link behaviour. */
			ov.ros_link = 1;
			s_bright    = WS_BRIGHT_MAX;

#if WS_DEMO_ENABLE
			{
				u8 painted = ws_demo_frame(&v, &ov, ms);
				if (!painted)
#endif
				{
					ws_state_step(&st, v);
					sp = ws_speed_map(v);

#if WS_CALIB_MODE
					ws_render_calib(calib);
					calib = (u8)((calib + 1) % WS_PIXELS);
#else
					ws_render(&st, sp, &ov, ms);
#endif
				}
#if WS_DEMO_ENABLE
			}
#endif
		}

		ws_show();
#if WS_LINK_ENABLE
		ws_link_status_tx(link_ok);
#endif

		/* LD1 heartbeat: 500 ms on / 500 ms off. */
		HAL_GPIO_WritePin(HAL_GPIOA, HAL_GPIO_PIN_5,
		                  ((ms / 500u) & 1u) ? HAL_GPIO_PIN_SET : HAL_GPIO_PIN_RESET);

		/* Keep the effect clock at exactly 40 ms even though ws_show()
		 * blocks for ~3 ms. */
		{
			u32 elapsed = HAL_GetTick() - t0;
			if (elapsed < WS_STEP_MS) HAL_Delay(WS_STEP_MS - elapsed);
		}
		ms += WS_STEP_MS;
	}
#endif /* WS_BRINGUP_STATIC */
}
/* ==== WS_HW_END ==== */
