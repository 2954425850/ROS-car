#ifndef __WS2811_H
#define __WS2811_H
#include "sys.h"
#include "system.h"

/* ---------------- master switch ---------------- */
/* 0 = whole effect engine compiled out at the call site, code kept intact */
#define WS2811_ENABLE       1

/* ---------------- geometry ---------------- */
#define WS_PIXELS           40      /* 2m * 60 LED/m / 3 LEDs-per-chip   */

/* ---------------- wiring / calibration ---------------- */
/* +1 or -1: increasing pixel index runs clockwise or counter-clockwise   */
#define WS_DIR              1
/* 0..39: physical index of the pixel facing straight ahead (nose)       */
#define WS_PHASE            0
/* 1 = boot into calibration mode: one white pixel walks the ring        */
#define WS_CALIB_MODE       0
/* Physical byte order of the strip. Set exactly ONE of these to 1.
 *
 * This is MEASURED, not guessed. Demo stage 16 (RGB PROBE) lights three
 * arcs -- the first third of the ring driven with pure red, the middle
 * third with pure green, the last third with pure blue -- so you can read
 * the real order straight off the strip.
 *
 *   RBG : this chassis's 2m strip, measured 2026-09-18. Emitting GRB made
 *         every colour come out cyclically rotated: red showed as blue,
 *         green as orange, deep blue as green.
 *   GRB : the most common WS2811 order
 *   RGB : the other common order
 */
#define WS_ORDER_RBG        1
#define WS_ORDER_GRB        0
#define WS_ORDER_RGB        0

/* ---------------- brightness / power ---------------- */
#define WS_BRIGHT_MAX       80      /* 0..255, global ceiling            */

/* ---------------- speed scale ---------------- */
#define WS_VMAX_MPS         0.75f   /* full-scale wheel speed, m/s       */
#define WS_VMIN_MPS         0.02f   /* below this the ring is "idle"     */

/* ---------------- low battery ---------------- */
#define WS_LOWBAT_V         10.0f   /* matches ROS_FAULT_UNDERVOLT doc   */

/* ---------------- FreeRTOS task ---------------- */
#define WS2811_TASK_PRIO    2
#define WS2811_STK_SIZE     256

/* ---------------- logic-layer API (host-testable) ---------------- */
typedef struct { u16 hue; u8 bright; } WS_SpeedOut;

void ws_hsv2rgb(u16 h, u8 s, u8 v, u8 *r, u8 *g, u8 *b);
WS_SpeedOut ws_speed_map(float mps);

/* One source byte -> the three SPI bytes carrying it (see ws2811.c).
 * Lives in the logic region so the host suite can assert the bit pattern. */
void ws_pack_byte(u8 src, u8 out[3]);

typedef struct {
	float filt;      /* 100ms low-passed |v|                    */
	u8    moving;    /* 0/1 latched motion state                */
	u8    blend;     /* 0 = idle layer, 255 = motion layer      */
	u16   start_ms;  /* dwell above the enter threshold         */
	u16   stop_ms;   /* dwell below the exit threshold          */
} WS_State;

void ws_state_init(WS_State *st);
void ws_state_step(WS_State *st, float v_abs);

/* ---------------- frame buffer / index mapping ---------------- */
extern u8 ws_frame[WS_PIXELS * 3];

void ws_clear(void);
void ws_put(u8 phys, u8 r, u8 g, u8 b);
u8   ws_map(u8 logical);
void ws_fill_solid(u8 r, u8 g, u8 b);
void ws_render_calib(u8 step);
void ws_render_rgb_probe(void);

/* ---------------- overlay + idle layer (host-testable) ---------------- */
typedef struct {
	u8  estop;         /* 1 = full-band red strobe              */
	u8  lowbat;        /* 1 = full-band orange breathe, capped  */
	u8  brake;         /* 1 = force red, quick flashes          */
	s8  turn;          /* +1 left / 0 none / -1 right           */
	u8  reverse;       /* 1 = white flow                        */
	u16 nav_permille;  /* 0..1000 nav progress, 0xFFFF = off    */
	u8  ros_link;      /* 0 waiting / 1 connected / 2 lost      */
} WS_Overlay;

u8   ws_breath(u32 ms, u16 period_ms, u8 lo, u8 hi);
void ws_render_idle(const WS_Overlay *ov, u32 ms);

/* ---------------- composite render (L2 motion + L3 overlays) ---------------- */
void ws_render(const WS_State *st, WS_SpeedOut sp, const WS_Overlay *ov, u32 ms);

/* ---------------- PE0 preview / demo mode ---------------- */
/* 0 = demo code compiled out at its hooks, block kept intact.
 * Set to 0 to kill the feature without touching anything else. */
#define WS_DEMO_ENABLE      1
#define WS_DEMO_STAGES      16      /* 1..16, plus 0 = off */

#if WS_DEMO_ENABLE
u8   ws_demo_frame(float *v, WS_Overlay *ov, u32 ms);
u8   ws_demo_active(void);
u8   ws_demo_stage_no(void);
void ws_demo_advance(void);
void ws_demo_oled(void);
#endif

/* ---------------- hardware-layer API ---------------- */
void ws2811_task(void *pvParameters);

#endif
