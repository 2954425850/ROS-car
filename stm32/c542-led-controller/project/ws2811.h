/**
  ******************************************************************************
  * file           : ws2811.h
  * brief          : WS2811/WS2812 LED-strip effect engine + STM32C542 SPI driver.
  *
  * This is the STM32C542 (NUCLEO-C542RC) port of the car's WS2811 task.
  * The effect engine (everything down to ws_pack_byte) is deliberately the
  * same code that runs on the F407 chassis; only the hardware layer at the
  * bottom is C5-specific.  Keeping the split means the F407 port later only
  * has to replace ws2811_hw_init()/ws_show() again.
  *
  * New strip: 12 V WS2811, 120 LED/m, 2 m -> 240 LEDs.
  * The usual 12 V strip groups 3 LEDs behind one WS2811 chip, so the frame
  * has 80 addressable pixels.
  *
  * Wiring on NUCLEO-C542RC:
  *   strip DIN  -> PA7  (Arduino D11, SPI1_MOSI, AF5)
  *   strip GND  -> board GND   (MANDATORY, even with a separate 12 V PSU)
  *   strip +12V -> external 12 V PSU
  *   B1 (blue user button, PC13) -> next demo stage
  *   LD1 (green user LED, PA5)   -> 1 Hz heartbeat
 *
 * Host link (the car's F407 sends "what to show", this board renders it):
 *   host TX -> PA10 (Arduino D2, USART1_RX)
 *   host RX <- PA9  (Arduino D8, USART1_TX)
 *   GND     -- GND
 *   USART2 (PA2/PA3) stays the ST-LINK VCP: logs out, and the same binary
 *   STATE frames may be injected there from a PC for bench testing.
  ******************************************************************************
  */
#ifndef __WS2811_H
#define __WS2811_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Legacy-style short types used by the shared effect engine. */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t   s8;
typedef int16_t  s16;

/* ---------------- master switch ---------------- */
#define WS2811_ENABLE       1

/* ---------------- geometry ---------------- */
/* Confirmed by the user: 3 LEDs per WS2811 IC -> 2 m * 120 / 3 = 80. */
#define WS_LEDS_PER_METER   120
#define WS_LEDS_PER_IC      3
#define WS_STRIP_LEN_M      2
#define WS_PIXELS           (WS_STRIP_LEN_M * WS_LEDS_PER_METER / WS_LEDS_PER_IC)

/* ---------------- wiring / calibration ---------------- */
/* +1 or -1: increasing pixel index runs clockwise or counter-clockwise   */
#define WS_DIR              1
/* 0..WS_PIXELS-1: physical index of the pixel facing straight ahead     */
#define WS_PHASE            0
/* 1 = boot into calibration mode: one white pixel walks the strip       */
#define WS_CALIB_MODE       0

/* Physical byte order of the strip. Set exactly ONE of these to 1.
 *
 * Measured 2026-11-25 from the 16 demo stages with BGR firmware:
 *   DEMO 0/1 cyan   -> purple      (R+B)
 *   DEMO 2   yellow -> cyan        (G+B)
 *   DEMO 3   purple -> yellow      (R+G)
 *   DEMO 7   red    -> green
 *   DEMO 16  probe  -> green/blue/red
 * All of these are exactly what a RBG strip shows when the firmware packs
 * BGR.  So the strip is **RBG** (same family as the old 60 LED/m strip).
 * With RBG packing the probe should read red/green/blue. */
#define WS_ORDER_RBG        1   /* measured on the new 120 LED/m strip */
#define WS_ORDER_GRB        0
#define WS_ORDER_RGB        0
#define WS_ORDER_BGR        0

/* ---------------- brightness / power ---------------- */
/* 0..255 global ceiling.  80 == ~31 %, same as the F407 firmware.  A 2 m
 * 240-LED 12 V strip is already noticeable at this level and a single
 * 12 V/3 A supply is comfortably enough. */
/* Max-power test: 100 % (255/255).  Full-white current is roughly
 * 80 IC x 3 channels x 16 mA = 3.8 A at 12 V; make sure the supply and the
 * ground return can take it.  Step back to 77 (30 %) if it sags. */
#define WS_BRIGHT_MAX       255

/* DEMO 0 idle styling.  1 = original F407 behaviour (cyan 2 s breathe).
 * 0 = steady cyan, used while isolating the "bright/dim flicker" report:
 * if steady cyan is stable, the flicker was the breathing effect itself. */
#define WS_IDLE_BREATHE     1

/* ---------------- bring-up wiring/decoder test ---------------- */
/* Set to 1 for a static, low-brightness test build:
 *   1) first 80 pixels white (tells 80-pixel vs 240-pixel strip)
 *   2) solid red / green / blue / white, each sent for 1.5 s and then
 *      held for 1.5 s with SPI completely stopped
 *   3) 240-pixel frame, so the same solid colour covers either a
 *      80-pixel (3 LED/IC) or a 240-pixel (1 LED/IC) strip
 *   4) WS_BRIGHT_MAX = 16 so it is dim and does not strain the supply
 * Set back to 0 after diagnosis. */
#define WS_BRINGUP_STATIC   0

/* Load test: steady 30 % frames in red (1 channel), cyan (2 channels),
 * white (3 channels), 5 s each.  If red is stable but cyan/white flicker,
 * the 12 V supply / ground return is current-limited; if all three are
 * stable, the earlier flicker was the effect, not the supply. */
#define WS_LOAD_TEST        0

/* Brightness ladder: six levels 5/10/15/20/25/30 %, each as steady white
 * (worst case, 3 channels) then steady cyan (2 channels), 4 s each.  Watch
 * the highest level that stays clean; report it. */
#define WS_BRIGHT_LADDER    0

/* One fixed state: 5 % white, sent continuously.  No labels to mismatch. */
#define WS_FIXED_TEST       0

#if WS_BRINGUP_STATIC
#undef  WS_PIXELS
#define WS_PIXELS           240
#undef  WS_BRIGHT_MAX
#define WS_BRIGHT_MAX       16
#endif

/* SPI prescaler from 144 MHz PCLK2.  64 -> 2.25 MHz WS-bit period 1.333 us
 * (800 kHz class).  If the strip turns out to be a 400 kHz variant, try 128
 * -> 1.125 MHz, period 2.667 us. */
#define WS_SPI_DIV          64

/* ---------------- speed scale ---------------- */
#define WS_VMAX_MPS         0.75f   /* full-scale wheel speed, m/s       */
#define WS_VMIN_MPS         0.02f   /* below this the strip is "idle"     */

/* ---------------- low battery ---------------- */
#define WS_LOWBAT_V         10.0f   /* matches ROS_FAULT_UNDERVOLT doc    */

/* ---------------- effect engine API (host-testable) ---------------- */
typedef struct { u16 hue; u8 bright; } WS_SpeedOut;

void ws_hsv2rgb(u16 h, u8 s, u8 v, u8 *r, u8 *g, u8 *b);
WS_SpeedOut ws_speed_map(float mps);

/* One source byte -> the three SPI bytes carrying it (see ws2811.c). */
void ws_pack_byte(u8 src, u8 out[3]);

typedef struct {
  float filt;      /* 100 ms low-passed |v|                  */
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

/* ---------------- demo / bench preview ---------------- */
#define WS_DEMO_ENABLE      1
#define WS_DEMO_STAGES      16      /* 1..16, plus 0 = off */

/* 0 = normal operation (idle cyan on boot, B1 to preview stages).
 * 1 = one-shot R/G/B/ramp probe at boot.  The strip has already been
 * characterised, so normal operation leaves this off. */
#define WS_BOOT_TEST        0

/* 0 = no VCP logging.  1 = stage/boot lines on USART2 (ST-LINK VCP,
 * 115200 8N1).  Logging is blocking but only happens on stage changes. */
#define WS_LOG_ENABLE       1

#if WS_DEMO_ENABLE
u8   ws_demo_frame(float *v, WS_Overlay *ov, u32 ms);
u8   ws_demo_active(void);
u8   ws_demo_stage_no(void);
void ws_demo_advance(void);
const char *ws_demo_stage_name(void);
#endif

/* ---------------- host link (USART1 PA9/PA10) ---------------- */
/* 1 = accept STATE frames from the F407 (or from a PC on the VCP).  When a
 * host has never spoken, the bench demo and B1 keep working exactly as
 * before; once frames arrive the host owns the strip. */
#define WS_LINK_ENABLE      1

/* Diagnostics, visible through the debugger. */
extern volatile u32 ws_link_frames;    /* valid STATE frames applied      */
extern volatile u32 ws_link_crc_err;   /* frames dropped on CRC/length    */
extern volatile u32 ws_link_last_ms;   /* tick of the last valid frame    */

/* ---------------- hardware-layer API ---------------- */
/* Initialise PA7/SPI1 and the on-board LED/button.  Returns 0 on success. */
int  ws2811_init(void);
/* Blocking 40 ms tick: button, effect engine, SPI frame.  Bare-metal
 * replacement for the FreeRTOS ws2811_task(). */
void ws2811_task(void) __attribute__((noreturn));

/* Diagnostics, visible through the debugger. */
extern volatile u32 ws_tx_ok;
extern volatile u32 ws_tx_err;

#ifdef __cplusplus
}
#endif

#endif /* __WS2811_H */
