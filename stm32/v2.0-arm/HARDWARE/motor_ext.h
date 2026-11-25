#ifndef __MOTOR_EXT_H
#define __MOTOR_EXT_H

#include "sys.h"

/* ------------------------------------------------------------------
 * External dual-channel motor driver, wired to the six reserved servo
 * ports instead of the on-board A/B/C/D outputs.
 *
 * The board takes a direction pair and a PWM per channel, and drives it
 * as sign-magnitude:
 *     INAx=L INBx=H  ->  forward, PWM out on Mx_A
 *     INAx=H INBx=L  ->  reverse, PWM out on Mx_B
 *     INAx=L INBx=L  ->  brake   (both motor terminals low)
 *     INAx=H INBx=H  ->  coast   (both motor terminals high-Z)
 * so duty 0 with both IN low is a brake, and the PWM frequency has to sit
 * between 18kHz and 60kHz. This code runs TIM8 at 20kHz.
 *
 * Wiring, per the C30D note that the six servo signals are
 * PC6 PC7 PC8 PC9 PB14 PB15:
 *     J1  PWM1  PC6   TIM8_CH1   right wheel
 *     J2  INA1  PC7   GPIO out
 *     J4  INB1  PC8   GPIO out
 *     J5  PWM2  PC9   TIM8_CH4   left wheel
 *     J6  INA2  PB14  GPIO out
 *     J7  INB2  PB15  GPIO out
 * J3 is not connected to the driver.
 *
 * Channel 1 (J1/J2/J4) is the right wheel and channel 2 (J5/J6/J7) the
 * left wheel, so MotorExt_SetTargets() takes its arguments in that order:
 * right first, left second.
 *
 * Every pin is written out literally below and again in motor_ext.c, the
 * way the rest of this firmware addresses its hardware (`PWMA1 TIM10->CCR1`
 * and friends). There is deliberately no table of ports and pins: a table
 * would have to be built from initialised data at startup, and the boot
 * self-test has to be able to run even if that data is wrong.
 * ------------------------------------------------------------------ */

/* 1 = the six servo ports drive the external board: this module owns
 *     PC6 / PC7 / PC8 / PC9 / PB14 / PB15 and TIM8, and the servo PWM
 *     code is wired off (balance.c, motor.c and system.c all key off this
 *     one macro).
 * 0 = the factory wiring: on-board A/B/C/D outputs plus the servo PWM. */
#define MOTOR_EXT_ENABLE            1

/* The six driver signals, one line each. Changing a pin here means
 * changing it in the GPIO_Init()/GPIO_PinAFConfig() calls in motor_ext.c
 * too - those are spelled out rather than derived from these macros, so
 * that a mistake here cannot silently move a pin at runtime. */
#define MOTOREXT_R_PWM              TIM8->CCR1   /* J1 PC6  TIM8_CH1 */
#define MOTOREXT_R_INA_PORT         GPIOC        /* J2 PC7           */
#define MOTOREXT_R_INA_PIN          GPIO_Pin_7
#define MOTOREXT_R_INB_PORT         GPIOC        /* J4 PC8           */
#define MOTOREXT_R_INB_PIN          GPIO_Pin_8
#define MOTOREXT_L_PWM              TIM8->CCR4   /* J5 PC9  TIM8_CH4 */
#define MOTOREXT_L_INA_PORT         GPIOB        /* J6 PB14          */
#define MOTOREXT_L_INA_PIN          GPIO_Pin_14
#define MOTOREXT_L_INB_PORT         GPIOB        /* J7 PB15          */
#define MOTOREXT_L_INB_PIN          GPIO_Pin_15

/* m/s -> PWM counts, for the open-loop path. Since step 2 the control task
 * feeds the driver through MotorExt_SetPwm() instead, so this only scales
 * MOTOREXT_BENCH_V and any future open-loop test:
 *     0.25 m/s -> 1000 counts -> 24% duty
 * Raise it if the car is sluggish, lower it if it lurches off. */
#define MOTOREXT_DUTY_PER_MPS       4000.0f

/* Full scale of the speed PI in balance.c: it saturates at +-16800 counts,
 * the same number the on-board driver used. MotorExt_SetPwm() maps that
 * onto the driver's +-95% duty, so 16800 at the PI output is full drive. */
#define MOTOREXT_PWM_FULL_SCALE     16800.0f

/* Set one of these to 0 if that wheel runs backwards while the car is
 * commanded forward. This only flips the sign of the duty; the pins are
 * not affected.
 *
 * Measured on the bench 2026-10-17 with MOTOREXT_BENCH_TEST = 1: the right
 * wheel ran forward and the left wheel ran backwards, so the left channel
 * is inverted here. The left driver channel (J5/J6/J7) had its motor wired
 * the other way round relative to the right one (J1/J2/J4). */
#define MOTOREXT_R_FORWARD_IS_POS   1
#define MOTOREXT_L_FORWARD_IS_POS   0

/* Ramp per 100Hz control tick, in counts. 420 counts is 10% of the full
 * duty per 10ms, so 0 to full takes 100ms. Only speeding up is rate
 * limited: slowing down, stopping and reversing keep full authority, so
 * a stop command is never delayed by this. */
#define MOTOREXT_SLEW               420

/* Bench check for the drive chain. 1 = the control task ignores the enable
 * switch, the battery gate AND the host link, and drives both wheels slowly
 * forward on the fixed command below, so a still car answers one question:
 * MCU -> board -> motors, is it wired and running at all?
 *
 * Only for a bench test with the wheels OFF THE GROUND. The O N / OFF
 * indicator on the OLED means nothing while this is 1, and the host cannot
 * stop the wheels - put it back to 0 for anything else. */
#define MOTOREXT_BENCH_TEST         0
#define MOTOREXT_BENCH_V            0.2f    /* m/s, both wheels forward */

/* Owns the six pins and TIM8. Leaves both channels braked. */
void MotorExt_Init(void);

/* What the six signal pins read back at boot, one bit per pin, latched
 * before anything drives them:
 *     bit0 J1 PC6  bit1 J2 PC7  bit2 J4 PC8
 *     bit3 J5 PC9  bit4 J6 PB14 bit5 J7 PB15
 * A set bit means the pin followed the MCU when it was driven high. A
 * clear bit means something outside the MCU held that pin low - the wire
 * is on the wrong row of the header (the GND row reads exactly like this)
 * or the pin is shorted. 63 (0x3F) = all six are reachable.
 * Shown on the OLED next to the EX marker, in decimal. */
u8 MotorExt_PinTest(void);

/* Step 1, open loop: wheel speeds in m/s, positive = forward. Only the
 * bench test uses this now. */
void MotorExt_SetTargets(float v_right, float v_left);

/* Step 2, closed loop: the two speed-PI outputs in counts, +-16800.
 * Positive means the wheel should push forward, so the encoder fed to the
 * PI must read positive for forward motion - the direction macros below
 * only flip the drive polarity and cannot repair an inverted encoder.
 * The slew limit and both direction macros still apply. */
void MotorExt_SetPwm(float pwm_right, float pwm_left);

/* Both bridges low: the wheels are locked. This is the state after
 * MotorExt_Init(), and what the car falls back to when the host link,
 * the enable switch or the battery says stop. */
void MotorExt_Brake(void);

/* Both bridges high-Z: the wheels roll free. Not used by the control
 * task - here so a bench test can tell a wiring fault from a brake. */
void MotorExt_Coast(void);

#endif
