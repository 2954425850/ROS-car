#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 
#include "system.h"

// No larger than 65535, because the timer of STM32F103 is 16 bit
//不可大于65535，因为STM32F103的定时器是16位的
#define ENCODER_TIM_PERIOD (u16)(65535)   

/* ROS 2026-10-23: motor A's encoder failed. A and B are wired in
 * parallel on the left driver channel (M+ to M+, M- to M-) and turn
 * together, so B's encoder (TIM3, PB4/PB5) stands in for the left
 * side: Get_Velocity_Form_Encoder() copies MOTOR_B.Encoder into
 * MOTOR_A.Encoder after the per-model sign fix, so the speed loop,
 * the OLED and the RosUp vA field all keep reading A.
 * B's encoder must be wired to PB4/PB5. If it is not, the left side
 * reads zero and the closed loop drives it to full duty.
 * 0 = original wiring. */
#define LEFT_ENCODER_IS_B   1

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
void Encoder_Init_TIM4(void);
void Encoder_Init_TIM5(void);
	
int Read_Encoder(u8 TIMX);

void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);
void TIM5_IRQHandler(void);

#endif
