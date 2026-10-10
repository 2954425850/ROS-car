#ifndef __MOTOR_H
#define __MOTOR_H

#include "system.h"


#define PWMA1   TIM10->CCR1  
#define PWMA2   TIM11->CCR1 

#define PWMB1   TIM9->CCR1 
#define PWMB2   TIM9->CCR2


#define PWMC1   TIM1->CCR1  
#define PWMC2   TIM1->CCR2 

#define PWMD1   TIM1->CCR3 
#define PWMD2   TIM1->CCR4

#define EN     PDin(3)  

/* Servo output channels on the RC/servo header.
 *   ch1..ch4 -> PC6 PC7 PC8 PC9   TIM8  CH1..CH4
 *   ch5..ch6 -> PB14 PB15         TIM12 CH1..CH2
 * The timer runs at 1MHz, so the compare value IS the pulse width in us. */
#define SERVO1_PWM  TIM8->CCR1     /* PC6  */
#define SERVO2_PWM  TIM8->CCR2     /* PC7  */
#define SERVO3_PWM  TIM8->CCR3     /* PC8  */
#define SERVO4_PWM  TIM8->CCR4     /* PC9  */
#define SERVO5_PWM  TIM12->CCR1    /* PB14 */
#define SERVO6_PWM  TIM12->CCR2    /* PB15 */

#define SERVO_NUM     6
#define SERVO_MIN     500          /* us, soft limit */
#define SERVO_MAX     2500         /* us, soft limit */

/* Legacy names kept for the Ackermann steering code in balance.c */
#define Servo_PWM  TIM8->CCR4
#define SERVO_INIT 1500  //Servo zero point //¶æ»úÁãµã

void Enable_Pin(void);
void MiniBalance_PWM_Init(u16 arr,u16 psc);

void Servo_PWM_Init(u16 arr,u16 psc);
void Servo_PWM12_Init(u16 arr,u16 psc);
void Servo_Set(u8 ch,int us);
extern int Servo_Ch[SERVO_NUM+1];
void MiniBalance_PWM11_Init(u16 arr,u16 psc);
void MiniBalance_PWM9_Init(u16 arr,u16 psc);
void MiniBalance_PWM1_Init(u16 arr,u16 psc);
#endif
