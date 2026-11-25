#include "motor_ext.h"

/* ------------------------------------------------------------------
 * PWM: TIM8 at 20kHz.
 *
 * TIM8 hangs off APB2, so its timer clock is the full 168MHz. Prescaler
 * 1 divides that by two and leaves an 84MHz counter, and ARR 4199 then
 * gives 84e6 / (4199 + 1) = 20.0kHz - inside the 18..60kHz the driver
 * board wants and above the audible band.
 *
 * Both PWM pins are TIM8 channels (J1 -> CH1, J5 -> CH4), so the one time
 * base drives both. TIM12 also counts at 84MHz, so if a PWM pin ever has
 * to move to it the same PSC/ARR give the same 20kHz and the duty scale
 * below still holds. That is why the duty is kept in counts rather than
 * in percent: 0 counts is 0% and MOTOREXT_DUTY_MAX counts is 100%.
 *
 * The soft limit stops at 95% so the driver's charge pump keeps the
 * headroom it needs to switch.
 * ------------------------------------------------------------------ */
#define MOTOREXT_PSC         1
#define MOTOREXT_ARR         4199
#define MOTOREXT_DUTY_MAX    4199
#define MOTOREXT_DUTY_LIMIT  3989          /* 95% of MOTOREXT_DUTY_MAX */

static int s_duty_r;                       /* signed counts, >0 forward */
static int s_duty_l;
static u8  s_pin_test;                     /* latched boot self-test */

/**************************************************************************
Function: Drive the right wheel's direction pair and duty.
Input   : signed duty in counts. Positive = forward.
Output  : none
Right wheel is driver channel 1: J2 INA1 (PC7), J4 INB1 (PC8), and the
PWM comes out of J1 PWM1 (PC6 = TIM8_CH1). Direction first, duty second,
so the direction is already correct when the PWM value lands.
 **************************************************************************/
static void motor_ext_right_out(int duty)
{
    if(duty > 0)
    {
        GPIO_ResetBits(MOTOREXT_R_INA_PORT, MOTOREXT_R_INA_PIN);   /* INA=L */
        GPIO_SetBits  (MOTOREXT_R_INB_PORT, MOTOREXT_R_INB_PIN);   /* INB=H */
        MOTOREXT_R_PWM = (u16)duty;
    }
    else if(duty < 0)
    {
        GPIO_SetBits  (MOTOREXT_R_INA_PORT, MOTOREXT_R_INA_PIN);   /* INA=H */
        GPIO_ResetBits(MOTOREXT_R_INB_PORT, MOTOREXT_R_INB_PIN);   /* INB=L */
        MOTOREXT_R_PWM = (u16)(-duty);
    }
    else
    {
        GPIO_ResetBits(MOTOREXT_R_INA_PORT, MOTOREXT_R_INA_PIN);   /* INA=INB=L */
        GPIO_ResetBits(MOTOREXT_R_INB_PORT, MOTOREXT_R_INB_PIN);   /*  = brake  */
        MOTOREXT_R_PWM = 0;
    }
}

/**************************************************************************
Function: Drive the left wheel's direction pair and duty.
Input   : signed duty in counts. Positive = forward.
Output  : none
Left wheel is driver channel 2: J6 INA2 (PB14), J7 INB2 (PB15), PWM out
of J5 PWM2 (PC9 = TIM8_CH4).
 **************************************************************************/
static void motor_ext_left_out(int duty)
{
    if(duty > 0)
    {
        GPIO_ResetBits(MOTOREXT_L_INA_PORT, MOTOREXT_L_INA_PIN);
        GPIO_SetBits  (MOTOREXT_L_INB_PORT, MOTOREXT_L_INB_PIN);
        MOTOREXT_L_PWM = (u16)duty;
    }
    else if(duty < 0)
    {
        GPIO_SetBits  (MOTOREXT_L_INA_PORT, MOTOREXT_L_INA_PIN);
        GPIO_ResetBits(MOTOREXT_L_INB_PORT, MOTOREXT_L_INB_PIN);
        MOTOREXT_L_PWM = (u16)(-duty);
    }
    else
    {
        GPIO_ResetBits(MOTOREXT_L_INA_PORT, MOTOREXT_L_INA_PIN);
        GPIO_ResetBits(MOTOREXT_L_INB_PORT, MOTOREXT_L_INB_PIN);
        MOTOREXT_L_PWM = 0;
    }
}

/**************************************************************************
Function: One 100Hz tick of the ramp.
Input   : current duty, wanted duty, both signed counts.
Output  : the duty to apply this tick.
Speeding up is rate limited so the gearbox and the driver's input
capacitors do not see a step. Slowing down and stopping are immediate, and
a reversal goes through zero instead of stepping from full forward to full
reverse - a zero duty is a brake, so the motor is stopped before it is
driven the other way.
 **************************************************************************/
static int motor_ext_slew(int current, int target)
{
    int mag_cur, mag_tgt;

    if((current > 0 && target < 0) || (current < 0 && target > 0))
        return 0;                                  /* reverse: brake first */

    mag_cur = (current < 0) ? -current : current;
    mag_tgt = (target  < 0) ? -target  : target;

    if(mag_tgt > mag_cur + MOTOREXT_SLEW)
        mag_tgt = mag_cur + MOTOREXT_SLEW;

    return (target < 0) ? -mag_tgt : mag_tgt;
}

/**************************************************************************
Function: Clamp, ramp and emit a pair of signed duty targets.
Input   : requested duty in counts, right and left
Output  : none
Shared tail of the open-loop (m/s) and closed-loop (PI counts) entry
points, so the 95% limit, the ramp and the two output hooks exist once.
 **************************************************************************/
static void motor_ext_apply(int tgt_r, int tgt_l)
{
    if(tgt_r >  MOTOREXT_DUTY_LIMIT) tgt_r =  MOTOREXT_DUTY_LIMIT;
    if(tgt_r < -MOTOREXT_DUTY_LIMIT) tgt_r = -MOTOREXT_DUTY_LIMIT;
    if(tgt_l >  MOTOREXT_DUTY_LIMIT) tgt_l =  MOTOREXT_DUTY_LIMIT;
    if(tgt_l < -MOTOREXT_DUTY_LIMIT) tgt_l = -MOTOREXT_DUTY_LIMIT;

    s_duty_r = motor_ext_slew(s_duty_r, tgt_r);
    s_duty_l = motor_ext_slew(s_duty_l, tgt_l);

    motor_ext_right_out(s_duty_r);
    motor_ext_left_out(s_duty_l);
}

/**************************************************************************
Function: Drive one pin high and read it back.
Input   : port, pin mask.
Output  : 1 if the pin followed the MCU, 0 if something held it low.
An output pin still reports its real level in the input register, so this
says whether the pin is reachable from the outside at all - without a
meter and without trusting anything but the register write itself.
 **************************************************************************/
static u8 motor_ext_probe(GPIO_TypeDef *port, u16 pin)
{
    u32 i;
    u8  ok;

    GPIO_SetBits(port, pin);
    for(i = 0; i < 400; i++) __NOP();              /* let the pin settle */
    ok = (GPIO_ReadInputData(port) & pin) ? 1 : 0;
    GPIO_ResetBits(port, pin);

    return ok;
}

/**************************************************************************
Function: Boot self-test of the six driver pins.
Input   : none
Output  : bit per pin, see MotorExt_PinTest().
Runs before anything is driving, so a pin held low outside the MCU shows
up as a clear bit. Every combination it drives is a stopped motor: both
IN low is a brake, and with IN high the PWM compare is still 0, so the
driver's output stays off. Nothing can move.
 **************************************************************************/
static u8 motor_ext_pin_selftest(void)
{
    GPIO_InitTypeDef g;
    u8 ok = 0;

    GPIO_StructInit(&g);
    g.GPIO_Mode  = GPIO_Mode_OUT;
    g.GPIO_OType = GPIO_OType_PP;
    g.GPIO_Speed = GPIO_Speed_100MHz;
    g.GPIO_PuPd  = GPIO_PuPd_DOWN;

    /* all six as outputs, low. MotorExt_Init() turns the two PWM pins back
     * into TIM8 alternate function once this has run. */
    g.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOC, &g);
    g.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &g);

    GPIO_ResetBits(GPIOC, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
    GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);

    if(motor_ext_probe(GPIOC, GPIO_Pin_6))  ok |= 0x01;   /* J1 PWM1  */
    if(motor_ext_probe(GPIOC, GPIO_Pin_7))  ok |= 0x02;   /* J2 INA1  */
    if(motor_ext_probe(GPIOC, GPIO_Pin_8))  ok |= 0x04;   /* J4 INB1  */
    if(motor_ext_probe(GPIOC, GPIO_Pin_9))  ok |= 0x08;   /* J5 PWM2  */
    if(motor_ext_probe(GPIOB, GPIO_Pin_14)) ok |= 0x10;   /* J6 INA2  */
    if(motor_ext_probe(GPIOB, GPIO_Pin_15)) ok |= 0x20;   /* J7 INB2  */

    return ok;
}

/**************************************************************************
Function: Initialise the six reserved servo ports as the external driver's
          inputs, and TIM8 as the two PWM outputs.
Input   : none
Output  : none
The direction pins come up low and the compares at 0, so the board is
braked rather than left floating between the pins being configured and the
first command arriving.
 **************************************************************************/
void MotorExt_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);

    /* ---- read the six pins back before anything drives them ---- */
    s_pin_test = motor_ext_pin_selftest();

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

    /* ---- direction pins: plain outputs, both low = brake ---- */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin  = MOTOREXT_R_INA_PIN | MOTOREXT_R_INB_PIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);                 /* J2 PC7, J4 PC8 */
    GPIO_InitStructure.GPIO_Pin  = MOTOREXT_L_INA_PIN | MOTOREXT_L_INB_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);                 /* J6 PB14, J7 PB15 */

    /* ---- PWM pins: alternate function, TIM8 ---- */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6 | GPIO_Pin_9;
    GPIO_Init(GPIOC, &GPIO_InitStructure);                 /* J1 PC6, J5 PC9 */
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);

    /* ---- time base: 20kHz, duty 0 ---- */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period        = MOTOREXT_ARR;
    TIM_TimeBaseStructure.TIM_Prescaler     = MOTOREXT_PSC;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    /* TIM8 is an advanced timer: TIM_OC1/4Init read all eight fields of
     * the struct, including the complementary output ones, so define them
     * all rather than letting the stack decide. */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = 0;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM8, &TIM_OCInitStructure);       /* PC6 - J1 PWM1, right */
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);       /* PC9 - J5 PWM2, left  */
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(TIM8, ENABLE);              /* advanced timer needs MOE */
    TIM_ARRPreloadConfig(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);

    /* the compares the OC init left at 0, plus the ramps starting from 0 */
    MotorExt_Brake();
}

/**************************************************************************
Function: What the six pins read back at boot. See motor_ext.h.
 **************************************************************************/
u8 MotorExt_PinTest(void)
{
    return s_pin_test;
}

/**************************************************************************
Function: Open loop - commanded wheel speed in, duty out.
Input   : v_right, v_left in m/s, positive = forward.
Output  : none
Only the bench test uses this since step 2.
 **************************************************************************/
void MotorExt_SetTargets(float v_right, float v_left)
{
    float k_r = MOTOREXT_DUTY_PER_MPS * (MOTOREXT_R_FORWARD_IS_POS ? 1.0f : -1.0f);
    float k_l = MOTOREXT_DUTY_PER_MPS * (MOTOREXT_L_FORWARD_IS_POS ? 1.0f : -1.0f);

    motor_ext_apply((int)(v_right * k_r), (int)(v_left * k_l));
}

/**************************************************************************
Function: Closed loop - the speed PI's output in, duty out.
Input   : pwm_right, pwm_left in counts, +-16800, positive = drive forward.
Output  : none
The PI saturates at MOTOREXT_PWM_FULL_SCALE, which maps to the 95% duty
limit, so the loop gains keep the same meaning they had on the on-board
driver. Sign errors belong to the encoder, not to these macros.
 **************************************************************************/
void MotorExt_SetPwm(float pwm_right, float pwm_left)
{
    float k_r = (MOTOREXT_DUTY_LIMIT / MOTOREXT_PWM_FULL_SCALE) *
                (MOTOREXT_R_FORWARD_IS_POS ? 1.0f : -1.0f);
    float k_l = (MOTOREXT_DUTY_LIMIT / MOTOREXT_PWM_FULL_SCALE) *
                (MOTOREXT_L_FORWARD_IS_POS ? 1.0f : -1.0f);

    motor_ext_apply((int)(pwm_right * k_r), (int)(pwm_left * k_l));
}

/**************************************************************************
Function: Stop with the windings shorted - both direction pins low.
Input   : none
Output  : none
Immediate, and the ramp state is reset, so whatever comes next starts from
zero instead of from the duty that was running when the command went away.
 **************************************************************************/
void MotorExt_Brake(void)
{
    s_duty_r = 0;
    s_duty_l = 0;

    motor_ext_right_out(0);
    motor_ext_left_out(0);
}

/**************************************************************************
Function: Stop with the windings open - both direction pins high.
Input   : none
Output  : none
 **************************************************************************/
void MotorExt_Coast(void)
{
    s_duty_r = 0;
    s_duty_l = 0;

    GPIO_SetBits(MOTOREXT_R_INA_PORT, MOTOREXT_R_INA_PIN);
    GPIO_SetBits(MOTOREXT_R_INB_PORT, MOTOREXT_R_INB_PIN);
    MOTOREXT_R_PWM = 0;

    GPIO_SetBits(MOTOREXT_L_INA_PORT, MOTOREXT_L_INA_PIN);
    GPIO_SetBits(MOTOREXT_L_INB_PORT, MOTOREXT_L_INB_PIN);
    MOTOREXT_L_PWM = 0;
}
