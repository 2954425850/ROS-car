#include "balance.h"

int Time_count=0; //Time variable //计时变量 

u8 Lidar_Detect = Lidar_Detect_ON;			//电磁巡线模式雷达检测障碍物，默认开启
int RC_Lidar_Avoid_FLAG = RC_Avoid_OFF;			//遥控模式雷达避障，默认关闭
u8 Mode;
float RC_Velocity_CCD=350,RC_Velocity_ELE=350; 
float PS2_Velocity,PS2_Turn_Velocity;			//遥控控制的速度
Encoder OriginalEncoder; //Encoder raw data //编码器原始数据    
u8 Lidar_Success_Receive_flag=0,Lidar_flag_count=0;//雷达在线标志位，雷达掉线计数器
/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
函数功能：运动学逆解，根据三轴目标速度计算各车轮目标转速
入口参数：X和Y、Z轴方向的目标运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //车轮目标速度限幅
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //全向移动小车才开启速度平滑处理
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //对输入速度进行平滑处理
  
      //Get the smoothed data 
			//获取平滑处理后的数据			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		//Mecanum wheel car
	  //麦克纳姆轮小车
	  if (Car_Mode==Mec_Car) 
    {
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//Omni car
		//全向轮小车
		else if (Car_Mode==Omni_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=0;	//Out of use //没有使用到
		}
		
		//Ackermann structure car
		//阿克曼小车
		else if (Car_Mode==Akm_Car) 
		{
			//Ackerman car specific related variables //阿克曼小车专用相关变量
			float R, Ratio=636.56, AngleR, Angle_Servo;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//对于阿克曼小车Vz代表右前轮转向角度
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
			AngleR=target_limit_float(AngleR,-0.49f,0.32f);
			
			//Inverse kinematics //运动学逆解
			if(AngleR!=0)
			{
				MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
				MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;			
			}
			else 
			{
				MOTOR_A.Target = Vx;
				MOTOR_B.Target = Vx;
			}
			// The PWM value of the servo controls the steering Angle of the front wheel
			//舵机PWM值，舵机控制前轮转向角度
			//Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
			Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.755f;
			Servo=SERVO_INIT + (Angle_Servo - 1.755f)*Ratio;

			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
			Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //舵机PWM值限幅
			}
		
		//Differential car
		//差速小车
		else if (Car_Mode==Diff_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //计算出右轮的目标速度
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
			
		}
		
		//FourWheel car
		//四驱车
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
					
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 		
		}
		
		//Tank Car
		//履带车
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //计算出右轮的目标速度
			
			//Wheel (motor) target speed limit //车轮(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用到
			MOTOR_D.Target=0; //Out of use //没有使用到
		}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
函数功能：FreeRTOS任务，核心运动控制任务
入口参数：无
返回  值：无
**************************************************************************/
u8 back_flag=0;
extern float temp1;
extern u8 flag;
void Balance_task(void *pvParameters)
{ 
	u32 lastWakeTime = getSysTickCnt();
    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//此任务以100Hz的频率运行（10ms控制一次）
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			//Time count is no longer needed after 30 seconds
			//时间计数，30秒后不再需要
			if(Time_count<3000)Time_count++;
			//Get the encoder data, that is, the real time wheel speed, 
			//and convert to transposition international units
			//获取编码器数据，即车轮实时速度，并转换位国际单位
			Get_Velocity_Form_Encoder();   
			/* ROS: the host owns motion. Local mode switching is gone - the
			 * kinematics, the link watchdog and the software stop now all live
			 * in ros_apply_command(). Mode and click_N_Double() stay compiled
			 * in, they are simply no longer driven from here.
			 *
			 * ros_send_frame() is called at the full 100Hz and divides itself
			 * down to the 50Hz uplink rate. */
			ros_apply_command();
			/* ROS 2026-10-16: unwired. The 0xAB servo frame used to move the six
			 * servo ports, and those pins are the external motor board's PWM and
			 * direction signals now, so a servo command would fight the drive. The
			 * decoder and the frame are still in usartx.c if a gimbal comes back;
			 * they simply have no pins to drive. */
			/* ros_apply_servo(); */
			ros_send_frame();
			//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
			//and the software failure flag is 0
			//如果电池电压不存在异常，而且使能开关在ON档位，而且软件失能标志位为0
			//KD_Lidar_Fun();
			if(MOTOREXT_BENCH_TEST || Turn_Off(Voltage)==0) 
			{ 			
           //Speed closed-loop control to calculate the PWM value of each motor, 
				 //PWM represents the actual wheel speed					 
				 //速度闭环控制计算各电机PWM值，PWM代表车轮实际转速
#if MOTOR_EXT_ENABLE
				 /* ROS 2026-10-23 step 2: external driver, closed loop.
				  *
				  * The six reserved servo ports drive the external board (see
				  * motor_ext.h) and the two encoders are wired again, so the
				  * speed PI owns the output as it did on the on-board driver.
				  * It runs in m/s and returns +-16800 counts; the driver
				  * scales that onto its +-95% duty and keeps its ramp.
				  *
				  * MOTOR_C is the right wheel and MOTOR_A the left one:
				  * FourWheel_Car gives A/B the Vx - Vz term and C/D the
				  * Vx + Vz term. If the car steers the wrong way round, swap
				  * the two arguments here.
				  *
				  * The loop assumes a positive encoder reading means the wheel
				  * rolls forward. MOTOREXT_*_FORWARD_IS_POS does NOT fix that
				  * - it only flips the drive polarity, and using it on an
				  * inverted encoder turns the loop into positive feedback.
				  * If a wheel runs away or pulls backwards, check the encoder
				  * sign on the OLED (show.c prints target and measured speed
				  * side by side) before touching the gains.
				  *
				  * Gains: the 70:1 gearbox turns the wheel 30/70 as fast per
				  * unit of duty as the old 30:1 motor, so both loop gains are
				  * scaled by 70/30 in system.c to keep the crossover. */
#if MOTOREXT_BENCH_TEST
				 /* Bench: fixed slow forward on both wheels. No host, no gate -
				  * if the wheels turn here, the drive chain is wired and running.
				  * Wheels off the ground, and MOTOREXT_BENCH_TEST back to 0 when
				  * the bench work is done. */
				 MotorExt_SetTargets(MOTOREXT_BENCH_V, MOTOREXT_BENCH_V);
#else
				 MotorExt_SetPwm(Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target),
				                 Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target));
#endif
#else
				 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
				 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
				 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
				 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
				 Limit_Pwm(16500) ;
				 //Set different PWM control polarity according to different car models
				 //根据不同小车型号设置不同的PWM控制极性
				 switch(Car_Mode)
				 {
						case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  -MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //麦克纳姆轮小车
						case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Omni car                //全向轮小车
						case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //阿克曼小车
						case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //两轮差速小车
						case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //四驱车 
						case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //履带车
				 }
#endif
			}
			//If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
			//如果Turn_Off(Voltage)返回值为1，不允许控制小车进行运动，PWM值设置为0
			else
			{
#if MOTOR_EXT_ENABLE
				/* No enable, low battery or a software stop: brake.
				 * Braking holds the wheels instead of letting the car roll, and it
				 * drops the ramp state, so the next command starts from zero.
				 * The PI is cleared too: its accumulator is the output, and a
				 * stale value would step straight back to the last duty.
				 *
				 * A lost host link is NOT this branch: ros_apply_command() zeroes
				 * the targets instead, so the loop does an active zero-speed hold.
				 * Only the three conditions above hard-brake. */
				Incremental_PI_Reset();
				MotorExt_Brake();
#else
				Set_Pwm(0,0,0,0,0);
#endif
			}
		 }  
}



/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
函数功能：赋值给PWM寄存器，控制车轮转速与方向
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	if(motor_a<0)			PWMA2=16800,PWMA1=16800+motor_a;
	else 	            PWMA1=16800,PWMA2=16800-motor_a;
			
	if(motor_b<0)			PWMB1=16800,PWMB2=16800+motor_b;
	else 	            PWMB2=16800,PWMB1=16800-motor_b;
			
	if(motor_c<0)			PWMC1=16800,PWMC2=16800+motor_c;
	else 	            PWMC2=16800,PWMC1=16800-motor_c;
	
	if(motor_d<0)			PWMD2=16800,PWMD1=16800+motor_d;
	else 	            PWMD1=16800,PWMD2=16800-motor_d;
	
	//Servo control
	//舵机控制
#if !MOTOR_EXT_ENABLE
	Servo_PWM =servo;
#else
	/* ROS 2026-10-16: Servo_PWM is TIM8->CCR4 = PC9, the external driver's
	 * left wheel PWM (J5). Nothing calls Set_Pwm() from the control task
	 * while MOTOR_EXT_ENABLE is on, but guarding the write means a later
	 * caller cannot silently wipe out the left wheel's duty. */
	(void)servo;
#endif
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
函数功能：限制PWM值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
函数功能：限幅函数
入口参数：幅值
返回  值：无
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
函数功能：检查电池电压、使能开关状态、软件失能标志位状态
入口参数：电压
返回  值：是否允许控制，1：不允许，0允许
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1; 
					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
函数功能：求绝对值
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)

函数功能：增量式PI控制器
入口参数：编码器测量值(实际速度)，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
/* Speed-loop state, one accumulator per motor. File scope because the
 * output IS the accumulator: freezing it inside a function-local static
 * leaves the loop ready to resume at the duty it held before a stop. */
static float s_pwm_a,s_pwm_b,s_pwm_c,s_pwm_d;
static float s_last_bias_a,s_last_bias_b,s_last_bias_c,s_last_bias_d;

static int pi_step(float *Pwm,float *Last_bias,float Encoder,float Target)
{
	 float Bias;

	 Bias=Target-Encoder; //Calculate the deviation //计算偏差
	 *Pwm+=Velocity_KP*(Bias-*Last_bias)+Velocity_KI*Bias;
	 if(*Pwm>16800.0f)*Pwm=16800.0f;
	 if(*Pwm<-16800.0f)*Pwm=-16800.0f;
	 *Last_bias=Bias; //Save the last deviation //保存上一次偏差
	 return (int)*Pwm;
}

/* Drop every speed loop. Called from the brake branch of the control
 * task, so re-enabling the car does not start from the last output. */
void Incremental_PI_Reset(void)
{
	 s_pwm_a=s_pwm_b=s_pwm_c=s_pwm_d=0.0f;
	 s_last_bias_a=s_last_bias_b=s_last_bias_c=s_last_bias_d=0.0f;
}

int Incremental_PI_A (float Encoder,float Target)
{
	 return pi_step(&s_pwm_a,&s_last_bias_a,Encoder,Target);
}
int Incremental_PI_B (float Encoder,float Target)
{
	 return pi_step(&s_pwm_b,&s_last_bias_b,Encoder,Target);
}
int Incremental_PI_C (float Encoder,float Target)
{
	 return pi_step(&s_pwm_c,&s_last_bias_c,Encoder,Target);
}
int Incremental_PI_D (float Encoder,float Target)
{
	 return pi_step(&s_pwm_d,&s_last_bias_d,Encoder,Target);
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
函数功能：对APP通过串口2发送过来的命令进行处理
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //全向轮运动小车可以进行横向移动
	{
		switch(Flag_Direction)  //Handle direction control commands //处理方向控制命令
		{ 
				case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
				case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
				case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
				case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
				case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
				case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
				case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
				case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
				default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
		}
		if(Flag_Move==0)		
		{	
			//If no direction control instruction is available, check the steering control status
			//如果无方向控制指令，检查转向控制状态
			if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/1000); //left rotation  //左自转  
			else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/1000); //right rotation //右自转
			else 		               Move_Z=0;                       //stop           //停止
		}
	}	
	else //Non-omnidirectional moving trolley //非全向移动小车
	{
		switch(Flag_Direction) //Handle direction control commands //处理方向控制命令
		{ 
				case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
				case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
				case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
				case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
				case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
				case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
				case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
				case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
				default:     Move_X=0;               Move_Z=0;         break;
		}
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //左自转 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //右自转	
	}
	//Z-axis data conversion //Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//阿克曼结构小车转换为前轮转向角度
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //差速控制原理系列需要此处理
		Move_Z=Move_Z*RC_Velocity/1000;
	}		
	//Unit conversion, mm/s -> m/s
  //单位转换，mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//得到控制目标值，进行运动学分析
	//Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
函数功能：读取编码器数值并计算车轮速度，单位m/s
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //获取编码器的原始数据
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	  //Decide the encoder numerical polarity according to different car models
		//根据不同小车型号决定编码器数值极性
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= -OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//编码器原始数据转换为车轮速度，单位m/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
#if LEFT_ENCODER_IS_B
		/* A's encoder is dead; B's stands in. See encoder.h. */
		MOTOR_A.Encoder= MOTOR_B.Encoder;
#endif
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
函数功能：对三轴目标速度做平滑处理
入口参数：三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
函数功能：浮点型数据计算绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}
//-------------------------雷达回程3轴位置PID----------------------------//
float Location_KP=0.6,Location_KD=0.003;
float Incremental_PI_X (float Current,float Target)
{ 	
	 static float Bias,result,int_bias;
	 Bias=Target-Current; //Calculate the deviation //计算偏差
	 result=Bias*Location_KP+Location_KD*(Bias-int_bias);
	int_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return result;    
}
float Incremental_PI_Y (float Current,float Target)
{ 	
	 static float Bias,result,int_bias;
	 Bias=Target-Current; //Calculate the deviation //计算偏差

	 result=Bias*Location_KP+Location_KD*(Bias-int_bias);
		int_bias=Bias; //Save the last deviation //保存上一次偏差 
	 return result;
}
float Angel_KP=0.1f,Angel_KD=0.0005f;
float Incremental_PI_Z (float Current,float Target)
{ 	
	 static float Bias,result;
	 Bias=Target-Current; //Calculate the deviation //计算偏差
	 if(Car_Mode==Omni_Car)
	{
		if(Bias>1.1f) result=0.1f;
		if(Bias<-1.1f) result=-0.1f;
	}else{
		if(Bias>1.0f) result=0.2f;
		if(Bias<-1.0f) result=-0.2f;
	}
	 return result;
}
//----------------------------------------------------------------------//
/**************************************************************************
函数功能：雷达避障功能（回程时）
入口参数：无
返回  值：无
**************************************************************************/
int time=0;//时间计数器
u8 turn=0;//转向标志位
void Lidar_back_Avoid(void)
{
	int barrier_cnt=0;//障碍点计数器
	if(Move_X>0)//小车前进时
	{
		for(int i=round(0/0.93f);i<round(30.0f/0.93f);i++)//扫描0~30°
		{
			if(PointCloud_Data[i].distance>30&&PointCloud_Data[i].distance<500)//距离500mm内有障碍物则记录
			{
				barrier_cnt++;
			}
		}
		for(int i=round(330/0.93f);i<round(360.0f/0.93f);i++)//扫描330~360°
		{
			if(PointCloud_Data[i].distance>30&&PointCloud_Data[i].distance<500)//距离500mm内有障碍物则记录
			{
				barrier_cnt++;
			}
		}
	}
	else//小车回退时
	{
		for(int i=round(150.0f/0.93f);i<round(210.0f/0.93f);i++)//扫描150~210°
		{
			if(PointCloud_Data[i].distance>30&&PointCloud_Data[i].distance<500)//距离500mm内有障碍物则记录
			{
				barrier_cnt++;
			}
		}
	}
	if(barrier_cnt>6)//障碍点数超过6，判定为有障碍
	{
		flag=1;//丢弃当前
		Move_X=0;//X轴速度设置为0，小车停止
		//时间计数器一个周期计数10s
		if(time<1000)time++;
		else time=0;
		//3s前障碍物消失，底盘会正常回程，第3s时障碍物依然存在则触发转向，
		if(time==300){
			turn=1;
		}else turn=0;
	}else flag=0,time=0;
	temp1=time;
}
/**************************************************************************
函数功能：雷达定位回程
入口参数：无
返回  值：无
**************************************************************************/
void Lidar_Back(void)
{
	float thresholds;
	if (Car_Mode==Diff_Car) thresholds = 5.0f;
	else thresholds = 2.0f;
	//简单状态机
    enum{
		X_BACK=0		,//X轴回程
		Y_BACK=1		,//Y轴回程
    };
	static uint8_t state_machine=Y_BACK;//初始状态为Y轴回程
	static float tar_angle=0;//目标角度
	//根据不同状态执行不同功能
	switch( state_machine )//实现X轴回程与Y轴回程的来回跳转
	{
		case X_BACK:
		{
			
			if(fabs(Lidar_Result[0])>50)//X轴偏差大于±50mm时再进行回程
			{
				if(Lidar_Result[0]<0) tar_angle=-90;//X轴回程时，位于原点左侧时目标角度为-90
				else if(Lidar_Result[0]>0) tar_angle=90;//X轴回程时，位于原点右侧时目标角度为90
				if(Yaw<=tar_angle-thresholds||Yaw>=tar_angle+thresholds)//角度偏差大于±2都时调整角度
				{
					Move_X=0;
					Move_Z=-Incremental_PI_Z(Yaw,tar_angle);
				}
				else//角度偏差小于±2度时，X轴回程
				{
					Move_X=-fabs(Incremental_PI_X(Lidar_Result[0],0)/1000.0f);
					Move_Z=0;
				}
			}else state_machine=Y_BACK;////X轴偏差小于±50mm时切换为Y轴回程
			if(turn) state_machine=Y_BACK;////转向标志置位时切换为Y轴回程
			break;
		}
		case Y_BACK:
		{
			tar_angle=0;//Y轴回程时，无论前进后退目标角度为0
			if(Yaw<=tar_angle-thresholds||Yaw>=tar_angle+thresholds)//角度偏差大于±2都时调整角度
			{
				Move_X=0;
				Move_Z=-Incremental_PI_Z(Yaw,tar_angle);
			}
			else//角度偏差小于±2度时，Y轴回程
			{
				Move_X=Incremental_PI_Y(Lidar_Result[1],0)/1000.0f;
				Move_Z=0;
			}
			if(fabs(Lidar_Result[1])<=20) state_machine=X_BACK;//Y轴偏差小于20mm时切换为X轴回程
			if(turn) state_machine=X_BACK;//转向标志置位时切换为X轴回程
			break;
		}

	}
}








