#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int Time_count;
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
函数功能：读取电池电压、蜂鸣器报警、开启自检、向APP发送数据、OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
int Buzzer_count=25;
int flag_cnt=0;
int Lidar_Result[3];
int a,b;
int lidar_data_ok;
float temp1;
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //此任务以10Hz的频率运行
		//开机时蜂鸣器短暂蜂鸣，开机提醒
		//The buzzer will beep briefly when the machine is switched on
		if(Time_count<50)Buzzer=1; 
		else if(Time_count>=51 && Time_count<100)Buzzer=0;
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer=1; //The buzzer is buzzing //蜂鸣器蜂鸣
		else if(Buzzer_count==5)Buzzer=0;
		//Read the battery voltage //读取电池电压
		for(i=0;i<10;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/10;
		Voltage_All=0;
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds //确保蜂鸣器只响0.5秒
		if(LowVoltage_2==1)LowVoltage_2++; //Make sure the buzzer only rings for 0.5 seconds //确保蜂鸣器只响0.5秒
		if(Voltage>=12.6f)Voltage=12.6f;
		else if(10<=Voltage && Voltage<10.5f && LowVoltage_1<2)LowVoltage_1++; //10.5V, first buzzer when low battery //10.5V，低电量时蜂鸣器第一次报警
		else if(Voltage<10 && LowVoltage_2<2)LowVoltage_2++; //10V, when the car is not allowed to control, the buzzer will alarm the second time //10V，小车禁止控制时蜂鸣器第二次报警			
		/* ROS: APP_Show() re-enabled 2026-09-20. It holds the only live printf()
	    calls, and fputc() targets USART2 -- which IS initialised again now
	    (see system.c). This is the readback path for the wheel-speed PI
	    tuning tool (tools/velocity_pid_tune.py): the USART2 ISR sets
	    PID_Send and this prints "{C<vel>:<kp>:<ki>}$". It only prints;
	    it writes no control state. */
	APP_Show();	 //Send data to the APP //向APP发送数据
		oled_show(); //Tasks are displayed on the screen //显示屏显示任务
		/* ROS: the N10P lidar front end is never started any more (see
		 * systemInit), so Positioning() and lidar_data_ok have no producer
		 * left. The lidar code stays compiled in but unreachable. */
   }
}  

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
函数功能：OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{  
/* ==== WS_DEMO_BEGIN ==== */
#if WS_DEMO_ENABLE
	if (ws_demo_active()) { ws_demo_oled(); return; }
#endif
/* ==== WS_DEMO_END ==== */
   static int count=0;	 
	 int Car_Mode_Show;
	
	 //Collect the tap information of the potentiometer, 
	 //and display the car model to be fitted when the car starts up in real time
	 //采集电位器档位信息，实时显示小车开机时要适配的小车型号
	 /* ROS: Car_Mode is fixed to FourWheel_Car in systemInit(), so show the
	  * model actually in use instead of the potentiometer tap. This also
	  * drops 10 ADC reads per 100ms. */
	 Car_Mode_Show = Car_Mode;
	 Voltage_Show=Voltage*100; 
	 count++;
	//The first line of the display shows the content // 
	//显示屏第1行显示内容//
	 switch(Car_Mode_Show)
	 {
		case Mec_Car:       OLED_ShowString(0,0,"Mec             "); break; 
		case Omni_Car:      OLED_ShowString(0,0,"Omni            "); break; 
		case Akm_Car:       OLED_ShowString(0,0,"Akm             "); break; 
		case Diff_Car:      OLED_ShowString(0,0,"Diff            "); break; 
		case FourWheel_Car: OLED_ShowString(0,0,"4WD             "); break; 
		case Tank_Car:      OLED_ShowString(0,0,"Tank            "); break; 
	 }
	if(Yaw<0){
		OLED_ShowString(82,0,"-");
		OLED_ShowNumber(90,0,-Yaw,4,12);
	}else {
		OLED_ShowString(82,0,"+");
		OLED_ShowNumber(90,0,Yaw,4,12);
	}
	
	/* ROS link health: LOST once the 200ms watchdog has fired, otherwise the
	  * running count of CRC-valid downlink frames, modulo 1000. */
	if(RosFault & ROS_FAULT_TIMEOUT) OLED_ShowString(32,0,"LOST");
	else                             OLED_ShowString(32,0,"ROS ");
	OLED_ShowNumber(58,0,RosDown.frames%1000,3,12);
	
	
	
	 //The second line of the display shows the content // 
	 //显示屏第2行显示内容//
	 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
	 {
		//麦轮、全向轮、四驱车显示电机A的目标速度和当前实际速度//
		OLED_ShowString(0,10,"A");
		if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
													OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
		else                 	OLED_ShowString(15,10,"+"),
													OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
		
		if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
													OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
		else                 	OLED_ShowString(60,10,"+"),
													OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
	 }
	 
	 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
	 {
		 //阿克曼、差速、履带车显示电机A的目标速度和当前实际速度//
		 OLED_ShowString(0,10,"L:");
		 if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
														OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
		 else                 	OLED_ShowString(15,10,"+"),
														OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12);  
		 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,10,"-"),
														OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
		 else                 	OLED_ShowString(60,10,"+"),
														OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
	 }
	 
			//The third line of the display shows the content // 
		 //显示屏第3行显示内容//
		 //阿克曼、差速、履带车显示电机B的目标速度和当前实际速度//
		 OLED_ShowString(0,20,"R:");
		 if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
														OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
		 else                 	OLED_ShowString(15,20,"+"),
														OLED_ShowNumber(20,20,  MOTOR_B.Target*1000,5,12);  
			
		 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,20,"-"),
														OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
		 else                 	OLED_ShowString(60,20,"+"),
														OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
	 

		if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		{
		//麦轮、全向轮、四驱车显示电机B的目标速度和当前实际速度//
		OLED_ShowString(0,20,"B");		
		if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
													OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
		else                 	OLED_ShowString(15,20,"+"),
													OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
		
		if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
													OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
		else                 	OLED_ShowString(60,20,"+"),
													OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
		}
	 
	  //The fourth line of the display shows the content // 
		//显示屏第4行显示内容//
		//麦轮、四驱车、全向轮显示电机C的目标速度和当前实际速度//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
													OLED_ShowString(0,30,"C");
		if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
													OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
		else                 	OLED_ShowString(15,30,"+"),
													OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
			
		if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
													OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
		else                 	OLED_ShowString(60,30,"+"),
													OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
			}
	  if(Car_Mode==Akm_Car)
	 {
			//阿克曼小车显示舵机的PWM的数值//
			OLED_ShowString(00,30,"SERVO:");
			if( Servo<0)		      OLED_ShowString(60,30,"-"),
														OLED_ShowNumber(80,30,-Servo,4,12);
			else                 	OLED_ShowString(60,30,"+"),
														OLED_ShowNumber(80,30, Servo,4,12); 
	 }
	 	 else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
	 {
		 //差速小车、履带车显示左电机的PWM的数值//
														 OLED_ShowString(00,30,"MA");
		 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(40,30,"-"),
														 OLED_ShowNumber(50,30,-MOTOR_A.Motor_Pwm,4,12);
		 else                 	 OLED_ShowString(40,30,"+"),
														 OLED_ShowNumber(50,30, MOTOR_A.Motor_Pwm,4,12); 
	 }	
	 
	 //The 5th line of the display shows the content // 
	 //显示屏第5行显示内容//
	 if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
	 {
			//麦轮小车显示电机D的目标速度和当前实际速度//
			OLED_ShowString(0,40,"D");
			if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
														OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
			else                 	OLED_ShowString(15,40,"+"),
														OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 			
			if( MOTOR_D.Encoder<0)OLED_ShowString(60,40,"-"),
														OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,40,"+"),
														OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
	 }

	 else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
	 {
		 //差速小车、履带车显示右电机的PWM的数值//
														 OLED_ShowString(00,40,"MB");
		 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(40,40,"-"),
														 OLED_ShowNumber(50,40,-MOTOR_B.Motor_Pwm,4,12);
		 else                 	 OLED_ShowString(40,40,"+"),
														 OLED_ShowNumber(50,40, MOTOR_B.Motor_Pwm,4,12);
	 }			 

	
		//The 6th line of the display shows the content // 
		//显示屏第6行显示内容// 
		//Display the current control mode 
	 //显示当前控制模式//
	/* ROS: no local modes left, show the latched fault byte instead */
	if(RosFault) OLED_ShowNumber(0,50,RosFault,2,12);
	else         OLED_ShowString(0,50,"--");
	
		
	 //显示当前小车是否允许控制//
	 if(EN==1&&Flag_Stop==0)   	OLED_ShowString(45,50,"O N");  
	 else                      	OLED_ShowString(45,50,"OFF"); 
		
															OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
		                          OLED_ShowString(88,50,".");
															OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
		                          OLED_ShowString(110,50,"V");
	 if(Voltage_Show%100<10) 		OLED_ShowNumber(92,50,0,2,12);
    //OLED_Clear();		
	OLED_Refresh_Gram();
}
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
函数功能：向APP发送数据
入口参数：无
返回  值：无
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //对电池电压处理成百分比形式
	 Voltage_Show=(Voltage*1000-10000)/27;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //车轮速度单位转换为0.01m/s，方便在APP显示
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //用于交替打印APP数据和显示波形
	 flag_show=!flag_show;
	
	 if(PID_Send==1) 
	 {	 
		
		//Send parameters to the APP, the APP is displayed in the debug screen
		//发送参数到APP，APP在调试界面显示
		//printf("{C%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI);
		printf("{C%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI);
		// printf("{B%d}$",(int)PointDataProcess[i].distance);
		 PID_Send=0;	
		  
	 }

    else if(flag_show==0)
	{
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //发送参数到APP，APP在首页显示
		printf("{:%d\t:%d\t:%d}\r\n",(int)Lidar_Result[0],(int)Lidar_Result[1],Lidar_Result[2]);
	  //printf("{A%d:%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(u8)Left_Figure,(u8)Right_Figure);
	}
	else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //发送参数到APP，APP在波形界面显示，把需要显示的波形填进相应的位置即可，最多可以显示5个波形
		//printf("{B%d:%d:%d}$",(int)RC_Velocity,(u8)Left_Figure,(u8)Right_Figure);
		 //printf("{%d}\r\n",b-a);
	
	 }
 		 
}

//OLED显示CCD函数
void OLED_Show_CCD(void)
{ 
	 u8 i,t;
	 for(i = 0;i<128; i++)
  {
		if(ADV[i]<CCD_Threshold) t=1; else t=0;
		OLED_DrawPoint_Shu(i,0,t);
  }
}

//OLED显示CCD画点函数
void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	 u8 i=0;
  OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	  for(i = 0;i<8; i++)
  {
      OLED_DrawPoint(x,y+i,t);
  }
}

//开机显示一次的内容
//void oled_show_once(void)
//{
//   OLED_ShowString(0,00,"Turn Right Wheel");  //转动右轮
//   OLED_ShowString(0,10,"TO Select Mode"); //选择模式
//	 OLED_ShowString(0,20,"Current Mode Is");//当前的模式是：
//	if(ELE_ON_Flag==1)				  OLED_ShowString(50,30,"ELE");//电磁巡线模式
//	if(CCD_ON_Flag==1)				  OLED_ShowString(50,30,"CCD");//CCD模式
//	if(APP_ON_Flag==1)				  OLED_ShowString(50,30,"APP");//其他模式，如APP、PS2、Remote
//	if(Along_wall==1)				    OLED_ShowString(50,30,"WAL");
//	if(Follow_ON_Flag==1)       OLED_ShowString(50,30,"FOL");
//	if(Avoid_ON_Flag==1)        OLED_ShowString(50,30,"Avo");
//	if(PS2_ON_Flag==1)        OLED_ShowString(50,30,"Ps2");
//	
//	OLED_ShowString(0,40,"Press User Key");// 按一下用户按键
//  OLED_ShowString(0,50,"TO End Selection");//结束选择
//	OLED_Refresh_Gram();	//OLED刷新
//}

