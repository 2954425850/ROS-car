#include "adc.h"

float Voltage,Voltage_Count,Voltage_All; //Variables related to battery voltage sampling //电池电压采样相关的变量  
/* ROS: battery voltage calibration.
 * Volt = Get_Adc(Battery_Ch) * 3.3 * 11.0 * Revise / 4096
 *
 * Measured on this board: displayed 11.78 for a true 11.35 (+3.8%), and
 * displayed 5.50 for a true 5.27 (+4.4%). Both are consistently ~4% high,
 * so it is a linear scale error, not a bad reading. The factory value of
 * 1.03 is a +3% correction which, on this board, moves the reading further
 * the wrong way.
 *
 *   required factor = true / displayed   = 0.9635 at 12V, 0.9582 at 5V
 *   new Revise      = 1.03 * 0.9635       = 0.9924  (12V, operating point)
 *                     1.03 * 0.9582       = 0.9869  (5V, bench only)
 *
 * 0.99 splits the difference: within 0.3% at both points. This only affects
 * the low-battery warning threshold; it feeds the uplink voltage field too.
 * To re-calibrate:  Revise_new = 0.99 * (multimeter reading / displayed). */
const float Revise=0.99;

//CCD相关变量
u16 CCD_Median,CCD_Threshold,ADV[128]={0}; 

//电磁巡线相关变量																										
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor,sum; 
/**************************************************************************
Function: ADC initializes battery voltage detection
Input   : none
Output  : none
函数功能：ADC初始化电池电压检测
入口参数：无
返回  值：无
**************************************************************************/
void  Adc_Init(void)
{  
		GPIO_InitTypeDef       GPIO_InitStructure;
		GPIO_StructInit(&GPIO_InitStructure);   /* ROS FIX 2026-10-13: define every member - *Init() reads the rest
		 * as stack garbage */
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		ADC_InitTypeDef       ADC_InitStructure;

		/* ROS FIX 2026-10-13: ADC_InitTypeDef was only partially filled in, and
		 * ADC_Init() ORs ADC_ExternalTrigConv straight into CR2 (bit 11 = ALIGN).
		 * Leftover stack contents in that member therefore decided the data
		 * alignment: garbage with bit 11 set makes the ADC left-aligned and every
		 * conversion comes back as raw<<4, i.e. 16x high - the battery reading
		 * then saturates on the 12.6 V clamp and never moves. Defining every
		 * member removes the dependency on the stack. */
		ADC_StructInit(&ADC_InitStructure);

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOA时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟


		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PB0 通道8
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
		GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化  
	
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PB1 通道9
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
		GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化


		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 

		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
		ADC_CommonInit(&ADC_CommonInitStructure);//初始化

		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
		ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
		ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
		ADC_Cmd(ADC1, ENABLE);//开启AD转换器	 
}		
/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
函数功能：AD采样
入口参数：ADC的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	//Sets the specified ADC rule group channel, one sequence, and sampling time
	//设置指定ADC的规则组通道，一个序列，采样时间
	
	//ADC1,ADC通道,采样时间为480周期
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );
  //Enable the specified ADC1 software transformation startup function	
  //使能指定的ADC1的软件转换启动功能	
	ADC_SoftwareStartConv(ADC1);
	//Wait for the conversion to finish
  //等待转换结束	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	//Returns the result of the last ADC1 rule group conversion
	//返回最近一次ADC1规则组的转换结果
	return ADC_GetConversionValue(ADC1);	
}

/**************************************************************************
Function: Collect multiple ADC values to calculate the average function
Input   : ADC channels and collection times
Output  : AD conversion results
函数功能：采集多次ADC值求平均值函数
入口参数：ADC通道和采集次数
返 回 值：AD转换结果
**************************************************************************/
u16 Get_adc_Average(u8 chn, u8 times)
{
  u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(chn);
		delay_ms(5);
	}
	return temp_val/times;
}

/**************************************************************************
Function: Read the battery voltage
Input   : none
Output  : Battery voltage in mV
函数功能：读取电池电压 
入口参数：无
返回  值：电池电压，单位mv
**************************************************************************/
float Get_battery_volt(void)   
{  
	float Volt;
	
	//The resistance partial voltage can be obtained by simple analysis according to the schematic diagram
	//电阻分压，具体根据原理图简单分析可以得到	
	Volt=Get_Adc(Battery_Ch)*3.3*11.0*Revise/1.0/4096;	
	return Volt;
}





