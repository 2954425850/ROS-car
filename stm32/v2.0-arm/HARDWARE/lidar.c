/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-07-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-07-05

All rights reserved
***********************************************/
#include "lidar.h"
#include <string.h>
#include "sys.h"

/**************************************************************************
Function: LIDAR_USART_Init
Input   : none
Output  : none
函数功能：雷达串口初始化
入口参数: 无 
返回  值：无
**************************************************************************/	 	
//串口5
//1圈雷达点云数据，N10P角度分辨率0.31,一圈1161个点左右
PointCloud_t PointCloud_Data[1161];

#define RECEIVE_BUF_MAX_SIZE 	108  //DMA单次最大搬运数据量--刚好一帧
u8 RX_BUF[RECEIVE_BUF_MAX_SIZE]={0};  //DMA数据缓冲区--刚好一帧

DMA_InitTypeDef DMA_TransmitInitStruct;

int Lidar_Deal_Flag=0;
void LIDAR_USART_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	USART_DeInit(UART5);  //复位串口5

	// 打开串口GPIO的时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOC D时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //使能USART时钟
	 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 

	// 将USART Tx的GPIO配置为推挽复用模式
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //初始化

	// 将USART Rx的GPIO配置为浮空输入模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化


	// 配置串口的工作参数
	// 配置波特率
	USART_InitStructure.USART_BaudRate =460800;
	// 配置 针数据字长
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	// 配置停止位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	// 配置校验位
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	// 配置硬件流控制
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	// 配置工作模式，收发一起
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	// 完成串口的初始化配置
	USART_Init(UART5, &USART_InitStructure);

//收发DMA配置	
	DMA_InitTypeDef DMA_ReceiveInitStruct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA_ReceiveInitStruct.DMA_Channel = DMA_Channel_4;	//设置DMA通道
    DMA_ReceiveInitStruct.DMA_PeripheralBaseAddr = (uint32_t) &UART5->DR;//设置外设地址
  	DMA_ReceiveInitStruct.DMA_Memory0BaseAddr = (uint32_t) RX_BUF;//设置内存地址
  	DMA_ReceiveInitStruct.DMA_DIR =  DMA_DIR_PeripheralToMemory;//设置搬运方向：外设到内存
  	DMA_ReceiveInitStruct.DMA_BufferSize = RECEIVE_BUF_MAX_SIZE+1;//搬运数据长度
  	DMA_ReceiveInitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址在搬运过程中是否自增(设置不自增)
  	DMA_ReceiveInitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址在搬运过程中是否自增(设置自增)
  	DMA_ReceiveInitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//数据大小为一个字节
  	DMA_ReceiveInitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//数据大小为一个字节
	DMA_ReceiveInitStruct.DMA_Mode = DMA_Mode_Normal;//无需循环
	DMA_ReceiveInitStruct.DMA_Priority = DMA_Priority_Medium;
 	DMA_ReceiveInitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  	DMA_ReceiveInitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_ReceiveInitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_ReceiveInitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  	DMA_Init(DMA1_Stream0, &DMA_ReceiveInitStruct);
	USART_DMACmd(UART5,USART_DMAReq_Rx,ENABLE);
	DMA_Cmd(DMA1_Stream0, ENABLE);//默认开启DMA接收
	
	//USART 空闲NVIC
	NVIC_InitStructure.NVIC_IRQChannel =UART5_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);//开启串口空闲中断
	USART_Cmd(UART5, ENABLE);  //使能串口

}
/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
函数功能：串口5接收中断
入口参数：无
返回  值：无
**************************************************************************/
int N10P1_DataHandle(uint8_t* oribuf);
extern int a,b;
int RxCounter=0;
u8 *start_ptr,*end_ptr;
void UART5_IRQHandler(void)                
{
	//空闲中断
	if(USART_GetITStatus(UART5,USART_IT_IDLE) != RESET){
    	 UART5->SR;
   		 UART5->DR;  		//清除IDLE标志			
		DMA_Cmd(DMA1_Stream0, DISABLE);  //关闭接收DMA,防止处理其间有数据	
		DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0);//清除DMA1_Steam0传输完成标志 
		
//		//本次接收的数据量（最大接收量 - 剩余的数据量 = 实际接收量）debug调试使用
//		RxCounter = (RECEIVE_BUF_MAX_SIZE+1) - DMA_GetCurrDataCounter(DMA1_Stream0);	
//		printf("%d\r\n",RxCounter);
		
		//N10P_DataHandle(RX_BUF);
		N10P1_DataHandle(RX_BUF);
		DMA_SetCurrDataCounter(DMA1_Stream0, RECEIVE_BUF_MAX_SIZE+1);
		DMA1_Stream0->M0AR=(uint32_t) RX_BUF;
		DMA_Cmd(DMA1_Stream0, ENABLE);
	}
}



//接收并处理N10P 1圈雷达点云数据，角度分辨率0.31°.
//PointCloud_t N10P_PointCloud_Data[1161];
//int N10P_DataHandle(uint8_t* oribuf)
//{
//	static int error_cnt=0;
//	float angle_temp;
//	//1帧数据校验
//	uint8_t CrcVal = 0;
//	for(uint8_t i=0;i<107;i++) CrcVal+=oribuf[i];
//	if( CrcVal!=oribuf[107] )
//	{
//		//调试使用，监测误码的次数
//		error_cnt++;
//		/* 数据校验失败的处理 */
//		return 0;
//	}
//	//雷达在线校验变量，与Balance_task中相关变量配合检验雷达是否在线
//	Lidar_Success_Receive_flag=1;
//	Lidar_flag_count=0;
//	//计算当前数据帧的起始角度，结束角度
//	float startangle,endangle,angleIncr;
//	startangle = (float)((uint16_t)(oribuf[5]<<8 | oribuf[6]))/100.f;
//	endangle = (float)((uint16_t)(oribuf[105]<<8 | oribuf[106]))/100.f;
//	//计算角度每个数据之间的角度增量值：角度范围/分辨率增量区间 N10P分辨率为0.31°左右
//	if( startangle > endangle ) angleIncr = (endangle + 360.f - startangle)/31.0f;
//	else                        angleIncr = (endangle - startangle)/31.0f;
//	
//	//数值下标索引值
//	uint16_t index = 0;            //当前角度对应的数组下标值
//	uint16_t lostindex = 0;        //用于检查是否有跳过下标的情况
//	static uint16_t lastindex = 0;//用于保存上一次的下标，辅组判断是否完成1圈的数据接收、是否有跳过下标
//	for(u8 i=0;i<32;i++)
//	{
//		angle_temp=startangle+i*angleIncr;//计算当前角度
//		if(angle_temp>360) angle_temp-=360;//防止角度超360°溢出
//		index = round(angle_temp/0.31f);//根据角度计算数组下标的位置，数组下标*0.31 = 实际的雷达角度
//		index = index%1161;           //防止数组溢出，使用1161个数据表示0~360°，取决于雷达分辨率，N10为0.8°
//		
//		//索引值完成了1次轮询,说明处理好了1圈雷达的数据.
//		if( index<lastindex )
//		{
//			/* 完成了1圈的数据处理，通知需要使用的任务，此时雷达数据可用了 */
//			// 使用 memcpy 进行赋值,将实时接收到的一圈数据直接一次赋值出去
//			memcpy(PointCloud_Data, N10P_PointCloud_Data, sizeof(N10P_PointCloud_Data));
//			/* 检查是否存在遗漏点 */
//			lostindex = (index + 1161) - lastindex;
//		}
//		else
//		{	/* 检查是否存在遗漏点 */
//			lostindex = index  - lastindex;
//		}
//		//计算当前角度下的距离与强度信息
//		PointCloud_t TmpPonit = { 0 };
//		TmpPonit.distance = oribuf[7+(i*3)] << 8 | oribuf[8+(i*3)];
//		TmpPonit.peak = oribuf[9+(i*3)];	
//		//根据索引值的不同情况进行数据赋值
//		if( 1 == lostindex || lostindex >= 3 ) //1为正常情况，3为雷达数据本身存在漏点，大于3则为刚上电瞬间的随机情况.这些情况直接赋值
//		{
//			N10P_PointCloud_Data[ index ].distance = TmpPonit.distance;
//			N10P_PointCloud_Data[ index ].peak = TmpPonit.peak;
//		}
//		else if( 0 == lostindex ) //出现下标重复的情况,则将两次结果取平均
//		{
//			N10P_PointCloud_Data[ index ].distance = (N10P_PointCloud_Data[ index ].distance + TmpPonit.distance)/2;
//			N10P_PointCloud_Data[ index ].peak    =  (N10P_PointCloud_Data[ index ].peak + TmpPonit.peak)/2;
//		}
//		else if( 2 == lostindex ) //若出现遗漏的点，将遗漏点的两边数值取平均，再赋值到遗漏点处
//		{
//			//先赋值当前角度
//			N10P_PointCloud_Data[ index ].distance = TmpPonit.distance;
//			N10P_PointCloud_Data[ index ].peak = TmpPonit.peak;
//			
//			//漏掉的上一个角度索引赋值,将左右两边值取平均作为该点的值.
//			N10P_PointCloud_Data[ (lastindex + 1)%1161 ].distance = ( N10P_PointCloud_Data[ index ].distance + N10P_PointCloud_Data[lastindex].distance ) / 2 ;
//			N10P_PointCloud_Data[ (lastindex + 1)%1161 ].peak =     ( N10P_PointCloud_Data[ index ].peak     + N10P_PointCloud_Data[lastindex].peak ) / 2 ;
//		}
//		//保存本次索引值
//		lastindex = index;
//	}
//	return 1;
//}

//接收并处理N10P 1圈雷达点云数据，角度分辨率0.93°（每隔3个点采一次样，其余点舍去，因此分辨率提高为0.93°，数据量过多会导致建图后检索时时间过长）.
PointCloud_t N10P1_PointCloud_Data[387];
int N10P1_DataHandle(uint8_t* oribuf)
{
	static int error_cnt=0;
	float angle_temp;
	//1帧数据校验
	uint8_t CrcVal = 0;
	for(uint8_t i=0;i<107;i++) CrcVal+=oribuf[i];
	if( CrcVal!=oribuf[107] )
	{
		//调试使用，监测误码的次数
		error_cnt++;
		/* 数据校验失败的处理 */
		return 0;
	}
	//雷达在线校验变量，与Balance_task中相关变量配合检验雷达是否在线
	Lidar_Success_Receive_flag=1;
	Lidar_flag_count=0;
	//计算当前数据帧的起始角度，结束角度
	float startangle,endangle,angleIncr;
	startangle = (float)((uint16_t)(oribuf[5]<<8 | oribuf[6]))/100.f;
	endangle = (float)((uint16_t)(oribuf[105]<<8 | oribuf[106]))/100.f;
	//计算角度每个数据之间的角度增量值：角度范围/分辨率增量区间 N10P分辨率为0.31°左右
	if( startangle > endangle ) angleIncr = (endangle + 360.f - startangle)/31.0f;
	else                        angleIncr = (endangle - startangle)/31.0f;
	
	//数值下标索引值
	uint16_t index = 0;            //当前角度对应的数组下标值
	uint16_t lostindex = 0;        //用于检查是否有跳过下标的情况
	static uint16_t lastindex = 0;//用于保存上一次的下标，辅组判断是否完成1圈的数据接收、是否有跳过下标
	for(u8 i=0;i<32;i++)
	{
		angle_temp=startangle+i*angleIncr;//计算当前角度
		if(angle_temp>360) angle_temp-=360;//防止角度超360°溢出
		if(i%3==0)
		{
			index = round(angle_temp/0.93f);//根据角度计算数组下标的位置，数组下标*0.31 = 实际的雷达角度
			index = index%387;           //防止数组溢出，使用1161个数据表示0~360°，取决于雷达分辨率，N10为0.8°
			//索引值完成了1次轮询,说明处理好了1圈雷达的数据.
			if( index<lastindex )
			{
				/* 完成了1圈的数据处理，通知需要使用的任务，此时雷达数据可用了 */
				// 使用 memcpy 进行赋值,将实时接收到的一圈数据直接一次赋值出去
				if(!lidar_data_ok)
				{
					memcpy(PointCloud_Data, N10P1_PointCloud_Data, sizeof(N10P1_PointCloud_Data));
					lidar_data_ok=1;
				}
				/* 检查是否存在遗漏点 */
				lostindex = (index + 387) - lastindex;
			}
			else
			{	/* 检查是否存在遗漏点 */
				lostindex = index  - lastindex;
			}
			//计算当前角度下的距离与强度信息
			PointCloud_t TmpPonit = { 0 };
			TmpPonit.distance = oribuf[7+(i*3)] << 8 | oribuf[8+(i*3)];
			TmpPonit.peak = oribuf[9+(i*3)];	
			//根据索引值的不同情况进行数据赋值
			if( 1 == lostindex || lostindex >= 3 ) //1为正常情况，3为雷达数据本身存在漏点，大于3则为刚上电瞬间的随机情况.这些情况直接赋值
			{
				N10P1_PointCloud_Data[ index ].distance = TmpPonit.distance;
				N10P1_PointCloud_Data[ index ].peak = TmpPonit.peak;
			}
			else if( 0 == lostindex ) //出现下标重复的情况,则将两次结果取平均
			{
				N10P1_PointCloud_Data[ index ].distance = (N10P1_PointCloud_Data[ index ].distance + TmpPonit.distance)/2;
				N10P1_PointCloud_Data[ index ].peak    =  (N10P1_PointCloud_Data[ index ].peak + TmpPonit.peak)/2;
			}
			else if( 2 == lostindex ) //若出现遗漏的点，将遗漏点的两边数值取平均，再赋值到遗漏点处
			{
				//先赋值当前角度
				N10P1_PointCloud_Data[ index ].distance = TmpPonit.distance;
				N10P1_PointCloud_Data[ index ].peak = TmpPonit.peak;
				
				//漏掉的上一个角度索引赋值,将左右两边值取平均作为该点的值.
				N10P1_PointCloud_Data[ (lastindex + 1)%387 ].distance = ( N10P1_PointCloud_Data[ index ].distance + N10P1_PointCloud_Data[lastindex].distance ) / 2 ;
				N10P1_PointCloud_Data[ (lastindex + 1)%387 ].peak =     ( N10P1_PointCloud_Data[ index ].peak     + N10P1_PointCloud_Data[lastindex].peak ) / 2 ;
			}
			//保存本次索引值
			lastindex = index;
		}
	}
	return 1;
}

















