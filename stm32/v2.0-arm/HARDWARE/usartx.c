#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;
/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
函数功能：串口1初始化
入口参数：无
返 回 值：无
**************************************************************************/
void uart1_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //使能GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //使能USART时钟

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //初始化
	
  //UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5 ;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		
	//Enable the IRQ channel //IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器	
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //初始化串口1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //使能串口1
}
/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
函数功能：串口2初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //使能GPIO时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //使能USART时钟
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //推挽输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //高速50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //初始化
	
	//UsartNVIC configuration //UsartNVIC配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=5 ;
	//Subpriority //子优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
  //Enable the IRQ channel //IRQ通道使能	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters 
	//根据指定的参数初始化VIC寄存器		
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART Initialization Settings 初始化设置
	USART_InitStructure.USART_BaudRate = bound; //Port rate //串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //一个停止
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //收发模式
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //初始化串口2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //开启串口接受中断
	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //使能串口2 
}

/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
函数功能：串口1接收中断
入口参数：无
返 回 值：无
**************************************************************************/

/**************************************************************************
 * ROS host link implementation - USART1, 115200 8N1
 *
 * Frame layout is documented in usartx.h.
 *
 * The receiver is a byte level state machine that runs entirely inside the
 * USART1 interrupt. It deliberately calls NO FreeRTOS API, so it stays safe
 * at whatever priority uart1_init() assigns it, and it re-synchronises on
 * the next 0xAA whenever a frame fails its CRC.
 **************************************************************************/
ROS_DOWN      RosDown;
ROS_UP        RosUp;
unsigned char RosFault = 0;

#define ROS_PI  3.14159265358979f

static u8  s_rx_buf[ROS_DOWN_SIZE];
static u8  s_rx_idx   = 0;
static u8  s_rx_state = 0;   /* 0 = hunt header, 1 = expect length, 2 = payload */

/* Live command copy. Owned by Balance_task, never touched by the ISR. */
static u8    s_flags     = 0;
static float s_vx = 0.0f, s_vy = 0.0f, s_wz = 0.0f;
static u32   s_last_rx   = 0;
static u8    s_have_link = 0;
static u8    s_tx_seq    = 0;

/**************************************************************************
Function: CRC-16/MODBUS. poly 0x8005 reflected 0xA001, init 0xFFFF,
          refin/refout true, xorout 0x0000.
          Check vector: crc16_modbus("123456789") == 0x4B37
 **************************************************************************/
u16 crc16_modbus(const u8 *buf, u16 len)
{
	u16 crc = 0xFFFF;
	u8  i;
	while(len--)
	{
		crc ^= (u16)(*buf++);
		for(i = 0; i < 8; i++)
			crc = (crc & 1) ? (u16)((crc >> 1) ^ 0xA001) : (u16)(crc >> 1);
	}
	return crc;
}

/* Little endian helpers. The wire format is little endian everywhere. */
static short ros_get_i16(const u8 *p)
{
	return (short)((u16)p[0] | ((u16)p[1] << 8));
}

static void ros_put_u16(u8 *p, u16 v)
{
	p[0] = (u8)(v & 0xFF);
	p[1] = (u8)((v >> 8) & 0xFF);
}

static void ros_put_u32(u8 *p, u32 v)
{
	p[0] = (u8)(v & 0xFF);
	p[1] = (u8)((v >> 8) & 0xFF);
	p[2] = (u8)((v >> 16) & 0xFF);
	p[3] = (u8)((v >> 24) & 0xFF);
}

/* Saturate before narrowing, so an out of range value can never wrap around
 * into a plausible looking but completely wrong command. */
static short ros_sat_i16(float v)
{
	if(v >  32767.0f) return (short) 32767;
	if(v < -32768.0f) return (short)-32768;
	return (short)v;
}

static void ros_rx_reset(void)
{
	s_rx_idx   = 0;
	s_rx_state = 0;
}

/* A complete, CRC verified frame is available: decode it and publish. */
static void ros_rx_publish(void)
{
	RosDown.flags = s_rx_buf[8];
	RosDown.seq   = s_rx_buf[9];
	RosDown.vx    = (float)ros_get_i16(&s_rx_buf[2]) * 0.001f;  /* mm/s   -> m/s   */
	RosDown.vy    = (float)ros_get_i16(&s_rx_buf[4]) * 0.001f;
	RosDown.wz    = (float)ros_get_i16(&s_rx_buf[6]) * 0.001f;  /* mrad/s -> rad/s */
	RosDown.frames++;
	RosDown.rx_ready = 1;
}

static void ros_rx_byte(u8 c)
{
	u16 crc;
	u8  k, n;

	switch(s_rx_state)
	{
		case 0:                                    /* hunting for the header */
			if(c == ROS_DOWN_HEAD)
			{
				s_rx_buf[0] = c;
				s_rx_idx    = 1;
				s_rx_state  = 1;
			}
			break;

		case 1:                                    /* the length byte is due */
			if(c == ROS_DOWN_LEN)
			{
				s_rx_buf[1] = c;
				s_rx_idx    = 2;
				s_rx_state  = 2;
			}
			else if(c == ROS_DOWN_HEAD)            /* a fresh header, stay in sync */
			{
				s_rx_buf[0] = c;
				s_rx_idx    = 1;
			}
			else
			{
				RosDown.head_err++;
				ros_rx_reset();
			}
			break;

		default:                                   /* collecting the payload */
			s_rx_buf[s_rx_idx++] = c;
			if(s_rx_idx < ROS_DOWN_SIZE) break;

			crc = (u16)s_rx_buf[ROS_DOWN_CRC_IDX]
			    | ((u16)s_rx_buf[ROS_DOWN_CRC_IDX + 1] << 8);

			if(crc == crc16_modbus(s_rx_buf, ROS_DOWN_CRC_IDX))
			{
				ros_rx_publish();
				ros_rx_reset();
			}
			else
			{
				/* CRC failed. A 0xAA inside the payload may have been taken for
				 * a header, so re-scan what we already hold and restart from
				 * the next 0xAA rather than throwing the whole window away. */
				RosDown.crc_err++;
				for(k = 1; k < ROS_DOWN_SIZE; k++)
					if(s_rx_buf[k] == ROS_DOWN_HEAD) break;

				if(k < ROS_DOWN_SIZE)
				{
					n = 0;
					while(k < ROS_DOWN_SIZE) s_rx_buf[n++] = s_rx_buf[k++];
					s_rx_idx   = n;
					s_rx_state = 1;            /* the length byte is still due */
				}
				else
				{
					ros_rx_reset();
				}
			}
			break;
	}
}

/**************************************************************************
 * Servo frame decoder, header 0xAB.
 *
 * A SECOND, independent state machine on the same byte stream. The headers
 * 0xAA (motion) and 0xAB (servo) are disjoint, so neither decoder can ever
 * lock onto the other's frame, and ros_rx_byte() above is left exactly as it
 * was when the motion path was verified on hardware.
 **************************************************************************/
ROS_SERVO RosServo;

static u8 s_sv_buf[ROS_SERVO_SIZE];
static u8 s_sv_idx   = 0;
static u8 s_sv_state = 0;   /* 0 = hunt 0xAB, 1 = expect length, 2 = payload */

static void ros_sv_reset(void)
{
	s_sv_idx   = 0;
	s_sv_state = 0;
}

static void ros_sv_publish(void)
{
	u8 i;
	for(i = 0; i < ROS_SERVO_NUM; i++)
		RosServo.us[i] = (short)((u16)s_sv_buf[2 + i * 2]
		                       | ((u16)s_sv_buf[3 + i * 2] << 8));
	RosServo.seq      = s_sv_buf[14];
	RosServo.frames++;
	RosServo.rx_ready = 1;
}

static void ros_sv_byte(u8 c)
{
	u16 crc;
	u8  k, n;

	switch(s_sv_state)
	{
		case 0:                                     /* hunting for 0xAB */
			if(c == ROS_SERVO_HEAD)
			{
				s_sv_buf[0] = c;
				s_sv_idx    = 1;
				s_sv_state  = 1;
			}
			break;

		case 1:                                     /* the length byte is due */
			if(c == ROS_SERVO_LEN)
			{
				s_sv_buf[1] = c;
				s_sv_idx    = 2;
				s_sv_state  = 2;
			}
			else if(c == ROS_SERVO_HEAD)            /* fresh header, stay in sync */
			{
				s_sv_buf[0] = c;
				s_sv_idx    = 1;
			}
			else
			{
				RosServo.crc_err++;
				ros_sv_reset();
			}
			break;

		default:                                    /* collecting the payload */
			s_sv_buf[s_sv_idx++] = c;
			if(s_sv_idx < ROS_SERVO_SIZE) break;

			crc = (u16)s_sv_buf[ROS_SERVO_CRC_IDX]
			    | ((u16)s_sv_buf[ROS_SERVO_CRC_IDX + 1] << 8);

			if(crc == crc16_modbus(s_sv_buf, ROS_SERVO_CRC_IDX))
			{
				ros_sv_publish();
				ros_sv_reset();
			}
			else
			{
				RosServo.crc_err++;
				for(k = 1; k < ROS_SERVO_SIZE; k++)
					if(s_sv_buf[k] == ROS_SERVO_HEAD) break;

				if(k < ROS_SERVO_SIZE)
				{
					n = 0;
					while(k < ROS_SERVO_SIZE) s_sv_buf[n++] = s_sv_buf[k++];
					s_sv_idx   = n;
					s_sv_state = 1;
				}
				else
				{
					ros_sv_reset();
				}
			}
			break;
	}
}

/**************************************************************************
Function: Push the newest servo frame to the outputs. Called from the 100Hz
          control task.
A channel value of 0 means "leave that channel unchanged". If the host stops
sending, the servos simply hold their last position - that is the safe
behaviour for a gimbal, so there is no link watchdog on this path.
 **************************************************************************/
void ros_apply_servo(void)
{
	short us[ROS_SERVO_NUM];
	u8    i;

	if(!RosServo.rx_ready) return;

	taskENTER_CRITICAL();
	for(i = 0; i < ROS_SERVO_NUM; i++) us[i] = RosServo.us[i];
	RosServo.rx_ready = 0;
	taskEXIT_CRITICAL();

	for(i = 0; i < ROS_SERVO_NUM; i++)
		if(us[i] != 0)
			Servo_Set((u8)(i + 1), us[i]);
}

/**************************************************************************
Function: Serial port 1 receive interrupt
Input   : none
Output  : none
 **************************************************************************/
/**************************************************************************
 * Arm frame decoder, header 0xAC.
 *
 * A THIRD independent state machine on the USART1 byte stream. 0xAA, 0xAB
 * and 0xAC are disjoint headers, so no decoder can lock onto another's
 * frame, and the two verified paths are untouched. Same shape as
 * ros_sv_byte() above on purpose - a reader who understands one
 * understands all three.
 **************************************************************************/
ROS_ARM RosArm;
ROS_ARM_FB RosArmFb;

static unsigned char s_arm_buf[ROS_ARM_SIZE];
static unsigned char s_arm_idx   = 0;
static unsigned char s_arm_state = 0;  /* 0 hunt, 1 length, 2 payload */

static void ros_arm_reset(void)
{
	s_arm_idx   = 0;
	s_arm_state = 0;
}

static void ros_arm_publish(void)
{
	unsigned char i;
	for(i = 0; i < ROS_ARM_NUM; i++)
		RosArm.pos[i] = (short)((unsigned short)s_arm_buf[2 + i * 2]
		                      | ((unsigned short)s_arm_buf[3 + i * 2] << 8));
	RosArm.t_ms    = (unsigned short)((unsigned short)s_arm_buf[14]
	                                | ((unsigned short)s_arm_buf[15] << 8));
	RosArm.flags   = s_arm_buf[16];
	RosArm.seq     = s_arm_buf[17];
	RosArm.frames++;
	RosArm.rx_ready = 1;
}

void ros_arm_byte(unsigned char c)
{
	unsigned short crc;
	unsigned char  k, n;

	switch(s_arm_state)
	{
		case 0:
			if(c == ROS_ARM_HEAD)
			{
				s_arm_buf[0] = c;
				s_arm_idx    = 1;
				s_arm_state  = 1;
			}
			break;

		case 1:
			if(c == ROS_ARM_LEN)
			{
				s_arm_buf[1] = c;
				s_arm_idx    = 2;
				s_arm_state  = 2;
			}
			else if(c == ROS_ARM_HEAD)
			{
				s_arm_buf[0] = c;
				s_arm_idx    = 1;
			}
			else
			{
				RosArm.head_err++;
				ros_arm_reset();
			}
			break;

		default:
			s_arm_buf[s_arm_idx++] = c;
			if(s_arm_idx < ROS_ARM_SIZE) break;

			crc = (unsigned short)s_arm_buf[ROS_ARM_CRC_IDX]
			    | ((unsigned short)s_arm_buf[ROS_ARM_CRC_IDX + 1] << 8);

			if(crc == crc16_modbus(s_arm_buf, ROS_ARM_CRC_IDX))
			{
				ros_arm_publish();
				ros_arm_reset();
			}
			else
			{
				RosArm.crc_err++;
				for(k = 1; k < ROS_ARM_SIZE; k++)
					if(s_arm_buf[k] == ROS_ARM_HEAD) break;

				if(k < ROS_ARM_SIZE)
				{
					n = 0;
					while(k < ROS_ARM_SIZE) s_arm_buf[n++] = s_arm_buf[k++];
					s_arm_idx   = n;
					s_arm_state = 1;
				}
				else
				{
					ros_arm_reset();
				}
			}
			break;
	}
}

/* ==== ARM_TASK_BEGIN ==== */

/**************************************************************************
 * Raw passthrough decoder, header 0xAD. Variable length: the length byte
 * gives the frame total, exactly like the other three decoders. Lives
 * after the ARM_TASK marker on purpose - that marker is the generator's
 * slice boundary for the 0xAC decoder.
 *
 * n is clamped to what the declared length can actually hold, so a frame
 * whose CRC happens to validate while n is garbage cannot make the read
 * loop inspect bytes that were never part of it.
 **************************************************************************/
ROS_RAW RosRaw;

static unsigned char s_raw_buf[ROS_RAW_SIZE];
static unsigned char s_raw_idx   = 0;
static unsigned char s_raw_want  = 0;
static unsigned char s_raw_state = 0;   /* 0 hunt, 1 length, 2 payload */

void ros_raw_byte(unsigned char c)
{
	unsigned short crc;

	if(s_raw_state == 0)
	{
		if(c == ROS_RAW_HEAD) { s_raw_buf[0] = c; s_raw_idx = 1; s_raw_state = 1; }
		return;
	}
	if(s_raw_state == 1)
	{
		if(c < ROS_RAW_MIN_LEN || c > ROS_RAW_SIZE)
		{
			RosRaw.crc_err++;
			s_raw_idx = 0; s_raw_state = 0;
			if(c == ROS_RAW_HEAD) { s_raw_buf[0] = c; s_raw_idx = 1; s_raw_state = 1; }
			return;
		}
		s_raw_buf[1] = c;
		s_raw_want   = c;
		s_raw_idx    = 2;
		s_raw_state  = 2;
		return;
	}
	s_raw_buf[s_raw_idx++] = c;
	if(s_raw_idx < s_raw_want) return;

	crc = (unsigned short)s_raw_buf[s_raw_want - 2]
	    | ((unsigned short)s_raw_buf[s_raw_want - 1] << 8);
	if(crc == crc16_modbus(s_raw_buf, (unsigned short)(s_raw_want - 2)))
	{
		unsigned char i;
		RosRaw.timeout_ms = (unsigned short)s_raw_buf[2]
		                  | ((unsigned short)s_raw_buf[3] << 8);
		RosRaw.n = s_raw_buf[4];
		if(RosRaw.n > (unsigned char)(s_raw_want - 6))
			RosRaw.n = (unsigned char)(s_raw_want - 6);
		for(i = 0; i < RosRaw.n && i < ROS_RAW_MAX_PAYLOAD; i++)
			RosRaw.raw[i] = s_raw_buf[5 + i];
		RosRaw.frames++;
		RosRaw.rx_ready = 1;              /* publish flag written last */
	}
	else
	{
		RosRaw.crc_err++;
	}
	s_raw_idx = 0; s_raw_state = 0;
}

/**************************************************************************
Function: Transmit the 0x57 passthrough reply. Same shape as the other
          uplink senders, so the host parser only needs one frame reader.
 **************************************************************************/
void ros_send_raw_fb(const unsigned char *rx, unsigned char n)
{
	unsigned char buf[ROS_RAWFB_SIZE];
	unsigned char i;
	static unsigned char seq = 0;

	if(n > (unsigned char)(ROS_RAWFB_SIZE - 8)) n = (unsigned char)(ROS_RAWFB_SIZE - 8);

	buf[0] = ROS_RAWFB_HEAD;
	buf[1] = (unsigned char)(n + 8);
	buf[2] = seq++;
	buf[3] = 0;                       /* status: reserved, 0 = ok */
	buf[4] = n;
	buf[5] = 0;
	for(i = 0; i < n; i++) buf[6 + i] = rx[i];
	ros_put_u16(&buf[6 + n], crc16_modbus(buf, (unsigned short)(6 + n)));
	for(i = 0; i < (unsigned char)(8 + n); i++) usart1_send(buf[i]);
}
int USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		u8 c = (u8)(USART1->DR & 0xFF);   /* reading DR is what clears RXNE */
		ros_rx_byte(c);                   /* motion frame, 0xAA - unchanged */
		ros_sv_byte(c);                   /* servo frame,  0xAB - independent */
		ros_arm_byte(c);                  /* arm frame,    0xAC - independent */
#if ARM_DEBUG_PASSTHROUGH
		ros_raw_byte(c);                  /* bring-up passthrough, 0xAD       */
#endif
	}
	return 0;
}

/**************************************************************************
Function: Decode the newest downlink frame, enforce the link watchdog and
          run the kinematics. Called from Balance_task at 100Hz.
 **************************************************************************/
void ros_apply_command(void)
{
	u32 now = getSysTickCnt();

	/* ---- take the frame the ISR published, if any ---- */
	if(RosDown.rx_ready)
	{
		taskENTER_CRITICAL();
		s_flags          = RosDown.flags;
		s_vx             = RosDown.vx;
		s_vy             = RosDown.vy;
		s_wz             = RosDown.wz;
		RosDown.rx_ready = 0;
		taskEXIT_CRITICAL();

		s_last_rx   = now;
		s_have_link = 1;
		RosFault   &= (u8)~ROS_FAULT_TIMEOUT;
	}

	/* ---- link watchdog: the host died or the cable came loose ---- */
	if(s_have_link && (u32)(now - s_last_rx) > ROS_RX_TIMEOUT_MS)
	{
		s_have_link = 0;
		RosFault   |= ROS_FAULT_TIMEOUT;
	}

	/* ---- anything that is not explicitly enabled is a stop ---- */
	if(!s_have_link
	   || !(s_flags & ROS_FLAG_ENABLE)
	   ||  (s_flags & ROS_FLAG_ESTOP))
	{
		Move_X = 0.0f;
		Move_Y = 0.0f;
		Move_Z = 0.0f;
	}
	else
	{
		Move_X = s_vx;
		Move_Y = s_vy;
		Move_Z = s_wz;
	}

	/* ---- hardware protection bits, reported on the uplink ---- */
	if(Voltage < 10.0f)           RosFault |= ROS_FAULT_UNDERVOLT;
	else                          RosFault &= (u8)~ROS_FAULT_UNDERVOLT;

	if(EN == 0 || Flag_Stop == 1) RosFault |= ROS_FAULT_ESTOP;
	else                          RosFault &= (u8)~ROS_FAULT_ESTOP;

	Drive_Motor(Move_X, Move_Y, Move_Z);
}

/**************************************************************************
Function: Build and transmit one uplink frame.
          Called from Balance_task at 50Hz. Every value it reads is owned by
          Balance_task, so no locking is needed here.
 **************************************************************************/
void ros_send_frame(void)
{
	static u8 s_tx_div = 0;
	u8    buf[ROS_UP_SIZE];
	float yaw, mv;
	u8    i;

	/* Called from the 100Hz control task, transmitted at 50Hz. */
	if(++s_tx_div < 2) return;
	s_tx_div = 0;

	RosUp.vA = MOTOR_A.Encoder;
	RosUp.vB = MOTOR_B.Encoder;
	RosUp.vC = MOTOR_C.Encoder;
	RosUp.vD = MOTOR_D.Encoder;

	/* the IMU keeps its angles in degrees, the wire format wants radians */
	RosUp.roll  = Roll  * 0.01745329252f;
	RosUp.pitch = Pitch * 0.01745329252f;

	yaw = Yaw * 0.01745329252f;
	while(yaw >  ROS_PI) yaw -= 2.0f * ROS_PI;     /* wrap to +-PI */
	while(yaw < -ROS_PI) yaw += 2.0f * ROS_PI;
	RosUp.yaw = yaw;

	RosUp.gz      = Imu_Gz;                       /* already rad/s */
	RosUp.voltage = Voltage;
	RosUp.fault   = RosFault;
	RosUp.seq     = s_tx_seq++;

	buf[0] = ROS_UP_HEAD;
	buf[1] = ROS_UP_LEN;
	ros_put_u32(&buf[2],  getSysTickCnt());
	ros_put_u16(&buf[6],  (u16)ros_sat_i16(RosUp.vA * 1000.0f));
	ros_put_u16(&buf[8],  (u16)ros_sat_i16(RosUp.vB * 1000.0f));
	ros_put_u16(&buf[10], (u16)ros_sat_i16(RosUp.vC * 1000.0f));
	ros_put_u16(&buf[12], (u16)ros_sat_i16(RosUp.vD * 1000.0f));
	ros_put_u16(&buf[14], (u16)ros_sat_i16(RosUp.roll  * 10000.0f));
	ros_put_u16(&buf[16], (u16)ros_sat_i16(RosUp.pitch * 10000.0f));
	ros_put_u16(&buf[18], (u16)ros_sat_i16(RosUp.yaw   * 10000.0f));
	ros_put_u16(&buf[20], (u16)ros_sat_i16(RosUp.gz    * 1000.0f));

	/* battery in mV, clamped so a bad ADC reading cannot wrap the field */
	mv = RosUp.voltage * 1000.0f;
	if(mv < 0.0f)     mv = 0.0f;
	if(mv > 65535.0f) mv = 65535.0f;
	ros_put_u16(&buf[22], (u16)mv);

	buf[24] = RosUp.fault;
	buf[25] = RosUp.seq;

	ros_put_u16(&buf[ROS_UP_CRC_IDX], crc16_modbus(buf, ROS_UP_CRC_IDX));

	for(i = 0; i < ROS_UP_SIZE; i++) usart1_send(buf[i]);
}
/**************************************************************************
Function: Transmit the 0x56 arm feedback frame. Called from arm_task at
          20Hz - the arm owns this frame, not the chassis task.
 **************************************************************************/
void ros_send_arm_fb(void)
{
	unsigned char buf[ROS_ARMFB_SIZE];
	unsigned char i, seq;
	/* transmit sequence for the 0x56 frame. Owned by arm_task, which is
	 * the only caller - hence no critical section. Kept separate from
	 * RosArm.seq, which the USART1 ISR overwrites with the RECEIVED
	 * downlink sequence on every accepted 0xAC frame. */
	static unsigned char s_armfb_seq = 0;
	short         sx;

	seq = (unsigned char)(++s_armfb_seq);

	buf[0] = ROS_ARMFB_HEAD;
	buf[1] = ROS_ARMFB_LEN;
	for(i = 0; i < ROS_ARM_NUM; i++)
	{
		buf[2 + i * 2] = (unsigned char)((unsigned short)RosArmFb.pos[i] & 0xFF);
		buf[3 + i * 2] = (unsigned char)(((unsigned short)RosArmFb.pos[i] >> 8) & 0xFF);
	}
	sx = RosArmFb.pot_raw;
	buf[14] = (unsigned char)((unsigned short)sx & 0xFF);
	buf[15] = (unsigned char)(((unsigned short)sx >> 8) & 0xFF);
	buf[16] = RosArmFb.status;
	buf[17] = RosArmFb.base_state;
	buf[18] = seq;
	buf[19] = RosArmFb.anchor;
	ros_put_u16(&buf[ROS_ARMFB_CRC_IDX], crc16_modbus(buf, ROS_ARMFB_CRC_IDX));

	for(i = 0; i < ROS_ARMFB_SIZE; i++) usart1_send(buf[i]);
}

/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART2_IRQHandler(void)
{
	/* USART2 is the bus-servo line (LX/HTS 0x55 protocol, 115200 8N1).
	 *
	 * Everything here is byte-at-a-time on purpose: the ISR runs at NVIC
	 * preempt priority 5, which is inside the range FreeRTOS masks, so it
	 * must not call a single FreeRTOS API. It only feeds a state machine
	 * and publishes completed replies into LxRx.
	 *
	 * The old APP / Bluetooth parsers that used to live here are still in
	 * this file but are no longer wired up - servo replies would have been
	 * eaten by them.
	 */
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		u8 c = (u8)(USART2->DR & 0xFF);   /* reading DR clears RXNE */
		lx_rx_byte(c);
	}
	return 0;
}

#if 0
/* ------------------------------------------------------------------
 * The pre-Task-2 USART2 ISR, kept here VERBATIM (every byte, including
 * the GBK comments) because the rule for this project is "unwire the
 * old entry points, never delete the code". USART2 now carries the
 * bus-servo line, so none of these APP / Bluetooth bytes can reach this
 * parser any more. It sits inside a disabled preprocessor block, so
 * nothing below is compiled, named or linked - a reference copy only.
 * ------------------------------------------------------------------ */
int USART2_IRQHandler(void)
{	
	static int temp_count = 0;				//用于记录前进的指令的次数，第一次连接蓝牙的时候需要用到
	int Usart_Receive;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //Check if data is received //判断是否接收到数据
	{	      
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
				
		Usart_Receive=USART2->DR; //Read the data //读取数据
		//抓包方法和使用
		if(AT_Command_Capture(Usart_Receive)) return 1;
	  
		if(Usart_Receive==0x4B) 
			//Enter the APP steering control interface
		  //进入APP转向控制界面
			Turn_Flag=1;  
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 
      // Enter the APP direction control interface		
			//进入APP方向控制界面	
			Turn_Flag=0;	
		
		if(Turn_Flag==0) 
		{
			//App rocker control interface command
			//APP摇杆控制界面命令
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				Flag_Direction=Usart_Receive-0x40;
			}
			else	if(Usart_Receive<=8)   
			{			
				Flag_Direction=Usart_Receive;
			}	
			else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)
		{
			//APP steering control interface command
			//APP转向控制界面命令
			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation //右自转
			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation  //左自转
			else                         Flag_Left=0,Flag_Right=0;
			if     (Usart_Receive==0x41||Usart_Receive==0x45) 
			{
				if((++temp_count) == 5)					//需要连续发送5次前进的指令，上拉转盘一段时间可开始app控制
				{
					temp_count = 0;
					APP_ON_Flag = RC_ON;		
					PS2_ON_Flag = RC_OFF;
				}
				Flag_Direction=Usart_Receive-0x40;
			}
			else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s //加速按键，+100mm/s
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons,   -100mm/s //减速按键，-100mm/s
		if(Usart_Receive=='a')	back_flag=!back_flag;
	 // The following is the communication with the APP debugging interface
	 //以下是与APP调试界面通讯
	 if(Usart_Receive==0x7B) Flag_PID=1;   //The start bit of the APP parameter instruction //APP参数指令起始位
	 if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stops the bit    //APP参数指令停止位

	 if(Flag_PID==1) //Collect data //采集数据
	 {
		Receive[i]=Usart_Receive;
		i++;
	 }
	 if(Flag_PID==2) //Analyze the data //分析数据
	 {
			if(Receive[3]==0x50) 	 PID_Send=1;
			else  if(Receive[1]!=0x23) 
      {								
				for(j=i;j>=4;j--)
				{
					Data+=(Receive[j-1]-48)*pow(10,i-j);
				}
				
				switch(Receive[1])			//APP电机调参
				{
					 case 0x30:  RC_Velocity=Data;break;
					 case 0x31:  Velocity_KP=Data;break;
					 case 0x32:  Velocity_KI=Data;break;
					 case 0x33:  break;
					 case 0x34:  break;
					 case 0x35:  break;
					 case 0x36:  break;
					 case 0x37:  break;
					 case 0x38:  break; 				
				} 
				}		
			//Relevant flag position is cleared			
			//相关标志位清零			
			Flag_PID=0;
			i=0;
			j=0;
			Data=0;
			memset(Receive, 0, sizeof(u8)*50); //Clear the array to zero//数组清零
	 }
   if(RC_Velocity<0)   RC_Velocity=0; 
	 if(RC_Velocity_CCD<0)   RC_Velocity_CCD=0;
	 if(RC_Velocity_ELE<0)   RC_Velocity_ELE=0;		 
  }
  return 0;	
}
#endif


/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来目标前进速度Vx、目标角速度Vz，转换为阿克曼小车的右前轮转角
入口参数：目标前进速度Vx、目标角速度Vz，单位：m/s，rad/s
返回  值：阿克曼小车的右前轮转角，单位：rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	//float AngleL;
	
	//Ackermann car needs to set minimum turning radius
	//If the target speed requires a turn radius less than the minimum turn radius,
	//This will greatly improve the friction force of the car, which will seriously affect the control effect
	//阿克曼小车需要设置最小转弯半径
	//如果目标速度要求的转弯半径小于最小转弯半径，
	//会导致小车运动摩擦力大大提高，严重影响控制效果
	Min_Turn_Radius=MINI_AKM_MIN_TURN_RADIUS;
	
	if(Vz!=0 && Vx!=0)
	{
		//If the target speed requires a turn radius less than the minimum turn radius
		//如果目标速度要求的转弯半径小于最小转弯半径
		if(float_abs(Vx/Vz)<=Min_Turn_Radius)
		{
			//Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			//降低目标角速度，配合前进速度，提高转弯半径到最小转弯半径
			if(Vz>0)
				Vz= float_abs(Vx)/(Min_Turn_Radius);
			else	
				Vz=-float_abs(Vx)/(Min_Turn_Radius);	
		}		
		R=Vx/Vz;
		//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR=atan(Axle_spacing/(R+0.5f*Wheel_spacing));
	}
	else
	{
		AngleR=0;
	}
	
	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//数据转换的中间变量
	short transition; 
	
	//将高8位和低8位整合成一个16位的short型数据
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //单位转换, mm/s->m/s						
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
	u32 guard = 100000;
	/* Wait for TXE (SR bit7 = 0x80), NOT TC (bit6 = 0x40). The original code
	 * wrote DR first and then waited on TC, which is benign in steady state
	 * because TC=1 implies TXE=1. It stops being benign once the wait can
	 * time out: a guard exit leaves TXE=0, and the next DR write would then
	 * clobber the byte still in flight. So the ordering and the guard have to
	 * be introduced together. TXE is also the correct bit for back-to-back
	 * transmission, since it frees the register one stop bit earlier than TC. */
	while((USART1->SR&0x80)==0 && --guard);
	USART1->DR = data;
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}




//蓝牙AT指令抓包，防止指令干扰到机器人正常的蓝牙通信
u8 AT_Command_Capture(u8 uart_recv)
{
	/*
	蓝牙链接时发送的字符，00:11:22:33:44:55为蓝牙的MAC地址
	+CONNECTING<<00:11:22:33:44:55\r\n
	+CONNECTED\r\n
	共44个字符
	
	蓝牙断开时发送的字符
	+DISC:SUCCESS\r\n
	+READY\r\n
	+PAIRABLE\r\n
	共34个字符
	\r -> 0x0D
	\n -> 0x0A
	*/
	
	static u8 pointer = 0; //蓝牙接受时指针记录器
	static u8 bt_line = 0; //表示现在在第几行
	static u8 disconnect = 0;
	static u8 connect = 0;
	
	//断开连接
	static char* BlueTooth_Disconnect[3]={"+DISC:SUCCESS\r\n","+READY\r\n","+PAIRABLE\r\n"};
	
	//开始连接
	static char* BlueTooth_Connect[2]={"+CONNECTING<<00:00:00:00:00:00\r\n","+CONNECTED\r\n"};


	//特殊标识符，开始警惕(使用时要-1)
	if(uart_recv=='+') 
	{
		bt_line++,pointer=0; //收到‘+’，表示切换了行数	
		disconnect++,connect++;
		return 1;//抓包，禁止控制
	}

	if(bt_line!=0) 
	{	
		pointer++;

		//开始追踪数据是否符合断开的特征，符合时全部屏蔽，不符合时取消屏蔽
		if(uart_recv == BlueTooth_Disconnect[bt_line-1][pointer])
		{
			disconnect++;
			if(disconnect==34) disconnect=0,connect=0,bt_line=0,pointer=0;
			return 1;//抓包，禁止控制
		}			

		//追踪连接特征 (bt_line==1&&connect>=13)区段是蓝牙MAC地址，每一个蓝牙MAC地址都不相同，所以直接屏蔽过去
		else if(uart_recv == BlueTooth_Connect[bt_line-1][pointer] || (bt_line==1&&connect>=13) )
		{		
			connect++;
			if(connect==44) connect=0,disconnect=0,bt_line=0,pointer=0;		
			return 1;//抓包，禁止控制
		}	

		//在抓包期间收到其他命令，停止抓包
		else
		{
			disconnect = 0;
			connect = 0;
			bt_line = 0;		
			pointer = 0;
			return 0;//非禁止数据，可以控制
		}			
	}
	
	return 0;//非禁止数据，可以控制
}
