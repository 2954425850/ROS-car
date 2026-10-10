#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"

#define SEND_DATA_SIZE    24
#define RECEIVE_DATA_SIZE 11

/*****A structure for storing triaxial data of a gyroscope accelerometer*****/
/*****���ڴ�������Ǽ��ٶȼ��������ݵĽṹ��*********************************/
typedef struct __Mpu6050_Data_ 
{
	short X_data; //2 bytes //2���ֽ�
	short Y_data; //2 bytes //2���ֽ�
	short Z_data; //2 bytes //2���ֽ�
}Mpu6050_Data;

/*******The structure of the serial port sending data************/
/*******���ڷ������ݵĽṹ��*************************************/
typedef struct _SEND_DATA_  
{
	unsigned char buffer[SEND_DATA_SIZE];
	struct _Sensor_Str_
	{
		unsigned char Frame_Header; //1���ֽ�
		short X_speed;	            //2 bytes //2���ֽ�
		short Y_speed;              //2 bytes //2���ֽ�
		short Z_speed;              //2 bytes //2���ֽ�
		short Power_Voltage;        //2 bytes //2���ֽ�
		Mpu6050_Data Accelerometer; //6 bytes //6���ֽ�
		Mpu6050_Data Gyroscope;     //6 bytes //6���ֽ�	
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Sensor_Str;
}SEND_DATA;

typedef struct _RECEIVE_DATA_  
{
	unsigned char buffer[RECEIVE_DATA_SIZE];
	struct _Control_Str_
	{
		unsigned char Frame_Header; //1 bytes //1���ֽ�
		float X_speed;	            //4 bytes //4���ֽ�
		float Y_speed;              //4 bytes //4���ֽ�
		float Z_speed;              //4 bytes //4���ֽ�
		unsigned char Frame_Tail;   //1 bytes //1���ֽ�
	}Control_Str;
}RECEIVE_DATA;
void uart1_init(u32 bound);
void uart2_init(u32 bound);

int USART1_IRQHandler(void);
int USART2_IRQHandler(void);

float Vz_to_Akm_Angle(float Vx, float Vz);
float XYZ_Target_Speed_transition(u8 High,u8 Low);
void usart1_send(u8 data);
void usart2_send(u8 data);
u8 AT_Command_Capture(u8 uart_recv);


/**************************************************************************
 * ROS host link protocol   (USART1 = PA9/PA10, 115200 8N1, common GND)
 *
 * NOTE: USART1 must run at NVIC preemption priority 5 (see uart1_init).
 *       FreeRTOSConfig.h sets configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 *       to 5, so a numerically lower priority would not be masked by
 *       taskENTER_CRITICAL() and would tear the shared command struct.
 *
 *   Pi -> STM32   downlink   12 bytes @ 20Hz
 *     0  header   0xAA
 *     1  len      0x0C              total frame length
 *     2  vx       int16   mm/s
 *     4  vy       int16   mm/s
 *     6  wz       int16   mrad/s
 *     8  flags    uint8   bit0 enable  bit1 estop
 *     9  seq      uint8
 *    10  crc      uint16  little endian, CRC-16/MODBUS over bytes 0..9
 *
 *   STM32 -> Pi   uplink     28 bytes @ 50Hz
 *     0  header   0x55
 *     1  len      0x1C              total frame length
 *     2  tick     uint32  ms
 *     6  vA       int16   m/s*1000  polarity corrected wheel speed
 *     8  vB       int16   m/s*1000
 *    10  vC       int16   m/s*1000
 *    12  vD       int16   m/s*1000
 *    14  roll     int16   rad*10000
 *    16  pitch    int16   rad*10000
 *    18  yaw      int16   rad*10000  wrapped to +-PI
 *    20  gz       int16   rad/s*1000
 *    22  voltage  uint16  mV
 *    24  fault    uint8   ROS_FAULT_*
 *    25  seq      uint8
 *    26  crc      uint16  little endian, CRC-16/MODBUS over bytes 0..25
 *
 *   CRC-16/MODBUS : poly 0x8005 reflected 0xA001, init 0xFFFF,
 *                   refin/refout true, xorout 0x0000
 *   Check vector  : crc16_modbus("123456789") == 0x4B37
 *
 *   Chassis is fixed to FourWheel_Car (four wheel differential):
 *   A/B = left pair, C/D = right pair, effective track = W + A = 1.000 m.
 *   The host derives vz = (v_right - v_left) / 1.000  (FULL track, not half).
 *   (it was 0.360 on the vendor reference car, 0.187 + 0.173)
 *   W = 0.460 and A = 0.540 are tape measured wheel centre distances of
 *   the real chassis. Only their SUM enters the model, so the host must
 *   divide by the same number. This sum is an initial guess: a skid steer
 *   chassis scrubs its tyres, so its true effective track is wider than
 *   the geometric one. Calibrate by spinning the car in place and
 *   comparing the angle the host integrates against the IMU yaw.
 **************************************************************************/

#define ROS_DOWN_SIZE        12      /* total downlink frame length      */
#define ROS_UP_SIZE          28      /* total uplink frame length        */
#define ROS_DOWN_HEAD        0xAA
#define ROS_UP_HEAD          0x55
#define ROS_DOWN_LEN         0x0C    /* downlink "len" field value       */
#define ROS_UP_LEN           0x1C    /* uplink   "len" field value       */
#define ROS_DOWN_CRC_IDX     10      /* crc offset in the downlink frame */
#define ROS_UP_CRC_IDX       26      /* crc offset in the uplink frame   */
#define ROS_RX_TIMEOUT_MS    200     /* link watchdog: stop the motors   */

/* downlink flags */
#define ROS_FLAG_ENABLE      0x01    /* bit0: host allows motion         */
#define ROS_FLAG_ESTOP       0x02    /* bit1: host emergency stop        */

/* uplink fault bits */
#define ROS_FAULT_UNDERVOLT  0x01    /* bit0: battery below 10V          */
#define ROS_FAULT_OVERVOLT   0x02    /* bit1: reserved                   */
#define ROS_FAULT_ESTOP      0x04    /* bit2: enable switch off / stop   */
#define ROS_FAULT_IMU        0x08    /* bit3: IMU init failed            */
#define ROS_FAULT_TIMEOUT    0x10    /* bit4: link watchdog fired        */
#define ROS_FAULT_ENCODER    0x20    /* bit5: reserved                   */

/**************************************************************************
 * Servo command frame  (second downlink frame type, header 0xAB)
 *
 *   0  header  0xAB
 *   1  len     0x11                  total frame length
 *   2  s1      int16  us } one per servo channel, in channel order
 *   4  s2      int16  us }
 *   6  s3      int16  us } 0 means "leave this channel unchanged", so the
 *   8  s4      int16  us } host can move one joint at a time
 *  10  s5      int16  us }
 *  12  s6      int16  us }
 *  14  seq     uint8
 *  15  crc     uint16 little endian, CRC-16/MODBUS over bytes 0..14
 *
 * Channels 1..4 = PC6 PC7 PC8 PC9 (TIM8), 5..6 = PB14 PB15 (TIM12).
 * Channels 5 and 6 drive the gimbal. Values are clamped by Servo_Set() to
 * SERVO_MIN..SERVO_MAX, so a bad command can never hit a mechanical stop.
 *
 * This is a separate frame type on purpose: the 0xAA motion frame was verified
 * on hardware, and keeping it untouched keeps that verification valid.
 **************************************************************************/
#define ROS_SERVO_SIZE       17
#define ROS_SERVO_HEAD       0xAB
#define ROS_SERVO_LEN        0x11
#define ROS_SERVO_CRC_IDX    15
#define ROS_SERVO_NUM        6

typedef struct _ROS_SERVO_
{
	volatile unsigned char rx_ready;
	volatile unsigned char seq;
	volatile short         us[ROS_SERVO_NUM];  /* 0 = leave unchanged */
	volatile unsigned int  frames;
	volatile unsigned int  crc_err;
} ROS_SERVO;

extern ROS_SERVO RosServo;
void ros_apply_servo(void);    /* Balance_task @100Hz: push values to the servos */

/**************************************************************************
 * Arm command frame  (third downlink frame type, header 0xAC)
 *
 *   0  header  0xAC
 *   1  len     0x14                  total frame length
 *   2  p[1]    int16  raw  0..1000    gripper      (servo id 1)
 *   4  p[2]    int16  raw  0..1000    wrist roll   (servo id 2)
 *   6  p[3]    int16  raw  0..1000    wrist pitch  (servo id 3)
 *   8  p[4]    int16  raw  0..1000    forearm      (servo id 4)
 *  10  p[5]    int16  raw  0..1000    upper arm    (servo id 5)
 *  12  p[6]    int16  signed -1000..+1000  base yaw  (servo id 6)
 *  14  t_ms    uint16 ms              travel time, 0 = firmware default
 *  16  flags   uint8  bit0 = re-home the base (EDGE: one search per
 *                           0->1 transition, not per frame)
 *  17  seq     uint8
 *  18  crc     uint16 little endian, CRC-16/MODBUS over bytes 0..17
 *
 * Units are raw servo counts on purpose: the ROS side already owns the
 * joint-angle <-> raw mapping (it has the URDF), and keeping the firmware
 * free of that layer is what makes it testable. Base field 0 is the centre
 * of the +-360 deg travel (SOFT_LIMIT, follow), one count 0.36 deg.
 *
 * One frame means "every joint walks to its target over t_ms". Streaming
 * is not a separate mode: the host simply sends these at 20-50 Hz with a
 * small t_ms, and point-to-point sends them rarely with a large one.
 **************************************************************************/
#define ROS_ARM_SIZE         20
#define ROS_ARM_HEAD         0xAC
#define ROS_ARM_LEN          0x14
#define ROS_ARM_CRC_IDX      18
#define ROS_ARM_NUM          6

#define ROS_ARM_FLAG_REHOME  0x01    /* bit0: re-run base homing */

typedef struct _ROS_ARM_
{
	volatile unsigned char rx_ready;
	volatile unsigned char flags;
	volatile unsigned char seq;
	volatile short         pos[ROS_ARM_NUM];  /* raw counts, see above    */
	volatile unsigned short t_ms;
	volatile unsigned int  frames;
	volatile unsigned int  crc_err;
	volatile unsigned int  head_err;
} ROS_ARM;

extern ROS_ARM RosArm;
void ros_arm_byte(unsigned char c);   /* called from the USART1 ISR */

/**************************************************************************
 * Arm feedback frame  (second uplink frame type, header 0x56)
 *
 *   0  header  0x56
 *   1  len     0x16                  total frame length
 *   2  a[1..5] int16  raw 0..1000    the five position joints, read back
 *  12  a[6]    int16  signed        base angle estimate, -1000..+1000
 *  14  pot_raw int16                base pot raw; -1 = anchor read failed
 *  16  status  uint8  ARM_ST_*
 *  17  base_st uint8  ARM_BASE_*: 0 = not tracking yet, 2 = OK. Those two are
 *                  the only ones the firmware ever emits, so the field is a
 *                  boolean in practice. For "the base is unusable" read
 *                  ARM_ST_BASE_FAULT in the status byte - NOT base_st == 4,
 *                  which never appears.
 *  18  seq     uint8
 *  19  anchor u8    1 = control live: the stage machine is in INIT_READY and
 *                  is therefore consuming 0xAC frames. 0 during power-up,
 *                  during a re-home, and until the base anchoring finishes.
 *                  The host gates its arm commands on this instead of
 *                  guessing how long a homing takes.
 *                  (Legacy name: it was a boot-anchor flag that nothing ever
 *                  set, so it used to ship as a constant 0.)
 *  20  crc     uint16 little endian, CRC-16/MODBUS over bytes 0..19
 *
 * A separate frame rather than an extension of the 28 byte 0x55 uplink:
 * that frame was verified on hardware and the host parser is known good,
 * so it stays byte for byte identical. Two uplink headers is also
 * symmetric with the two downlink headers.
 **************************************************************************/
#define ROS_ARMFB_SIZE       22
#define ROS_ARMFB_HEAD       0x56
#define ROS_ARMFB_LEN        0x16
#define ROS_ARMFB_CRC_IDX    20

/* status bits for ROS_ARMFB.status */
#define ARM_ST_ONLINE        0x01
#define ARM_ST_BASE_KNOWN    0x02
#define ARM_ST_BASE_BLIND    0x04
#define ARM_ST_HOMING        0x08
#define ARM_ST_BASE_FAULT    0x10
#define ARM_ST_OVERTEMP      0x20
#define ARM_ST_UNDERVOLT     0x40
#define ARM_ST_ESTOP         0x80

/**************************************************************************
 * Raw bus passthrough  (bring-up only, header 0xAD down / 0x57 up)
 *
 * USART2 is the servo bus, so a PC cannot be plugged into it any more.
 * These two frames let a script on the Pi drive the bus by proxy, which
 * is how the base joint has to be characterised (motor-mode pot
 * behaviour, speed calibration, inter-frame gap).
 *
 *   0xAD down:  0 hdr  1 len  2..3 timeout_ms u16  4 n u8
 *               5 .. 5+n-1  payload bytes
 *               5+n .. 6+n  crc u16 little endian over bytes 0..4+n
 *               len = 7 + n  (total frame length, header included)
 *
 *   0x57 up:    0 hdr  1 len  2 seq  3 status  4 n u16  5 = 0
 *               6.. rx bytes  6+n_rx.. crc u16
 *               len = 8 + n_rx
 *
 * Guarded by ARM_DEBUG_PASSTHROUGH so it can be compiled out: the shipped
 * build must not carry a frame that lets any host drive the servo bus raw.
 **************************************************************************/
#define ARM_DEBUG_PASSTHROUGH   1    /* set to 1 for a bring-up build */

#define ROS_RAW_SIZE         26      /* frame buffer, >= 7 + MAX_PAYLOAD  */
#define ROS_RAW_HEAD         0xAD
#define ROS_RAW_MIN_LEN      7       /* 1 + 1 + 2 + 1 + 0 payload + 2 crc */
#define ROS_RAW_MAX_PAYLOAD  16

#define ROS_RAWFB_SIZE       24
#define ROS_RAWFB_HEAD       0x57

typedef struct _ROS_RAW_
{
	volatile unsigned char rx_ready;
	volatile unsigned char n;
	volatile unsigned char raw[ROS_RAW_MAX_PAYLOAD];
	volatile unsigned short timeout_ms;
	volatile unsigned int  frames;
	volatile unsigned int  crc_err;   /* bad length OR bad crc          */
} ROS_RAW;

extern ROS_RAW RosRaw;
void ros_raw_byte(unsigned char c);
void ros_send_raw_fb(const unsigned char *rx, unsigned char n);

typedef struct _ROS_ARM_FB_
{
	short         pos[ROS_ARM_NUM];
	short         pot_raw;
	unsigned char status;
	unsigned char base_state;
	unsigned char anchor;
} ROS_ARM_FB;

extern ROS_ARM_FB RosArmFb;
void ros_send_arm_fb(void);

/* Latest downlink frame, written by the USART1 ISR and read by Balance_task
 * under taskENTER_CRITICAL(). rx_ready is the single publish flag. */
typedef struct _ROS_DOWN_
{
	volatile unsigned char rx_ready;  /* ISR set 1 on a CRC-valid frame   */
	volatile unsigned char flags;
	volatile unsigned char seq;
	volatile float         vx;        /* m/s                              */
	volatile float         vy;        /* m/s                              */
	volatile float         wz;        /* rad/s                            */
	volatile unsigned int  frames;    /* valid frames received            */
	volatile unsigned int  crc_err;   /* CRC failures                     */
	volatile unsigned int  head_err;  /* framing resyncs                  */
} ROS_DOWN;

/* Uplink payload, filled once per transmitted frame */
typedef struct _ROS_UP_
{
	float         vA, vB, vC, vD;     /* wheel speed m/s                  */
	float         roll, pitch, yaw;   /* rad                              */
	float         gz;                 /* rad/s                            */
	float         voltage;            /* V                                */
	unsigned char fault;
	unsigned char seq;
} ROS_UP;

extern ROS_DOWN      RosDown;
extern ROS_UP        RosUp;
extern unsigned char RosFault;        /* latched fault bits for the uplink */

unsigned short crc16_modbus(const unsigned char *buf, unsigned short len);
void ros_apply_command(void);  /* Balance_task @100Hz: decode + watchdog + Drive_Motor */
void ros_send_frame(void);     /* Balance_task @100Hz: transmits the uplink at 50Hz    */

#endif

