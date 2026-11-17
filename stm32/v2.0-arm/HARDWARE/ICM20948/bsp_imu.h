#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "sys.h"

//选择Bank值
#define REG_VAL_SELECT_BANK_0 0x00
#define REG_VAL_SELECT_BANK_1 0x10
#define REG_VAL_SELECT_BANK_2 0x20
#define REG_VAL_SELECT_BANK_3 0x30


typedef struct{ //中间私有变量
	float x;
	float y;
	float z;
}PrivateBuf_t;
	
typedef struct{
	PrivateBuf_t gyro; //3轴角速度
	PrivateBuf_t accel;//3轴加速度
	PrivateBuf_t magn;//3轴磁力计
}IMU_DATA_t;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}ATTITUDE_DATA_t;


//IMU操作接口
typedef struct {
    uint8_t (*Init)(void);   //返回值：1错误 0无异常
    uint8_t (*DeInit)(void); //返回值：1错误 0无异常
	
	void (*UpdateZeroPoint)(IMU_DATA_t* point,ATTITUDE_DATA_t* attitude);
    void (*Update_9axisVal)(IMU_DATA_t* imudata); //更新9轴数据
	void (*UpdateAttitude)(IMU_DATA_t imudata,ATTITUDE_DATA_t *attitude); //更新姿态角
	
}IMUInterface_t,*pIMUInterface_t;


extern IMUInterface_t UserICM20948;
//定义imu数据
extern IMU_DATA_t axis_9Val;                   //9轴数值
extern ATTITUDE_DATA_t AttitudeVal;            //欧拉角
extern pIMUInterface_t imu;
#endif /* __BSP_IMU_H */
