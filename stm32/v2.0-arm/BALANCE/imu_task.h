#ifndef __IMU_TASK_H
#define __IMU_TASK_H
#include "system.h"

#define IMU_TASK_PRIO      3
#define IMU_STK_SIZE       256
#define IMU_TASK_RATE      RATE_100_HZ

/* Published atomically by ICM20948_task, read by Balance_task when it builds
 * the ROS uplink frame. */
extern volatile float Pitch, Roll, Yaw;   /* degrees */
extern volatile float Imu_Gz;             /* rad/s   */

void ICM20948_task(void *pvParameters);

#endif
