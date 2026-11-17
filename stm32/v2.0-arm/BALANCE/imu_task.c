#include "imu_task.h"
/**************************************************************************
Function: Get Angle function
Input   : none
Output  : none
函数功能：获取角度信息
入口参数：无
返回  值：无
**************************************************************************/
volatile float Pitch,Roll,Yaw;
/* rad/s. Published together with the three angles below. */
volatile float Imu_Gz = 0.0f;
void Get_Angle(void)
{
	imu->Update_9axisVal(&axis_9Val);           //陀螺仪数据更新
	if(axis_9Val.gyro.z>-0.02f&&axis_9Val.gyro.z<0.02f) axis_9Val.gyro.z=0;
	imu->UpdateAttitude(axis_9Val,&AttitudeVal);//更新姿态角
	//*57.2958是将弧度转化为角度
	/* Publish the attitude and the yaw rate as ONE consistent set.
	 *
	 * Balance_task runs at a HIGHER priority than this task, so it can preempt
	 * here at any instruction. Without this critical section it can build an
	 * uplink frame whose roll came from this cycle while yaw is still the
	 * previous cycle's.
	 *
	 * The gyro needs the same treatment for a worse reason: Update_9axisVal()
	 * rewrites gyro.z in place several times over (raw counts, zero-offset,
	 * x0.061, x0.01745, then the drift correction), so a read landing between
	 * two of those steps returns a raw ADC count presented as rad/s - wrong by
	 * a factor of ~57, not merely stale.
	 *
	 * The fix has to live here, in the WRITER. A critical section in the reader
	 * would not help: the preemption happens before the reader starts, so the
	 * partial update is already done by then.
	 *
	 * No reader side locking is needed once this is atomic, because this task
	 * can never preempt Balance_task - nothing else writes these values. */
	taskENTER_CRITICAL();
	Roll   = -( AttitudeVal.roll  *57.2958f);
	Pitch  = -( AttitudeVal.pitch *57.2958f);
	Yaw    = -( AttitudeVal.yaw   *57.2958f);
	Imu_Gz = axis_9Val.gyro.z;
	taskEXIT_CRITICAL();
}
void ICM20948_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
    while(1)
    {
        //This task runs at 200Hz
        //此任务以200Hz的频率运行
        vTaskDelayUntil(&lastWakeTime, F2T(RATE_200_HZ));
		//解算角度
		Get_Angle();
    }
}




