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

#ifndef __LIDAR_H
#define	__LIDAR_H
#include "system.h"
#include <stdint.h>

//1个雷达点云数据信息
typedef struct {
	uint16_t distance;//距离信息
	uint8_t  peak;    //强度信息
}PointCloud_t;

extern PointCloud_t N10P_PointCloud_Data[1161];//1圈雷达点云数据，角度分辨率0.31°.可直接使用
extern PointCloud_t PointCloud_Data[1161];

void LIDAR_USART_Init(void);

void Positioning(PointCloud_t* PointCloud,int length,float resolution,int* result);


//N10P雷达数据处理函数
int N10P_DataHandle(uint8_t* oribuf);



#endif

