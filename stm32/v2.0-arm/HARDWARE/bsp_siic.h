#ifndef __BSP_sIIC_H
#define __BSP_sIIC_H

#include "sys.h"
#include "system.h"



//IO操作函数	 
#define IIC_SCL    PBout(10) //SCL
#define IIC_SDA    PBout(11) //SDA	 
#define READ_SDA()   PBin(11)  //输入SDA 
//IO方向设置
#define SDA_IN()  {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=0<<11*2;}	//PB5输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(11*2));GPIOB->MODER|=1<<11*2;} //PB5输出模式

#define sIIC_SDA_H PBout(11)=1
#define sIIC_SDA_L PBout(11)=0
#define sIIC_SCL_H PBout(10)=1
#define sIIC_SCL_L PBout(10)=0

typedef enum 
{
  IIC_OK       = 0x00U,
  IIC_ERR      = 0x01U,
  IIC_BUSY     = 0x02U,
  IIC_TIMEOUT  = 0x03U
} IIC_Status_t;

typedef struct {
	//参数：设备地址，要写入的数据，要写入的数据长度，超时时间
    IIC_Status_t (*write)(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
    IIC_Status_t (*read)(uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	
	//参数：设备地址，要访问的寄存器地址，要写入的数据，要写入的数据长度，超时时间
    IIC_Status_t (*write_reg)(uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
	IIC_Status_t (*read_reg)(uint16_t DevAddress, uint16_t MemAddress,  uint8_t *pData, uint16_t Size, uint32_t Timeout);
	
	void (*delay_ms)(u32 ms);
}IICInterface_t,*pIICInterface_t;

extern IICInterface_t UserII2Dev;
void IIC_Init(void);
#endif /* __BSP_sIIC_H */
