#ifndef __MYIIC_H__
#define __MYIIC_H__

#include "board.h"
#include "stdio.h"

//����SDA���ģʽ
#define SDA_OUT()   {                                                \
                        DL_GPIO_initDigitalOutput(I2C_SDA_IOMUX);    \
                        DL_GPIO_setPins(I2C_PORT, I2C_SDA_PIN);      \
                        DL_GPIO_enableOutput(I2C_PORT, I2C_SDA_PIN); \
                    }
//����SDA����ģʽ
#define SDA_IN()    { DL_GPIO_initDigitalInput(I2C_SDA_IOMUX); }
//��ȡSDA���ŵĵ�ƽ�仯
#define SDA_GET()   ( ( ( DL_GPIO_readPins(I2C_PORT,I2C_SDA_PIN) & I2C_SDA_PIN ) > 0 ) ? 1 : 0 )
//SDA��SCL���
#define SDA(x)      ( (x) ? (DL_GPIO_setPins(I2C_PORT,I2C_SDA_PIN)) : (DL_GPIO_clearPins(I2C_PORT,I2C_SDA_PIN)) )
#define SCL(x)      ( (x) ? (DL_GPIO_setPins(I2C_PORT,I2C_SCL_PIN)) : (DL_GPIO_clearPins(I2C_PORT,I2C_SCL_PIN)) )

void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Ack(uint8_t ack);
uint8_t I2C_WaitAck(void);
void Send_Byte(uint8_t data);
uint8_t Read_Byte(void);
uint8_t IIC_WriteRegLen(uint8_t addr,uint8_t regaddr,uint8_t num,uint8_t *regdata);
uint8_t IIC_ReadDataLen(uint8_t addr, uint8_t regaddr,uint8_t num,uint8_t* Read);



//float SHT20_Read(unsigned char regaddr);

#endif
