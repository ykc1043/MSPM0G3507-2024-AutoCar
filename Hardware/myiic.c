#include "myiic.h"



/******************************************************************
 * �� �� �� �ƣ�IIC_Start
 * �� �� ˵ ����IIC��ʼ�ź�
 * �� �� �� �Σ���
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void IIC_Start(void)
{
        SDA_OUT();
        
        SCL(0);
        SDA(1);
        SCL(1);
        
        delay_us(5);
        
        SDA(0);
        delay_us(5);
        SCL(0);
        delay_us(5);
        
        
}

/******************************************************************
 * �� �� �� �ƣ�IIC_Stop
 * �� �� ˵ ����IICֹͣ�ź�
 * �� �� �� �Σ���
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void IIC_Stop(void)
{
        SDA_OUT();
        
        SCL(0);
        SDA(0);
        
        SCL(1);
        delay_us(5);
        SDA(1);
        delay_us(5);
        
}

/******************************************************************
 * �� �� �� �ƣ�IIC_Send_Ack
 * �� �� ˵ ������������Ӧ��
 * �� �� �� �Σ�0Ӧ��  1��Ӧ��
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void IIC_Send_Ack(uint8_t ack)
{
        SDA_OUT();
        SCL(0);
        SDA(0);
        delay_us(5);
        if(!ack) SDA(0);
        else         SDA(1);
        SCL(1);
        delay_us(5);
        SCL(0);
        SDA(1);
}

/******************************************************************
 * �� �� �� �ƣ�IIC_Wait_Ack
 * �� �� ˵ �����ȴ��ӻ�Ӧ��
 * �� �� �� �Σ���
 * �� �� �� �أ�1=��Ӧ��   0=��Ӧ��
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
uint8_t I2C_WaitAck(void)
{
        char ack = 0;
        unsigned char ack_flag = 10;
        SDA_IN();
    SDA(1);
        delay_us(5);
        SCL(1);
        delay_us(5);
        while( (SDA_GET()==1) && ( ack_flag ) )
        {
                ack_flag--;
                delay_us(5);
        }
        
        if( ack_flag <= 0 )
        {
                IIC_Stop();
                return 1;
        }
        else
        {
                SCL(0);
                SDA_OUT();
        }
        return ack;
}
/******************************************************************
 * �� �� �� �ƣ�IIC_Write
 * �� �� ˵ ����IICдһ���ֽ�
 * �� �� �� �Σ�datд�������
 * �� �� �� �أ���
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
void Send_Byte(uint8_t data)
{
        int i = 0;
        SDA_OUT();
        SCL(0);//����ʱ�ӿ�ʼ���ݴ���
        
        for( i = 0; i < 8; i++ )
        {
                SDA( (data & 0x80) >> 7 );
                delay_us(2);
        data<<=1;
        delay_us(6); 
                SCL(1);
                delay_us(4);
                SCL(0);
                delay_us(4);
                
        }
}

/******************************************************************
 * �� �� �� �ƣ�IIC_Read
 * �� �� ˵ ����IIC��1���ֽ�
 * �� �� �� �Σ���
 * �� �� �� �أ�������1���ֽ�����
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
uint8_t Read_Byte(void)
{
        unsigned char i,receive=0;
    SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
        {
        SCL(0);
        delay_us(5);
        SCL(1);
        delay_us(5);
        receive<<=1;
        if( SDA_GET() )
        {        
            receive|=1;   
        }
        delay_us(5); 
    }                
  return receive;
}

/******************************************************************
 * �� �� �� �ƣ�IIC_WriteReg
 * �� �� ˵ ����IIC����д������
 * �� �� �� �Σ�addr������ַ regaddr�Ĵ�����ַ numҪд��ĳ��� regdataд������ݵ�ַ
 * �� �� �� �أ�0=��ȡ�ɹ�   ����=��ȡʧ��
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
uint8_t IIC_WriteRegLen(uint8_t addr,uint8_t regaddr,uint8_t num,uint8_t *regdata)
{
    uint16_t i = 0;
        IIC_Start();
        Send_Byte((addr<<1)|0);
        if( I2C_WaitAck() == 1 ) {IIC_Stop();return 1;}
        Send_Byte(regaddr);
        if( I2C_WaitAck() == 1 ) {IIC_Stop();return 2;}
    
        for(i=0;i<num;i++)
    {
        Send_Byte(regdata[i]);
        if( I2C_WaitAck() == 1 ) {IIC_Stop();return (3+i);}
    }        
        IIC_Stop();
    return 0;
}


/******************************************************************
 * �� �� �� �ƣ�IIC_ReadData
 * �� �� ˵ ����IIC������ȡ����
 * �� �� �� �Σ�addr������ַ regaddr�Ĵ�����ַ numҪ��ȡ�ĳ��� Read��ȡ��������Ҫ�洢�ĵ�ַ
 * �� �� �� �أ�0=��ȡ�ɹ�   ����=��ȡʧ�� 
 * ��       �ߣ�LC
 * ��       ע����
******************************************************************/
uint8_t IIC_ReadDataLen(uint8_t addr, uint8_t regaddr,uint8_t num,uint8_t* Read)
{
        uint8_t i;
        IIC_Start();
        Send_Byte((addr<<1)|0);
        if( I2C_WaitAck() == 1 ) {IIC_Stop();return 1;}
        Send_Byte(regaddr);
        if( I2C_WaitAck() == 1 ) {IIC_Stop();return 2;}
        
        IIC_Start();
        Send_Byte((addr<<1)|1);
        if( I2C_WaitAck() == 1 ) {IIC_Stop();return 3;}
        
        for(i=0;i<(num-1);i++){
				delay_us(5);
                Read[i]=Read_Byte();
                IIC_Send_Ack(0);
        }
		delay_us(5);
        Read[i]=Read_Byte();
        IIC_Send_Ack(1);         
        IIC_Stop();
        return 0;
}

///******************************************************************
// * �� �� �� �ƣ�SHT20_Read
// * �� �� ˵ ����������ʪ��
// * �� �� �� �Σ�regaddr�Ĵ�����ַ regaddr=0xf3�����¶� =0xf5����ʪ�� 
// * �� �� �� �أ�regaddr=0xf3ʱ�����¶ȣ�regaddr=0xf5ʱ����ʪ��
// * ��       �ߣ�LC
// * ��       ע����
//******************************************************************/
//float SHT20_Read(uint8_t regaddr)
//{        
//    unsigned char data_H = 0;
//    unsigned char data_L = 0;
//    float temp = 0;
//    IIC_Start();
//    IIC_Write(0x80|0);
//    if( IIC_Wait_Ack() == 1 ) printf("error -1\r\n");
//    IIC_Write(regaddr);
//    if( IIC_Wait_Ack() == 1 ) printf("error -2\r\n");
//       
//    do{
//    delay_us(10);
//    IIC_Start();
//    IIC_Write(0x80|1);
//    
//    }while( IIC_Wait_Ack() == 1 );

//    delay_us(20);
//    
//    data_H = IIC_Read();
//    IIC_Send_Ack(0);
//    data_L = IIC_Read();
//    IIC_Send_Ack(1);
//    IIC_Stop();
//    
//    if( regaddr == 0xf3 )
//    {
//        temp = ((data_H<<8)|data_L) / 65536.0 * 175.72 - 46.85;
//    }
//    if( regaddr == 0xf5 )
//    {
//        temp = ((data_H<<8)|data_L) / 65536.0 * 125.0 - 6;
//    }
//   return temp;

//}
