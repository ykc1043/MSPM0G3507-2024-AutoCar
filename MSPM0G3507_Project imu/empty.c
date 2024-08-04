/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "board.h"
#include "stdio.h"
extern F32 AngleX,AngleY,AngleZ;
#define KEY_N1_READ() 	( ((DL_GPIO_readPins(GPIOA,DL_GPIO_PIN_17) & DL_GPIO_PIN_17) > 0) ? 1 : 0 )

uint8_t f_sendcmd=0;
extern uint8_t f_sendcmd;

void send_packet(float data) {
    // 数据包定义
    uint8_t packet[6];
    
    // 帧头
    packet[0] = 0xAC;
    packet[1] = 0xAC;
    
    // 将浮点型数据转换为整型数据
    int16_t int_data = (int16_t)data;
    
    // 数据部分
    packet[2] = (uint8_t)(int_data >> 8); // 高字节
    packet[3] = (uint8_t)(int_data & 0xFF); // 低字节
    
    // 校验和
    packet[4] = packet[2] + packet[3];
    
    // 帧尾
    packet[5] = 0x2B;

    // 通过串口发送数据包
    for(int i = 0; i < 6; i++) {
        while(DL_UART_isBusy(UART_0_INST));
        DL_UART_Main_transmitData(UART_0_INST, packet[i]);
    }
}

int main(void)
{
//	board_init();
	SYSCFG_DL_init();
	//清除串口中断标志
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//使能串口中断
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	//清除串口中断标志
	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	//使能串口中断
	
	//printf("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\nBoard Init [[ ** LCKFB ** ]]\r\n");
	//DL_WWDT_restart(WWDT0);
    int i = 30000000*2; while (i--);// 延时一下让传感器上电准备完毕，传感器上电后需要初始化完毕后才会接收指令的

    // 唤醒传感器，并配置好传感器工作参数，然后开启主动上报---------------
    Cmd_03();// 1 唤醒传感器
    /**
       * 设置设备参数
     * @param accStill    惯导-静止状态加速度阀值 单位dm/s?
     * @param stillToZero 惯导-静止归零速度(单位cm/s) 0:不归零 255:立即归零
     * @param moveToZero  惯导-动态归零速度(单位cm/s) 0:不归零
     * @param isCompassOn 1=需开启磁场 0=需关闭磁场
     * @param barometerFilter 气压计的滤波等级[取值0-3],数值越大越平稳但实时性越差
     * @param reportHz 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
     * @param gyroFilter    陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
     * @param accFilter     加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
     * @param compassFilter 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
     * @param Cmd_ReportTag 功能订阅标识
     */
	int imu_freq=5;
    Cmd_12(5, 255, 0,  0, 3, imu_freq, 2, 4, 9, 0x40);// 2 设置设备参数(内容1)
    //Cmd_19();// 3 开启数据主动上报
	Cmd_18();// 关闭数据主动上报
	
//    Cmd_05();// z轴角归零
//    Cmd_06();// xyz世界坐标系清零
	//DL_WWDT_restart(WWDT0);
//	for(int i=0;i<=360;i++)
//	{
//		send_packet(i);
//	}
//	
//	while(1)
//	{
//		DL_WWDT_restart(WWDT0);
//		printf("%d\r\n",KEY_N1_READ());
//		delay_ms(100);
//	}
	while(1)
	{
		DL_WWDT_restart(WWDT0);
		if(KEY_N1_READ()==0)
		{
			delay_ms(10);
			if(KEY_N1_READ()==0)
			{
				Cmd_05();// z轴角归零
				Cmd_06();// xyz世界坐标系清零
				delay_ms(500);
//				while(1)
//				{
//					if(KEY_N1_READ()!=0)
//						break;
//				}
			}
		}
		
		NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
		Cmd_11();// 获取1次订阅的功能数据
		delay_ms(5);
//		printf("%d\r\n",(int)AngleZ);
		send_packet(AngleZ+180);
		NVIC_DisableIRQ(UART_1_INST_INT_IRQN);
		
		if(f_sendcmd>0)
		{
			if(f_sendcmd==5)
			{
				Cmd_05();// z轴角归零
			}else if(f_sendcmd==6)
			{
				Cmd_06();// xyz世界坐标系清零
			}
			f_sendcmd=0;
		}
		if(AngleZ!=0)
			DL_GPIO_togglePins(LED1_PORT, LED1_PIN_14_PIN);
		delay_ms(1000/imu_freq);
		//im948_test();
	}

    while (0) 
    {                        
        //LED引脚输出高电平
        DL_GPIO_setPins(LED1_PORT, LED1_PIN_14_PIN);
		//printf("LED [ON]\r\n");
        delay_ms(500);
		
        //LED引脚输出低电平
        DL_GPIO_clearPins(LED1_PORT, LED1_PIN_14_PIN);
		//printf("LED [OFF]\r\n");
        delay_ms(500);
    }
}


	



void SysTick_Handler(void)
{
	SysTick->CTRL &= ~(1 << 16); // Clear the SysTick interrupt flag


}










