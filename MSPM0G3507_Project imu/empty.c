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
    // ���ݰ�����
    uint8_t packet[6];
    
    // ֡ͷ
    packet[0] = 0xAC;
    packet[1] = 0xAC;
    
    // ������������ת��Ϊ��������
    int16_t int_data = (int16_t)data;
    
    // ���ݲ���
    packet[2] = (uint8_t)(int_data >> 8); // ���ֽ�
    packet[3] = (uint8_t)(int_data & 0xFF); // ���ֽ�
    
    // У���
    packet[4] = packet[2] + packet[3];
    
    // ֡β
    packet[5] = 0x2B;

    // ͨ�����ڷ������ݰ�
    for(int i = 0; i < 6; i++) {
        while(DL_UART_isBusy(UART_0_INST));
        DL_UART_Main_transmitData(UART_0_INST, packet[i]);
    }
}

int main(void)
{
//	board_init();
	SYSCFG_DL_init();
	//��������жϱ�־
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//ʹ�ܴ����ж�
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	//��������жϱ�־
	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	//ʹ�ܴ����ж�
	
	//printf("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\nBoard Init [[ ** LCKFB ** ]]\r\n");
	//DL_WWDT_restart(WWDT0);
    int i = 30000000*2; while (i--);// ��ʱһ���ô������ϵ�׼����ϣ��������ϵ����Ҫ��ʼ����Ϻ�Ż����ָ���

    // ���Ѵ������������úô���������������Ȼ���������ϱ�---------------
    Cmd_03();// 1 ���Ѵ�����
    /**
       * �����豸����
     * @param accStill    �ߵ�-��ֹ״̬���ٶȷ�ֵ ��λdm/s?
     * @param stillToZero �ߵ�-��ֹ�����ٶ�(��λcm/s) 0:������ 255:��������
     * @param moveToZero  �ߵ�-��̬�����ٶ�(��λcm/s) 0:������
     * @param isCompassOn 1=�迪���ų� 0=��رմų�
     * @param barometerFilter ��ѹ�Ƶ��˲��ȼ�[ȡֵ0-3],��ֵԽ��Խƽ�ȵ�ʵʱ��Խ��
     * @param reportHz ���������ϱ��Ĵ���֡��[ȡֵ0-250HZ], 0��ʾ0.5HZ
     * @param gyroFilter    �������˲�ϵ��[ȡֵ0-2],��ֵԽ��Խƽ�ȵ�ʵʱ��Խ��
     * @param accFilter     ���ټ��˲�ϵ��[ȡֵ0-4],��ֵԽ��Խƽ�ȵ�ʵʱ��Խ��
     * @param compassFilter �������˲�ϵ��[ȡֵ0-9],��ֵԽ��Խƽ�ȵ�ʵʱ��Խ��
     * @param Cmd_ReportTag ���ܶ��ı�ʶ
     */
	int imu_freq=5;
    Cmd_12(5, 255, 0,  0, 3, imu_freq, 2, 4, 9, 0x40);// 2 �����豸����(����1)
    //Cmd_19();// 3 �������������ϱ�
	Cmd_18();// �ر����������ϱ�
	
//    Cmd_05();// z��ǹ���
//    Cmd_06();// xyz��������ϵ����
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
				Cmd_05();// z��ǹ���
				Cmd_06();// xyz��������ϵ����
				delay_ms(500);
//				while(1)
//				{
//					if(KEY_N1_READ()!=0)
//						break;
//				}
			}
		}
		
		NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
		Cmd_11();// ��ȡ1�ζ��ĵĹ�������
		delay_ms(5);
//		printf("%d\r\n",(int)AngleZ);
		send_packet(AngleZ+180);
		NVIC_DisableIRQ(UART_1_INST_INT_IRQN);
		
		if(f_sendcmd>0)
		{
			if(f_sendcmd==5)
			{
				Cmd_05();// z��ǹ���
			}else if(f_sendcmd==6)
			{
				Cmd_06();// xyz��������ϵ����
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
        //LED��������ߵ�ƽ
        DL_GPIO_setPins(LED1_PORT, LED1_PIN_14_PIN);
		//printf("LED [ON]\r\n");
        delay_ms(500);
		
        //LED��������͵�ƽ
        DL_GPIO_clearPins(LED1_PORT, LED1_PIN_14_PIN);
		//printf("LED [OFF]\r\n");
        delay_ms(500);
    }
}


	



void SysTick_Handler(void)
{
	SysTick->CTRL &= ~(1 << 16); // Clear the SysTick interrupt flag


}










