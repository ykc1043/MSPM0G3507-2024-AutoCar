/*
 * ������������Ӳ�������������չ����Ӳ�����Ϲ���ȫ����Դ
 * �����������www.lckfb.com
 * ����֧�ֳ�פ��̳���κμ������⻶ӭ��ʱ����ѧϰ
 * ������̳��https://oshwhub.com/forum
 * ��עbilibili�˺ţ������������塿���������ǵ����¶�̬��
 * ��������׬Ǯ���������й�����ʦΪ����
 * Change Logs:
 * Date           Author       Notes
 * 2024-06-26     LCKFB     first version
 */
#include "board.h"
#include "stdio.h"

#define RE_0_BUFF_LEN_MAX	128

volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;

#define RE_1_BUFF_LEN_MAX	128

volatile uint8_t  recv1_buff[RE_1_BUFF_LEN_MAX] = {0};
volatile uint16_t recv1_length = 0;
volatile uint8_t  recv1_flag = 0;

void board_init(void)
{
	// SYSCFG��ʼ��
	SYSCFG_DL_init();
	//��������жϱ�־
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	//ʹ�ܴ����ж�
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	
	printf("Board Init [[ ** LCKFB ** ]]\r\n");
}

//����δ�ʱ��ʵ�ֵľ�ȷus��ʱ
void delay_us(unsigned long __us) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    // ������Ҫ��ʱ���� = �ӳ�΢���� * ÿ΢���ʱ����
    ticks = __us * (32000000 / 1000000);

    // ��ȡ��ǰ��SysTickֵ
    told = SysTick->VAL;

    while (1)
    {
        // �ظ�ˢ�»�ȡ��ǰ��SysTickֵ
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            // ����ﵽ����Ҫ��ʱ���������˳�ѭ��
            if (tcnt >= ticks)
                break;
        }
    }
}
//����δ�ʱ��ʵ�ֵľ�ȷms��ʱ
void delay_ms(unsigned long ms) 
{
	delay_us( ms * 1000 );
}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

//���ڷ��͵����ַ�
void uart0_send_char(char ch)
{
	//������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
	while( DL_UART_isBusy(UART_0_INST) == true );
	//���͵����ַ�
	DL_UART_Main_transmitData(UART_0_INST, ch);

}
//���ڷ����ַ���
void uart0_send_string(char* str)
{
	//��ǰ�ַ�����ַ���ڽ�β ���� �ַ����׵�ַ��Ϊ��
	while(*str!=0&&str!=0)
	{
		//�����ַ����׵�ַ�е��ַ��������ڷ������֮���׵�ַ����
		uart0_send_char(*str++);
	}
}
//���ڷ��͵����ַ�
void uart1_send_char(char ch)
{
	//������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
	while( DL_UART_isBusy(UART_1_INST) == true );
	//���͵����ַ�
	DL_UART_Main_transmitData(UART_1_INST, ch);

}
//���ڷ����ַ���
void uart1_send_string(char* str)
{
	//��ǰ�ַ�����ַ���ڽ�β ���� �ַ����׵�ַ��Ϊ��
	while(*str!=0&&str!=0)
	{
		//�����ַ����׵�ַ�е��ַ��������ڷ������֮���׵�ַ����
		uart1_send_char(*str++);
	}
}


#if !defined(__MICROLIB)
//��ʹ��΢��Ļ�����Ҫ�������ĺ���
#if (__ARMCLIB_VERSION <= 6000000)
//�����������AC5  �Ͷ�����������ṹ��
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
	x = x;
}
#endif


//printf�����ض���
int fputc(int ch, FILE *stream)
{
	//������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
	while( DL_UART_isBusy(UART_0_INST) == true );
	
	DL_UART_Main_transmitData(UART_0_INST, ch);
	
	return ch;
}
#define BUFFER_SIZE 6
uint8_t rx_buffer[BUFFER_SIZE];
uint8_t rx_index = 0;
bool packet_ready = false;

int parse_packet() {
    if (packet_ready) {
        packet_ready = false;
        
        // ���֡ͷ��֡β
        if (rx_buffer[0] == 0xAC && rx_buffer[1] == 0xAC && rx_buffer[5] == 0x2B) {
            // ��ȡ����
            int16_t data = (rx_buffer[2] << 8) | rx_buffer[3];
            uint8_t checksum = rx_buffer[4];
            
            // У�����֤
            if (checksum == (rx_buffer[2] + rx_buffer[3])) {
                //printf("Received valid data: %d\n", data);
				return data;
            } else {
                //printf("Checksum error\n");
            }
        } else {
            //printf("Frame error\n");
        }
    }
	return -999;
}
extern uint8_t f_sendcmd;
void UART0_IRQHandler(void) {
    // ����һ���ֽ�
    uint8_t data = DL_UART_Main_receiveData(UART_0_INST);
    
	if(data=='5')
	{
		f_sendcmd=5;
	}else if(data=='6')
	{
		f_sendcmd=6;
	}
//    // �洢��������
//    rx_buffer[rx_index++] = data;
//    
//    // ����Ƿ��յ������������ݰ�
//    if (rx_index >= BUFFER_SIZE) {
//        packet_ready = true;
//        rx_index = 0; // ���������Խ�����һ�����ݰ�
//    }
}
void UART_1_INST_IRQHandler(void)
{
	uint8_t receivedData = 0;
	
	//��������˴����ж�
	switch( DL_UART_getPendingInterrupt(UART_1_INST) )
	{
		case DL_UART_IIDX_RX://����ǽ����ж�
			
			// ���շ��͹��������ݱ���
			receivedData = DL_UART_Main_receiveData(UART_1_INST);

			// ��黺�����Ƿ�����
			if (recv1_length < RE_1_BUFF_LEN_MAX - 1)
			{
				recv0_buff[recv0_length++] = receivedData;
				Cmd_GetPkt(receivedData);
				// ������������ٷ��ͳ�ȥ������ش�����ע�͵�
//				uart1_send_string("1R:");
//				uart1_send_char(receivedData);
//				uart1_send_string("\r\n");
				//printf("0R<-1R:%c\r\n",receivedData);
			}
			else
			{
				recv1_length = 0;
			}

			// ��ǽ��ձ�־
			recv1_flag = 1;
		
			break;
		
		default://�����Ĵ����ж�
			break;
	}
}

void HardFault_Handler(void)
{

}