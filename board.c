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
#include "string.h"
#define RE_0_BUFF_LEN_MAX 128

volatile uint8_t recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t recv0_flag = 0;

#define RE_1_BUFF_LEN_MAX 128

volatile uint8_t recv1_buff[RE_1_BUFF_LEN_MAX] = {0};
volatile uint16_t recv1_length = 0;
volatile uint8_t recv1_flag = 0;

volatile uint8_t gRxPacket1[UART_PACKET_SIZE];
volatile bool gCheckUART1;
volatile bool Datavalid = false;
volatile int DatavalidCt = 0;
extern volatile uint8_t gRxPacket1[UART_PACKET_SIZE];
extern volatile bool gCheckUART1;
extern volatile bool Datavalid;
extern volatile int DatavalidCt;

void board_init(void)
{
	// SYSCFG��ʼ��
	SYSCFG_DL_init();
	// ��������жϱ�־
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	// ʹ�ܴ����ж�
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

	printf("Board Init [[ ** LCKFB ** ]]\r\n");
}

// ����δ�ʱ��ʵ�ֵľ�ȷus��ʱ
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
// ����δ�ʱ��ʵ�ֵľ�ȷms��ʱ
void delay_ms(unsigned long ms)
{
	delay_us(ms * 1000);
}

void delay_1us(unsigned long __us) { delay_us(__us); }
void delay_1ms(unsigned long ms) { delay_ms(ms); }

// ���ڷ��͵����ַ�
void uart0_send_char(char ch)
{
	// ������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
	while (DL_UART_isBusy(UART_0_INST) == true)
		;
	// ���͵����ַ�
	DL_UART_Main_transmitData(UART_0_INST, ch);
}
void uart0_send_chars(uint8_t *ch, int len)
{
	for (int i = 0; i < len; i++)
	{
		while (DL_UART_isBusy(UART_0_INST) == true)
			;
		DL_UART_Main_transmitData(UART_0_INST, ch[i]);
	}
}
// ���ڷ����ַ���
void uart0_send_string(char *str)
{
	// ��ǰ�ַ�����ַ���ڽ�β ���� �ַ����׵�ַ��Ϊ��
	while (*str != 0 && str != 0)
	{
		// �����ַ����׵�ַ�е��ַ��������ڷ������֮���׵�ַ����
		uart0_send_char(*str++);
	}
}
// ���ڷ��͵����ַ�
void uart1_send_char(char ch)
{
	// ������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
	while (DL_UART_isBusy(UART_1_INST) == true)
		;
	// ���͵����ַ�
	DL_UART_Main_transmitData(UART_1_INST, ch);
}
// ���ڷ����ַ���
void uart1_send_string(char *str)
{
	// ��ǰ�ַ�����ַ���ڽ�β ���� �ַ����׵�ַ��Ϊ��
	while (*str != 0 && str != 0)
	{
		// �����ַ����׵�ַ�е��ַ��������ڷ������֮���׵�ַ����
		uart1_send_char(*str++);
	}
}

#if !defined(__MICROLIB)
// ��ʹ��΢��Ļ�����Ҫ�������ĺ���
#if (__ARMCLIB_VERSION <= 6000000)
// �����������AC5  �Ͷ�����������ṹ��
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

// ����_sys_exit()�Ա���ʹ�ð�����ģʽ
void _sys_exit(int x)
{
	x = x;
}
#endif

// printf�����ض���
int fputc(int ch, FILE *stream)
{
	// ������0æ��ʱ��ȴ�����æ��ʱ���ٷ��ʹ��������ַ�
	while (DL_UART_isBusy(UART_0_INST) == true)
		;

	DL_UART_Main_transmitData(UART_0_INST, ch);

	return ch;
}

// ���ڵ��жϷ�����
void UART_0_INST_IRQHandler(void)
{
	uint8_t receivedData = 0;

	// ��������˴����ж�
	switch (DL_UART_getPendingInterrupt(UART_0_INST))
	{
	case DL_UART_IIDX_RX: // ����ǽ����ж�

		// ���շ��͹��������ݱ���
		receivedData = DL_UART_Main_receiveData(UART_0_INST);

		// ��黺�����Ƿ�����
		if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
		{
			recv0_buff[recv0_length++] = receivedData;

			// ������������ٷ��ͳ�ȥ������ش�����ע�͵�
			// uart0_send_char(receivedData);
			printf("0R:%c\r\n", receivedData);
		}
		else
		{
			recv0_length = 0;
		}

		// ��ǽ��ձ�־
		recv0_flag = 1;

		break;

	default: // �����Ĵ����ж�
		break;
	}
}
// #define BUFFER_SIZE 6
uint8_t rx_buffer[6];
extern uint8_t rx_buffer[6];
uint8_t rx_index = 0;
bool packet_ready = false;

int parse_packet()
{
	// printf("a\r\n");
	if (packet_ready)
	{
		packet_ready = false;

		// ���֡ͷ��֡β
		if (rx_buffer[0] == 0xAC && rx_buffer[1] == 0xAC && rx_buffer[5] == 0x2B)
		{
			// ��ȡ����
			int16_t data = (rx_buffer[2] << 8) | rx_buffer[3];
			uint8_t checksum = rx_buffer[4];

			// У�����֤
			if (checksum == (rx_buffer[2] + rx_buffer[3]))
			{
				printf("Received valid data: %d\n", data);
				return data;
			}
			else
			{
				printf("Checksum error\n");
			}
		}
		else
		{
			for (int i = 0; i < 6; i++)
			{
				printf("%02X ", rx_buffer[i]);
			}
			printf("Frame error\n");
		}
	}
	return -999;
}

#define DATA_LENGTH 6

// ����ĺ���ԭ��
void process_frame_data(uint8_t data_source[], uint8_t data_target[], int data_length);

// ��������Ƿ�Ϊ֡ͷ
bool is_frame_header(const uint8_t *data)
{
	return data[0] == 0xAC && data[1] == 0xAC;
}

// ��������Ƿ�Ϊ֡β
bool is_frame_tail(const uint8_t *data)
{
	return data[0] == 0x2B;
}

void process_frame_data(uint8_t data_source[], uint8_t data_target[], int data_length)
{
    uint8_t header[2] = {0xAC, 0xAC};
    uint8_t footer = 0x2B;
    int header_index = -1;
    int footer_index = -1;

    // Find the position of the header and footer
    for (int i = 0; i < data_length; i++)
    {
        if (data_source[i] == header[0] && data_source[(i + 1) % data_length] == header[1])
        {
            header_index = i;
        }
        if (data_source[i] == footer)
        {
            footer_index = i;
        }
    }

    // Copy the header to the target
    memcpy(data_target, &data_source[header_index], 2 * sizeof(uint8_t));

    // Copy the body to the target
    int body_start = (header_index + 2) % data_length;
    int body_end = (footer_index - 1 + data_length) % data_length;
    if (body_start <= body_end)
    {
        memcpy(&data_target[2], &data_source[body_start], 3 * sizeof(uint8_t));
    }
    else
    {
        memcpy(&data_target[2], &data_source[body_start], (data_length - body_start) * sizeof(uint8_t));
        memcpy(&data_target[2 + data_length - body_start], data_source, (3 - data_length + body_start) * sizeof(uint8_t));
    }

    // Copy the footer to the target
    data_target[5] = footer;

	Datavalid=true;
	DatavalidCt++;
}

void UART_1_INST_IRQHandler(void)
{
	switch (DL_UART_Main_getPendingInterrupt(UART_1_INST))
	{
	case DL_UART_MAIN_IIDX_DMA_DONE_RX:
		gCheckUART1 = true;

		// Disable DMA channel
		DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);

		// Optionally clear data buffer if necessary
		//            memset((void *)gRxPacket1, 0, UART_PACKET_SIZE);

		// Clear UART FIFO
		while (!DL_UART_isRXFIFOEmpty(UART_1_INST))
		{
			volatile uint8_t dummy = DL_UART_Main_receiveData(UART_1_INST); // Replace with the actual function to read data
			(void)dummy;													// Prevent unused variable warning
		}

		// Reconfigure DMA
		DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_1_INST->RXDATA));
		DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)&gRxPacket1[0]);
		DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, UART_PACKET_SIZE);

		// Re-enable DMA channel
		DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);

		break;
	default:
		break;
	}
}
// void UART_1_INST_IRQHandler(void)
// {
// 	switch (DL_UART_Main_getPendingInterrupt(UART_1_INST))
// 	{
// 	case DL_UART_MAIN_IIDX_DMA_DONE_RX:
// 		gCheckUART1 = true;

// 		// Disable DMA channel
// 		DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);

// 		// Optionally clear data buffer if necessary
// 		//            memset((void *)gRxPacket1, 0, UART_PACKET_SIZE);

// 		// Clear UART FIFO
// 		while (!DL_UART_isRXFIFOEmpty(UART_1_INST))
// 		{
// 			volatile uint8_t dummy = DL_UART_Main_receiveData(UART_1_INST); // Replace with the actual function to read data
// 			(void)dummy;													// Prevent unused variable warning
// 		}

// 		// Reconfigure DMA
// 		DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_1_INST->RXDATA));
// 		DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)&gRxPacket1[0]);
// 		DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, UART_PACKET_SIZE);

// 		if (gRxPacket1[0] != 0xAC && gRxPacket1[1] != 0xAC && gRxPacket1[5] != 0x2B)
// 		{
// 			while (!DL_UART_isRXFIFOEmpty(UART_1_INST))
// 			{
// 				volatile uint8_t dummy = DL_UART_Main_receiveData(UART_1_INST); // Replace with the actual function to read data
// 				(void)dummy;													// Prevent unused variable warning
// 			}

// 			// Reconfigure DMA
// 			DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_1_INST->RXDATA));
// 			DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)&gRxPacket1[0]);
// 			DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, UART_PACKET_SIZE);
// 			memset((void *)gRxPacket1, 0, UART_PACKET_SIZE);
// 			Datavalid = false;

// 		}
// 		else
// 		{
// 			Datavalid = true;
// 			DatavalidCt++;
// 		}
// 		// Re-enable DMA channel
// 		DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
// 		if(Datavalid==false)
// 		{
// 			for(int i=0;i<2234567;i++)
// 			{
// 				__ASM("NOP");
// 			}

// 		}
// 		break;
// 	default:
// 		break;
// 	}
// }

// void UART_1_INST_IRQHandler11(void)
// {
// 	uint8_t receivedData = 0;

// 	// ��������˴����ж�
// 	switch (DL_UART_getPendingInterrupt(UART_1_INST))
// 	{
// 	case DL_UART_IIDX_RX: // ����ǽ����ж�

// 		// ���շ��͹��������ݱ���
// 		receivedData = DL_UART_Main_receiveData(UART_1_INST);
// 		// uart0_send_char(receivedData);
// 		if (rx_index == 0 && receivedData == 0xAC)
// 		{
// 			rx_buffer[rx_index++] = receivedData;
// 		}
// 		else
// 		{
// 			packet_ready = true;
// 			rx_index = 0;
// 		}
// 		if (rx_index == 1 && receivedData == 0xAC)
// 		{
// 			rx_buffer[rx_index++] = receivedData;
// 		}
// 		else
// 		{
// 			packet_ready = true;
// 			rx_index = 0;
// 		}
// 		if (rx_index == 5 && receivedData == 0x2B)
// 		{
// 			rx_buffer[rx_index++] = receivedData;
// 		}
// 		else
// 		{
// 			packet_ready = true;
// 			rx_index = 0;
// 		}
// 		// ����Ƿ��յ������������ݰ�
// 		if (rx_index >= BUFFER_SIZE)
// 		{
// 			packet_ready = true;
// 			rx_index = 0; // ���������Խ�����һ�����ݰ�
// 		}
// 		break;

// 	default: // �����Ĵ����ж�
// 		break;
// 	}
// }

// void HardFault_Handler(void)
//{
//	printf(" err");
//	printf(" err");
//	printf(" err");
//	printf(" err");
//	while(1);
// }