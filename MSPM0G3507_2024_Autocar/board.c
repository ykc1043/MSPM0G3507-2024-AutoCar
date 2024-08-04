/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
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
	// SYSCFG初始化
	SYSCFG_DL_init();
	// 清除串口中断标志
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	// 使能串口中断
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

	printf("Board Init [[ ** LCKFB ** ]]\r\n");
}

// 搭配滴答定时器实现的精确us延时
void delay_us(unsigned long __us)
{
	uint32_t ticks;
	uint32_t told, tnow, tcnt = 38;

	// 计算需要的时钟数 = 延迟微秒数 * 每微秒的时钟数
	ticks = __us * (32000000 / 1000000);

	// 获取当前的SysTick值
	told = SysTick->VAL;

	while (1)
	{
		// 重复刷新获取当前的SysTick值
		tnow = SysTick->VAL;

		if (tnow != told)
		{
			if (tnow < told)
				tcnt += told - tnow;
			else
				tcnt += SysTick->LOAD - tnow + told;

			told = tnow;

			// 如果达到了需要的时钟数，就退出循环
			if (tcnt >= ticks)
				break;
		}
	}
}
// 搭配滴答定时器实现的精确ms延时
void delay_ms(unsigned long ms)
{
	delay_us(ms * 1000);
}

void delay_1us(unsigned long __us) { delay_us(__us); }
void delay_1ms(unsigned long ms) { delay_ms(ms); }

// 串口发送单个字符
void uart0_send_char(char ch)
{
	// 当串口0忙的时候等待，不忙的时候再发送传进来的字符
	while (DL_UART_isBusy(UART_0_INST) == true)
		;
	// 发送单个字符
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
// 串口发送字符串
void uart0_send_string(char *str)
{
	// 当前字符串地址不在结尾 并且 字符串首地址不为空
	while (*str != 0 && str != 0)
	{
		// 发送字符串首地址中的字符，并且在发送完成之后首地址自增
		uart0_send_char(*str++);
	}
}
// 串口发送单个字符
void uart1_send_char(char ch)
{
	// 当串口0忙的时候等待，不忙的时候再发送传进来的字符
	while (DL_UART_isBusy(UART_1_INST) == true)
		;
	// 发送单个字符
	DL_UART_Main_transmitData(UART_1_INST, ch);
}
// 串口发送字符串
void uart1_send_string(char *str)
{
	// 当前字符串地址不在结尾 并且 字符串首地址不为空
	while (*str != 0 && str != 0)
	{
		// 发送字符串首地址中的字符，并且在发送完成之后首地址自增
		uart1_send_char(*str++);
	}
}

#if !defined(__MICROLIB)
// 不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
// 如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

// 定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
#endif

// printf函数重定义
int fputc(int ch, FILE *stream)
{
	// 当串口0忙的时候等待，不忙的时候再发送传进来的字符
	while (DL_UART_isBusy(UART_0_INST) == true)
		;

	DL_UART_Main_transmitData(UART_0_INST, ch);

	return ch;
}

// 串口的中断服务函数
void UART_0_INST_IRQHandler(void)
{
	uint8_t receivedData = 0;

	// 如果产生了串口中断
	switch (DL_UART_getPendingInterrupt(UART_0_INST))
	{
	case DL_UART_IIDX_RX: // 如果是接收中断

		// 接收发送过来的数据保存
		receivedData = DL_UART_Main_receiveData(UART_0_INST);

		// 检查缓冲区是否已满
		if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
		{
			recv0_buff[recv0_length++] = receivedData;

			// 将保存的数据再发送出去，不想回传可以注释掉
			// uart0_send_char(receivedData);
			printf("0R:%c\r\n", receivedData);
		}
		else
		{
			recv0_length = 0;
		}

		// 标记接收标志
		recv0_flag = 1;

		break;

	default: // 其他的串口中断
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

		// 检查帧头和帧尾
		if (rx_buffer[0] == 0xAC && rx_buffer[1] == 0xAC && rx_buffer[5] == 0x2B)
		{
			// 提取数据
			int16_t data = (rx_buffer[2] << 8) | rx_buffer[3];
			uint8_t checksum = rx_buffer[4];

			// 校验和验证
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

// 假设的函数原型
void process_frame_data(uint8_t data_source[], uint8_t data_target[], int data_length);

// 检查数据是否为帧头
bool is_frame_header(const uint8_t *data)
{
	return data[0] == 0xAC && data[1] == 0xAC;
}

// 检查数据是否为帧尾
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

// 	// 如果产生了串口中断
// 	switch (DL_UART_getPendingInterrupt(UART_1_INST))
// 	{
// 	case DL_UART_IIDX_RX: // 如果是接收中断

// 		// 接收发送过来的数据保存
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
// 		// 检查是否收到了完整的数据包
// 		if (rx_index >= BUFFER_SIZE)
// 		{
// 			packet_ready = true;
// 			rx_index = 0; // 重置索引以接收下一个数据包
// 		}
// 		break;

// 	default: // 其他的串口中断
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