/******************************************************************************
                        STM32的串口驱动
*******************************************************************************/
#ifndef __drv_Uart_h
#define __drv_Uart_h

#define Dbp_UartNum        0  // 用户调试口
#define CMD_UartNum        1  // 模块通信口


#define FifoSize 200   // 串口3的fifo大小 字节数
typedef struct // 串口 Fifo缓冲区
{
    U8 RxBuf[FifoSize];
    volatile U8 In;
    volatile U8 Out;
    volatile U8 Cnt;
}struct_UartFifo;
extern struct_UartFifo UartFifo;

//------------------------------------------------------------------------------
// 描述: 打开串口
// 输入: n       =串口号(串口号为0~4，0~4对应CPU串口1~5);
//       speed   =波特率;
//       databits=数据位数(8);
//       stopbits=停止位数(1或2);
//       paparity=校验位('n'=无  'o'=奇  'e'=偶);
//------------------------------------------------------------------------------
extern void UART_Open(U8 n, U32 speed, U8 databits, U8 stopbits, U8 parity);
//------------------------------------------------------------------------------
// 描述: Uart同步发送数据，等待发送完毕
// 输入: n=串口号, buf[Len]=要发送的内容
// 返回: 返回发送字节数
//------------------------------------------------------------------------------
extern int UART_Write(U8 n, const U8 *buf, int Len);


#endif

