
#include "board.h"
#include "stdio.h"
#include "myiic.h"
#include "motor.h"
#include "oled.h"
#include "stdlib.h"
#include "pid.h"
#include "key.h"
#include "string.h"
extern F32 AngleZ;

unsigned int utick = 0;

extern volatile uint8_t gRxPacket1[UART_PACKET_SIZE];
extern volatile bool gCheckUART1;
extern volatile bool Datavalid;
extern volatile int DatavalidCt;

extern uint8_t rx_buffer[6];

uint8_t task_num = 0;

int turns = 0;

// 函数声明
void OLED_SHOWDATA(void);

void scheduler(void);
void every_10ms_task(void);
void every_100ms_task(void);
void every_200ms_task(void);
void every_second_task(void);
void get_lvbo();
void updateAngle(float current_angle);
// 全局变量声明

volatile uint8_t F_Tim10ms = 0;
volatile uint8_t F_Tim100ms = 0;
volatile uint8_t F_Tim200ms = 0;
volatile uint8_t F_Tim1s = 0;

// gray api
uint8_t GetNum[1];
void BUZZER(uint8_t stat)
{
	if (stat == 1)
	{
		DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_0_PIN);
	}
	else
	{
		DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_0_PIN);
	}
}

uint8_t gray_readNum()
{

	// HAL_I2C_Mem_Read(&hi2c1, 0x4f << 0x01, 0xDD, 1, GetNum, 1, HAL_MAX_DELAY);

	IIC_ReadDataLen(0x4f, 0xDD, 1, GetNum);
	for (int i = 0; i < 8; i++)
	{
		printf("%d ", (GetNum[0] >> i) & 0x01);
	}
	printf("\r\n");
	return 0;
}
uint8_t GetANum[8];
uint8_t gray_readAdNum()
{

	// HAL_I2C_Mem_Read(&hi2c1, 0x4f << 0x01, 0xB0, 1, GetANum, 8, HAL_MAX_DELAY);
	IIC_ReadDataLen(0x4f, 0xB0, 8, GetANum);
	// for (int i = 0; i < 8; i++)
	// {
	// 	printf("%d	", GetANum[i]);
	// }
	// printf("\r\n");
	return 0;
}
uint8_t gray_ping()
{
	uint8_t GetNum[1];
	static uint8_t ct = 0;
	// HAL_I2C_Mem_Read(&hi2c1, 0x4f << 0x01, 0xDD, 1, GetNum, 1, HAL_MAX_DELAY);
	IIC_ReadDataLen(0x4f, 0xAA, 1, GetNum);
	// etNum[7]-=127;
	if (GetNum[0] == 0x66)
	{
		printf("Gray ping ok\r\n");
		return 0;
	}
	else
	{
		printf("Gray ping NOT ok 0x%x %d\r\n", GetNum[0], ct++);
		if (GetNum[0] != 0x00 && ct > 5)
			return 0;
		return 1;
	}
}

// #define DEBUG_MAIN
#ifdef DEBUG_MAIN

int main(void)
{
	SYSCFG_DL_init();
	delay(1000);
	// 清除串口中断标志
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	// 使能串口中断
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	// 清除串口中断标志
	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	// 使能串口中断
	DL_TimerA_startCounter(PWM_0_INST);
	DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, DL_TIMER_CC_0_INDEX);
	DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, DL_TIMER_CC_1_INDEX);
	Set_PWM(200, 200);
	delay_ms(1000);
	Set_PWM(400, 400);
	delay_ms(1000);
	Set_PWM(600, 600);
	delay_ms(1000);
	Set_PWM(0, 0);
	/*
	Set_PWM(0, 100);
	delay_ms(1000);
	Set_PWM(0, -100);
	delay_ms(1000);
	Set_PWM(0, 100);
	delay_ms(1000);
	Set_PWM(0, -100);
	delay_ms(1000);
	Set_PWM(0, 100);
	delay_ms(1000);
	Set_PWM(0, -100);
	delay_ms(1000);

	Set_PWM(100, 0);
	delay_ms(1000);
	Set_PWM(-100, 0);
	delay_ms(1000);
	Set_PWM(100, 0);
	delay_ms(1000);
	Set_PWM(-100, 0);
	delay_ms(1000);
	Set_PWM(100, 0);
	delay_ms(1000);
	Set_PWM(-100, 0);
	delay_ms(1000);

	Set_PWM(100, 100);
	delay_ms(1000);
	Set_PWM(-100, -100);
	delay_ms(1000);
	Set_PWM(100, 100);
	delay_ms(1000);
	Set_PWM(-100, -100);
	delay_ms(1000);
	Set_PWM(100, 100);
	delay_ms(1000);
	Set_PWM(-100, -100);
	delay_ms(1000);
	Set_PWM(0, 0);
	*/
	while (1)
	{
	}
}

#else

void get_AngleZ(float *get) // 获取imu角度
{

	int data = parse_packet();

	if (data != -999)
	{
		*get = (float)(data - 180);
	}
}

void enter_curve() // 入口执行
{
	Set_PWM(0, 0);
	printf("Car entering the curve\n");
	BUZZER(1);
	delay_ms(150); // change more time
	BUZZER(0);
	delay_ms(50);
	// BUZZER(1);
	// delay_ms(150);
	// BUZZER(0);
	// delay_ms(50);

	// Add sound/light indication code here
}

void exit_curve() // 出口执行
{
	Set_PWM(0, 0);
	printf("Car exiting the curve\n");
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(50);
	// BUZZER(1);
	// delay_ms(100);
	// BUZZER(0);
	// delay_ms(50);
	// BUZZER(1);
	// delay_ms(100);
	// BUZZER(0);
	// delay_ms(50);

	// Add sound/light indication code here
}

#define THRESHOLD_HIGH 210 // 高AD值阈值，具体数值需根据实际情况调整
#define THRESHOLD_DIFF 50  // 灰度值差异阈值，具体数值需根据实际情况调整

int detect_curve_entry() // 入口检测逻辑 entry conor,high differ
{
	int is_line_detected = 0;
	int max_value = GetANum[0];
	int min_value = GetANum[0];

	// 遍历灰度传感器的值，找出最大值和最小值
	for (int i = 1; i < 8; i++)
	{
		if (GetANum[i] > max_value)
		{
			max_value = GetANum[i];
		}
		if (GetANum[i] < min_value)
		{
			min_value = GetANum[i];
		}
	}

	// 如果最大值和最小值的差异小于阈值且最大值大于高AD值阈值，表明未检测到线
	if ((max_value - min_value < THRESHOLD_DIFF) && (max_value > THRESHOLD_HIGH))
	{
		is_line_detected = 0;
	}
	else
	{
		is_line_detected = 1;
	}
	// printf("detect_curve_entry() max_value - min_value = %d .#define THRESHOLD_HIGH 170\r\n", max_value - min_value);
	// printf("max_value = %d .#define THRESHOLD_HIGH 170\r\n", max_value);

	// 如果当前状态是未检测到线，而之前状态是检测到线，则表明进入了弧线
	static int previous_state = 0;
	int entry_detected = (!previous_state && is_line_detected);
	previous_state = is_line_detected;

	if (entry_detected)
	{
		printf("Car entering curve\n");
	}

	return entry_detected;
}

int detect_curve_exit() // 出口检测逻辑
{
	int is_line_detected = 0;
	int max_value = GetANum[0];
	int min_value = GetANum[0];

	// 遍历灰度传感器的值，找出最大值和最小值
	for (int i = 1; i < 8; i++)
	{
		if (GetANum[i] > max_value)
		{
			max_value = GetANum[i];
		}
		if (GetANum[i] < min_value)
		{
			min_value = GetANum[i];
		}
	}

	// 如果最大值和最小值的差异小于阈值且最大值大于高AD值阈值，表明未检测到线
	if ((max_value - min_value < THRESHOLD_DIFF) && (max_value > THRESHOLD_HIGH))
	{
		is_line_detected = 0;
	}
	else
	{
		is_line_detected = 1;
	}

	// 如果当前状态是检测到线，而之前状态是未检测到线，则表明离开了弧线
	static int previous_state = 1;
	int exit_detected = (previous_state && !is_line_detected);
	previous_state = is_line_detected;

	if (exit_detected)
	{
		printf("Car exiting curve\n");
	}

	return exit_detected;
}
/*
float kp_e = 20.0;
float ki_e = 0;
float kd_e = 0.0;
set_p_i_d(&pid_line, kp_e, ki_e, kd_e); // Adjust Kp, Ki, Kd values for curve
char str1[16] = {0};
sprintf(str, "%02.2f %02.2f %02.2f", kp_e, ki_e, kd_e);
OLED_ShowString(0, 48, (u8 *)str, 8, 1);
OLED_Refresh();
*/
void task1(void) // 任务1 开环走1m
{
	// 1.直走1m
	printf("zhizou\r\n");
	OLED_ShowString(0, 16, (uint8_t *)"1.zhizou", 8, 1); // Cast the string literal to uint8_t *
	OLED_Refresh();
	Set_PWM(500, 512);
	delay_ms(1900);
	Set_PWM(0, 0);
	BUZZER(1);
	delay_ms(250); // change more time
	BUZZER(0);
	delay_ms(50);
	BUZZER(1);
	delay_ms(250);
	BUZZER(0);
	delay_ms(50);
}
int yaw;
int yaw_offset;
/*
直线走x米，参数：(启用圈数计算，基础pwm，角度，走的时间，调用标识(用于初始化配置))
注意:!!所有mcu内置的东西，比如pwm，按键不能和串口中断冲突，否则死机。如果需要，先关串口中断再调用mcu内设再开中断
*/
void zhizouxm(int ena_turns, int basepwm, int ang, unsigned int run_time, int tag)
{
	// 30 = 1m

	_pid pid_yaw; // yaw pid控制器

	uint8_t sign_start;		 // 信号_开始，为1时运行
	uint8_t sign_imurst;	 // 信号_软复位imu，硬复位使用gnd短接从机PA17
	int wh_ct;				 // while 循环次数
	int wh_ct_start;		 // 小车开始行走时的 while次数，用于计算走的距离
	unsigned int tick_start; //

	sign_imurst = 0;
	wh_ct = 0;
	wh_ct_start = 0;

	// Variables to store PWM values
	int pwm_a = 0;
	int pwm_b = 0;

	float control_output;
	int BASE_PWM = basepwm;

	// Configure DMA for UART1 启用DMA 串口中断
	DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_1_INST->RXDATA));
	DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)&gRxPacket1[0]);
	DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, UART_PACKET_SIZE);
	DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
	// Enable UART1 interrupt
	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

	// Initialize the PID controller parameters 初始化pid控制器
	PID_param_init();
	set_p_i_d(&pid_yaw, 4.0, 0.0, 20.0); // Set your Kp, Ki, Kd values
	if (tag == 1)						 // 第2题，默认不行走，不复位imu,设置目标角度
	{
		sign_start = 0;
		set_pid_target(&pid_yaw, ang); // Desired yaw angle, set to 0 for straight line
	}
	if (tag == 2) // 默认行走，不复位imu,设置目标角度
	{
		set_pid_target(&pid_yaw, ang);
		sign_start = 1;
		sign_imurst = 0;
		for (int i = 0; i < 10; i++) // 用于稳定角度读取
		{
			process_frame_data((uint8_t *)gRxPacket1, rx_buffer, 6);
			delay_ms(75);
		}
	}
	if (tag == 3) // 和1一样，第3题
	{
		sign_start = 0;
		sign_imurst = 0;
		set_pid_target(&pid_yaw, ang); // Desired yaw angle, set to 0 for straight line
	}
	// 如果当前旋转圈数不为0（已经转过一圈），则跳过第一阶段的旋转
	if (tag == 4)
	{
		// 设置目标角度
		set_pid_target(&pid_yaw, ang);

		// 等待 PID 系统稳定
		sign_start = 1;
		sign_imurst = 0;
		for (int i = 0; i < 10; i++)
		{
			process_frame_data((uint8_t *)gRxPacket1, rx_buffer, 6);
			delay_ms(75);
		}
		tick_start = utick;

		// 如果当前已经旋转到目标角度附近，可以设置一个阈值来判断是否需要进一步旋转
		if (abs(yaw - ang) < 5.0) // 例如设定一个角度误差范围
		{
			// 目标角度已经达到，可以跳过进一步旋转的处理
			sign_start = 0; // 停止旋转
		}
	}
	if (tag == 5) // 和1一样，第3题
	{
		sign_start = 1;
		sign_imurst = 0;
		set_pid_target(&pid_yaw, ang); // Desired yaw angle, set to 0 for straight line
	}
	// pid_yaw.target_val = 0;

	while (1)
	{
		wh_ct++;
		if (utick > tick_start + run_time && sign_start == 1) // 用于计算跑多久，runtime
		{
			Set_PWM(0, 0);
			break;
		}

		process_frame_data((uint8_t *)gRxPacket1, rx_buffer, 6); // 读取imu

		AngleZ = (int)((rx_buffer[2] << 8) | rx_buffer[3]); // 读取imu 中间
		yaw = (int)AngleZ - yaw_offset;						// 读取imu 最终

		if (sign_imurst == 1) // imu软复位 暂时用不到
		{
			yaw_offset = yaw;

			sign_imurst = 0;
		}

		if (sign_start == 1) // 开始行走
		{
			if (ena_turns == 1)
				updateAngle(yaw); // 更新圈数

			// Update the PID controller with the current yaw angle
			control_output = turn_pid_realize(&pid_yaw, (float)yaw);

			// Adjust PWM values based on the control output
			pwm_a = basepwm - (int)control_output;
			pwm_b = basepwm + (int)control_output;
		}
		// Set the PWM values to control the motors
		NVIC_DisableIRQ(UART_0_INST_INT_IRQN); // 关中断
		if (sign_start == 1)
		{
			Set_PWM(pwm_a, pwm_b);
		}
		else
		{
			if (wh_ct % 3 == 0) // 3个按键控制
			{
				if (KEY_N1_READ() == 0)
				{
					delay_ms(10);
					if (KEY_N1_READ() == 0)
					{
						sign_imurst = 1;
					}
				}
			}
			else if (wh_ct % 3 == 1)
			{
				if (KEY_N2_READ() == 0)
				{
					delay_ms(10);
					if (KEY_N2_READ() == 0)
					{
						OLED_SHOWDATA();
					}
				}
			}
			else if (wh_ct % 3 == 2)
			{
				if (KEY_N3_READ() == 0)
				{
					delay_ms(10);
					if (KEY_N3_READ() == 0)
					{
						sign_start = 1;
						wh_ct_start = wh_ct;
						delay_ms(500);
						tick_start = utick;
					}
				}
			}
		}
		NVIC_EnableIRQ(UART_0_INST_INT_IRQN); // 启用中断
		//		// Set_PWM(pwm_a, pwm_b);
		//		// OLED_SHOWDATA();
		//		printf("tick:%d ,Yaw: %d, Control Output: %.2f, PWM_A: %d, PWM_B: %d\n", utick, yaw, control_output, pwm_a, pwm_b);

		delay_ms(75);
		printf("start_tick:%d tick:%d now_yaw:%d tar_yaw%d turns:%d\n", tick_start, utick, yaw, (int)get_pid_target(&pid_yaw), turns);
	}
	printf("real runtime:%d tar_yaw=%d end_yaw:%d \r\n", utick - tick_start, ang, yaw);
}
int count = 0;
_pid pid_line;
/*
循迹函数，参数(启用圈数统计，出去时左边pwm，出去时右边pwm，出去时偏转时间)
 */
void xunji(int ena_turns, int ena_entr_outr_chek, int basepwm, int exited_curve_left_pwm, int exited_curve_right_pwm, int exited_curve_left_pwm_time)
{
	// 2.走半径40cm弧线半圈
	// Initialize the PID controller parameters
	int pwm_a, pwm_b;
	int BASE_PWM = basepwm; // 基础pwm
	int pwm_offset_a = 0;
	int pwm_offset_b = 0;
	float control_output;
	pid_line.err = 0.0;
	pid_line.err_last = 0.0;
	pid_line.integral = 0.0;

	set_p_i_d(&pid_line, 20.0, 0.0, 0.0); // Initial Kp, Ki, Kd values
	set_pid_target(&pid_line, 4.0);		  // Desired line angle, set to 4 (center of 8 sensors)

	// GetANum[8]
	OLED_ShowString(0, 16, (uint8_t *)"2.xunji", 8, 1);
	OLED_Refresh();

	int entered_curve = 0;
	int exited_curve = 0;
	uint32_t enter_time = 0;			 // Variable to store the time when the car enters the curve
	const uint32_t debounce_time = 3000; // Time in milliseconds to avoid false exit detection

	while (1)
	{
		gray_readAdNum(); // 灰度读取

		process_frame_data((uint8_t *)gRxPacket1, rx_buffer, 6); // imu读取
		AngleZ = (int)((rx_buffer[2] << 8) | rx_buffer[3]);		 // imu读取2
		// printf("%d\r\n",yaw);
		yaw = (int)AngleZ - yaw_offset;		   // imu读取最终
		if (ena_turns == 1)					   //
			updateAngle(yaw);				   // 更新转向圈数
		NVIC_DisableIRQ(UART_0_INST_INT_IRQN); // 关闭中断
		// if (KEY_N1_READ() == 0)				   // 读取按键，调试用
		// {
		// 	delay_ms(10);
		// 	if (KEY_N1_READ() == 0)
		// 	{
		// 		for (int i = 0; i < 8; i++)
		// 		{
		// 			printf("%d\t", GetANum[i]);
		// 		}
		// 		printf("\r\n");
		// 	}
		// }

		NVIC_EnableIRQ(UART_0_INST_INT_IRQN); // 启用中断
		// 获取当前线的位置（最小值的位置）
		int min_index = 0; // 最小索引
		for (int i = 1; i < 8; i++)
		{
			if (GetANum[i] < GetANum[min_index])
			{
				min_index = i;
			}
		}
		pid_line.actual_val = min_index; // 当前线的位置

		// Update the PID controller with the current line angle
		control_output = location_pid_realize(&pid_line, (float)min_index);
		if (min_index >= 4 && min_index <= 4) // 中心，正常计算pid
		{
			pwm_offset_a = 0;
			pwm_offset_b = 0;
		} /////
		else if (min_index == 1 && ((GetANum[0] < 100) || (GetANum[1] < 150))) // 在最左边，强制向右偏转
		{
			pwm_offset_b = -control_output;
			pwm_offset_a = +700;
		}
		else if (min_index == 2 && GetANum[2] < 150) // 弱强制右偏转
		{
			pwm_offset_b = -control_output - 100;
			pwm_offset_a = +100;
		}
		else if (min_index == 3 && GetANum[4] < 150) // 弱右偏转
		{
			pwm_offset_b = -control_output + control_output - 100;
			pwm_offset_a = +100;
		}
		/////444444444 在中心
		else if (min_index == 5 && GetANum[5] < 150) // 弱左偏转
		{
			pwm_offset_b = -100;
			pwm_offset_a = -control_output + control_output + 100;
		}
		else if (min_index == 6 && GetANum[6] < 150) // 弱强制左偏转
		{
			pwm_offset_b = -100;
			pwm_offset_a = +control_output - 100;
		}
		else if (min_index == 7 && GetANum[7] < 150) // 弱强制左偏转
		{
			pwm_offset_b = -700;
			pwm_offset_a = +control_output;
		}

		// Adjust PWM values based on the control output 。pid计算
		pwm_a = BASE_PWM - (int)control_output - pwm_offset_a;
		pwm_b = BASE_PWM + (int)control_output + pwm_offset_b;
		if (pwm_a > 700)
			pwm_a = 700;
		if (pwm_b > 700)
			pwm_b = 700;
		if (pwm_a < -700)
			pwm_a = -700;
		if (pwm_b < -700)
			pwm_b = -700;
		NVIC_DisableIRQ(UART_0_INST_INT_IRQN); // 关中断
		// Set the PWM values to control the motors
		Set_PWM(pwm_a, pwm_b);				  // pwm最终赋值
		NVIC_EnableIRQ(UART_0_INST_INT_IRQN); // 开中断
		// 打印位置和PWM值以调试
		// printf("pos:%d pwm:%d,%d\n", min_index, pwm_a, pwm_b);

		if (ena_entr_outr_chek == 1)
		{
			if (!entered_curve && detect_curve_entry()) // 出口判定
			{
				printf("Car entering curve\n");
				set_p_i_d(&pid_line, 60.0, 0.0, 0.0); // Adjust Kp, Ki, Kd values for curve

				pid_line.err = 0.0;
				pid_line.err_last = 0.0;
				pid_line.integral = 0.0;

				entered_curve = 1;
				enter_time = utick; // Record the time when entering the curve
				enter_curve();
				Set_PWM(0, 0);
			}

			if (entered_curve && !exited_curve && detect_curve_exit()) // 入口判定
			{
				if (utick - enter_time > debounce_time)
				{
					printf("Car exiting curve\n");
					set_p_i_d(&pid_line, 20.0, 0.0, 0.0); // Adjust Kp, Ki, Kd values for straight line
					exited_curve = 1;
					exit_curve();
					// 检测到出口右拐的速度和时间，也就是角度
					Set_PWM(exited_curve_left_pwm, exited_curve_right_pwm);
					delay_ms(exited_curve_left_pwm_time);
				}
			}
			if (entered_curve == 1)
			{
				for (int i = 1; i < 8; i++)
				{
					if (GetANum[i] <180)
					{
						break;

					}
				}
			}

			// Add a break condition to exit the loop after completing the half-circle
			if (entered_curve && exited_curve) // 完成入出，退出 完成半圈
			{
				// while(1);
				exit_curve();
				break;
			}
		}
		printf("tick:%d now_yaw:%d tar_yaw%d\n", utick, yaw, (int)get_pid_target(&pid_line));
	}
	// 3.停止
	Set_PWM(0, 0);
	delay_ms(100);
}

void xiesi(void) // 任务2
{
	count++;

	printf("%dms count %d start\r\n", utick, count);
	OLED_ShowNum(96, 0, 2, count, 8, 1);
	// 1.直走1m 写死直走
	printf("zhizou\r\n");
	OLED_ShowString(0, 16, (uint8_t *)"1.zhizou", 8, 1); // Cast the string literal to uint8_t *
	OLED_Refresh();
	// 1m直线速度
	Set_PWM(500, 514);
	if (count == 1)
		delay_ms(1570); // 第1圈直线时间
	else if (count == 2)
		delay_ms(1570); // 第2圈直线时间
	else
		delay_ms(1500); // 第3圈直线时间
	Set_PWM(0, 0);
	// 1.闭环直走1m 有问题但可以调
	// if (count == 1)
	// 	zhizouxm(1, 400, 180, 30, 1);
	// else if (count == 2)
	// 	zhizouxm(1, 400, 5, 30, 2);
	printf("xunji1\r\n");

	// 2.走半径40cm弧线半圈
	// Initialize the PID controller parameters
	xunji(0, 1, 300, 120, 0, 350); // 100,0,400
	// 3.停止
	Set_PWM(0, 0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	// xiesi();//开启循环
	printf("%d ms\r\n", utick);
}

void task3(void) // 任务3
{
	// 1
	zhizouxm(1, 400, 145, 2700, 3); // 1.28m A-C
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	// Set_PWM(-100, 100);
	// delay_ms(100);
	// 2
	xunji(1, 1, 400, -200, 200, 320); // C-B
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);

	// 3
	// zhizouxm(1, 400, 45, 3000, 4); // 1.28m B-D
	Set_PWM(400, 400);
	delay_ms(2800);
	Set_PWM(0, 0);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	// 4
	xunji(1, 1, 400, 200, -200, 300); // C-B
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
}
// #define UART1_PACKET_SIZE (3)
void task4(void) // 任务3
{
	static int ct = 0;
	ct++;
	// 1
	if (ct == 1)
	{

		zhizouxm(1, 400, 145, 2700, 3); // 1.28m A-C
	}
	else if (ct == 2)
	{
		zhizouxm(1, 400, 145, 2700, 5); // 1.28m A-C
	}

	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	// Set_PWM(-100, 100);
	// delay_ms(100);
	// 2
	xunji(1, 1, 400, -200, 200, 350); // C-B
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);

	// 3
	// zhizouxm(1, 400, 45, 3000, 4); // 1.28m B-D
	Set_PWM(400, 400);
	delay_ms(2800);
	Set_PWM(0, 0);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);

	int min_index;
	int basepwm = 100;
	int BASE_PWM = 100;
	int pwm_offset_b, pwm_offset_a;
	int pwm_a, pwm_b;
	int control_output;
	while (1)
	{
		gray_readAdNum(); // 灰度读取

		NVIC_DisableIRQ(UART_0_INST_INT_IRQN); // 关闭中断

		NVIC_EnableIRQ(UART_0_INST_INT_IRQN); // 启用中断
		// 获取当前线的位置（最小值的位置）
		int min_index = 0; // 最小索引
		for (int i = 1; i < 8; i++)
		{
			if (GetANum[i] < GetANum[min_index])
			{
				min_index = i;
			}
		}

		NVIC_DisableIRQ(UART_0_INST_INT_IRQN); // 关中断
		// Set the PWM values to control the motors
		if (GetANum[min_index] < 150)
		{
			break;
		}
		else
		{
			Set_PWM(250, 100);
		} // pwm最终赋值
		NVIC_EnableIRQ(UART_0_INST_INT_IRQN); // 开中断
	}

	// 4
	xunji(1, 1, 400, 200, -200, 300); // C-B
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
	delay_ms(100);
	BUZZER(1);
	delay_ms(100);
	BUZZER(0);
}
int main(void)
{
	//	board_init();
	SYSCFG_DL_init();
	// 清除串口中断标志
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	// 使能串口中断
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	// 清除串口中断标志
	//	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	//	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);

	// 使能串口中断
	DL_TimerA_startCounter(PWM_0_INST);
	DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, DL_TIMER_CC_0_INDEX);
	DL_TimerG_setCaptureCompareValue(PWM_0_INST, 0, DL_TIMER_CC_1_INDEX);
	OLED_Init();

	printf("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\nBoard Init [[ ** LCKFB ** ]]\r\n");
	//	while (1)
	//	{
	//		if (Datavalid == true)
	//		{
	//			for (int i = 0; i < 6; i++)
	//			{
	//				uart0_send_char(gRxPacket1[i]);
	//			}
	//		}
	//		delay_ms(100);
	//	}

	while (gray_ping() != 0) // 灰度自检，
	{
		uint64_t i = 4000000;
		while (i--)
		{
			__asm("NOP");
		};
		OLED_ShowString(0, 0, (u8 *)"Gray err", 8, 1);
		OLED_Refresh();
	}
	OLED_Clear();
	OLED_ShowString(0, 0, (u8 *)"K1 to start", 8, 1);

	OLED_Refresh();

	while (1) // 按key1 开始
	{

		delay_ms(100);
		if (KEY_N1_READ() == 0)
		{
			delay_ms(10);
			if (KEY_N1_READ() == 0)
			{
				break;
			}
		}
	}
	//	xiesi();
	//	xiesi();
	//	while(1);
	OLED_ShowString(0, 0, (u8 *)"K1 K2 K3   ", 8, 1);
	OLED_ShowString(0, 8, (u8 *)"-- ** ++", 8, 1);
	OLED_Refresh();
	// 按键任务选择
	uint8_t task_tmp = 0;
	while (1)
	{

		if (KEY_N1_READ() == 0)
		{
			delay_ms(10);
			if (KEY_N1_READ() == 0)
			{
				task_tmp--;
				if (task_tmp < 1)
				{
					task_tmp = 4;
				}
				delay_ms(100);
			}
		}
		else if (KEY_N2_READ() == 0)
		{
			if (KEY_N2_READ() == 0)
			{
				task_num = task_tmp;
				break;
			}
		}
		else if (KEY_N3_READ() == 0)
		{
			delay_ms(10);
			if (KEY_N3_READ() == 0)
			{
				task_tmp++;
				if (task_tmp > 4)
				{
					task_tmp = 1;
				}
				delay_ms(100);
			}
		}
		OLED_ShowNum(0, 24, task_tmp, 2, 8, 1);
		OLED_Refresh();
	}
	OLED_Clear();
	OLED_Refresh();
	OLED_ShowString(0, 0, (u8 *)"RUN", 8, 1);
	OLED_Refresh();
	// 最终任务进入选择
	if (task_num == 1)
	{
		printf("task1\r\n");
		task1(); // 进入任务1
	}
	else if (task_num == 2)
	{
		printf("task2\r\n");
		xiesi(); // 进入任务2，2次半圈
		xiesi();
	}
	else if (task_num == 3)
	{
		task3(); // 进入任务3
		printf("task3\r\n");
	}
	else if (task_num == 4)
	{

		task4();
	}
	while ((1))
	{
		OLED_ShowString(0, 16, (u8 *)" car end", 8, 1);
		OLED_Refresh();
	}

	//  while(1)
	//  {
	//  	//gray_readAdNum();
	//  	gray_readNum();
	//  delay_ms(100);
	//  }
	// int i = 4000000; while (i--){__ASM("NOP");};// 延时一下让传感器上电准备完毕，传感器上电后需要初始化完毕后才会接收指令的

	// 唤醒传感器，并配置好传感器工作参数，然后开启主动上报---------------
	//	Cmd_03(); // 1 唤醒传感器
	//	/**
	//	 * 设置设备参数
	//	 * @param accStill    惯导-静止状态加速度阀值 单位dm/s?
	//	 * @param stillToZero 惯导-静止归零速度(单位cm/s) 0:不归零 255:立即归零
	//	 * @param moveToZero  惯导-动态归零速度(单位cm/s) 0:不归零
	//	 * @param isCompassOn 1=需开启磁场 0=需关闭磁场
	//	 * @param barometerFilter 气压计的滤波等级[取值0-3],数值越大越平稳但实时性越差
	//	 * @param reportHz 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
	//	 * @param gyroFilter    陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
	//	 * @param accFilter     加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
	//	 * @param compassFilter 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
	//	 * @param Cmd_ReportTag 功能订阅标识
	//	 */
	//	int imu_freq = 10;
	//	Cmd_12(5, 255, 0, 0, 3, imu_freq, 2, 4, 9, 0x40); // 2 设置设备参数(内容1)
	//	// Cmd_19();// 3 开启数据主动上报
	//	Cmd_18(); // 关闭数据主动上报

	//	Cmd_05(); // z轴角归零
	//	Cmd_06(); // xyz世界坐标系清零

	//	get_AngleZ();
	//	printf("AngleZ=%.2f\r\n", AngleZ);
	//	delay_ms(1000);

	//	while(1)
	//	{

	//		//keynum=click();
	//		if(KEY_N1_READ()==0)
	//		{
	//			Cmd_05();
	//			Cmd_06();
	//			delay_ms(500);
	//		}else if(KEY_N2_READ()==0)
	//		{
	//			mode=1;
	//			delay_ms(100);
	//			break;
	//		}
	//		char str[16];
	//		sprintf(str,"key 1:%d  ",get_AngleZ());
	//		OLED_ShowString(0,0,(u8 *)str,8,1);
	//		OLED_Refresh();
	//		delay_ms(200);
	//
	//	}
	//	OLED_Clear();
	//	OLED_Refresh();
	//	// Variables to store PWM values
	//	int pwm_a = 0;
	//	int pwm_b = 0;
	//	int yaw;
	//	float control_output;
	//	int BASE_PWM = 0;

	//	pid_yaw.target_val = 0;
	// while(1)
	// {
	// 	int data;
	// 	get_AngleZ(&data);
	// 	printf("%d\r\n",data);
	// 	delay_ms(200);
	// }
	//	xiesi();
	//	xiesi();
	while (1)
	{

		// im948_test();
	}

	while (0)
	{
		// LED引脚输出高电平
		DL_GPIO_setPins(LED1_PORT, LED1_PIN_14_PIN);
		// printf("LED [ON]\r\n");
		delay_ms(500);

		// LED引脚输出低电平
		DL_GPIO_clearPins(LED1_PORT, LED1_PIN_14_PIN);
		// printf("LED [OFF]\r\n");
		delay_ms(500);
	}
}

#endif

void SysTick_Handler(void)
{
	SysTick->CTRL &= ~(1 << 16); // Clear the SysTick interrupt flag

	utick++;

	if (utick % 10 == 0)
	{
		F_Tim10ms = 1;
	}
	if (utick % 100 == 0)
	{
		F_Tim100ms = 1;
	}
	if (utick % 200 == 0)
	{
		F_Tim200ms = 1;
	}
	if (utick % 1000 == 0)
	{
		F_Tim1s = 1;
	}
}
void OLED_SHOWDATA(void) // oled显示
{

	// OLED_ShowNum(0, 0, mode, 2, 8, 1);
	char str[8];
	sprintf(str, "%d  ", (int)yaw);
	OLED_ShowString(0, 16, (u8 *)str, 8, 1);
	OLED_ShowNum(0, 24, turns, 2, 8, 1);
	OLED_Refresh();
}
void scheduler(void)
{
	if (F_Tim10ms)
	{
		F_Tim10ms = 0;
		every_10ms_task();
	}
	if (F_Tim100ms)
	{
		F_Tim100ms = 0;
		every_100ms_task();
	}
	if (F_Tim200ms)
	{
		F_Tim200ms = 0;
		every_200ms_task();
	}
	if (F_Tim1s)
	{
		F_Tim1s = 0;
		every_second_task();
	}
}
// 10ms 100hz task
void every_10ms_task(void)
{

	F_Tim10ms = 0;
}
// 100ms 10hz task
void every_100ms_task(void)
{

	F_Tim100ms = 0;
}
// 200ms 5hz task
void every_200ms_task(void)
{

	//	printf("%d\r\n",(int)AngleZ);
	F_Tim200ms = 0;
}
// 每秒执行的任务
void every_second_task(void)
{

	F_Tim1s = 0;
}
void get_lvbo() // 无用
{
	(void)yaw;
	updateAngle(yaw);
	yaw = yaw + turns * 360.0;
	//				yaw=yaw+wenpiao_adc;
}

void updateAngle(float current_angle) // 更新圈数
{
	static float previous_angle = 180;
	static int turns = 0;

	float angle_difference = current_angle - previous_angle;

	// 考虑到跨越0度和360度的情况
	if (angle_difference > 180.0)
	{
		angle_difference -= 360.0;
	}
	else if (angle_difference < -180.0)
	{
		angle_difference += 360.0;
	}

	if (angle_difference < -180.0)
	{
		turns++;
	}
	else if (angle_difference > 180.0)
	{
		turns--;
	}

	previous_angle = current_angle;
	yaw = current_angle + turns * 360;

	// 将yaw角度限制在0到360度之间
	while (yaw < 0)
	{
		yaw += 360;
	}
	while (yaw >= 360)
	{
		yaw -= 360;
	}
}

// 	if((GetNum[0] >> 0) & 0x01)//1111 1110
// 	{
// 		Set_PWM(200,0);
// 	}else if((GetNum[0] >> 1) & 0x01)// 1111 1101
// 	{
// 		Set_PWM(200,0);
// 	}else if(((GetNum[0] >> 2) & 0x01) ==0)// 1111 1011
// 	{
// 		// Set_PWM(0,0);//
// 		Set_PWM(200,0);
// 	}else if(((GetNum[0] >> 3) & 0x01)==0)// 1111 0111
// 	{
// 		// Set_PWM(0,0);//
// 		Set_PWM(200,0);
// 	}else if(((GetNum[0] >> 4) & 0x01)==0)// 1110 1111
// 	{
// 		// Set_PWM(0,0);//
// 		Set_PWM(200,0);
// 	}else if(((GetNum[0] >> 5) & 0x01)==0)// 1101 1111
// 	{
// 		// Set_PWM(0,0);//
// 		Set_PWM(0,200);
// 	}else if(((GetNum[0] >> 6) & 0x01)==0)// 1011 1111
// 	{
// 		// Set_PWM(0,0);//
// 		Set_PWM(0,200);
// 	}
// 	// else if(((GetNum[0] >> 7) & 0x01)==0)// 1011 1111
// 	// {
// 	// 	// Set_PWM(0,0);//
// 	// 	Set_PWM(0,200);
// 	// }else
// 	// if((GetNum[0] >> 8) & 0x01)// 0111 1111
// 	// {
// 	// 	Set_PWM(200,0);
// 	// }
//     // Set_PWM(500 + gray_status[0] * 100, -(500 - gray_status[0] * 100));
// 	delay_ms(100);

// while (1)
// {
// 	feed_dog();
// 	if (read_pin == 0)
// 	{
// 		delay_ms(10);
// 		{
// 			if (read_pin == 0)
// 			{
// 				while (1)
// 				{
// 					if (read_pin == 0)
// 					{
// 						break;
// 					}
// 				}
// 			}
// 		}
// 	}
// }