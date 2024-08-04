#include "motor.h"
float Velcity_Kp=10.0,  Velcity_Ki=0.3,  Velcity_Kd; //����ٶ�PID����

int myAbs(int a)
{
	if(a>0)
		return a;
	if(a<0)
		return -a;
	return 0;
}

/***************************************************************************
�������ܣ������PID�ջ�����
��ڲ��������ҵ���ı�����ֵ
����ֵ  �������PWM
***************************************************************************/
int Velocity_A(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //������ر���
		static int ControlVelocityA, Last_biasA; //��̬�������������ý�������ֵ��Ȼ����
		
		Bias=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
		
		ControlVelocityA+=Velcity_Ki*(Bias-Last_biasA)+Velcity_Kp*Bias;  //����ʽPI������
                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
		Last_biasA=Bias;	
	    if(ControlVelocityA>3199) ControlVelocityA=3199;
	    else if(ControlVelocityA<-3199) ControlVelocityA=-3199;
		return ControlVelocityA; //�����ٶȿ���ֵ
}

/***************************************************************************
�������ܣ������PID�ջ�����
��ڲ��������ҵ���ı�����ֵ
����ֵ  �������PWM
***************************************************************************/
int Velocity_B(int TargetVelocity, int CurrentVelocity)
{  
    int Bias;  //������ر���
		static int ControlVelocityB, Last_biasB; //��̬�������������ý�������ֵ��Ȼ����
		
		Bias=TargetVelocity-CurrentVelocity; //���ٶ�ƫ��
		
		ControlVelocityB+=Velcity_Ki*(Bias-Last_biasB)+Velcity_Kp*Bias;  //����ʽPI������
                                                                   //Velcity_Kp*(Bias-Last_bias) ����Ϊ���Ƽ��ٶ�
	                                                                 //Velcity_Ki*Bias             �ٶȿ���ֵ��Bias���ϻ��ֵõ� ƫ��Խ����ٶ�Խ��
		Last_biasB=Bias;	
	    if(ControlVelocityB>3199) ControlVelocityB=3199;
	    else if(ControlVelocityB<-3199) ControlVelocityB=-3199;
		return ControlVelocityB; //�����ٶȿ���ֵ
}
/*
#define MotorIn_A_Set() DL_GPIO_setPins(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN_0_L_PIN);
#define MotorIn_A_Reset() DL_GPIO_clearPins(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN_0_L_PIN);

#define MotorIn_B_Set() DL_GPIO_setPins(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN_1_R_PIN);
#define MotorIn_B_Reset() DL_GPIO_clearPins(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN_1_R_PIN);


DL_TimerG_setCaptureCompareValue(PWM_0_INST, myAbs(pwma), DL_TIMER_CC_1_INDEX);

DL_TimerG_setCaptureCompareValue(PWM_0_INST, myAbs(pwma), DL_TIMER_CC_0_INDEX);
*/

void Set_PWM(int pwma,int pwmb)
{
	//limit pwm
//	if (pwma < 1)
//		pwma = 1;
	if (pwma > 999)
		pwma = 999;
//	if (pwmb < 1)
//		pwmb = 1;
	if (pwmb > 999)
		pwmb = 999;

	pwmb=-pwmb;
	if(pwma>=0)
	{MotorIn_A_Set();}
	else
	{MotorIn_A_Reset();}
	if(pwmb>=0)
	{MotorIn_B_Set();}
	else
	{MotorIn_B_Reset();}

	
	if(pwma==0)
	{
		DL_TimerG_setCaptureCompareValue(PWM_0_INST, 1, DL_TIMER_CC_0_INDEX);
		
	}else if(pwmb==0)
	{
		DL_TimerG_setCaptureCompareValue(PWM_0_INST, 1, DL_TIMER_CC_1_INDEX);
	}
	DL_TimerG_setCaptureCompareValue(PWM_0_INST, myAbs(pwma), DL_TIMER_CC_0_INDEX);
	DL_TimerG_setCaptureCompareValue(PWM_0_INST, myAbs(pwmb), DL_TIMER_CC_1_INDEX);

}



