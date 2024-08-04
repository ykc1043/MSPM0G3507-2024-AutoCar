#include "ti_msp_dl_config.h"
#include "pid.h"

//����ȫ�ֱ���
_pid pid_speed, pid_speed2;    
_pid pid_location, pid_location2;
_pid pid_turn,pid_turn_speed;


/**
  * @brief  PID������ʼ��
	*	@note 	��
  * @retval ��
  */
void PID_param_init()
{
	/* λ����س�ʼ������ */
    pid_location.target_val=0.0;				
    pid_location.actual_val=0.0;
    pid_location.err=0.0;
    pid_location.err_last=0.0;
    pid_location.integral=0.0;
  
		pid_location.Kp = 0.10;
		pid_location.Ki = 0.0;
		pid_location.Kd = 0.0;
  
  	/* �ٶ���س�ʼ������ */
    pid_speed.target_val=0.0;				
    pid_speed.actual_val=0.0;
    pid_speed.err=0.0;
    pid_speed.err_last=0.0;
    pid_speed.integral=0.0;
  
		pid_speed.Kp = 22.0;
		pid_speed.Ki = 5.0;
		pid_speed.Kd = 0.0;


		
			/* λ����س�ʼ������ */
    pid_location2.target_val=0.0;				
    pid_location2.actual_val=0.0;
    pid_location2.err=0.0;
    pid_location2.err_last=0.0;
    pid_location2.integral=0.0;
  
		pid_location2.Kp = 0.10;
		pid_location2.Ki = 0.0;
		pid_location2.Kd = 0.0;
  
  	/* �ٶ���س�ʼ������ */
    pid_speed2.target_val=0.0;				
    pid_speed2.actual_val=0.0;
    pid_speed2.err=0.0;
    pid_speed2.err_last=0.0;
    pid_speed2.integral=0.0;
  
		pid_speed2.Kp = 20.0;
		pid_speed2.Ki = 5.0;
		pid_speed2.Kd = 0.0;
		
	/* �ǶȻ���س�ʼ������ */

		pid_turn.target_val=0.0;
		pid_turn.actual_val=0.0;
		pid_turn.err=0.0;
		pid_turn.err_last=0.0;
		pid_turn.integral=0.0;
		
		pid_turn.Kp=2.0;
		pid_turn.Ki=0.0;
		pid_turn.Kd=0.0;

	/* ת���ٶ���س�ʼ������ */

		pid_turn_speed.target_val=160.0;
		pid_turn_speed.actual_val=0.0;
		pid_turn_speed.err=0.0;
		pid_turn_speed.err_last=0.0;
		pid_turn_speed.integral=0.0;
		
		pid_turn_speed.Kp=0.3;
		pid_turn_speed.Ki=0.005;
		pid_turn_speed.Kd=0.0;
//#if defined(PID_ASSISTANT_EN)
//    float pid_temp[3] = {pid.Kp, pid.Ki, pid.Kd};   //����ô����Ӧ��pid�ṹ�����
//    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // ��ͨ�� 1 ���� P I D ֵ
//#endif
}

/**
  * @brief  ����Ŀ��ֵ
  * @param  val		Ŀ��ֵ
	*	@note 	��
  * @retval ��
  */
void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_val = temp_val;    // ���õ�ǰ��Ŀ��ֵ
}

/**
  * @brief  ��ȡĿ��ֵ
  * @param  ��
	*	@note 	��
  * @retval Ŀ��ֵ
  */
float get_pid_target(_pid *pid)
{
  return pid->target_val;    // ���õ�ǰ��Ŀ��ֵ
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // ���ñ���ϵ�� P
		pid->Ki = i;    // ���û���ϵ�� I
		pid->Kd = d;    // ����΢��ϵ�� D
}

/**
  * @brief  λ��PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */
float location_pid_realize(_pid *pid, float actual_val)  //λ�û����Kp����Ҳ����
{
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err=pid->target_val-actual_val;
  
//    /* �趨�ջ����� */   //�⻷�������Բ�Ҫ 
    if((pid->err >= -0.1) && (pid->err <= 0.1)) 
    {
      pid->err = 0;
      pid->integral = 0;
    }
    
    pid->integral += pid->err;    // ����ۻ�

		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                  +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}

/**
  * @brief  �ٶ�PID�㷨ʵ��
  * @param  actual_val:ʵ��ֵ
	*	@note 	��
  * @retval ͨ��PID���������
  */
float speed_pid_realize(_pid *pid, float actual_val)
{
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err=pid->target_val-actual_val;

    if((pid->err<0.5f ) && (pid->err>-0.5f))   //��������������ٶ�ƫ������1���ӣ���������ƫ��Ϊ��Ȧ
		{
      pid->err = 0.0f;
		}
	
		
    pid->integral += pid->err;    // ����ۻ�
	
	
//	  /*�����޷�*/
//	   	 if (pid->integral >= 1000) {pid->integral =1000;}
//      else if (pid->integral < -1000)  {pid->integral = -1000;}

		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}

float turn_pid_realize(_pid *pid, float actual_val)  //λ�û����Kp����Ҳ����
{
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err=pid->target_val-actual_val;
  
    /* �趨�ջ����� */   //�⻷�������Բ�Ҫ 
//    if((pid->err >= -0.5f) && (pid->err <= 0.5f)) 
//    {
//      pid->err = 0;
//      pid->integral = 0;
//    }
    
    pid->integral += pid->err;    // ����ۻ�

		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                  +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}



float turn_speed_pid_realize(_pid *pid, float actual_val)
{
		/*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err=pid->target_val-actual_val;

    if((pid->err<1 ) && (pid->err>-1))   //��������������ٶ�ƫ������1���ӣ���������ƫ��Ϊ��Ȧ
		{
      pid->err = 1;
		}
	
		
    pid->integral += pid->err;    // ����ۻ�
	
	
	  /*�����޷�*/
	   	 if (pid->integral >= 1000) {pid->integral =1000;}
      else if (pid->integral < -1000)  {pid->integral = -1000;}

		/*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp*pid->err
		                  +pid->Ki*pid->integral
		                   +pid->Kd*(pid->err-pid->err_last);
  
		/*����*/
    pid->err_last=pid->err;
    
		/*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}


