#ifndef _PID_H  
#define _PID_H


#include <stdio.h>
#include <stdlib.h>
#include <math.h>





typedef struct
{
    float target_val;           //Ŀ��ֵ
    float actual_val;        		//ʵ��ֵ
    float err;             			//����ƫ��ֵ
    float err_last;          		//������һ��ƫ��ֵ
    float Kp,Ki,Kd;          		//������������֡�΢��ϵ��
    float integral;          		//�������ֵ
}_pid;

extern _pid pid_speed, pid_speed2;    
extern _pid pid_location,pid_location2;
extern _pid pid_turn,pid_turn_speed;



 void PID_param_init(void);
 void set_pid_target(_pid *pid, float temp_val);
 float get_pid_target(_pid *pid);
 void set_p_i_d(_pid *pid, float p, float i, float d);

float turn_pid_realize(_pid *pid, float actual_val);  
float turn_speed_pid_realize(_pid *pid, float actual_val);

float location_pid_realize(_pid *pid, float actual_val);
float speed_pid_realize(_pid *pid, float actual_val);
float speed_pid2_realize(_pid *pid, float actual_val);


#endif


