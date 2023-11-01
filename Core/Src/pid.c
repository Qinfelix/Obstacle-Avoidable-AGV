
#include "main.h"
#include "pwm.h"
#include "pid.h"





void pid_control(PID_CONTROLLER* pid)
{
	pid->error_integral+=pid->error;
	pid->control = pid->kp*pid->error + pid->ki*pid->error_integral + pid->kd*(pid->error-pid->error_last);//get the control number

	pid->pwm_para = pid->pwm_para + pid->control;

	pid->error_last=pid->error;

	if(pid->pwm_para<0)
		{
			pid->pwm_para=0;
		}

	if(pid->pwm_para > 8500)
	{
		pid->pwm_para=8500;
	}

	if(pid->error_integral>500000)
	{
		pid->error_integral=0;
	}
}


void pid_controller_init(PID_CONTROLLER* pid, uint8_t sys_control_frequency,float kp,float ki,float kd)
{
	pid->control=0;
	pid->error=0;
	pid->error_integral=0;
	pid->error_last=0;
	pid->control_frequency=sys_control_frequency;
	pid->pwm_para=0;

	pid->kd=kd;
	pid->ki=ki;
	pid->kp=kp;

}












