/**
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  */
/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "main.h"
#include "math.h"
#include "string.h"
#include "struct_typedef.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LimitMax(input, max)   \
{                          \
	if (input > max)       \
	{                      \
		input = max;       \
	}                      \
	else if (input < -max) \
	{                      \
		input = -max;      \
	}                      \
}
/* Private macro -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*********************增量式PID控制***********************/
void PID_IncrementMode(s_pid_increase_t *pid)
{
	 if(pid->kp<0) pid->kp=-pid->kp;
	 if(pid->ki<0) pid->ki=-pid->ki;
	 if(pid->kd<0) pid->kd=-pid->kd;
	
	 if(pid->errNow >5 || pid->errNow<-5)pid->errNow=0;

	 pid->dErrP=pid->errNow-pid->errOld1;
	 pid->dErrI=pid->errNow;
	 pid->dErrD=pid->errNow-2*pid->errOld1+pid->errOld2;
	
	 pid->errOld2=pid->errOld1;
	 pid->errOld1=pid->errNow;
	
	 pid->dCtrOut=pid->kp*pid->dErrP+pid->ki*pid->dErrI+pid->kd*pid->dErrD;
	
	 if(pid->dCtrOut>pid->dOutMAX) pid->dCtrOut=pid->dOutMAX;
     else if(pid->dCtrOut<-pid->dOutMAX) pid->dCtrOut=-pid->dOutMAX;

	
	 if(pid->kp==0 && pid->ki==0 && pid->kd==0) pid->ctrOut=0;
	 else pid->ctrOut+=pid->dCtrOut;
	 
	 if(pid->ctrOut>pid->OutMAX) pid->ctrOut=pid->OutMAX;
     else if(pid->ctrOut<-pid->OutMAX)
   
	 pid->ctrOut=-pid->OutMAX; 
}
/********************绝对式PID控制**************************/

/**
 * @brief 绝对式PID计算
 * @param s_pid_absolute_t *pid
 * @return float PIDout
 */
void PID_AbsoluteMode(s_pid_absolute_t *pid)
{
    //PID各环节偏差
	pid->Perror = pid->NowError;                  //P环节偏差是当前偏差
	pid->Ierror += pid->NowError;                 //I环节偏差是上电后一直持续到现在的偏差
	pid->Derror = pid->NowError - pid->LastError; //D环节偏差是当前偏差与上次偏差的差值，即偏差增量
	pid->LastError = pid->NowError;               //更新偏差	
	//限制积分历史偏差
	if( pid->Ierror >= pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror <= -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID各环节输出量
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID总输出量
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//限制PID总输出量
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief 绝对式PID计算(积分分离)
 * @param s_pid_absolute_t *pid
 * @param float integral_apart_val
 * @return float PIDout
 */
void PID_AbsoluteMode_integral_apart(s_pid_absolute_t *pid,float integral_apart_val)
{
    //PID各环节偏差
	pid->Perror = pid->NowError;                  //P环节偏差是当前偏差
    if(fabs(pid->NowError)<integral_apart_val)
	    pid->Ierror += pid->NowError;                 //I环节偏差是上电后一直持续到现在的偏差
    else pid->Ierror = 0;
	pid->Derror = pid->NowError - pid->LastError; //D环节偏差是当前偏差与上次偏差的差值，即偏差增量
	pid->LastError = pid->NowError;               //更新偏差
	//限制积分历史偏差
	if( pid->Ierror >= pid->IerrorLim) pid->Ierror =  pid->IerrorLim;
	else if( pid->Ierror <= -pid->IerrorLim)  pid->Ierror =  -pid->IerrorLim;
	//PID各环节输出量
	pid->Pout = pid->Kp * pid->Perror;
	pid->Iout = pid->Ki * pid->Ierror;
	pid->Dout = pid->Kd * pid->Derror;
	//PID总输出量
	pid->PIDout = pid->Pout + pid->Iout + pid->Dout;
	//限制PID总输出量
	if(pid->PIDout > pid->PIDoutMAX) pid->PIDout = pid->PIDoutMAX;
	else if(pid->PIDout < -pid->PIDoutMAX) pid->PIDout = -pid->PIDoutMAX;
}
/**
 * @brief   PID参数初始化，可以放在初始化函数中，也可以放在循环里
 * @param 	PID_AbsoluteType *pid
 * @param   float kp
 * @param   float ki
 * @param   float kd
 * @param   float errILim
 * @param   float MaxOutCur		
 * @return None
 */
void pid_abs_param_init(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur)
{
	memset(pid,0,sizeof(s_pid_absolute_t));
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->IerrorLim = errILim;
	pid->PIDoutMAX = MaxOutCur;
}
/**
 * @brief   PID参数赋值，可以放在初始化函数中，也可以放在循环里，我用来放到循环里调试参数
 * @param 	PID_AbsoluteType *pid
 * @param   float kp
 * @param   float ki
 * @param   float kd
 * @param   float errILim
 * @param   float MaxOutCur		
 * @return None
 */
void pid_abs_evaluation(s_pid_absolute_t *pid, float kp, float ki, float kd, float errILim, float MaxOutCur)
{
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->IerrorLim = errILim;
	pid->PIDoutMAX = MaxOutCur;
}

/**
 * @brief   单环PID
 * @param 	s_pid_absolute_t *single_pid
 * @param   float get
 * @param   float targeti		
 * @return  float pid_output
 * @attention None
 */
int16_t motor_single_loop_PID(s_pid_absolute_t *single_pid , float target , float get)
{
	static float pid_output;
	single_pid->NowError = (float)(target - get);
	PID_AbsoluteMode(single_pid);
	pid_output = single_pid->PIDout;

	return pid_output;
}
/**
 * @brief   串级PID
 * @param 	s_pid_absolute_t *pos_pid
 * @param   s_pid_absolute_t *spd_pid
 * @param   float externGet
 * @param   float externSet
 * @param   float internGet		
 * @return  pid_output(float)
 * @attention None
 */
float motor_double_loop_PID(s_pid_absolute_t *pos_pid, s_pid_absolute_t *spd_pid, float externGet, float externSet, float internGet)
{
	static float pid_output;
	static float out_st;

	pos_pid->NowError = (float)externSet - (float)externGet;
	PID_AbsoluteMode(pos_pid);
	out_st = pos_pid->PIDout;

	spd_pid->NowError = out_st - (float)internGet;
	PID_AbsoluteMode(spd_pid);
	pid_output = spd_pid->PIDout;

	return pid_output;
}
/**
 * @brief 串级PID(速度环积分分离)
 * @param s_pid_absolute_t *pos_pid
 * @param s_pid_absolute_t *spd_pid
 * @param float externGet
 * @param float externSet
 * @param float internGet
 * @param float integral_apart_val
 * @return pid_output(float)
 * @attention None
 */
float motor_double_loop_PID_integral_apart(s_pid_absolute_t *pos_pid, s_pid_absolute_t *spd_pid, float externGet, float externSet, \
                                            float internGet,float integral_apart_val)
{
	static float pid_output;
	static float out_st;

	pos_pid->NowError = (float)externSet - (float)externGet;
    PID_AbsoluteMode(pos_pid);
	out_st = pos_pid->PIDout;

	spd_pid->NowError = out_st - (float)internGet;
	PID_AbsoluteMode_integral_apart(spd_pid,integral_apart_val);
	pid_output = spd_pid->PIDout;

	return pid_output;
}


/***********************************************官方例程PID***************************************************************/

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}
