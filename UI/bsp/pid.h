#ifndef PID_H
#define PID_H

/* 绝对式 PID 数据结构体 */
typedef struct
{
  float Kp;         // 比例环节参数
	float Ki;         // 积分环节参数
	float Kd;         // 微分环节参数
    
    // 是这样，去看 pid.c 中的绝对式 PID 算法核心函数
    // 你会发现这个“偏差值”就是各个环节中用于乘以 K 的量
	float Perror;     // 比例偏差值
	float Ierror;     // 积分偏差值
	float Derror;     // 微分偏差值
	
	float Pout;       // 比例环节输出量
	float Iout;       // 积分环节输出量
	float Dout;       // 微分环节输出量
	
	float NowError;   //当前偏差
	float LastError;  //上次偏差
	float IerrorLim;  //积分偏差上限
	float PIDout;     //PID运算后输出量
	float PIDoutMAX;  //PID运算后输出量上限
} s_pid_absolute_t;

/* 增量式 PID 数据结构体 */
typedef struct 
{
    float Kp;
	float Ki;
	float Kd;
	
	float dErrP;
	float dErrI;
	float dErrD;
	
	float errNow;
	float errOld1;
	float errOld2;

	float dCtrOut;
	float dOutMAX;
	float ctrOut;
	float OutMAX;
	
}s_pid_increase_t;

void PID_AbsoluteMode(s_pid_absolute_t *pid_data);
void pid_abs_param_init(s_pid_absolute_t *pid_data, float Kp, float Ki, float Kd, float errILim, float MaxOutCur);

#endif  /* PID_H */
