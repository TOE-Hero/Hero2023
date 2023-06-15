#include <cmsis_os.h>
#include <main.h>
#include <math.h>
#include <string.h>
#include "bsp_can.h"
#include "STMGood.h"
#include "bsp_usart.h"
#include "motor.h"
#include "bsp_RC.h"
#include "math.h"
#include "pid.h"
#include "INS_task.h"
#include "Chassis.h"
#include "Gimbal.h"
#include "Mode_Switch.h"
#include "ramp.h"
#include "Monitor.h"

/******************************** define ****************************************/
/********************* Debug ************************/
#define  C_FOLLOW_YAW_DEBUG         0	//pid_abs_evaluation(&YAW_motor_pid_pos, P5, 0, 0, 2000, S5);
										//pid_abs_evaluation(&YAW_motor_pid_speed, P6, I6, D6, 2000, 10000);
/********************* VAL ************************/
#if	ROBOT_ID == SUN
	
	#define YAW_MID_VAL             7065//国赛英雄YAW轴中值
#endif

#if	ROBOT_ID == MOON
	
	#define YAW_MID_VAL             1253//分区赛英雄YAW轴中值
#endif

#define RAMP_MAX_VAL                8000//键盘按下后速度最大值(高速)
#define RAMP_MID_VAL                5000//键盘按下后速度最大值(中速)
#define RAMP_LOW_VAL                3000//键盘按下后速度最大值（低速）
#define CHASSIS_SPEED_MAX		    10000//按下shift底盘速度
#define THETA_OFFSET			    360.0f/469309.7f//电机与中值差值除以8191，乘360°（转为度）再乘pi/180（转为弧度）
#define SuperCap_Vol_MAX            29.0f//超级电容最大电压
/**************************** global variable ************************************/
//底盘信息结构体声明，包含小陀螺与底盘分离目标前进速度，超级电容信息，裁判系统传过来的有关底盘的信息
chassisMove_t   s_chassisMove;
//超级电容期望功率CAN发送数组
uint8_t         capExpect_txBuff[8];
//底盘电机目标输出电流数组
int16_t         motor[4];

/************************** static declaration ***********************************/
//小陀螺或者底盘分离时通过角度计算的sin
static float   sin_Gimbal_ang=0.0f;
//小陀螺或者底盘分离时通过角度计算的cos
static float   cos_Gimbal_ang=0.0f;
//目标前进速度，用来被赋值遥控器值与键盘按下后的速度
static int     Target_Vx=0;
//目标后退速度，用来被赋值遥控器值与键盘按下后的速度
static int     Target_Vy=0;
//与YAW轴所定的正前方刻度相距离的刻度值转化为角度
static float   opposite_ang=0;
/************************** extern declaration ***********************************/
extern s_robo_Mode_Setting robot_Mode;
extern uint8_t             RobotId;
extern uint8_t             RobotLevel;
extern u_data_32bit        chassis_power;
extern s_FPS_monitor       finalFps;
extern s_pid_absolute_t    LF_motor_pid_stop_pos;
extern s_pid_absolute_t    RF_motor_pid_stop_pos;
extern s_pid_absolute_t    LB_motor_pid_stop_pos;
extern s_pid_absolute_t    RB_motor_pid_stop_pos;
extern s_pid_absolute_t    LF_motor_pid_stop_speed;
extern s_pid_absolute_t    RF_motor_pid_stop_speed;
extern s_pid_absolute_t    LB_motor_pid_stop_speed;
extern s_pid_absolute_t    RB_motor_pid_stop_speed;
/************************** Function declaration *********************************/

/**
  * @brief          按键控制，用斜坡函数对目标速度进行线性控制
  * @param[in]      none
  * @retval         none
  */
static void key_Control_chassic(void);//按键控制
/*********************************************************************************/

//底盘跟随YAW轴6020串级PID，适量加一些内环的D有助于英雄地盘跟随很跟手
/**
  * @brief          底盘跟随YAW轴6020串级PID计算函数
  * @param[in]      int16_t target 
  * @param[in]      s_pid_absolute_t *s_chassisMove_pos_pid
  * @param[in]      s_pid_absolute_t *s_chassisMove_spd_pid
  * @retval         float yawRot
  */
static float ChasFollow_yawrot(int16_t target, s_pid_absolute_t *s_chassisMove_pos_pid, s_pid_absolute_t *s_chassisMove_spd_pid)
{
	
	float yawRot;

	s_chassisMove_pos_pid->NowError = target - YAW_motor.back_position;
	//从小于半圈处回正
	if(s_chassisMove_pos_pid->NowError > 4095)
		s_chassisMove_pos_pid->NowError -= 8191;
	else if(s_chassisMove_pos_pid->NowError < -4096)
		s_chassisMove_pos_pid->NowError += 8191;
	s_chassisMove_pos_pid->NowError *=  0.0439506f;//360/8191（每个电机刻度对应角度）,编码器刻度转化成角度
	PID_AbsoluteMode(s_chassisMove_pos_pid);
	
	s_chassisMove_spd_pid->NowError = s_chassisMove_pos_pid->PIDout - YAW_motor.back_motor_speed;
	PID_AbsoluteMode(s_chassisMove_spd_pid);
	yawRot = s_chassisMove_spd_pid->PIDout;

	return yawRot;//输出PID
}

/**
  * @brief          功率控制
  * @param[in]      chassisMove_t *s_chassisMove
  * @param[in]      int16_t motorr[4]
  * @retval         none
  */
static void chassis_power_control(chassisMove_t *s_chassisMove, int16_t *motor_)
{
	//超级电容功率偏置电压，用于修改超级电容控制板校准问题
	int16_t					SuperCAP_Power_Offset = -10;

#if	ROBOT_ID == SUN
		SuperCAP_Power_Offset = -5;
#endif
#if	ROBOT_ID == MOON
		SuperCAP_Power_Offset = -4;
#endif

    static uint16_t 		basic_power = 55*100;//基础功率
    //限制输出最大.总电流
    if(finalFps.cap_board >= 4)//如果电容控制板发送频率正常，则执行
    {
		//超级电容门限电压标志位
		static uint8_t		SuperCAP_Low_Vol_Flag = 0;
		//超级电容欠压放电倍率
		static uint8_t		SuperCAP_Low_Vol_Multiple = 30;
		//超级电容放电限制电压,当低于此电压值时,应减小功率,防止超功率
		static uint8_t		SuperCap_Vol_Threshold = 18;
		//超级电容恢复正常功率电压
		static uint8_t		SuperCap_Vol_Wide = 22;

		//根据裁判系统反馈的功率限制，判断电容的电压阈值
        if(s_chassisMove->chassis_power_limit == 50)
        {
            SuperCAP_Low_Vol_Multiple = 30;
            SuperCap_Vol_Threshold = 21;
            SuperCap_Vol_Wide = 25;
        }else if(s_chassisMove->chassis_power_limit == 60)
        {
            SuperCAP_Low_Vol_Multiple = 35;
            SuperCap_Vol_Threshold = 21;
            SuperCap_Vol_Wide = 25;
        }else if(s_chassisMove->chassis_power_limit == 70)
        {
            SuperCAP_Low_Vol_Multiple = 40;
            SuperCap_Vol_Threshold = 19;
            SuperCap_Vol_Wide = 23;
        }else if(s_chassisMove->chassis_power_limit == 80)
        {
            SuperCAP_Low_Vol_Multiple = 40;
            SuperCap_Vol_Threshold = 19;
            SuperCap_Vol_Wide = 23;
        }else if(s_chassisMove->chassis_power_limit == 90)
        {
            SuperCAP_Low_Vol_Multiple = 40;
            SuperCap_Vol_Threshold = 19;
            SuperCap_Vol_Wide = 23;
        }else if(s_chassisMove->chassis_power_limit == 100)
        {
            SuperCAP_Low_Vol_Multiple = 40;
            SuperCap_Vol_Threshold = 19;
            SuperCap_Vol_Wide = 23;
        }else if(s_chassisMove->chassis_power_limit == 120)
        {
            SuperCAP_Low_Vol_Multiple = 40;
            SuperCap_Vol_Threshold = 19;
            SuperCap_Vol_Wide = 23;
        }else
        {
            SuperCAP_Low_Vol_Multiple = 30;
            SuperCap_Vol_Threshold = 21;
            SuperCap_Vol_Wide = 25;
        }
		//电容电压置位标志位
        // if(s_chassisMove->capVol < SuperCap_Vol_Threshold)
        // {
        //     SuperCAP_Low_Vol_Flag = 1;
        // }
		// else if (s_chassisMove->capVol > SuperCap_Vol_Wide)
		// {
		// 	SuperCAP_Low_Vol_Flag = 0;
		// }
		// //根据标志位限制最大电流
		// if (SuperCAP_Low_Vol_Flag == 0)
        // {
        //     //if(!PRESS_V) s_chassisMove->cur_sum_limit = 30000;//不按V键走的快
		// 	if(!PRESS_V) s_chassisMove->cur_sum_limit = 30000;//不按V键走的快
        //     else s_chassisMove->cur_sum_limit = 16000;//按住V键走的慢点
        // }
		// else
		// {
			float capVol_max = 29.0f;//电容电压最大值
			float capVol_limit = 20.0f;//电容电压你想要限制的值
			float current_max = 20000.0f;//电流输出总和->你想限制的最大值
			if(PRESS_SHIFT)//如果按下shift
			{
				capVol_limit = 15.0f;//修改电容电压限幅为更低
				current_max = 30000.0f;//修改电流输出为更高
			}
			float capVol_scal = fabs((s_chassisMove->capVol-capVol_limit)/(capVol_max - capVol_limit));
			s_chassisMove->cur_sum_limit = current_max * capVol_scal * capVol_scal;
		// }

    }
    else//如果电容控制板帧率不正常，认为电容控制板断联，以最低功率运行
    {
        s_chassisMove->cur_sum_limit = 4 * 3000;
        // s_chassisMove->cur_sum_limit = 0;
    }

	//如果缓冲能量小于一定值,限制最大电流输出,每次运行该线程都会减小
	if(s_chassisMove->chassis_power_buffer < 10)
	{
		s_chassisMove->cur_sum_limit -= 2000;
	}

    s_chassisMove->cur_sum = fabs(motor_[0])+fabs(motor_[1])+fabs(motor_[2])+fabs(motor_[3]);//计算总电流
    /*为什么要根据裁判系统帧率决定发送的功率呢？因为在检录后，电容的电被放干净，如果那时候关闭车的电源不给电容充电的话，在进场上电后，裁判系统
    在刚开始的一段时间发送的功率限制值可能有问题（帧率在10左右，不稳定，正常8~12），因为还没初始化完，所以如果以电容没电的情况给电容控制板发送
    功率限制值来充电的话，很可能在上电一瞬间超功率，所以在发送功率限制值的ID的帧率不正常时，就以固定功率发送。*/
    if(finalFps.judge > 8)//如果裁判系统频率高于8Hz
    /*由于功率最大限制值的发送频率与裁判系统有关这个信息的ID发送频率有关，查具体手册看发送频率，我这里大概是8~12Hz*/
    {
        if(s_chassisMove->chassis_power_buffer > 10)
        {
            //功率限制加缓冲能量
            capExpect_txBuff[0] = (uint16_t)((s_chassisMove->chassis_power_limit + SuperCAP_Power_Offset + s_chassisMove->chassis_power_buffer * 0.1) * 100) >> 8;//功率限制值
            capExpect_txBuff[1] = (uint16_t)((s_chassisMove->chassis_power_limit + SuperCAP_Power_Offset + s_chassisMove->chassis_power_buffer * 0.1) * 100);
            //memset(&capExpect_txBuff[2],0, 6*sizeof(uint8_t));//初始化数组中D2~D7位为0，发送给can打包时是长度为8的数组，虽然只使用了前两位。
        }
        else
        {
            capExpect_txBuff[0] = (uint16_t)((s_chassisMove->chassis_power_limit + SuperCAP_Power_Offset) * 100) >> 8;//功率限制值
            capExpect_txBuff[1] = (uint16_t)((s_chassisMove->chassis_power_limit + SuperCAP_Power_Offset) * 100);
            //memset(&capExpect_txBuff[2],0, 6*sizeof(uint8_t));//初始化数组中D2~D7位为0，发送给can打包时是长度为8的数组，虽然只使用了前两位。
        }
    }
    else if(finalFps.judge > 2)//如果裁判系统工作频率低于8Hz大于2Hz
    {
        capExpect_txBuff[0] = (uint16_t)((s_chassisMove->chassis_power_limit + SuperCAP_Power_Offset) * 100) >> 8;//功率限制值
        capExpect_txBuff[1] = (uint16_t)((s_chassisMove->chassis_power_limit + SuperCAP_Power_Offset) * 100);
        //memset(&capExpect_txBuff[2],0, 6*sizeof(uint8_t));
    }
    else//如果裁判系统工作频率低于2Hz
    {
        //功率数值位移操作，同给大疆电机发送数据同理
        capExpect_txBuff[0] = (uint8_t)(basic_power >> 8);//功率基础值55W
        capExpect_txBuff[1] = (uint8_t) basic_power;
        //memset(&capExpect_txBuff[2],0, 6*sizeof(uint8_t));
    }
    if(s_chassisMove->cur_sum > s_chassisMove->cur_sum_limit)//如果目标电流大于限制值
    {
        //为什么限定值是四个电机最大电流：因为如果限定的是单个电机最大电流，那么就有可能在转弯时转不过来，让每个电机电流成比例降低不会太影响运行流畅性
        float motor_Scale = (float)s_chassisMove->cur_sum_limit / (float)s_chassisMove->cur_sum;//最大电流限幅除以当前总电流算出一个比例值
        for(int8_t i=0; i<4; i++)
        {
            motor_[i] *= motor_Scale;//每个电机输出目标电流乘比例值令其等于最大电流限幅
        }
    }
}
/**
  * @brief          底盘初始化
  * @param[in]      none
  * @retval         none
  */
void Chassis_Init(void)
{
//初始化电机信息结构体
	memset(&LF_motor , 0 , sizeof(LF_motor));
	memset(&RF_motor , 0 , sizeof(RF_motor));
	memset(&LB_motor , 0 , sizeof(LB_motor));
	memset(&RB_motor , 0 , sizeof(RB_motor));
	memset(&s_chassisMove , 0 , sizeof(chassisMove_t));

#if	ROBOT_ID == SUN
	//底盘跟随YAW轴串级环PID
	pid_abs_param_init(&YAW_motor_pid_pos, -60.0f, 0, 0, 2000, 2000);
	pid_abs_param_init(&YAW_motor_pid_speed, 1.8f, 0, 20.0f, 2000, 10000);
	//底盘电机单环PID
	pid_abs_param_init(&LF_motor_pid_speed, 12.0f, 0.3, 0, 4000, 10000);
	pid_abs_param_init(&RF_motor_pid_speed, 12.0f, 0.3, 0, 4000, 10000);
	pid_abs_param_init(&LB_motor_pid_speed, 12.0f, 0.3, 0, 4000, 10000);
	pid_abs_param_init(&RB_motor_pid_speed, 12.0f, 0.3, 0, 4000, 10000);
#endif
#if	ROBOT_ID == MOON 
	//底盘跟随YAW轴串级环PID
	pid_abs_param_init(&YAW_motor_pid_pos, -60.0f, 0, 0, 2000, 2000);
	pid_abs_param_init(&YAW_motor_pid_speed, 1.7f, 0, 20.0f, 2000, 13000);
	//底盘电机单环PID
	pid_abs_param_init(&LF_motor_pid_speed, 12.0f, 1, 0, 4000, 10000);
	pid_abs_param_init(&RF_motor_pid_speed, 12.0f, 1, 0, 4000, 10000);
	pid_abs_param_init(&LB_motor_pid_speed, 12.0f, 1, 0, 4000, 10000);
	pid_abs_param_init(&RB_motor_pid_speed, 12.0f, 1, 0, 4000, 10000);
#endif

}
/**
  * @brief          底盘解算函数，放到Task里
  * @param[in]      none
  * @retval         none
  */
void Chassis_Move(void)
{
	//目标前进与左右速度，设定此中间值的作用是方便后面解算，看起来也更直观，也方便确定正反向
	#if	ROBOT_ID == SUN
	Target_Vx=(rc_ctrl.rc.ch[3]*600)/66;
	Target_Vy=(rc_ctrl.rc.ch[2]*600)/66;
	#endif
	#if	ROBOT_ID == MOON
	Target_Vx=(rc_ctrl.rc.ch[3]*600)/66;
	Target_Vy=(rc_ctrl.rc.ch[2]*600)/66;
	#endif
	
	key_Control_chassic();//键盘按键控制底盘电机函数（含斜坡）
	
	switch(robot_Mode.chassisMode)
	{
		case C_NULL://发送电流为0
		 {
			ramp_cFllow.count = 50;//让离开该模式进入跟随模式时的目标速度能有一个基础值，不至于从0开始计算斜坡，反应会更迅速些
			CANTx_SendCurrent(&hcan1,0x200, 0 , 0 , 0 , 0 );
			LF_motor.out_current = 0;
			RF_motor.out_current = 0;
			LB_motor.out_current = 0;
			RB_motor.out_current = 0;
			//为了进入底盘制动模式时目标位置是刚离开该模式时的值
			LF_motor.target_pos=LF_motor.serial_position;
			RF_motor.target_pos=RF_motor.serial_position;
			LB_motor.target_pos=LB_motor.serial_position;
			RB_motor.target_pos=RB_motor.serial_position;
			break;
		 }
		 case C_APART://底盘分离
		 {
			ramp_cFllow.count = 50;//让离开该模式进入跟随模式时的目标速度能有一个基础值，不至于从0开始计算斜坡
			s_chassisMove.W =0;
			opposite_ang=-(YAW_motor.back_position-YAW_MID_VAL)*THETA_OFFSET;//差值除以8191，乘360°（转为度）再乘pi/180（转为弧度）
			
			sin_Gimbal_ang=sin(opposite_ang);
			cos_Gimbal_ang=cos(opposite_ang);
			
			s_chassisMove.Vx=( Target_Vx*cos_Gimbal_ang+Target_Vy*sin_Gimbal_ang);
			s_chassisMove.Vy=(-Target_Vx*sin_Gimbal_ang+Target_Vy*cos_Gimbal_ang);
			
			LF_motor.target_motor_speed= s_chassisMove.Vx+s_chassisMove.Vy+s_chassisMove.W;
			RF_motor.target_motor_speed=-s_chassisMove.Vx+s_chassisMove.Vy+s_chassisMove.W;
			LB_motor.target_motor_speed= s_chassisMove.Vx-s_chassisMove.Vy+s_chassisMove.W;
			RB_motor.target_motor_speed=-s_chassisMove.Vx-s_chassisMove.Vy+s_chassisMove.W;
			LF_motor.out_current = motor_single_loop_PID(&LF_motor_pid_speed, LF_motor.target_motor_speed, LF_motor.back_motor_speed);
			RF_motor.out_current = motor_single_loop_PID(&RF_motor_pid_speed, RF_motor.target_motor_speed, RF_motor.back_motor_speed);
			LB_motor.out_current = motor_single_loop_PID(&LB_motor_pid_speed, LB_motor.target_motor_speed, LB_motor.back_motor_speed);
			RB_motor.out_current = motor_single_loop_PID(&RB_motor_pid_speed, RB_motor.target_motor_speed, RB_motor.back_motor_speed);
			//为了进入底盘制动模式时目标位置是刚离开该模式时的值
			LF_motor.target_pos=LF_motor.serial_position;
			RF_motor.target_pos=RF_motor.serial_position;
			LB_motor.target_pos=LB_motor.serial_position;
			RB_motor.target_pos=RB_motor.serial_position;
			break;
		 }
		 case C_FOLLOW://底盘跟随
		 {
			 #if C_FOLLOW_YAW_DEBUG == 1
			//  -60.0f, 0, 0, 2000, 2000
			//  1.7f, 0, 20.0f, 2000, 10000
				pid_abs_evaluation(&YAW_motor_pid_pos, P5, 0, 0, 2000, S5);
				pid_abs_evaluation(&YAW_motor_pid_speed, P6, I6, D6, 2000, 10000);
            #endif
			s_chassisMove.Vx=Target_Vx;
			s_chassisMove.Vy=Target_Vy;
///************************************** Chassis fllow Gimbal PID***************************************************/
			if( (abs(YAW_motor.back_position-YAW_MID_VAL) < 10) && (!PRESS_FRONT) && (!PRESS_BACK) && (!PRESS_LEFT) && (!PRESS_RIGHT) )
			{
				//ramp_cFllow.count = 550;
				s_chassisMove.W = 0;
			}
			else
			{
				s_chassisMove.W = ChasFollow_yawrot(YAW_MID_VAL, &YAW_motor_pid_pos, &YAW_motor_pid_speed);//*ramp_cal(&ramp_cFllow);
			}
			//此处用斜坡是为了在从其他模式转回到跟随模式时（比如从陀螺转到跟随）防止因为速度过大而转过了，使用的是时间较长的斜坡
			LF_motor.target_motor_speed=( s_chassisMove.Vx+s_chassisMove.Vy+s_chassisMove.W)*ramp_cal(&ramp_cFllow);
			RF_motor.target_motor_speed=(-s_chassisMove.Vx+s_chassisMove.Vy+s_chassisMove.W)*ramp_cal(&ramp_cFllow);
			LB_motor.target_motor_speed=( s_chassisMove.Vx-s_chassisMove.Vy+s_chassisMove.W)*ramp_cal(&ramp_cFllow);
			RB_motor.target_motor_speed=(-s_chassisMove.Vx-s_chassisMove.Vy+s_chassisMove.W)*ramp_cal(&ramp_cFllow);
			/***转向环分离***/
			/*转向环分离原因：
			当四个轮子的target_motor_speed都大于pid所限制的最大值时，由于pid输出最大值的限制，导致四个轮子的输出电流都是pid限制的最大值，
			这就导致在高速运行时车无法转向(转向环失效)，这就需要在输出电流处增加一个转向电流，令最大电流达到最大时，能够令要转向的方向
			内侧减电流，外侧增加电流，同时四个轮子的电流加和不变。
			PS:正常是需要像一下这样写的：
			out_current=pid1(&pid_speed_struct, target_speed, back_speed)+pid2(&pid_speed_struct, target_speed, back_speed)
			pid1 用来计算XY速度，pid2 用来计算W，每个pid最大值分离，我是偷懒了，但是效果是一样的。
			*/
			int16_t W_ = 2.5 * s_chassisMove.W;//转向环，打印一下s_chassisMove.W值看最大值，然后调倍数
			if(abs(W_)>5000) W_ = 5000 * W_ / abs(W_);//如果转向环值大于(+-)5000，则将转向环限制在(+-)5000，5000这个值也是调出来的
			LF_motor.out_current = motor_single_loop_PID(&LF_motor_pid_speed, LF_motor.target_motor_speed, LF_motor.back_motor_speed)+W_;
			RF_motor.out_current = motor_single_loop_PID(&RF_motor_pid_speed, RF_motor.target_motor_speed, RF_motor.back_motor_speed)+W_;
			LB_motor.out_current = motor_single_loop_PID(&LB_motor_pid_speed, LB_motor.target_motor_speed, LB_motor.back_motor_speed)+W_;
			RB_motor.out_current = motor_single_loop_PID(&RB_motor_pid_speed, RB_motor.target_motor_speed, RB_motor.back_motor_speed)+W_;
			//为了进入底盘制动模式时目标位置是刚离开该模式时的值
			LF_motor.target_pos=LF_motor.serial_position;
			RF_motor.target_pos=RF_motor.serial_position;
			LB_motor.target_pos=LB_motor.serial_position;
			RB_motor.target_pos=RB_motor.serial_position;
			break;
		}
		 case C_TOP://小陀螺
		{
			//按照等级来定小陀螺转速，有时候裁判系统返给的底盘功率是不太正常的，比如五秒倒计时之前，功率上限是350，所以用等级来确定小陀螺转速
			if(RobotLevel == 1)
				s_chassisMove.W = 3000;
			else if(RobotLevel == 2)
				s_chassisMove.W = 4000;
			else if(RobotLevel == 3)
				s_chassisMove.W = 5000;
			else
				s_chassisMove.W = 3000;
			
			ramp_cFllow.count = 50;//让离开该模式进入跟随模式时的目标速度能有一个基础值，不至于从0开始计算斜坡，反应会更迅速些
			
			opposite_ang=-(YAW_motor.back_position-YAW_MID_VAL) * THETA_OFFSET;//差值除以8191，乘360°（转为度）再乘pi/180（转为弧度）
			//根据相对枪口朝前时6020的编码器值，求出相对sin与cos，方便后面做运动速度合成
			sin_Gimbal_ang=sin(opposite_ang);
			cos_Gimbal_ang=cos(opposite_ang);
			//速度合成，将x，y分解后的速度合成为电机目标速度
			s_chassisMove.Vx=( Target_Vx*cos_Gimbal_ang+Target_Vy*sin_Gimbal_ang);
			s_chassisMove.Vy=(-Target_Vx*sin_Gimbal_ang+Target_Vy*cos_Gimbal_ang);
			
			LF_motor.target_motor_speed=( s_chassisMove.Vx*sqrt(2)+s_chassisMove.Vy*sqrt(2))*0.5+s_chassisMove.W;
			RF_motor.target_motor_speed=(-s_chassisMove.Vx*sqrt(2)+s_chassisMove.Vy*sqrt(2))*0.5+s_chassisMove.W;
			LB_motor.target_motor_speed=( s_chassisMove.Vx*sqrt(2)-s_chassisMove.Vy*sqrt(2))*0.5+s_chassisMove.W;
			RB_motor.target_motor_speed=(-s_chassisMove.Vx*sqrt(2)-s_chassisMove.Vy*sqrt(2))*0.5+s_chassisMove.W;
			LF_motor.out_current = motor_single_loop_PID(&LF_motor_pid_speed, LF_motor.target_motor_speed, LF_motor.back_motor_speed);
			RF_motor.out_current = motor_single_loop_PID(&RF_motor_pid_speed, RF_motor.target_motor_speed, RF_motor.back_motor_speed);
			LB_motor.out_current = motor_single_loop_PID(&LB_motor_pid_speed, LB_motor.target_motor_speed, LB_motor.back_motor_speed);
			RB_motor.out_current = motor_single_loop_PID(&RB_motor_pid_speed, RB_motor.target_motor_speed, RB_motor.back_motor_speed);
			//为了进入底盘制动模式时目标位置是刚离开该模式时的值
			LF_motor.target_pos=LF_motor.serial_position;
			RF_motor.target_pos=RF_motor.serial_position;
			LB_motor.target_pos=LB_motor.serial_position;
			RB_motor.target_pos=RB_motor.serial_position;
			break;
		 }
		case C_STOP://制动,用的串级环
		{
			pid_abs_evaluation(&LF_motor_pid_stop_pos, 0.5f , 0 , 0 , 2000 , 3000);
			pid_abs_evaluation(&LF_motor_pid_stop_speed, 7.0f , 0.2f , 0 , 2000 , 10000);
			pid_abs_evaluation(&RF_motor_pid_stop_pos, 0.5f , 0 , 0 , 2000 , 3000);
			pid_abs_evaluation(&RF_motor_pid_stop_speed, 7.0f , 0.2f , 0 , 2000 , 10000);
			pid_abs_evaluation(&LB_motor_pid_stop_pos, 0.5f , 0 , 0 , 2000 , 3000);
			pid_abs_evaluation(&LB_motor_pid_stop_speed, 7.0f , 0.2f , 0 , 2000 , 10000);
			pid_abs_evaluation(&RB_motor_pid_stop_pos, 0.5f , 0 , 0 , 2000 , 3000);
			pid_abs_evaluation(&RB_motor_pid_stop_speed, 7.0f , 0.2f , 0 , 2000 , 10000);
			ramp_cFllow.count = 50;//让离开该模式进入跟随模式时的目标速度能有一个基础值，不至于从0开始计算斜坡，反应会更迅速些

			LF_motor.out_current = motor_double_loop_PID(&LF_motor_pid_stop_pos,&LF_motor_pid_stop_speed, \
														LF_motor.serial_position , LF_motor.target_pos , LF_motor.back_motor_speed);
			RF_motor.out_current = motor_double_loop_PID(&RF_motor_pid_stop_pos,&RF_motor_pid_stop_speed, \
														RF_motor.serial_position , RF_motor.target_pos , RF_motor.back_motor_speed);
			LB_motor.out_current = motor_double_loop_PID(&LB_motor_pid_stop_pos,&LB_motor_pid_stop_speed, \
														LB_motor.serial_position , LB_motor.target_pos , LB_motor.back_motor_speed);
			RB_motor.out_current = motor_double_loop_PID(&RB_motor_pid_stop_pos,&RB_motor_pid_stop_speed, \
														RB_motor.serial_position , RB_motor.target_pos , RB_motor.back_motor_speed);
			break;
		 }
	}
//这里赋值成数组是因为懒了，刚开始学习的时候用的结构体，但是后来太繁杂，后面的控制功率函数如果要参数化的话要加太多参数了，所以在这里赋值为数组
	motor[0]=RF_motor.out_current;
	motor[1]=LF_motor.out_current;
	motor[2]=LB_motor.out_current;
	motor[3]=RB_motor.out_current;
	
	chassis_power_control(&s_chassisMove, motor);//功率控制
	
	//CANTx_SendCurrent(&hcan1,0x200, RF_motor.out_current , LF_motor.out_current , LB_motor.out_current , RB_motor.out_current );
	CANTx_SendCurrent(&hcan1,0x200, motor[0] , motor[1] , motor[2] , motor[3] );

}

/**
  * @brief          按键控制，用斜坡函数对目标速度进行线性控制
  * @param[in]      none
  * @retval         none
  */
static void key_Control_chassic(void)
{
	/*
	没有按下V+没有按下SHIFT = 中速
	没有按下V+按下SHIFT = 高速
	按下V+没有按下SHIFT = 低速
	按下V+按下SHIFT = 低速
	*/
	//前进
	if (PRESS_FRONT)
	{
		if(!PRESS_V && PRESS_SHIFT) Target_Vx = RAMP_MAX_VAL * ramp_cal(&ramp_w);
		else if(!PRESS_V && !PRESS_SHIFT) Target_Vx = RAMP_MID_VAL*ramp_cal(&ramp_w);
		else if(PRESS_V && !PRESS_SHIFT) Target_Vx = RAMP_LOW_VAL*ramp_cal(&ramp_w);
		else Target_Vx = RAMP_LOW_VAL*ramp_cal(&ramp_w);

		ramp_w.interval = 0;//清空W按键防抖值
		ramp_s.count = RAMP_INIT;//重置S按键的斜坡函数计数值
	}
	else
	{
		ramp_w.interval++;//计算松开W的时间，chassis线程5ms执行一次，也就是检测15ms没有按下W，重置斜坡函数计数值
		if(ramp_w.interval>3) ramp_w.count = RAMP_INIT;//松开W，重置斜坡函数计数值
	}
	//后退
	if (PRESS_BACK)
	{
		if(!PRESS_V && PRESS_SHIFT) Target_Vx = -RAMP_MAX_VAL * ramp_cal(&ramp_s);
		else if(!PRESS_V && !PRESS_SHIFT) Target_Vx = -RAMP_MID_VAL*ramp_cal(&ramp_s);
		else if(PRESS_V && !PRESS_SHIFT) Target_Vx = -RAMP_LOW_VAL*ramp_cal(&ramp_s);
		else Target_Vx = -RAMP_LOW_VAL*ramp_cal(&ramp_s);

		ramp_s.interval = 0;
		ramp_w.count = RAMP_INIT;
	}
	else
	{
		ramp_s.interval++;
		if(ramp_s.interval>3) ramp_s.count = RAMP_INIT;//重置斜坡函数计数值
	}
	//向左
	if (PRESS_LEFT)
	{
		if(!PRESS_V && PRESS_SHIFT) Target_Vy = -RAMP_MAX_VAL * ramp_cal(&ramp_a);
		else if(!PRESS_V && !PRESS_SHIFT) Target_Vy = -RAMP_MID_VAL*ramp_cal(&ramp_a);
		else if(PRESS_V && !PRESS_SHIFT) Target_Vy = -RAMP_LOW_VAL*ramp_cal(&ramp_a);
		else Target_Vy = -RAMP_LOW_VAL*ramp_cal(&ramp_a);

		// Target_Vy = -RAMP_MID_VAL * ramp_cal(&ramp_a);
		ramp_a.interval = 0;
		ramp_d.count = RAMP_INIT;
	}
	else
	{
		ramp_a.interval++;
		if(ramp_a.interval>3) ramp_a.count = RAMP_INIT;//重置斜坡函数计数值
	}
	//向右
	if (PRESS_RIGHT)
	{
		if(!PRESS_V && PRESS_SHIFT) Target_Vy = RAMP_MAX_VAL * ramp_cal(&ramp_d);
		else if(!PRESS_V && !PRESS_SHIFT) Target_Vy = RAMP_MID_VAL*ramp_cal(&ramp_d);
		else if(PRESS_V && !PRESS_SHIFT) Target_Vy = RAMP_LOW_VAL*ramp_cal(&ramp_d);
		else Target_Vy = RAMP_LOW_VAL*ramp_cal(&ramp_d);

		// Target_Vy = RAMP_MID_VAL * ramp_cal(&ramp_d);
		ramp_d.interval = 0;
		ramp_a.count = RAMP_INIT;
	}
	else
	{
		ramp_d.interval++;//计算松开D的时间
		if(ramp_d.interval>3) ramp_d.count = RAMP_INIT;//重置斜坡函数计数值
	}

}


