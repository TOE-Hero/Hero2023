#include <cmsis_os.h>
#include <main.h>
#include <stdbool.h>
#include "bsp_can.h"
#include "pid.h"
#include "STMGood.h"
#include "bsp_usart.h"
#include "dji_motor.h"
#include "bsp_remote.h"
#include "math.h"
#include "printf.h"
#include "RM_Judge.h"

#include "Hero_control.h"
/**************************** Debug *****************************/
#define T_SWING_DEBUG       	0	//pid_abs_evaluation(&TRANS_motor_pid_pos,P7,0,0,0,S7);
								//pid_abs_evaluation(&TRANS_motor_pid_speed,P8,I8,D8,0,16000.0f);
/**************************** define ****************************/
#if	ROBOT_ID == SUN
	#define TRANS_REDUCTION_RATIO   REDUCTION_RATIO_3508_1_19//宏定义拨弹盘电机减速比
	#define TRANS_STEP			    MOTOR_ONE_TURN_SCALE_3508/6*TRANS_REDUCTION_RATIO
	// #define TRANS_STEP			    26215.6//8191/6*19.203208556
	#define FRI_SPD_H       	4250//4050// 16m/s弹速,一般白弹丸要比透明弹丸少70左右
	#define FRI_SPD_H_LOB		4300// 16m/s弹速，吊射弹速，同上 
	#define FRI_SPD_L       	2700// 10m/s弹速
#endif
#if	ROBOT_ID == MOON
	#define TRANS_REDUCTION_RATIO   REDUCTION_RATIO_3508_1_19//宏定义拨弹盘电机减速比
	#define TRANS_STEP			    MOTOR_ONE_TURN_SCALE_3508/6*TRANS_REDUCTION_RATIO
	#define FRI_SPD_H       	4320//4050// 16m/s弹速,一般白弹丸要比透明弹丸少70左右
	#define FRI_SPD_H_LOB		4350 //4180// 16m/s弹速，吊射弹速，同上
	#define FRI_SPD_L       	2700// 10m/s弹速
#endif


#define SHOOT_ONCE_DECLR 	 	0//shoot_once()函数是否定义宏定义，0为不定义，没用到这个函数，但为了没worning就给0了

#define BULL_HEAT 			 	100//每发弹丸的热量
//射频，测试用
#define S_LOW_FREQUENCY 	 	1500//低射频
#define S_MID_FREQUENCY 	 	800//中射频
#define S_HIGH_FREQUENCY 	 	500//高射频

#define INTERVAL_VAL		 	340.0f//区间值，防止按下鼠标左键时在转向下一步的途中再转一步
#define SHOOT_ONE_BALL    		({do{TRANS_motor.target_pos-= TRANS_STEP;}while(0);1;})//播弹盘转一步，发射一发，这是个宏函数，最后一个表达式的值为该函数返回值
#define SHOOT_KILL_ME   	 	do{TRANS_motor.target_pos-= 10*TRANS_STEP;}while(0)//播弹盘转10步，将自己射死（按住B再按鼠标左键）
#define SHOOT_BACK_ONE_STEP		do{TRANS_motor.target_pos+= TRANS_STEP;}while(0)//卡住的话回转一步

/**************************** global variable ************************************/
int16_t 					 	Fric_SpeedTarget=0;//裁判系统弹速
float 						 	temperture_offset=0;//随着摩擦轮温度升高，降低摩擦轮转速的补偿值
float				         	average_temperture;//两摩擦轮平均温度
float                        	average_speed;//两摩擦轮平均速度
uint32_t					 	fric_target_speed;//摩擦轮目标速度，用作模式转换
/************************** static declaration ***********************************/
static int16_t				 	count;//计算射频用，数到射频值重载为0
static int16_t				 	count_judge_loss;//计算射频用，裁判系统如果掉线，也能够发弹，只不过是很慢的射频
static int16_t 				 	shootFreq=S_MID_FREQUENCY; // 设定射频

static uint8_t 					keyLock_Z=0;//键盘按键Z锁标志位
static uint8_t 					keyLock_X=0;//键盘按键X锁标志位
static uint8_t 					keyLock_E=0;//键盘按键E锁标志位

static int16_t					fric_speed_offset_key=0;//摩擦轮速度补偿值,用键盘控制
static float 					bullet_speed_ = 0;//弹速信息，当弹速过低时用作反向补偿
static uint8_t 					over_ball_speed_flag = 0;//超弹速补偿标志位
static uint16_t 				over_ball_speed_amount = 0;//超弹速补偿计数值，超一次弹速加一次1
static uint16_t 				last_shot_ball_amount = 0;//上一次超弹速补偿计数值，用来与超弹速计数值做比较
static int16_t 					over_ball_speed_offset = 0;//超弹速补偿值
static bool    					shoot_flag;//是否发弹标志位，根据摩擦轮转速判定
/************************** extern declaration ***********************************/
extern s_robo_Mode_Setting		robot_Mode;//机器人模式
extern u_data_32bit 			bullet_speed;//裁判系统弹速
extern uint16_t 				shot_ball_amount;//发射弹丸数量
extern s_FPS_monitor			finalFps;//最终帧率计算值
extern s_FPS_monitor	    	Fps;//每个要检查FPS的地方++
extern s_rm_judge_shoot_data_t 	Judge_ShootData;
/************************** Function declaration *********************************/

static void ShootBall(void);//控制播弹盘电机函数
/*********************************************************************************/
/**
  * @brief          发射机构初始化
  * @param[in]      none
  * @retval         none
  */
void Shoot_Init(void)
{
	TRANS_motor.target_pos = TRANS_motor.serial_position;//让上电时的编码器值为目标值
#if	ROBOT_ID == SUN
	pid_abs_param_init(&FIRE_L_motor_pid_speed,12.0f,0.3,0,2000,16000.0f);
	pid_abs_param_init(&FIRE_R_motor_pid_speed,12.0f,0.3,0,2000,16000.0f);
	pid_abs_param_init(&TRANS_motor_pid_pos,0.25f,0,0,0,1500);
	pid_abs_param_init(&TRANS_motor_pid_speed,20.0f,0.3f,0,2000,16000.0f);
#endif
#if	ROBOT_ID == MOON
	pid_abs_param_init(&FIRE_L_motor_pid_speed,12.0f,0.3,0,2000,16000.0f);
	pid_abs_param_init(&FIRE_R_motor_pid_speed,12.0f,0.3,0,2000,16000.0f);
	pid_abs_param_init(&TRANS_motor_pid_pos,0.25f,0,0,0,1500);
	pid_abs_param_init(&TRANS_motor_pid_speed,15.3f,0.3f,0,2000,16000.0f);
#endif
}

/**
  * @brief          发射机构解算函数，放到Task里
  * @param[in]      none
  * @retval         none
  */

void Shoot_Move(void)
{
	// Judge_ShootData.coolingheat_limit = 34400;
	// Judge_ShootData.coolingheat=0;
	
	ShootBall();
	average_speed = (abs(FIRE_L_motor.back_motor_speed) + abs(FIRE_R_motor.back_motor_speed)) / 2;
	fric_target_speed = (uint32_t)((fabs(FIRE_R_motor.target_motor_speed)+fabs(FIRE_L_motor.target_motor_speed)) / 2);
	average_temperture=(FIRE_R_motor.temperature+FIRE_R_motor.temperature)/2;//摩擦轮电机平均温度
	//弹速限幅，如果裁判系统不返回弹速的话，一般bullet_speed.f这个值一直为0，那么就有可能做反向补偿而超弹速，所以做一个限幅
	bullet_speed_ = bullet_speed.f;
	if(bullet_speed_<15.0) bullet_speed_=15.0;
	if(bullet_speed_>15.9) bullet_speed_=15.9;
	//模式选择
	switch(robot_Mode.shootMode)
	{
/***************************** FIRE_R(+) & L(-)***********************************/
		case S_NULL://摩擦轮发送电流0
		{
		FIRE_R_motor.target_motor_speed = FIRE_L_motor.target_motor_speed = 0;

		FIRE_L_motor.out_current=0;
		FIRE_R_motor.out_current=0;
		break;
		}
		case S_STOP://摩擦轮制动
		{
		FIRE_R_motor.target_motor_speed = FIRE_L_motor.target_motor_speed = 0;

		FIRE_L_motor.out_current = motor_single_loop_PID(&FIRE_L_motor_pid_speed , 0 , FIRE_L_motor.back_motor_speed);
		FIRE_R_motor.out_current = motor_single_loop_PID(&FIRE_R_motor_pid_speed , 0 , FIRE_R_motor.back_motor_speed);
		break;
		}
		case S_SWING://摩擦轮转
		{
			//Z、X键分别用来增加和减小摩擦轮目标转速
			if(PRESS_Z) {keyLock_Z=1;}
			if((!PRESS_Z) && keyLock_Z==1)
			{
				fric_speed_offset_key+=10;
				keyLock_Z = 0;
			}
			if(PRESS_X) {keyLock_X=1;}
			if((!PRESS_X) && keyLock_X==1)
			{
				fric_speed_offset_key-=10;
				keyLock_X = 0;
			}

			//如果裁判系统返回的弹速上限大于15（裁判系统正常）,或者返回的最大弹速上限为0（裁判系统不正常）,或者最大弹速上限为负值（主控设置弹速无限）
			if(Fric_SpeedTarget>15.0f || Fric_SpeedTarget<=0)
			{
				/*进行摩擦轮补偿的思想是，摩擦轮转速只减不加，由于机械做的十分优秀，在固定温度下给摩擦轮固定转速打出的弹速是一个很标准的正态分布，
				（所以这是前提）所以做了这个效果还不错，但是不做纯粹的每个温度对应多少摩擦轮转速的原因是，恒定摩擦轮转速下，温度对应弹速并非是简单一
				次线性，所以需要多次测试去整定超参数，为什么要只减不加呢，因为如果做的是完全的线性，那么有可能出现温度降下来后超射速，也造成了每辆车
				可能由于机械精度不同，每辆车的公式要重新整定，十分麻烦（毕竟调参要更简单），也可以避免由于机械误差导致弹速不稳定，给了机械一个余量*/

				//速换模式
				if(robot_Mode.gimbalMode == G_ENCODE)//吊射模式
				{
					//如果两摩擦轮温度平均值小于30，不进行摩擦轮速度补偿
					if(average_temperture<30) temperture_offset=0;
					else//如果两摩擦轮温度平均值大于30，开始摩擦轮速度补偿
					{
						#if ROBOT_ID == SUN
							//temperture_offset=-fabs(((average_temperture-30)/100)*900);
							temperture_offset=-fabs(((average_temperture-30)/100)*(22*average_temperture));
							// temperture_offset = 1.0f-(0.8*(average_temperture-30.0)/100 + 0.2*(bullet_speed_-15.5f)/100); //0.8*0.01+0.2*0.003=0.008+0.0006=0.86%
						#endif
						#if ROBOT_ID == MOON
							temperture_offset=fabs(((average_temperture-20)/100)*800);
						#endif
					}
					// FIRE_R_motor.target_motor_speed = FRI_SPD_H_LOB+(temperture_offset+fric_speed_offset_key);
					// FIRE_L_motor.target_motor_speed =-FRI_SPD_H_LOB-(temperture_offset+fric_speed_offset_key);
					FIRE_R_motor.target_motor_speed = FRI_SPD_H_LOB+(fric_speed_offset_key);
					FIRE_L_motor.target_motor_speed = -FRI_SPD_H_LOB-(fric_speed_offset_key);
				}
				else//云台模式在其他模式时
				{
					if(average_temperture<30) temperture_offset=0;
					else//如果两摩擦轮温度平均值大于30，开始摩擦轮较大速度补偿
					{
						#if ROBOT_ID == SUN
							temperture_offset=(-fabs(((average_temperture-30)/100)*(22*average_temperture) ));//*((bullet_speed_-15.0)/fabs(bullet_speed_-15.0));
						#endif
						#if ROBOT_ID == MOON
							temperture_offset=fabs(((average_temperture-20)/100)*800);
						#endif
					}
					//超弹速补偿，每超一次，减一次固定摩擦轮速度
					if((bullet_speed.f >= 15.85) && (last_shot_ball_amount != shot_ball_amount))
					{
						over_ball_speed_flag=0;
					}
					if(bullet_speed.f >= 15.85 && over_ball_speed_flag==0)
					{
						++over_ball_speed_amount;
						over_ball_speed_offset=-17*over_ball_speed_amount;
						over_ball_speed_flag=1;
						last_shot_ball_amount = shot_ball_amount;
					}
					//摩擦轮目标转速
					FIRE_R_motor.target_motor_speed = FRI_SPD_H;//+temperture_offset+over_ball_speed_offset;
					FIRE_L_motor.target_motor_speed =-FRI_SPD_H;//-temperture_offset-over_ball_speed_offset;
				}
				
			}
			else if(Fric_SpeedTarget<=15.0f && Fric_SpeedTarget>0.0f)//如果裁判系统返回的弹速上限不大于15并且大于0的话，低弹速
			{
				FIRE_R_motor.target_motor_speed = FRI_SPD_L;
				FIRE_L_motor.target_motor_speed =-FRI_SPD_L;
			}
		
		FIRE_L_motor.out_current = motor_single_loop_PID(&FIRE_L_motor_pid_speed , FIRE_L_motor.target_motor_speed , \
															FIRE_L_motor.back_motor_speed);
		FIRE_R_motor.out_current = motor_single_loop_PID(&FIRE_R_motor_pid_speed , FIRE_R_motor.target_motor_speed , \
															FIRE_R_motor.back_motor_speed);
		break;
		}
	
	}
/********************************** TRANS *****************************************/
	switch(robot_Mode.transMode)
	{
		case T_NULL://播弹盘电机发送电流0
		{
			robot_Mode.shootOnce = 0;//不准发弹
			TRANS_motor.target_pos = TRANS_motor.serial_position;
			TRANS_motor.out_current=0;
		break;
		}
		case T_STOP:
		{
			robot_Mode.shootOnce = 0;//不准发弹
			TRANS_motor.out_current=motor_double_loop_PID(&TRANS_motor_pid_pos,&TRANS_motor_pid_speed,\
												   TRANS_motor.serial_position, TRANS_motor.target_pos,TRANS_motor.back_motor_speed);
		break;
		}
		case T_SWING://播弹盘电机转
		{
            #if T_SWING_DEBUG == 1
			//0.25f,0,0,0,1500
			//15.3f,0.3f,0,2000,16000.0f
				TRANS_motor.target_pos+=(rc_ctrl.rc.ch[4]*100)/66;//调试用
				pid_abs_evaluation(&TRANS_motor_pid_pos,P7,0,0,0,S7);
				pid_abs_evaluation(&TRANS_motor_pid_speed,P8,I8,D8,0,16000.0f);
			#endif
			
			TRANS_motor.out_current=motor_double_loop_PID(&TRANS_motor_pid_pos,&TRANS_motor_pid_speed,\
													   TRANS_motor.serial_position, TRANS_motor.target_pos,TRANS_motor.back_motor_speed);
		break;
		}
	}
	//发送CAN2路摩擦轮与拨弹盘电流
	can_send_state.shoot = (&hcan2,0x200, 0 , TRANS_motor.out_current , FIRE_L_motor.out_current , FIRE_R_motor.out_current );
	
}


/**
  * @brief          拨弹盘控制
  * @param[in]      none
  * @retval         none
  */
static void ShootBall(void)
{

	static uint16_t Last_fric_l;
	//static uint16_t  Last_Last_fric_l;

	#if T_SWING_DEBUG == 0 //如果调试拨弹盘的话，遥控器侧边拨轮不作为固定射频使用
	if( (rc_ctrl.rc.ch[4]>=600) && (LS_UP) )//按固定射频发射，射频在下面设定，左拨杆UP且左上角拨轮拨到最上方时生效
	{
		
		if((Judge_ShootData.coolingheat_limit - BULL_HEAT) >= Judge_ShootData.coolingheat)//枪口热量闭环，防止超枪口热量
		{
			count++;
			if(count>shootFreq)	count=shootFreq;//count值重载，也用来控制射频
		}
		else
		{
			count = 0;
		}
		//Judge_ShootData.coolingheat_limit = 34400;// 没接裁判系统的时候固定热量
	}
	else	count = 0;
	#endif
	//为了防止卡弹，也为了防止鼠标左键连点造成的连发，INTERVAL_VAL是区间值，是发弹临界值
	if(fabs((double)TRANS_motor.target_pos - (double)TRANS_motor.serial_position) > INTERVAL_VAL)
	{
		//printf("fric_error_INTERVAL_VAL\r\n");
		robot_Mode.shootOnce = 0;
		count = 0;
	}

	if(count==shootFreq || robot_Mode.shootOnce==1)//按下左键或者达到计算的射频时发射一发
	{
		/*防止卡弹，防止连发*/
		if(fabs((double)TRANS_motor.target_pos - (double)TRANS_motor.serial_position) > 1.1*TRANS_STEP)
		{
			printf("fric_error_1.1*TRANS_STEP\r\n");
			SHOOT_BACK_ONE_STEP;//不是电机回转一步，而是因为卡了，目标值要减一步，实际电机没有回转
		}
		else
		{
			//防止超热量，最大冷却值减一个弹丸的热量大于实时热量时可发弹
			if((Judge_ShootData.coolingheat_limit - BULL_HEAT) >= Judge_ShootData.coolingheat)
			{
				if((average_speed > (fric_target_speed * 0.7)) && finalFps.fric_l  > 850 && finalFps.fric_r > 850)
				{
					if(PRESS_G)//如果按住G键，再按左键松开，拨弹盘电机目标位置加10步,是为了破釜沉舟，连发到死，一级6发，二级8发，三级10发
						SHOOT_KILL_ME;//Kill myself
					else//如果没有按G键，拨弹盘电机目标加1步
						SHOOT_ONE_BALL;//Shoot one ball
				}
				else
				{
				count = 0;
				robot_Mode.shootOnce = 0;
				}
			}
			else
			{
				count = 0;
				robot_Mode.shootOnce = 0;
			}

		}
	//不满足上面的条件或者上面的条件运行完，标志位清零
	count = 0;
	robot_Mode.shootOnce = 0;
	}
}

static void CalculateCoolHeat(uint8_t robot_level, s_rm_judge_shoot_data_t *judge_shoot_data, float ball_speed_limit, bool *shoot_flag)
{
static int16_t cal_freq = 0;
cal_freq++;
int16_t cal_now_coolingheat;
//弹速模式,16m/s
if(ball_speed_limit > 15.0f || ball_speed_limit < 0.0f)
{
	switch (robot_level)
	{
	case 1:
		judge_shoot_data->coolingheat_limit = 100;
		judge_shoot_data->coolingheat_every_second = 20;
		break;
	case 2:
		judge_shoot_data->coolingheat_limit = 200;
		judge_shoot_data->coolingheat_every_second = 60;
		break;
	case 3:
		judge_shoot_data->coolingheat_limit = 300;
		judge_shoot_data->coolingheat_every_second = 100;
		break;
	default:
		judge_shoot_data->coolingheat_limit = 100;
		judge_shoot_data->coolingheat_every_second = 20;
		break;
	}
}
else if(ball_speed_limit<=15.0f && ball_speed_limit>0.0f)//爆发模式，10m/s
{
	switch (robot_level)
	{
	case 1:
		judge_shoot_data->coolingheat_limit = 200;
		judge_shoot_data->coolingheat_every_second = 40;
		break;
	case 2:
		judge_shoot_data->coolingheat_limit = 350;
		judge_shoot_data->coolingheat_every_second = 80;
		break;
	case 3:
		judge_shoot_data->coolingheat_limit = 500;
		judge_shoot_data->coolingheat_every_second = 120;
		break;
	default:
		judge_shoot_data->coolingheat_limit = 100;
		judge_shoot_data->coolingheat_every_second = 20;
		break;
	}
}

if(!(cal_freq%1000))
{
	judge_shoot_data->coolingheat = judge_shoot_data->coolingheat - judge_shoot_data->coolingheat_every_second;
}

}
