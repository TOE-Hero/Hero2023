#include <cmsis_os.h>
#include <main.h>
#include <math.h>
#include "bsp_remote.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "laser.h"
#include "STMGood.h"
#include "dji_motor.h"
#include "pid.h"
#include "INS_task.h"

#include "Hero_control.h"
/**
 **********************************************************************************
 * @FilePath: \Hero2022-vscode\Module\Mode_Switch.c
 * @Brief: 
 * @Date: 2022-05
 * @Author: Rosenyin
 **********************************************************************************
 */
/**
 ******************************************************************************
  遥控器:
          UP  |UP  |  G_GYRO 	+ C_TOP 	+ S_SWING 	+ T_SWING
              |MID |  G_GYRO 	+ C_FOLLOW 	+ S_SWING 	+ T_SWING
              |DOWN|  G_GYRO 	+ C_APART 	+ S_SWING 	+ T_SWING
			  
          MID |UP  |  G_GYRO 	+ C_TOP 	+ S_STOP 	+ T_NULL
              |MID |  G_GYRO 	+ C_FOLLOW 	+ S_STOP 	+ T_NULL
              |DOWN|  G_ENCODE 	+ C_STOP 	+ S_STOP 	+ T_NULL
			  
          DOWN|UP  |  keyboard--------->键盘模式,若没有按下Ctrl\R\C则进入默认模式，G_GYRO + C_FOLLOW
              |MID |  G_GYRO + C_APART + S_STOP + T_NULL
              |DOWN|  security_mode---->发送电流0
  键盘:
			MOUSE_L	|发射一枚弹丸
			MOUSE_L |按住开启自瞄
			keyF	|打开摩擦轮
			keyR	|开启陀螺移动模式
			keyC	|开启底盘分离模式
			keyV    |开启慢速模式
			keyB    |按住B再按鼠标左键连发10发弹丸
			keyCtrl	|开启吊射模式，底盘制动

 *****************************************************************************
 **/
/**************************** #define *******************************************/

/************************ global variables **************************************/
s_robo_Mode_Setting		robot_Mode;//定义机器人模式结构体

/************************** extern declaration***********************************/
extern RC_ctrl_t		rc_ctrl;//遥控器信息结构体
extern s_FPS_monitor	finalFps;//最终帧率计算值
extern float			average_temperture;//两摩擦轮平均温度
extern float			average_speed;//两摩擦轮平均速度
extern uint16_t			fric_target_speed;//摩擦轮目标速度，用作模式转换

/********************************************************************************/
static void DBUS_Ctrl(void);
/**
  * @brief          设定所有模块模式为零电流模式
  * @param[in]      none
  * @retval         none
  */
void ModeInit(void)
{
	robot_Mode.gimbalMode  = G_NULL;
	robot_Mode.chassisMode = C_NULL;
	robot_Mode.shootMode   = S_NULL;
	robot_Mode.transMode   = T_NULL;
}

/**
  * @brief          红点激光控制函数，如果在自瞄模式，则关闭红点 / 如果不在自瞄模式并且摩擦轮处于开启状态，开启红点 / 如果不在自瞄
  * 				模式也没有开启摩擦轮，则关闭红点
  * @param[in]      none
  * @retval         none
  */
static void laser_ctrl(void)
{
	if((robot_Mode.gimbalMode == G_FULL_VISION)||(robot_Mode.gimbalMode == G_HALF_VISION)) 
	{
		LaserOff();
	}
	else if(robot_Mode.shootMode == S_SWING)
	{
		// 如果两个摩擦轮帧率正常，并且摩擦轮的返回的平均速度大于摩擦轮目标速度的1/2，打开激光（前提是摩擦轮处于旋转模式）
		if(average_speed > fric_target_speed * 0.5 && finalFps.fric_l > 800 && finalFps.fric_r > 800)
			LaserOn();
	}
	else 
	{
		LaserOff();
	}
}

/**
 * @brief 遥控器拨杆与键盘控制模式函数
 * 
 */
static void DBUS_Ctrl(void)
{
	static int8_t keyLock_PressL, keyLock_PressR = 0;//鼠标左右键标志位
	static int8_t keyLock_E, keyLock_R, keyLock_CTRL, keyLock_F, \
					keyLock_C, keyLock_B, keyLock_Q, keyLock_G = 0;//按键抬起触发标志位，置1后要清零
	static int8_t deafult_mode_flag = 1;//默认模式标志位
	//防止死掉再复活的时候，不是默认模式，比如陀螺死亡，再复活还是陀螺模式
	if((finalFps.LB_motor==0)&&(finalFps.LF_motor==0)&&(finalFps.RB_motor==0)&&(finalFps.RF_motor==0)) deafult_mode_flag = 1;
	static int8_t g_gyro_flag = 0;//云台纯陀螺仪控制模式标志位
	static uint8_t vis_mode = G_FULL_VISION;//自瞄模式切换标志位
	if(LS_DOWN)//DOWN
	{
		if(LS_DOWN && RS_DOWN)//DOWN | DOWN-------------------------->发送电流0,保护模式
		{deafult_mode_flag=1;
			robot_Mode.gimbalMode = G_NULL;
			robot_Mode.chassisMode= C_NULL;
			robot_Mode.shootMode  = S_NULL;
			robot_Mode.transMode  = T_NULL;
		}
		if(LS_DOWN && RS_MID)//DOWN | MID
		{deafult_mode_flag=1;
			robot_Mode.gimbalMode = G_HALF_GYRO;
			robot_Mode.chassisMode= C_APART;
			robot_Mode.shootMode  = S_STOP;
			robot_Mode.transMode  = T_STOP;
		}
		if(LS_DOWN && RS_UP)//DOWN | UP------------------------------>键盘控制
		{
			//F键锁 开摩擦轮
			if(PRESS_F) {keyLock_F=1;	deafult_mode_flag=0;}
			if((!PRESS_F) && keyLock_F==1)
			{deafult_mode_flag=0;
				if( (robot_Mode.shootMode == S_STOP) && (robot_Mode.transMode==T_STOP) )
					{robot_Mode.shootMode = S_SWING;robot_Mode.transMode=T_SWING;}
				else {robot_Mode.shootMode = S_STOP;robot_Mode.transMode=T_STOP;}
				keyLock_F = 0;
			}
			//左键 拨盘目标增
			if(PRESS_MOUSE_L) {keyLock_PressL=1;	deafult_mode_flag=0;}
			if((!PRESS_MOUSE_L) && keyLock_PressL==1)
			{deafult_mode_flag=0;
				robot_Mode.shootOnce = 1;
				keyLock_PressL = 0;
			}
			//C键锁 底盘分离
			if(PRESS_C) {keyLock_C=1;	deafult_mode_flag=0;}
			if(!PRESS_C && keyLock_C==1)
			{deafult_mode_flag=0;
				if(robot_Mode.gimbalMode==G_ENCODE) {robot_Mode.gimbalMode=G_HALF_GYRO;}
				if(robot_Mode.chassisMode == C_APART)	{ robot_Mode.chassisMode = C_FOLLOW;}
				else {robot_Mode.chassisMode = C_APART;}
				keyLock_C = 0;
			}
			//R键锁 陀螺
			if(PRESS_R) {keyLock_R=1;	deafult_mode_flag=0;}
			if(!PRESS_R && keyLock_R==1)
			{deafult_mode_flag=0;
				if(robot_Mode.gimbalMode==G_ENCODE) {robot_Mode.gimbalMode=G_HALF_GYRO;}
				if(robot_Mode.chassisMode == C_TOP)	{robot_Mode.chassisMode = C_FOLLOW;}
				else 
				{
					robot_Mode.chassisMode	= C_TOP;
				}
				keyLock_R = 0;
			}
			//Ctrl键锁 吊射 制动
			if(PRESS_CTRL) {keyLock_CTRL=1;		deafult_mode_flag=0;}
			if(!PRESS_CTRL && keyLock_CTRL==1)
			{deafult_mode_flag=0;
				if(robot_Mode.gimbalMode == G_ENCODE)
				{robot_Mode.gimbalMode	= G_HALF_GYRO;
				robot_Mode.chassisMode	= C_FOLLOW;}
				else
				{robot_Mode.gimbalMode	= G_ENCODE;
				robot_Mode.chassisMode = C_STOP;
				}
				keyLock_CTRL = 0;
			}
			//按Q切换自瞄模式
			if(PRESS_Q) {keyLock_Q=1;  deafult_mode_flag=0;}
			if(!PRESS_Q && keyLock_Q==1)
			{
				deafult_mode_flag = 0;
				if(vis_mode == G_FULL_VISION) vis_mode = G_HALF_VISION;
				else vis_mode = G_FULL_VISION;
				keyLock_Q=0;
			}
			
			//按住鼠标右键，开启自瞄，关闭恢复半陀螺仪模式
			if(PRESS_MOUSE_R)
			{
				deafult_mode_flag = 0;
				if(robot_Mode.gimbalMode!=G_ENCODE)//在吊射模式下不生效
					robot_Mode.gimbalMode = vis_mode;
			}
			else if((robot_Mode.gimbalMode!=G_ENCODE))
			{
				if(g_gyro_flag==1)
					robot_Mode.gimbalMode = G_GYRO;
				else
					robot_Mode.gimbalMode = G_HALF_GYRO;
			}
			// G键锁，开启云台陀螺仪模式，这个按键被连续发弹自杀模式占用了
			// if(PRESS_X) {keyLock_G=1;deafult_mode_flag=0;}
			// if(!PRESS_X && keyLock_X==1)
			// {
			// 	if(robot_Mode.gimbalMode == G_GYRO)
			// 	{
			// 		robot_Mode.gimbalMode	= G_HALF_GYRO;
			// 		g_gyro_flag=0;
			// 	}
			// 	else 
			// 	{
			// 		robot_Mode.gimbalMode	= G_GYRO;
			// 		g_gyro_flag=1;
			// 	}
			// 	keyLock_X = 0;
			// }
			//如果默认模式标志位不被置为0，则默认模式标志位为1有效，进入默认模式
			if(deafult_mode_flag == 1)
			{
				robot_Mode.gimbalMode	= G_HALF_GYRO;
				robot_Mode.chassisMode	= C_FOLLOW;
				robot_Mode.shootMode	= S_STOP;
				robot_Mode.transMode	= T_STOP;
			}
			
		}//LS_DOWN && RS_UP
	}//LS_DOWN
	/****************************遥控器控制调试*******************************/
	if(LS_MID)//MID
	{
		if(LS_MID && RS_DOWN)//MID | DOWN
		{deafult_mode_flag=1;
			robot_Mode.gimbalMode = G_HALF_GYRO;//G_FULL_VISION;//G_HALF_GYRO;//G_ENCODE;
			robot_Mode.chassisMode= C_STOP;
			robot_Mode.shootMode  = S_STOP;
			robot_Mode.transMode  = T_NULL;
		}
		if(LS_MID && RS_MID)//MID | MID
		{deafult_mode_flag=1;
			robot_Mode.gimbalMode = G_FULL_VISION;//G_GYRO;//G_HALF_VISION;//G_FULL_VISION;//G_HALF_VISION;
			robot_Mode.chassisMode= C_APART;
			robot_Mode.shootMode  = S_STOP;
			robot_Mode.transMode  = T_NULL;
		}
		if(LS_MID && RS_UP)//MID | UP
		{deafult_mode_flag=1;
			robot_Mode.gimbalMode = G_HALF_GYRO;//G_HALF_GYRO;//G_FULL_VISION;//G_HALF_GYRO;//G_FULL_VISION;
			robot_Mode.chassisMode= C_TOP;// C_APART;//C_TOP;
			robot_Mode.shootMode  = S_STOP;
			robot_Mode.transMode  = T_NULL;
		}
	}//LS_MID
	if(LS_UP)//UP
	{
		if(LS_UP && RS_DOWN)//UP | DOWN
		{deafult_mode_flag=1;
			robot_Mode.gimbalMode = G_ENCODE;//G_FULL_VISION;//G_HALF_GYRO;
			robot_Mode.chassisMode= C_STOP;
			robot_Mode.shootMode  = S_SWING;
			robot_Mode.transMode  = T_SWING;
		}
		if(LS_UP && RS_MID)//UP | MID
		{deafult_mode_flag=1;
			robot_Mode.gimbalMode = G_FULL_VISION;//G_HALF_VISION;//G_HALF_GYRO;
			robot_Mode.chassisMode= C_APART;//C_FOLLOW;
			robot_Mode.shootMode  = S_SWING;
			robot_Mode.transMode  = T_SWING;
		}
		if(LS_UP && RS_UP)//UP | UP
		{deafult_mode_flag=1;
			robot_Mode.gimbalMode = G_FULL_VISION;//G_ENCODE;//G_FULL_VISION;
			robot_Mode.chassisMode= C_TOP;//C_APART;//C_FOLLOW;
			robot_Mode.shootMode  = S_SWING;
			robot_Mode.transMode  = T_SWING;
		}
	}//LS_UP
}
/**
 * @brief 摩擦轮温度保护，如果电机温度大于90度，让摩擦轮零电流
 * 
 */
static void FricTempProtect(void)
{
	if(average_temperture>90) 
	{
		robot_Mode.shootMode=S_NULL;
		robot_Mode.transMode=T_NULL;
	}
}

/**
  * @brief          模式控制函数，放到每一个task中
  * @param[in]      none
  * @retval         none
  */
void ModeSwitch(void)
{
	UI_API();//给底盘c板传需要显示在UI上的信息
	//摩擦轮温度保护，如果电机温度大于90度，让摩擦轮零电流
	FricTempProtect();
	//红点激光控制
	laser_ctrl();
	//如果DBUS帧率正常
	if(Is_DBUS_Ok())
	{
		DBUS_Ctrl();
	}//Is_DBUS_Ok()
	else
	{
		ModeInit();
	}
}

