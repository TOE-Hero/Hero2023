/**
 ******************************************************************************
 * @FilePath: \for_infantry\App\nuc_interface.c
 * @Brief: 
 * @Date: 2021-01-21 18:56:17
 * @Author: Rio
 ******************************************************************************
 */
#include "nuc_interface.h"
#include "usart.h"
#include "bsp_usart.h"
#include "Mode_Switch.h"
#include "Gimbal.h"
#include "user_lib.h"
#include "CRC_Check.h"
#include "kalman_filter.h"
#include "STMGood.h"
#include "Mode_Switch.h"
#include "pid.h"
#include <string.h>
#include <math.h>
#include "usbd_cdc_if.h"

#if ROBOT_ID == SUN
	#define CMERA_TO_MUZZLE         300//271.6024//摄像头到枪口真实距离，先机械测量获得，然后再细调参整定
	#define PIXEL_CONV_WORLD        0.00043//0.000412//0.00037//0.000453//像素坐标转换到世界坐标参数,(越小枪口越上抬)
	#define PIXLE_INTERCEPT_VAL     520//像素被截取的值（摄像头取流值），英雄是600
	#define BALL_SPEED              15.5//平均弹速
	#define YAW_DELTA_X             5.276//5.254//帧差乘一个系数，令幅值与陀螺仪角速度相等（陀螺仪角速度单位为°）
	#define YAW_PRE_P               30//yaw轴预测量，系数超参数获得，为了能够让云台跟上装甲板的同时再超前一些
	#define PC_FPS					200
#endif
#if ROBOT_ID == MOON
	#define CMERA_TO_MUZZLE         354.3489
	#define PIXEL_CONV_WORLD        0.00045
	#define PIXLE_INTERCEPT_VAL     600
	#define BALL_SPEED              15.0
	#define YAW_DELTA_X             6.262
	#define YAW_PRE_P               30//yaw轴预测量，系数超参数获得，为了能够让云台跟上装甲板的同时再超前一些
	#define PC_FPS					200
#endif

#define C_LANGUAGE   0
#define PY_LANGUAGE  1
#define SWQ_PY_LANGUAGE  2
//定义视觉程序用的是哪一套
#define VISION_LANGUAGE SWQ_PY_LANGUAGE

typedef struct  {
    float fdata[2];
    unsigned char tail[4];
}Frame;

Frame send;
/***************************** static declaration ******************************/
static float bullet_speed_pit_pre = 0;
/***************************** extern declaration ******************************/
extern s_motor_data_t 		YAW_motor;//YAW轴电机信息结构体
extern s_motor_data_t 		YAW_motor_imu;//YAW轴用陀螺仪完全控制结构体
extern s_motor_data_t 		YAW_motor_encode;//YAW用编码器值控制位置环，陀螺仪角速度控制速度环结构体
extern s_pid_absolute_t 	YAW_motor_pid_pos_encode;//YAW轴编码器控制PID位置环结构体
extern s_pid_absolute_t 	YAW_motor_pid_speed_encode;//YAW轴编码器控制PID速度环结构体
extern s_motor_data_t 		PIT_motor_imu;
extern s_pid_absolute_t 	PIT_motor_pid_pos_imu;
extern s_pid_absolute_t 	PIT_motor_pid_speed_imu;
extern s_robo_Mode_Setting 	robot_Mode;//机器人模式结构体内嵌枚举
extern s_FPS_monitor 		Fps;

extern visInf_t s_visionInform;//自瞄结构体
extern float	yawSpd_buff[30];//陀螺仪延迟赋值数组
extern u_data_32bit bullet_speed;

/*****************************  declaration ******************************/
float dtaX = 0.0f, visImu = 0.0f, visX;
float lastX = 0.0f, lastY = 0.0f, lastZ= 0.0f;
float lastdtaX = 0.0f;
su_PC_DATA pcData;//PC数据，将串口接收到的pc数据识别 帧头&CRC校验 并赋值
uint16_t Final_rotate_times, Final_rotate_times_inv;
/************************* fuction declaration ***************************/



/**
  * @brief          给PC发送数据（USB）（C++版）
  * @param[in]      uint8_t selectRB
  * @retval         none
  */
void sendMessageToPc(uint8_t selectRB)
{
	uint8_t pcTxBuf[6];
	pcTxBuf[0] = 0xA6;
	pcTxBuf[1] = selectRB;
	pcTxBuf[2] = 0;
	Append_CRC8_Check_Sum(pcTxBuf,4);
	pcTxBuf[4] = '\r';
	pcTxBuf[5] = '\n';
	CDC_Transmit_FS(pcTxBuf,sizeof(pcTxBuf));
}

#if VISION_LANGUAGE==SWQ_PY_LANGUAGE
uint8_t pcTxData[5];
/**
  * @brief          给PC发送数据（USB）（孙闻崎Python版）
  * @param[in]      uint8_t selectRB,红方英雄为1，蓝方英雄为101
  * @param[in]      uint8_t visionMode,0地面，3哨兵
  * @retval         none
  */
void sendMessageToPc_Python(uint8_t selectRB , uint8_t visionMode)
{
	pcTxData[0] = 0xA6;//帧头
	pcTxData[1] = selectRB;//机器人自己ID，红方英雄为1，蓝方英雄为101
	pcTxData[2] = visionMode;//0地面，3哨兵
	pcTxData[3] = '\r';
	pcTxData[4] = '\n';
	CDC_Transmit_FS(pcTxData,sizeof(pcTxData));
	
}
/**
  * @brief          处理PC传输过来的数据，为Python视觉的数据格式，放到USB接收中，480取流帧率260
  * @param[in]      su_PC_DATA *pcdata ：自定义的pc数据数组
  * @param[in]      uint8_t *visionBuf ：串口或USB获取的数据数组
  * @retval         none
  */
void DealPcData(su_PC_DATA *pcdata, uint8_t *visionBuf)
{
	
	//先把从串口或者USB获取的数据存到定义的pc共用体中
	pcdata->x.bit[0] = visionBuf[0];
	pcdata->x.bit[1] = visionBuf[1];
	pcdata->x.bit[2] = visionBuf[2];
	pcdata->x.bit[3] = visionBuf[3];

	pcdata->y.bit[0] = visionBuf[4];
	pcdata->y.bit[1] = visionBuf[5];
	pcdata->y.bit[2] = visionBuf[6];
	pcdata->y.bit[3] = visionBuf[7];

	pcdata->z.bit[0] = visionBuf[8];
	pcdata->z.bit[1] = visionBuf[9];
	pcdata->z.bit[2] = visionBuf[10];
	pcdata->z.bit[3] = visionBuf[11];
	//将存到pc共用体的数据传参到自瞄结构体中
	s_visionInform.yawPos	= pcdata->x.data;//yaw轴装甲板当前位置
	s_visionInform.pitPos	= pcdata->y.data;//pit轴装甲板当前位置
	s_visionInform.distance	= pcdata->z.data;//装甲板距离
	{//PIT-offset
		
		bullet_speed_pit_pre = bullet_speed.f;
		float k1, k2, z, y, offsetY;//作用域在这个大括号内的参数
		#if ROBOT_ID == SUN
		if(PRESS_E)
		{
			offsetY = 520;//P5;//PIXLE_INTERCEPT_VAL;//1080-摄像头取流数值（英雄取流480）
			k1 = 0.0005;//PIXEL_CONV_WORLD;//P6;//0.00039;//坐标转换系数，将像素坐标转化为实际坐标(越小枪口越上抬)
		}
		else
		{
			offsetY = 430;//520;//P5;//PIXLE_INTERCEPT_VAL;//1080-摄像头取流数值（英雄取流480）
			k1 =0.00045;// 0.0005;//PIXEL_CONV_WORLD;//P6;//0.00039;//坐标转换系数，将像素坐标转化为实际坐标(越小枪口越上抬)
		}
		#endif
		#if ROBOT_ID == MOON
		if(PRESS_E)
		{
			offsetY = 490;//520;//P5;//PIXLE_INTERCEPT_VAL;//1080-摄像头取流数值（英雄取流480）
			k1 = 0.00034;//0.0005;//PIXEL_CONV_WORLD;//P6;//0.00039;//坐标转换系数，将像素坐标转化为实际坐标(越小枪口越上抬)
		}
		else
		{
			offsetY = 420;//430;//PIXLE_INTERCEPT_VAL;//1080-摄像头取流数值（英雄取流480）
			k1 = 0.0004;//0.00045;//PIXEL_CONV_WORLD;//P6;//0.00039;//坐标转换系数，将像素坐标转化为实际坐标(越小枪口越上抬)
		}
		#endif
		k2 = CMERA_TO_MUZZLE;//P7;//摄像头与枪口的实际距离（超参数整定，前提是测量了实际值得出一个模糊值，再进行细调参）
		z = s_visionInform.distance / 100.0f;//目标的实际距离（单位:m）
		y = s_visionInform.pitPos;//纵轴像素数值，从左上角为起始为0（0~480）
		float h = 1.0 * k1 * z * (540 + k2/z - offsetY - y);//540为1080/2,目标值距离枪口的实际纵向距离
		//H=h;//是为了打印h值整定k2，将枪口与装甲板放水平，然后将k1赋值为1，接着调整k2的值直到h为0（最起码要测两个距离（z）值下的k2）
		float theta,ay,ax, t;//theta为视觉补偿角度
		if(bullet_speed_pit_pre<15.3) bullet_speed_pit_pre=15.3;
		if(bullet_speed_pit_pre>15.9) bullet_speed_pit_pre=15.9;
		float v0 = 15.5;//bullet_speed_pit_pre;//弹速，取一个概率最大的值,或者依照上一发弹丸的弹速
		t = 1.0 * z / v0 / cosf(IMU_All_Value.pit.pitAng*3.1415926/180);//通过弹速求得弹丸飞行时间
		ay = h + 0.5 * 9.8 * t * t;//目标值距离枪口的实际纵向距离与弹道下坠的加和
		ax = v0 * t;//实际上就是z
		theta = 180.0 * asinf(ay / ax);
		PIT_motor_imu.visPitTarget = theta;//pit_imu视觉目标角度（带补偿）（陀螺仪角度转换）
	}
	{//YAW-pre
		visImu = yawSpd_buff[14];//陀螺仪yaw角速度延迟赋值
		dtaX = pcdata->x.data - lastX;//帧差
		visX = 4.46252 * dtaX;//P7*dtaX;//目标像素移动速度，帧差要乘的系数要整定，为了y轴的值与陀螺仪是一样的，最终波形效果为两个sin函数重合
		//if(fabs(visX) > 500.0f)	visX = visImu;//如果帧差过大那么大概率是没识别到，所以不使用这种帧差作预测计算
		//if(fabs(visX) < 1.3f)   visX = 0;//当帧差比较小下的时候直接去掉预测
		s_visionInform.yawRelV = visX - visImu;//目标移动速度减去自身yaw轴角速度，获得yaw轴与目标的相对速度
		if(fabs(s_visionInform.yawRelV) > 300.0f) s_visionInform.yawRelV = 0;//当absV大于300时，absV=0，不做预测
		if(fabs(s_visionInform.yawRelV) > 40.0f ) s_visionInform.yawRelV = 40*s_visionInform.yawRelV / fabs(s_visionInform.yawRelV);//当absV大于40时，做循环限幅
		
		{// 判断小陀螺
			static bool rotate_flag = 0;
			static bool rotate_flag_inv = 0;
			static uint16_t rotate_times, rotate_times_inv;
			// static uint16_t Final_rotate_times, Final_rotate_times_inv;//这里改为了全局变量，方便打印看陀螺计次值
			float rotate_cali_deltaX;//敌方小陀螺移动帧差
			if(pcdata->z.data > 0)// 如果没有距离信息，滤掉
				// 距离×像素速度，为了近处和远处统一单位
				rotate_cali_deltaX = dtaX * pcdata->z.data/100.0f;
			else
				rotate_cali_deltaX = 0;
			// 如果检测到的帧差大于某值（这里的负数是为了识别反向小陀螺，不可以改为绝对值形式）
			if( rotate_cali_deltaX < -800)
			{
				// 反向小陀螺标志位置1
				rotate_flag_inv = 1;
			}
			/*如果已经检测到反向小陀螺标志位置1，并且陀螺帧差大于0<-这里的大于0，是因为顺时针小陀螺是左边装甲板消失，
			右边装甲板突然出现，所以帧差是负值，但是由于右侧突然检测到以后，会稳定识别一段时间，这段时间内装甲板是从右往左移动的，
			帧差为正值，所以要加一个判断装甲板帧差的判断*/
			if(rotate_flag_inv == 1 && rotate_cali_deltaX >=0 )
			{
				// 清除反向小陀螺标志位
				rotate_flag_inv = 0;
				// 反向小陀螺计次值连续++
				rotate_times_inv++;
			}
			// 如果检测到的帧差大于某值（这里的正数是为了识别正向小陀螺，不可以改为绝对值形式）
			if( rotate_cali_deltaX > 800)
			{
				// 正向小陀螺标志位置1
				rotate_flag = 1;
			}
			/*如果已经检测到反向小陀螺标志位置1，并且陀螺帧差小于0<-这里的小于0，是因为逆时针小陀螺是右边装甲板消失，
			左边装甲板突然出现，所以帧差是正值，但是由于左侧突然检测到以后，会稳定识别一段时间，这段时间内装甲板是从左往右移动的，
			帧差为负值，所以要加一个判断装甲板帧差的判断*/
			if(rotate_flag == 1 && rotate_cali_deltaX <=0 )
			{
				// 清除正向小陀螺标志位
				rotate_flag = 0;
				// 正向小陀螺计次值连续++
				rotate_times ++ ;
			}
			// 大概0.8s计算一次陀螺次数,根据PC帧率修改
			if(!(Fps.pc%((uint16_t)(PC_FPS*0.8f))))
			{
				// 取0.8s的陀螺次数最终值
				Final_rotate_times = rotate_times;
				Final_rotate_times_inv = rotate_times_inv;
				// 陀螺次数清零
				rotate_times = rotate_times_inv = 0;
			}
			// 如果每0.8s检测到小陀螺次数超过3次，认为敌方车是小陀螺，取消预测
			if((Final_rotate_times >= 3 || Final_rotate_times_inv >=3 ) )
			{
				s_visionInform.yawRelV = 0;
			}
			
		}
		float Ka,Kb;
		#if ROBOT_ID == SUN
		Ka = 0.0453;
		Kb = 2.4;
		#endif
		#if ROBOT_ID ==MOON
		Ka = 0.0378;
		Kb = 2.4;
		#endif
		s_visionInform.yawPre = s_visionInform.yawRelV * (s_visionInform.distance * Ka + Kb);//(P8 + s_visionInform.distance * I8);// YAW_PRE_P;//P8;//yaw轴真实速度乘系数，系数超参数获得

		/*如果底盘模式不是陀螺模式 && 目标真实速度大于10 && 目标真实速度小于13，预测值乘1.3倍系数，因为在这个区间内响应不够*/
		// if(robot_Mode.chassisMode != C_TOP && fabs(s_visionInform.yawRelV) > 10.0 && fabs(s_visionInform.yawRelV) < 13.0)
		// 	s_visionInform.yawPre *= 1.3f;
		
		//yaw轴目标装甲板实际值（像素）加预测值作为PID获得值，（最终是这个值减yaw轴中值作为位置pid解算）
		YAW_motor.visTarget = s_visionInform.yawPos + s_visionInform.yawPre;
	}
	/*为了下一次计算帧差*/
	lastX = pcdata->x.data;
	lastY = pcdata->y.data;
	lastdtaX = dtaX;
	// if((pcdata->z.data) > 30.0f && (pcdata->z.data) < 700.0f)//识别距离限幅
	// lastZ = pcdata->z.data;
	//if((pcdata->z.data) > 700.0f) pcdata->z.data = 700.0f;//7米外识别到认定在7米，为了不让pit补偿时抬枪过高看不到目标
	#endif //#if VISION_LANGUAGE==SWQ_PY_LANGUAGE
	Fps.pc++;//计算PC接收帧率
}


