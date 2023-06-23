#include "Print_task.h"
#include <main.h>
#include "bsp_delay.h"
#include "dji_motor.h"
#include "bsp_can.h"
#include "bsp_remote.h"
#include "bsp_usart.h"
#include "pid.h"
#include "STMGood.h"
#include "INS_task.h"
#include "printf.h"
#include "usbd_cdc_if.h"

#include "Hero_control.h"
/*********************************************************** Debug ********************************************************************/
/*********************** PC **********************/
#define PC_DATA_DEBUG                 0
#define PIT_OFFSET_DEBUG              0
#define YAW_PRE_WAVE                  0
#define BALL_SPEED					  0
/********************* Gimbal ********************/

/******************** Chassic ********************/
#define CAP_VOL_DEBUG                 0
#define CHASSIS_MOTOR_CURRENT         0
#define CHASSIS_POWER_BUFFER          0
/********************* Fire **********************/
#define SPEED_CLOSED_LOOP_DEBUG       0
#define FRIC_BACK_SPEED_DEBUG         0
/********************** IMU **********************/
#define IMU_YAW_ANG_DEBUG             0
#define IMU_YAW_V_DEBUG               0
#define IMU_PIT_ANG_DEBUG             0
#define IMU_PIT_V_DEBUG               0
#define IMU_ROLL_ANG_DEBUG            0
#define IMU_ROLL_V_DEBUG              0
/********************** FPS **********************/
#define FPS_DEBUG                     0

/***************************************************** extern declaration *************************************************************/

/********************** PC **********************/
extern double              H;
extern float               dtaX;
extern float               visX;
extern float               yawSpd_buff[30];
extern float	           visImu;
extern fp32                gyro_offset[3];
extern uint8_t             pcTxData[2];
extern visInf_t            s_visionInform;//自瞄结构体
extern su_PC_DATA          pcData;
extern uint16_t 		   Final_rotate_times, Final_rotate_times_inv;
/********************* Gimbal ********************/
extern float               pit_offset_ang;
extern float	 	       RC_accumulate_ch_1;
extern float               RC_accumulate_ch_1_gyro;
extern float               Gyro_up_lim;
extern float               Gyro_down_lim;
extern float               Gyro_mid;
extern s_motor_data_t      YAW_motor;//YAW轴电机信息结构体
extern s_motor_data_t      YAW_motor_imu;//YAW轴用陀螺仪完全控制结构体
extern s_motor_data_t      YAW_motor_encode;//YAW用编码器值控制位置环，陀螺仪角速度控制速度环结构体
extern s_pid_absolute_t    YAW_motor_pid_pos;//云台pit轴双环
extern s_pid_absolute_t    YAW_motor_pid_speed;
extern s_pid_absolute_t    YAW_motor_pid_pos_imu;//云台yaw轴陀螺仪双环
extern s_pid_absolute_t    YAW_motor_pid_speed_imu;
extern s_pid_absolute_t    YAW_motor_pid_pos_encode;//YAW轴编码器控制PID位置环结构体
extern s_pid_absolute_t    YAW_motor_pid_speed_encode;//YAW轴编码器控制PID速度环结构体
extern s_pid_absolute_t    YAW_motor_pid_pos_imu_vis;
extern s_pid_absolute_t    YAW_motor_pid_speed_imu_vis;

extern s_motor_data_t      PIT_motor;//PIT轴电机信息结构体
extern s_motor_data_t      PIT_motor_imu;//PIT轴用陀螺仪完全控制结构体
extern s_pid_absolute_t    PIT_motor_pid_pos;//云台pit轴双环
extern s_pid_absolute_t    PIT_motor_pid_speed;
extern s_pid_absolute_t    PIT_motor_pid_pos_imu;
extern s_pid_absolute_t    PIT_motor_pid_speed_imu;
extern s_pid_absolute_t    PIT_motor_pid_pos_imu_vis;
extern s_pid_absolute_t    PIT_motor_pid_speed_imu_vis;
/******************** Chassic *********************/
extern chassisMove_t 	   s_chassisMove;//底盘运动结构体，包括超级电容
extern s_motor_data_t      LF_motor;
extern s_motor_data_t      RF_motor;
extern s_motor_data_t      LB_motor;
extern s_motor_data_t      RB_motor;
extern s_pid_absolute_t    LF_motor_pid_stop_pos;
extern s_pid_absolute_t    RF_motor_pid_stop_pos;
extern s_pid_absolute_t    LB_motor_pid_stop_pos;
extern s_pid_absolute_t    RB_motor_pid_stop_pos;
extern s_pid_absolute_t    LF_motor_pid_stop_speed;
extern s_pid_absolute_t    RF_motor_pid_stop_speed;
extern s_pid_absolute_t    LB_motor_pid_stop_speed;
extern s_pid_absolute_t    RB_motor_pid_stop_speed;
/******************** Fire *************************/
extern s_motor_data_t      FIRE_L_motor;
extern s_motor_data_t      FIRE_R_motor;
/********************* IMU *************************/
extern s_IMU_all_Value     IMU_All_Value;
/********************* FPS *************************/
extern s_FPS_monitor	   finalFps;//每秒频率结构体
extern s_FPS_monitor	   Fps;//每秒频率结构体
/******************** Judge *************************/
extern int16_t             Fric_SpeedTarget;//裁判系统弹速
/******************** Module ***********************/
extern s_robo_Mode_Setting robot_Mode;//机器人模式结构体内嵌枚举
/******************** STMGood **********************/
extern double	           P1,I1,D1,S1, \
                           P2,I2,D2,S2, \
				           P3,I3,D3,S3, \
				           P4,I4,D4,S4, \
				           P5,I5,D5,S5, \
				           P6,I6,D6,S6, \
				           P7,I7,D7,S7, \
				           P8,I8,D8,S8, \
				           P9,I9,D9,S9;
extern int8_t g_gyro_flag;
extern int8_t deafult_mode_flag;
extern u_data_32bit bullet_speed;
extern uint16_t shot_ball_amount;
extern uint8_t trans_can_state;
/*******************************************************************************/
typedef struct
{
	union
	{
		int16_t integer;
		uint8_t arr[2];
	}motor_temp;
	union
	{
		int16_t integer;
		uint8_t arr[2];
	}motor_back_spd;
	union
	{
		int16_t integer;
		uint8_t arr[2];
	}motor_target_spd;
	union
	{
		uint32_t u_integer;
		uint8_t arr[4];
	}ball_spd;
}s_pc_ball_speed;

s_pc_ball_speed pc_ball_speed;
uint8_t pcdata_ball_speed[10];

#define PRINT_FREQUENCY    20//单位（Hz）.任务延时也就是1000/PRINT_FREQUENCY,注意要无符号整型

/**
  * @brief          PrintDebug任务,UART1
  * @param[in]      
  * @retval        
  */
void Print_task(void const * argument)
{
	
	while(1)
	{
	ModeSwitch();//模式选择
	osDelay((uint16_t)(1000/PRINT_FREQUENCY));//打印频率
	
	//printf("%s\r\n","HERO_test");

	// printf("%d\r\n",trans_can_state);
/********************************************************** Judge ************************************************************/
	//printf("%d\r\n",JudgeGameState.arr2[0]);
	//printf("%d\r\n",g_gyro_flag);
	//printf("%d\r\n",deafult_mode_flag);
	//printf("%d\r\n",shot_ball_amount);
	
	//printf("%d\r\n",Fric_SpeedTarget);
/************************************************************ PC ************************************************************/
		{
			// printf("%d,%d\n",Final_rotate_times,Final_rotate_times_inv);
			//printf("%f  %f   %f\r\n",visX,s_visionInform.yawRelV,s_visionInform.yawPre);
			//printf("%f\r\n",visTarget_t.visYawTarget);
			//printf("%f\r\n",s_visionInform.distance);
			//printf("%d\r\n",Fps.pc);
			//printf("%d   %f\r\n",visTarget_t.visPitTarget,PIT_motor.out_current);
			//printf("%f %f %f %d %f\r\n",pcData.x.data,YAW_motor_pid_pos_imu_vis.NowError,s_visionInform.yawRelV,finalFps.pc,s_visionInform.distance);
		#if YAW_PRE_WAVE == 1
			printf("%f,%f,%f\n",visX,yawSpd_buff[14],yawSpd_buff[11]);
		#endif
		#if PC_DATA_DEBUG == 1
			printf("%f\r\n %f\r\n %f\r\n \r\n",pcData.x.data,pcData.y.data,pcData.z.data);
		#endif
		#if PIT_OFFSET_DEBUG == 1
			printf("H=%lf , \
					s_visionInform.distance=%lf, \
					visTarget_t.visPitTarget%lf\r\n, \
					\r\n" , \
					H, \
					s_visionInform.distance, \
					visTarget_t.visPitTarget);
		#endif
		#if BALL_SPEED == 1
			pc_ball_speed.ball_spd.u_integer=(uint16_t)(bullet_speed.f*10);
			//printf("%d\r\n",pc_ball_speed.ball_spd.u_integer);
			pc_ball_speed.motor_back_spd.integer=abs(FIRE_L_motor.back_motor_speed);
			pc_ball_speed.motor_target_spd.integer=abs(FIRE_L_motor.target_motor_speed);
			pc_ball_speed.motor_temp.integer=FIRE_L_motor.temperature;
			pcdata_ball_speed[0]=0xA7;
			pcdata_ball_speed[1]=0;
			pcdata_ball_speed[2]=pc_ball_speed.ball_spd.arr[0];
			pcdata_ball_speed[3]=pc_ball_speed.ball_spd.arr[1];
			pcdata_ball_speed[4]=pc_ball_speed.motor_target_spd.arr[0];
			pcdata_ball_speed[5]=pc_ball_speed.motor_target_spd.arr[1];
			pcdata_ball_speed[6]=pc_ball_speed.motor_back_spd.arr[0];
			pcdata_ball_speed[7]=pc_ball_speed.motor_back_spd.arr[1];
			pcdata_ball_speed[8]=pc_ball_speed.motor_temp.arr[0];
			pcdata_ball_speed[9]=pc_ball_speed.motor_temp.arr[1];
			CDC_Transmit_FS(pcdata_ball_speed,sizeof(pcdata_ball_speed));
		#endif
		}
/*********************************************************** Firc ***********************************************************/
		{
		#if FRIC_BACK_SPEED_DEBUG == 1
			printf("%d,%f\n",FIRE_L_motor.back_motor_speed,FIRE_L_motor_pid_speed.PIDout);
		#endif
		#if SPEED_CLOSED_LOOP_DEBUG == 1
			printf("%d  %d\r\n \
					%d  %d\r\n \r\n", \
					FIRE_L_motor.temperature, \
					FIRE_R_motor.temperature, \
					FIRE_L_motor.back_motor_speed, \
					FIRE_R_motor.back_motor_speed);
		#endif
		#if SPEED_CLOSED_LOOP_DEBUG == 1
		printf("%f\r\n%d %d\r\n%f  %f\r\n \r\n",bullet_speed.f,FIRE_L_motor.temperature, \
					FIRE_R_motor.temperature,FIRE_L_motor.target_motor_speed, \
					FIRE_R_motor.target_motor_speed);
		#endif

		//printf("%d\r\n",TRANS_motor.coolingheat_limit);
		// printf("%d  %d %f\r\n",TRANS_motor.out_current,TRANS_motor.back_current,TRANS_motor_pid_pos.NowError);
		
		}
/********************************************************** Gimbal *********************************************************/
		{
			// printf("%f   %d   %f\r\n",PIT_motor.serial_motor_ang*100, \
			// 						YAW_motor.back_position - 1253, \
			// 						bullet_speed.f);
			// printf("%f   %d   %f %f\r\n",visTarget_t.visYawTarget,	YAW_motor.out_current, \
			// 													YAW_motor_pid_pos_imu_vis.PIDout, \
			// 													YAW_motor_pid_speed_imu_vis.PIDout);
			// //printf("%f   %f   \r\n",PIT_motor_pid_pos.PIDout,PIT_motor_pid_speed.PIDout);
			//printf("%f\r\n",visTarget_t.visPitTarget);
			//printf("%d  \r\n",PIT_motor.back_current);
			//printf("%d  %d\r\n",PIT_motor.back_position,YAW_motor.back_position);
			//printf("PID_POS=%f  ,  PID_SPD=%f\n",PIT_motor_pid_pos.PIDout,PIT_motor_pid_speed.PIDout);
			//printf("%f  %d  %f\r\n",PIT_motor_pid_pos.NowError,PIT_motor.back_motor_speed,RC_accumulate_ch_1);
			//printf("%f  %f  %f\r\n",PIT_motor_pid_pos.NowError,YAW_motor_pid_pos_imu.NowError,YAW_motor_pid_speed_imu.Ierror);
			//printf("%f  %f\r\n",PIT_motor_pid_pos.NowError,YAW_motor_pid_pos_encode.NowError);
			//printf("%lf,%lf,%d\r\n", PIT_motor_pid_pos.NowError,PIT_motor.target_pos,PIT_motor.back_position);
			//printf("%lf,%lf,%f\r\n", PIT_motor_pid_pos_imu.NowError,PIT_motor_imu.target_ang_imu,IMU_All_Value.pit.pitAng);
			
			//printf("%f %f %f %d\r\n", pit_offset_ang,IMU_All_Value.pit.pitAng,(((float)PIT_motor.back_position-2715)*360)/8192/3.4,PIT_motor.back_position);
			//printf("%d   %lf\r\n", YAW_motor.back_position,IMU_All_Value.pit.pitAng);
			//printf("%f %f %f %d\r\n",PIT_motor_imu.target_ang_imu,PIT_motor_pid_speed_imu.NowError,PIT_motor_pid_speed_imu.Iout,PIT_motor.out_current);
			//printf("%f %f %lf\r\n",Gyro_up_lim,Gyro_down_lim,Gyro_mid);
			//printf("%d   %lf\r\n", PIT_motor.back_position,IMU_All_Value.pit.pitAng);
			// printf("%d  %f  %d  %lf\r\n",YAW_motor.back_position,YAW_motor_pid_pos_encode.NowError, \
			// 								YAW_motor.out_current,YAW_motor_pid_speed_encode.NowError);
			// printf("%d  %f  %d  %lf\r\n",YAW_motor.back_position,YAW_motor_pid_pos_imu.NowError, \
			// 								YAW_motor.out_current,YAW_motor_pid_speed_imu.NowError);
		}
/********************************************************* Chassis ********************************************************/
		{
		#if CAP_VOL_DEBUG == 1
			printf("capVol=%f  CAPexpect=%f  curSum=d\r\n",s_chassisMove.capVol,s_chassisMove.cap_expect,s_chassisMove.cur_sum);
		#endif
		// printf("%d\r\n",s_chassisMove.W);
		//printf("power_buffer=%d\r\ncapVol=%f\r\n",s_chassisMove.chassis_power_buffer, s_chassisMove.capVol);
		// printf("%lf,%d\r\n",s_chassisMove.capVol,cap_precentV.integer);
		#if CHASSIS_MOTOR_CURRENT == 1
			printf("LF=%d,RF=%d,LB=%d,RB=%d\r\n LFO=%d,RFO=%d,LBO=%d,RBO=%d\r\n",	LF_motor.back_current, \
																					RF_motor.back_current, \
																					LB_motor.back_current, \
																					RB_motor.back_current, \
																					LF_motor.out_current, \
																					RF_motor.out_current, \
																					LB_motor.out_current, \
																					RB_motor.out_current);
		#endif
		#if CHASSIS_POWER_BUFFER == 1
			printf("power_buffer=%d J\r\n",s_chassisMove.chassis_power_buffer);
		#endif

		}
/********************************************************** Fps **********************************************************/
		{
		#if FPS_DEBUG == 1
			printf("%d %d %d %d %d %d %d %d %d %d %d %d %d\r\n", \
													finalFps.pitch,	\
													finalFps.RF_motor, \
													finalFps.LF_motor, \
													finalFps.LB_motor, \
													finalFps.RB_motor, \
													finalFps.yaw, \
													finalFps.trans,	\
													finalFps.fric_l, \
													finalFps.fric_r, \
													finalFps.dbus, \
													finalFps.cap_board, \
													finalFps.pc, \
													finalFps.judge);
		#endif
		//printf("%d    %f\r\n", finalFps.pc,s_visionInform.distance);
		}
/********************************************************** IMU ***********************************************************/
		{
			#if IMU_YAW_ANG_DEBUG == 1
				printf( "YAW_ang=%f \r\n  ",IMU_All_Value.yaw.yawAng);
			#endif 
			#if IMU_YAW_V_DEBUG == 1
				printf( "YAW_v=%f  ",IMU_All_Value.yaw.yawAngV);
			#endif
			#if IMU_PIT_ANG_DEBUG == 1
				printf( "PIT_ang=%f  ",IMU_All_Value.pit.pitAng);
			#endif
			#if IMU_PIT_V_DEBUG == 1
				printf( "PIT_ang=%f  ",IMU_All_Value.pit.pitAng);
			#endif
			#if IMU_ROLL_ANG_DEBUG == 1
				printf( "PIT_ang=%f  ",IMU_All_Value.roll.rollAng);
			#endif
			#if IMU_ROLL_V_DEBUG == 1
				printf( "PIT_ang=%f  ",IMU_All_Value.roll.rollAngV);
			#endif
			//printf("%f,%f,%f\r\n",gyro_offset[0],gyro_offset[1],gyro_offset[2]);
		}

/**************************************************************************************************************************/
	}

}

