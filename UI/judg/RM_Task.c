#include "RM_Task.h"
#include "bsp_can.h"
#include "myTask03.h"

extern ext_game_robot_HP_t Judge_GameRobotSurvivors;//机器人血量
extern int JudgeReceivedNewData;
extern uint8_t PowerReciveNewDataSignal;
extern int16_t datatype;
extern uint8_t ToJudgeMask1;
extern UI_RX uiRx;
extern uint8_t fric_change_mode_flag;
uint8_t candata[8];
wl4data  data4bytes;
wl2data  data2bytes;

int aaaa=0;
void Start_UI_Task(void)
{
	static uint8_t blinkCount = 0;
	static uint8_t isInit = 0;
	static uint16_t fric_last_mode;
	static uint8_t fric_mode_count=5;
	
	if(uiRx.UI_sync_flag == 1 || isInit < 30)
	{
		osDelay(10);
		Crosshair(1);//准星  //UpData0
		osDelay(15);
		Crosshair1();//下面的吊射线
		osDelay(15);
		Collision_Border();
		osDelay(15);
		DrawCapVolRectangle(uiRx.realPowData,1 , uiRx.realYawData);
		osDelay(20);
		FloatDataShow(uiRx.realPitData, uiRx.realPowData, uiRx.realYawData,uiRx.realfricSpdData,uiRx.realfricTempData,1, 1);
		//osDelay(10);
		//CharacterCHASSIS_MODE(290, 840, 1);  //UpData4
		//osDelay(20);
		//CharacterCHASSISMODstateShow(350, 800, 1);  //csadd: 
		//osDelay(10);
		//CharacterGIMBAL_MODE(290, 750, 1);  //csadd: 云台模式  //UpData8
		//osDelay(10);
		//CharacterGIMBALstateShow(350, 710, 1);  //UpData9
		osDelay(20);
		uint16_t gimbal_x = 860;
		CharacterGimbal_NULL_Show(gimbal_x, 880, 1);
		osDelay(15);
		CharacterGimbal_MNAUALShow(gimbal_x-100, 880, 1);
		osDelay(15);
		CharacterGimbal_HOIST_Show(gimbal_x-200, 880, 1);
		osDelay(15);
		CharacterGimbal_FULL_VISION_Show(gimbal_x-350, 880, 1);
		osDelay(15);
		CharacterGimbal_HALF_VISION_Show(gimbal_x-500, 880, 1);
		osDelay(15);
		uint16_t chassis_x = 1000;
		Character_CHASSIS_NULL_Show(chassis_x, 880, 1);
		osDelay(15);
		Character_CHASSIS_FOLLOW_Show(chassis_x+100, 880, 1);
		osDelay(15);
		Character_CHASSIS_TOP_Show(chassis_x+220, 880, 1);
		osDelay(15);
		Character_CHASSIS_STOP_Show(chassis_x+300, 880, 1);
		osDelay(15);
		Character_CHASSIS_APART_Show(chassis_x+390, 880, 1);
		osDelay(15);
		CharacterFricSPEEDShow(1560, 680, 1);
		osDelay(15);
		CharacterFricTEMPShow(1560, 580, 1);
		osDelay(15);
		CharacterFRIC_MODEShow(290, 660, 1); //弹仓状态  //UpData2
		osDelay(20);
		CharacteFRIC_stateShow(350, 620, 1);  //UpData3
		osDelay(20);
		CharacterPitchShow(1100, 600, 1); // x坐标，y坐标，操作类型  //UpData1
		osDelay(20);
		Draw_CIRCLE_mode(1);
		++isInit;
	}
	else
	{
		
		FloatDataShow(uiRx.realPitData, uiRx.realPowData, uiRx.realYawData,uiRx.realfricSpdData,uiRx.realfricTempData,2, 2);
		osDelay(20);
		
		DrawCapVolRectangle(uiRx.realPowData,2 , uiRx.realYawData);
		osDelay(20);
		if(fric_change_mode_flag || uiRx.firc_error)
		{
			osDelay(15);
			CharacteFRIC_stateShow(350, 620, 2);
			osDelay(15);
		}
		Draw_CIRCLE_mode(2);
		osDelay(20);
	}
	fric_last_mode = uiRx.fricMode;
}

void Start_RM_Task(void){
    if(1){//JudgeReceivedNewDataSignal[13] == 1){
				//JudgeReceivedNewDataSignal[13] = 0;
				//candata[0] = Judge_ShootData.bullet_type;
				//candata[1] = Judge_ShootData.shooter_id;
				//candata[2] = Judge_ShootData.bullet_freq;
				//data4bytes.f = Judge_ShootData.bullet_speed;
				candata[3] = chas.spd.bit[0];//data4bytes.c[0];
				candata[4] = chas.spd.bit[1];//data4bytes.c[1];
				candata[5] = chas.spd.bit[2];//data4bytes.c[2];
				candata[6] = chas.spd.bit[3];//data4bytes.c[3];
				candata[7] = 0x00;
//				CAN_Send_bytes(0x112,candata);
	}
	//成为1的条件：云台通过can发送给底盘要传输的数据，后底盘通过串口发送给裁判系统，哨兵貌似没用这个
	if(JudgeSendFresh == 1){
#ifdef VERSION19
//		RobotSendMsgToClient(ToJudgeData[0],ToJudgeData[1],ToJudgeData[2],ToJudgeMask);
#endif
		JudgeSendFresh = 0;
	}
	

	if(JudgeReceivedNewData != 0)
	{
		JudgeReceivedNewData = 0;

#ifdef VERSION19
		/* 比赛状态数据 */
			/*0  未开始比赛 
			 *1  准备阶段  
			 *2  自检阶段
			 *3  5s倒计时 
			 *4  对战中
			 *5  比赛结算中  */
			if(JudgeReceivedNewDataSignal[0] == 1){
				JudgeReceivedNewDataSignal[0] = 0;
//				candata[0] = Judge_GameState.game_progress;
//				memset(&candata[1],0,7);
//				CAN_Send_bytes(0x101,candata);
			}
			/* 比赛结果数据 */
			if(JudgeReceivedNewDataSignal[1] == 1){
				JudgeReceivedNewDataSignal[1] = 0;
//				candata[0] = Judge_GameResult.winner;
//				memset(&candata[1],0,7);
//				CAN_Send_bytes(0x1002,candata);				
			}
			/* 机器人存活数据 */
			if(JudgeReceivedNewDataSignal[2] == 1){
				JudgeReceivedNewDataSignal[2] = 0;
//				if(Judge_GameRobotState.robot_id==1)
//					data2bytes.ud = Judge_GameRobotSurvivors.red_1_robot_HP;//
//				if(Judge_GameRobotState.robot_id==101)
//					data2bytes.ud = Judge_GameRobotSurvivors.blue_1_robot_HP;
//				candata[0] = data2bytes.c[0];
//				candata[1] = data2bytes.c[1];
//				memset(&candata[2],0,6);
//				CAN_Send_bytes(0x1003,candata);				
			}
			/* 场地事件数据 */
			if(JudgeReceivedNewDataSignal[3] == 1){
				JudgeReceivedNewDataSignal[3] = 0;
//				data4bytes.f = Judge_EventData.event_type;
//				candata[0] = data4bytes.c[0];
//				candata[1] = data4bytes.c[1];
//				candata[2] = data4bytes.c[2];
//				candata[3] = data4bytes.c[3];
//				memset(&candata[4],0,4);
//				CAN_Send_bytes(0x1101,candata);
			}
			/* 补给站动作标识 */
			if(JudgeReceivedNewDataSignal[4] == 1){
				JudgeReceivedNewDataSignal[4] = 0;
//				candata[0] = Judge_SupplyProjectileAction.supply_projectile_id;
//				candata[1] = Judge_SupplyProjectileAction.supply_robot_id;
//				candata[2] = Judge_SupplyProjectileAction.supply_projectile_step;
//				candata[3] = Judge_SupplyProjectileAction.supply_projectile_num;
//				memset(&candata[4],0,4);
//				CAN_Send_bytes(0x1102,candata);			
			}
			/* 请求补给站补弹 对抗赛未开放 */
			if(JudgeReceivedNewDataSignal[5] == 1){
				JudgeReceivedNewDataSignal[5] = 0;
//				candata[0] = Judge_SupplyProjectileBooking.supply_projectile_id;
//				candata[1] = Judge_SupplyProjectileBooking.supply_robot_id;
//				candata[2] = Judge_SupplyProjectileBooking.supply_num;
//				memset(&candata[3],0,5);
//				CAN_Send_bytes(0x1103,candata);		
			}
				/* 裁判系统警告信息 */
			if(JudgeReceivedNewDataSignal[6] == 1){
				JudgeReceivedNewDataSignal[6] = 0;
			}
			/* 比赛机器人状态 */
			if(JudgeReceivedNewDataSignal[7] == 1){
				JudgeReceivedNewDataSignal[7] = 0;
//				data2bytes.ud = Judge_GameRobotState.remain_HP;
//				candata[0] = data2bytes.c[0];
//				candata[1] = data2bytes.c[1];
//				candata[0] = Judge_GameRobotState.robot_id;
//				data2bytes.ud = Judge_GameRobotState.shooter_id1_17mm_speed_limit;
//				candata[1] = data2bytes.c[0];
//				candata[2] = data2bytes.c[1];
//				data2bytes.ud = Judge_GameRobotState.shooter_id1_17mm_cooling_limit;
//				candata[3] = data2bytes.c[0];
//				candata[4] = data2bytes.c[1];
//				data2bytes.ud = Judge_GameRobotState.shooter_id1_17mm_cooling_rate;
//				candata[5] = data2bytes.c[0];
//				candata[6] = data2bytes.c[1];
//				candata[7] = Judge_GameRobotState.robot_level;
//				CAN_Send_bytes(0x102,candata);	
/*----------------------------------------------------------------------------------------------*/
				candata[0] = Judge_GameRobotState.robot_id;//机器人ID：1为红1英雄，101为蓝1英雄
				candata[1] = Judge_GameRobotState.robot_level;//机器人等级
				data2bytes.ud = Judge_GameRobotState.shooter_id1_42mm_cooling_limit;;;//机器人42mm枪口热量上限
				candata[2] = data2bytes.c[0];
				candata[3] = data2bytes.c[1];
				data2bytes.ud = Judge_GameRobotState.shooter_id1_42mm_speed_limit;;//机器人42mm弹速上限
				candata[4] = data2bytes.c[0];
				candata[5] = data2bytes.c[1];
				data2bytes.ud = Judge_GameRobotState.chassis_power_limit;//机器人底盘底盘功率上限
				candata[6] = data2bytes.c[0];
				candata[7] = data2bytes.c[1];
				CAN_Send_bytes(0x103,candata);

			}
			/* 实时功率热量数据 因一个ID 的八位数据不够，故使用两个ID */
			if(JudgeReceivedNewDataSignal[8] == 1){
				JudgeReceivedNewDataSignal[8] = 0;
				data2bytes.ud =  Judge_PowerHeatData.chassis_volt;//底盘输出电压
				candata[0] = data2bytes.c[0];
				candata[1] = data2bytes.c[1];
				data2bytes.ud = Judge_PowerHeatData.chassis_current;//底盘输出电流
				candata[2] = data2bytes.c[0];
				candata[3] = data2bytes.c[1];

//				CAN_Send_bytes(0x104,candata);		
/*---------------------------------------------------------------------------*/
				data2bytes.ud =  Judge_PowerHeatData.chassis_power_buffer;//底盘缓冲能量
				candata[0] = data2bytes.c[0];
				candata[1] = data2bytes.c[1];
				data4bytes.f = Judge_PowerHeatData.chassis_power;//底盘输出功率
				candata[2] = data4bytes.c[0];
				candata[3] = data4bytes.c[1];
				candata[4] = data4bytes.c[2];
				candata[5] = data4bytes.c[3];
				data2bytes.ud = Judge_PowerHeatData.shooter_id1_42mm_cooling_heat;//42mm枪口热量
				candata[6] = data2bytes.c[0];
				candata[7] = data2bytes.c[1];
				//printf("power = %f",Judge_PowerHeatData.chassis_power);
				CAN_Send_bytes(0x105,candata);
				
			}
			/* 机器人位置信息与枪口朝向 */
//			if(JudgeReceivedNewDataSignal[9] == 1){
//				JudgeReceivedNewDataSignal[9] = 0;
//				data4bytes.f = Judge_GameRobotPos.x;
//				candata[0] = data4bytes.c[0];
//				candata[1] = data4bytes.c[1];
//				candata[2] = data4bytes.c[2];
//				candata[3] = data4bytes.c[3];
//				data4bytes.f = Judge_GameRobotPos.y;
//				candata[4] = data4bytes.c[0];
//				candata[5] = data4bytes.c[1];
//				candata[6] = data4bytes.c[2];
//				candata[7] = data4bytes.c[3];
//				CAN_Send_bytes(0x105,candata);
//				
//				data4bytes.f = Judge_GameRobotPos.z;
//				candata[0] = data4bytes.c[0];
//				candata[1] = data4bytes.c[1];
//				candata[2] = data4bytes.c[2];
//				candata[3] = data4bytes.c[3];
//				data4bytes.f = Judge_GameRobotPos.yaw;
//				candata[4] = data4bytes.c[0];
//				candata[5] = data4bytes.c[1];
//				candata[6] = data4bytes.c[2];
//				candata[7] = data4bytes.c[3];
//				CAN_Send_bytes(0x106,candata);
//			}
			/* 机器人状态增益 */
			if(JudgeReceivedNewDataSignal[10] == 1){
				JudgeReceivedNewDataSignal[10] = 0;
//				candata[0] = Judge_BuffMusk.power_rune_buff;
//				memset(&candata[1],0,7);
//				CAN_Send_bytes(0x107,candata);				
			}
						/* 电源管理供电状态 */
			if(PowerReciveNewDataSignal==1){
				PowerReciveNewDataSignal = 0;
				candata[0] = Judge_GameRobotState.mains_power_gimbal_output;
				candata[1] = Judge_GameRobotState.mains_power_chassis_output;
				candata[2] = Judge_GameRobotState.mains_power_shooter_output;
				memset(&candata[3],0,5);
				CAN_Send_bytes(0x108,candata);				
			}
			/* 空中机器人能量状态，只有空中机器人可用 */
//			if(JudgeReceivedNewDataSignal[11] == 1){
//				JudgeReceivedNewDataSignal[11] = 0;
//				candata[0] = Judge_AerialRobotEnergy.energy_point;
//				candata[1] = Judge_AerialRobotEnergy.attack_time;
//				memset(&candata[2],0,6);
//				CAN_Send_bytes(0x110,candata);				
//			}
			/* 伤害信息 */
			if(JudgeReceivedNewDataSignal[12] == 1){
				JudgeReceivedNewDataSignal[12] = 0;
//				candata[0] = Judge_RobotHurt.armor_id;
//				candata[1] = Judge_RobotHurt.hurt_type;
//				memset(&candata[2],0,6);
//				CAN_Send_bytes(0x111,candata);				
			}
//			/* 实时射击信息 */
			if(JudgeReceivedNewDataSignal[13] == 1){
				JudgeReceivedNewDataSignal[13] = 0;
//				//candata[0] = Judge_ShootData.bullet_type;
//				//candata[1] = Judge_ShootData.shooter_id;
//				//candata[2] = Judge_ShootData.bullet_freq;
				data4bytes.f = Judge_ShootData.bullet_speed;
				candata[3] = data4bytes.c[0];
				candata[4] = data4bytes.c[1];
				candata[5] = data4bytes.c[2];
				candata[6] = data4bytes.c[3];
//				candata[7] = 0x00;
				//CAN_Send_bytes(0x112,candata);
			}
			/* 剩余子弹数量信息 */
			if(JudgeReceivedNewDataSignal[14] == 1){
				JudgeReceivedNewDataSignal[14] = 0;
//				candata[0] = Judge_ShootData.bullet_type;
//				candata[1] = Judge_ShootData.bullet_freq;
//				data4bytes.f = Judge_ShootData.bullet_speed;
//				candata[2] = data4bytes.c[0];
//				candata[3] = data4bytes.c[1];
//				candata[4] = data4bytes.c[2];
//				candata[5] = data4bytes.c[3];
//				candata[6] = 0x00;
//				candata[7] = 0x00;
//				CAN_Send_bytes(0x112,candata);
			}
			/* 机器人之间通信数据 */
			if(JudgeReceivedNewDataSignal[15] == 1){
				JudgeReceivedNewDataSignal[15] = 0;
			}
#endif

	}
}
