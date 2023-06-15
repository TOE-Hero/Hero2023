#ifndef _JUDGE_H_
#define _JUDGE_H_

#include "stm32f4xx_hal.h"
#include "CRC.h"
#include <string.h>
#include <bsp_usart.h>

#define VERSION19

typedef union{
	uint8_t c[2];//2字节
	int16_t d;
	uint16_t ud;
}wl2data;

typedef union{
	uint8_t c[4];
	float f;
	uint32_t ud;
	int32_t  d;
}wl4data;

extern int JudgeSendFresh;

#ifdef VERSION19
//0x0001	比赛状态数据
typedef __packed struct{ 
	uint8_t game_type : 4; /*1 机甲大师赛   
												  *2  单项赛   
												  *3  ICRA	*/
	uint8_t game_progress : 4;/*0  未开始比赛 
														 *1  准备阶段  
														 *2  自检阶段
														 *3  5s倒计时 
														 *4  对战中
														 *5  比赛结算中  */
	uint16_t stage_remain_time; /* 当前阶段剩余时间 s*/
}ext_game_state_t;

//0x0002	比赛结果数据
typedef __packed struct{ 
	uint8_t winner; /* 0 平局
									 * 1 红方胜利
									 * 2 蓝方胜利 */
}ext_game_result_t;

//0x0003	机器人存活数据
typedef __packed struct{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;  
	uint16_t red_3_robot_HP; 
	uint16_t red_4_robot_HP;   
	uint16_t red_5_robot_HP; 
	uint16_t red_7_robot_HP; 
	uint16_t red_base_HP;  
	uint16_t blue_1_robot_HP;  
	uint16_t blue_2_robot_HP;   
	uint16_t blue_3_robot_HP;   
	uint16_t blue_4_robot_HP;  
	uint16_t blue_5_robot_HP; 
	uint16_t blue_7_robot_HP;   
	uint16_t blue_base_HP; 
} ext_game_robot_HP_t;  /* 红方 位0开始 英雄、工程、步兵1、步兵2、步兵3、哨兵、基地
												 * 蓝方 位8开始 英雄、工程、步兵1、步兵2、步兵3、哨兵、基地 */


//0x0101	场地事件数据
typedef __packed struct{
	uint32_t event_type; /* 无用 如需要，查手册 */
}ext_event_data_t;

//0x0102	补给站动作标识
typedef __packed struct { 
	uint8_t supply_projectile_id;/* 补给站ID    
															  * 1：1号补给口 
															  * 2：2号补给口 */
	uint8_t supply_robot_id;/* 补弹机器人ID   
													 * 0 ： 无机器人补弹 
													 * 1 ： 红英雄
													 * 2 ： 红工程
													 * 3/4/5 ： 红步兵
													 * 11 ： 蓝英雄
													 * 12 ： 蓝工程
													 * 13/14/15 ： 蓝步兵		*/
	uint8_t supply_projectile_step;/* 出弹口开闭状态 
																  * 0：关闭
																  * 1：子弹准备中
																  * 2：子弹下落 */
	uint8_t supply_projectile_num;  /* 补弹数量  50/100/150/200 */
} ext_supply_projectile_action_t;

//0x0103	请求补给站补弹 对抗赛未开放
typedef __packed struct {
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_num; 
} ext_supply_projectile_booking_t;

//0x104 警告信息
typedef __packed struct { 
	uint8_t level;    /* 等级 */
	uint8_t foul_robot_id;  /* 犯规机器人ID */
} ext_referee_warning_t;

//0x0201  比赛机器人状态
typedef __packed struct {
	uint8_t robot_id;/* 机器人ID 
									  * 1： 红英雄
									  * 2： 红工程
									  * 3/4/5：红步兵
									  * 6： 红空中
									  * 7： 红哨兵
										  * 11：蓝英雄
										  * 12：蓝工程
										  * 13/14/15：蓝步兵		
										  * 16  蓝空中
										  * 17  蓝哨兵*/
	uint8_t robot_level; /* 机器人等级 1/2/3*/
	uint16_t remain_HP; /* 机器人剩余血量 */
	uint16_t max_HP; /* 机器人上限血量 */
	uint16_t shooter_id1_17mm_cooling_rate; /* 机器人17mm枪口每秒冷却值 */
	uint16_t shooter_id1_17mm_cooling_limit; /* 机器人17mm枪口热量上限 */
	uint16_t shooter_id1_17mm_speed_limit;	/* 机器人17mm枪口速度上限 */
	uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;
  uint16_t chassis_power_limit;
	uint8_t mains_power_gimbal_output : 1; /* 云台电源输出情况 */
	uint8_t mains_power_chassis_output : 1; /* 底盘电源输出情况 */
	uint8_t mains_power_shooter_output : 1; /* 发射摩擦轮电源输出情况 */
} ext_game_robot_state_t;


typedef __packed struct
{
 uint16_t chassis_volt;/* 底盘输出电压 mV */
 uint16_t chassis_current;/* 底盘输出电流 mA*/
 float chassis_power;/* 底盘输出功率 W */
 uint16_t chassis_power_buffer;/* 底盘功率缓冲 J */
 uint16_t shooter_id1_17mm_cooling_heat;/* 17mm id1枪口热量 */
 uint16_t shooter_id2_17mm_cooling_heat;/* 17mm id2枪口热量 */
 uint16_t shooter_id1_42mm_cooling_heat;/* 42mm枪口热量 */
} ext_power_heat_data_t;

//0x0203 机器人位置以及枪口朝向角度
typedef __packed struct {
	float x; 
	float y; 
	float z; 
	float yaw; 
} ext_game_robot_pos_t;

//0x0204 机器人增益
typedef __packed struct { 
	uint8_t power_rune_buff; /* bit0  补血状态 
													  * bit1  热量冷却加倍
													  * bit2  防御加成
													  * bit3  攻击加成 
													  * 其余保留 */
}ext_buff_musk_t;

//0x0205 空中机器人能量状态 只有空中机器人能够接收
typedef __packed struct {
	uint8_t energy_point; /* 积累的能量 */
	uint8_t attack_time; /* 剩余可攻击时间 */
}aerial_robot_energy_t;

//0x0206 伤害状态
typedef __packed struct {
	uint8_t armor_id : 4; /* 装甲伤害时为装甲ID 其余为0 */
	uint8_t hurt_type : 4; /* 0 装甲伤害 
												  * 1 模块离线扣血 
												  * 2 枪口超射速扣血 
													* 3 枪口超热量扣血
												  * 4 底盘超功率扣血
													* 5 装甲撞击扣血*/
} ext_robot_hurt_t;

//0x0207 实时射击信息
//typedef __packed struct {
//	uint8_t bullet_type; /* 子弹类型 17mm为1 42mm为2*/
//	uint8_t bullet_freq; /* 射频 */
//	float bullet_speed; /* 射速 */
//} ext_shoot_data_t;
typedef __packed struct
{
 uint8_t bullet_type;/* 子弹类型 17mm为1 42mm为2*/
 uint8_t shooter_id;/*发射机构 ID： 1：1 号 17mm 发射机构  2：2 号 17mm 发射机构*/
 uint8_t bullet_freq;/* 射频  单位Hz*/
 float bullet_speed; /* 射速  单位m/s */
} ext_shoot_data_t;
//0x208	子弹剩余可发射数量（只限无人机与哨兵可用）
typedef __packed struct {   
	uint16_t bullet_remaining_num;  /* 剩余数量 */ 
} ext_bullet_remaining_t;

//0x0301 机器人间交互数据
typedef __packed struct { 
	uint16_t data_cmd_id; /* 数据段的内容ID */
	uint16_t send_ID; /* 发送者的ID 若红1， 则ID为 1*/
	uint16_t receiver_ID; /* 接受者的ID */
}ext_student_interactive_header_data_t;

//客户端信息 内容ID 0xD180
/*发送客户端信息时，内容ID为0xD180 
										发送者ID 为发送的机器人ID 
										接受者的ID 为机器人对应客户端的ID*/
typedef __packed struct{
	float data1; /* 自定义浮点数据 1 */ 
	float data2; /* 自定义浮点数据 2 */
	float data3; /* 自定义浮点数据 3 */
	uint8_t masks; /* 自定义八位数据 位0-5 分别控制数据面板的六个指示灯 */
} client_custom_data_t;

typedef __packed struct{
	uint8_t operate_tpye;/* 操作类型 */
	uint8_t graphic_tpye;/* 图形类型 */
	uint8_t graphic_name[5];/* 图像名称 */
	uint8_t layer;/* 图层 */
	uint8_t color;/* 颜色 */
	uint8_t width;/* 线宽 */
	uint16_t start_x;/* 起始点X */
	uint16_t start_y;/* 起始点Y */
	uint16_t radius;/* 半径 */
	uint16_t end_x;/* 终止点X */
	uint16_t end_y;/* 终止点Y */
	int16_t start_angle;/* 起始角度 */
	int16_t end_angle;/* 终止角度 */
	uint8_t text_lenght;/* 文本长度 */
	uint8_t text[30];/* 文本字符 */
} ext_client_graphic_draw_t;

typedef __packed struct
{
uint8_t graphic_name[3];
uint32_t operate_tpye:3;
uint32_t graphic_tpye:3;
uint32_t layer:4;
uint32_t color:4;
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10;
uint32_t start_x:11;
uint32_t start_y:11;
uint32_t radius:10;
uint32_t end_x:11;
uint32_t end_y:11;
} graphic_data_struct_t;

typedef __packed struct {
    graphic_data_struct_t grapic_data_struct;
    uint8_t data[30];
} ext_client_custom_character_t;

typedef __packed struct {
    graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

typedef __packed struct
{
 	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

typedef __packed struct
{
 	graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;



//机器人之间通信数据 
/* 发送机器人之间通信信息时， 内容ID 为0x0200-0x02FF 
															发送者ID 为发送机器人的ID 
															接受者ID 为接受机器人的ID 
															数据段的字节数要小于113 */
typedef __packed struct
{
	uint8_t data[112];
} robot_interactive_data_t;


extern ext_game_state_t 														Judge_GameState;
extern ext_game_result_t     												Judge_GameResult;
extern ext_game_robot_HP_t 									Judge_GameRobotSurvivors;

extern ext_event_data_t 														Judge_EventData;
extern ext_supply_projectile_action_t     					Judge_SupplyProjectileAction;
extern ext_supply_projectile_booking_t 							Judge_SupplyProjectileBooking;

extern ext_game_robot_state_t												Judge_GameRobotState;
extern ext_power_heat_data_t												Judge_PowerHeatData;
extern ext_game_robot_pos_t													Judge_GameRobotPos;
extern ext_buff_musk_t															Judge_BuffMusk;
extern aerial_robot_energy_t												Judge_AerialRobotEnergy;
extern ext_robot_hurt_t															Judge_RobotHurt;
extern ext_shoot_data_t  														Judge_ShootData;
extern ext_student_interactive_header_data_t				Judge_StudentInfoHeader;
extern client_custom_data_t													Judge_ClientData;
extern robot_interactive_data_t  										Judge_RobotInterfaceData;
extern ext_client_graphic_draw_t										Judge_ClientGraphicDraw;
extern ext_referee_warning_t												Judge_RefereeWarning;
extern ext_bullet_remaining_t												Judge_BulletRemaining;

extern float ToJudgeData[3];
extern uint8_t ToJudgeMask;
extern uint8_t JudgeReceivedNewDataSignal[16];
extern int32_t fps;

void CheckID(void);
void RobotSendMsgToClient(float data1,float data2,float data3,uint8_t mask);
void JudgeData(uint8_t data);
void Crosshair(uint8_t operate_type);
void Crosshair1(void);
void Collision_Border(void);

void CharacterPitchShow(uint32_t start_x,uint32_t start_y,uint8_t operate_tpye);
void CharacterFRIC_MODEShow(uint32_t start_x,uint32_t start_y,uint8_t operate_tpye);
void CharacterCHASSIS_MODE(uint32_t start_x,uint32_t start_y,uint8_t operate_tpye);
void CharacteFRIC_stateShow(uint32_t start_x,uint32_t start_y,uint8_t operate_tpye);
void AddFloatdata(uint16_t name , uint8_t i,uint8_t operate_type,uint32_t graphic_tpye,uint8_t layer,uint32_t color,uint32_t start_angle,uint32_t end_angle,uint32_t width,uint32_t start_x,uint32_t start_y);
void CharacterCapvoltShow(uint32_t start_x,uint32_t start_y,uint8_t operate_tpye);
void CharacterCHASSISMODstateShow(uint32_t start_x,uint32_t start_y,uint8_t operate_tpye);
void FloatDataShow(int32_t data1, int32_t data2, int32_t data3,int32_t data4,int32_t data5,uint8_t operate_type1,uint8_t operate_type2);
void CharacterGIMBAL_MODE(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);
void CharacterGIMBALstateShow(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);

void DrawCapVolRectangle(int32_t capVol,uint8_t operate_tpye,int16_t yaw_data);

void CharacterGimbal_MNAUALShow(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);
void CharacterGimbal_NULL_Show(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);
void CharacterGimbal_HOIST_Show(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);
void CharacterGimbal_FULL_VISION_Show(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);
void CharacterGimbal_HALF_VISION_Show(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);

void Character_CHASSIS_NULL_Show(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);
void Character_CHASSIS_FOLLOW_Show(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);
void Character_CHASSIS_TOP_Show(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);
void Character_CHASSIS_STOP_Show(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);
void Character_CHASSIS_APART_Show(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);

void CharacterFricSPEEDShow(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);
void CharacterFricTEMPShow(uint32_t start_x, uint32_t start_y, uint8_t operate_tpye);

void Draw_CIRCLE_mode(uint8_t operate_tpye);
#endif
/***********************************************************************************************/
/***********************************************************************************************/
/***********************************************************************************************/
/***********************************************************************************************/
/***********************************************************************************************/

extern int imu_receive;

#endif


