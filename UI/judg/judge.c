#include "Judge.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "bsp_can.h"

#define _IS_DJI_STILL_SB_     0

#define CROSS_HAIR_LAYER    0//准星周围的线的图层
#define DYNAMIC_NUMBER_LAYER  1//动态变化数字的图层
#define CROSS1_HAIR_LAYER   2//准星周围的线的图层
#define CHASSIS_MODE_LAYER    3//底盘模式可变化字符的图层
#define CAP_V_RECTANGLE_LAYHER  4//电容电压滑动条外的矩形
#define CAP_V_LINE_LAYHER   5//电容电压滑动条内的直线
#define GIMBAL_MODE_LAYER   5//云台模式可变化字符的图层
#define FRIC_MODE_LAYER     6//摩擦轮模式可变化字符的图层

#define STATIC_CHARACTER_LAYER  8//静态不会变的字符的图层
#define BOARDER_LAYER     9//防撞条图层(其实是准星)

/******************* define **************************/
#define MOON  0x00
#define SUN   0x01

/*******************哪台英雄***************************/
#define ROBOT_ID  SUN
//#define ROBOT_ID  MOON



extern UI_RX uiRx;
uint8_t rbuff[255];
uint8_t HeaderData[20], ReceiveData[255];
uint8_t UpData[128];
uint8_t UpData0[128];
uint8_t UpData1[112];
uint8_t UpData2[112];
uint8_t UpData3[112];
uint8_t UpData4[112];
uint8_t UpData5[112];
uint8_t UpData6[112];
uint8_t UpData7[112];
uint8_t UpData8[112];
uint8_t UpData9[112];
uint8_t UpData10[112];
uint8_t UpData11[112];
uint8_t UpData12[112];
uint8_t UpData13[112];
uint8_t UpData14[120];
uint8_t UpData15[128];

int32_t fps;
int16_t datatype;
int JudgeReceivedNewData = 0;
int JudgeSendFresh = 0;
int imu_receive = 0;
char* cha[18] =
{
  "PITCH:",      "CAPVOLT:",        "CHASIS_MODE:",  "VISION_MODE:", "FRIC:    ",  //0~4
  "NULL     ",   "APART   ",        "FLLOW   ",  "TOP     ", "STOP    ",       //5~9
  "NULL       ", "MANUAL     ",     "ENCODE     ",   "FULL_VIS   ", "HALF_VIS  ",//10~14
  "OPEN!!",      "CLOSED",  //15~16
  "ERROR"//17
};

#ifdef VERSION19

int Robot_ID;
float ToJudgeData[3];
uint8_t ToJudgeMask;
uint8_t ToJudgeMask1 = 0;
uint8_t JudgeReceivedNewDataSignal[16] = {0};
uint8_t PowerReciveNewDataSignal = 0;

wl2data w2data;
wl2data w2data1;
wl4data w4data;

ext_game_state_t                  Judge_GameState;
ext_game_result_t                     Judge_GameResult;
ext_game_robot_HP_t                     Judge_GameRobotSurvivors;
ext_event_data_t                  Judge_EventData;
ext_supply_projectile_action_t              Judge_SupplyProjectileAction;
ext_supply_projectile_booking_t           Judge_SupplyProjectileBooking;

ext_game_robot_state_t                Judge_GameRobotState;
ext_power_heat_data_t               Judge_PowerHeatData;
ext_game_robot_pos_t                Judge_GameRobotPos;
ext_buff_musk_t                   Judge_BuffMusk;
aerial_robot_energy_t               Judge_AerialRobotEnergy;
ext_robot_hurt_t                  Judge_RobotHurt;
ext_shoot_data_t                    Judge_ShootData;
ext_student_interactive_header_data_t       Judge_StudentInfoHeader;
client_custom_data_t                Judge_ClientData;
robot_interactive_data_t                Judge_RobotInterfaceData;
ext_client_graphic_draw_t             Judge_ClientGraphicDraw;
ext_referee_warning_t               Judge_RefereeWarning;
ext_bullet_remaining_t                Judge_BulletRemaining;

ext_client_custom_graphic_seven_t Judge_ClientDrawSeven;
ext_client_custom_graphic_seven_t Judge_ClientDrawSeven1;
ext_client_custom_character_t Judge_ClientCharacter;
ext_client_custom_graphic_five_t Judge_ClientDrawFive;

ext_client_custom_graphic_single_t Judge_ClientDrawSingle;
ext_client_custom_graphic_double_t Judge_ClientDrawDouble;
//赋值机器人ID
void CheckID (void)
{
  Robot_ID = (int) Judge_GameRobotState.robot_id;
}
//接收裁判系统数据
void JudgeData (uint8_t data)
{
  static int HeaderIndex;
  static int dataIndex;
  static int InfoStartReceive;
  static int16_t datalength;
  static uint8_t packindex;
  if (data == 0xA5)
  {
    HeaderIndex = 1;
    HeaderData[0] = data;
    InfoStartReceive = 0;
  }
  else
  {
    if (HeaderIndex < 5)
    {
      HeaderData[HeaderIndex++] = data;
      if (HeaderIndex == 5 && Verify_CRC8_Check_Sum (HeaderData, 5))
      {
        w2data.c[0] = HeaderData[1];
        w2data.c[1] = HeaderData[2];
        datalength = w2data.d;
        packindex = HeaderData[3];
        InfoStartReceive = 1;
        dataIndex = 5;
        memcpy (ReceiveData, HeaderData, 5);
        return;
      }
    }
    if (InfoStartReceive)
    {
      if (dataIndex < datalength + 9)
      {
        //9: frame(5)+cmd_id(2)+crc16(2)
        ReceiveData[dataIndex++] = data;
      }
      if (dataIndex == datalength + 9)
      {
        //do the deal once
        InfoStartReceive = 0;
        if (Verify_CRC16_Check_Sum (ReceiveData, datalength + 9))
        {
          w2data.c[0] = ReceiveData[5];
          w2data.c[1] = ReceiveData[6];
          datatype = w2data.d;//cmd_id
          fps++;
          JudgeReceivedNewData = 1;
          switch (datatype)
          {
          case 0x0001:
          {
            JudgeReceivedNewDataSignal[0] = 1;
            memcpy (&Judge_GameState, &ReceiveData[7], sizeof (ext_game_state_t));
            break;
          }
          case 0x0002:
          {
            JudgeReceivedNewDataSignal[1] = 1;
            memcpy (&Judge_GameResult, &ReceiveData[7], sizeof (ext_game_result_t));
            break;
          }
          case 0x0003:
          {
            JudgeReceivedNewDataSignal[2] = 1;
            memcpy (&Judge_GameRobotSurvivors, &ReceiveData[7], sizeof (ext_game_robot_HP_t));
            break;
          }
          case 0x0101:
          {
            JudgeReceivedNewDataSignal[3] = 1;
            memcpy (&Judge_EventData, &ReceiveData[7], sizeof (ext_event_data_t));
            break;
          }
          case 0x0102:
          {
            JudgeReceivedNewDataSignal[4] = 1;
            memcpy (&Judge_SupplyProjectileAction, &ReceiveData[7], sizeof (ext_supply_projectile_action_t));
            break;
          }
          case 0x0103:
          {
            JudgeReceivedNewDataSignal[5] = 1;
            memcpy (&Judge_SupplyProjectileBooking, &ReceiveData[7], sizeof (ext_supply_projectile_booking_t));
            break;
          }
          case 0x0104:
          {
            JudgeReceivedNewDataSignal[6] = 1;
            memcpy (&Judge_RefereeWarning, &ReceiveData[7], sizeof (ext_referee_warning_t));
            break;
          }
          case 0x0201:
          {
            JudgeReceivedNewDataSignal[7] = 1;
						PowerReciveNewDataSignal = 1;
            memcpy (&Judge_GameRobotState, &ReceiveData[7], sizeof (ext_game_robot_state_t));
            break;
          }
          case 0x0202:
          {
            JudgeReceivedNewDataSignal[8] = 1;
            memcpy (&Judge_PowerHeatData, &ReceiveData[7], sizeof (ext_power_heat_data_t));
            //printf("real_power=%f\r\n", Judge_PowerHeatData.chassis_power);
            //printf("judge.current=%d\r\n",  Judge_PowerHeatData.chassis_current);
            break;
          }
          case 0x0203:
          {
            JudgeReceivedNewDataSignal[9] = 1;
            memcpy (&Judge_GameRobotPos, &ReceiveData[7], sizeof (ext_game_robot_pos_t));
            break;
          }
          case 0x0204:
          {
            JudgeReceivedNewDataSignal[10] = 1;
            memcpy (&Judge_BuffMusk, &ReceiveData[7], sizeof (ext_buff_musk_t));
            break;
          }
          case 0x0205:
          {
            JudgeReceivedNewDataSignal[11] = 1;
            memcpy (&Judge_AerialRobotEnergy, &ReceiveData[7], sizeof (aerial_robot_energy_t));
            break;
          }
          case 0x0206:
          {
            JudgeReceivedNewDataSignal[12] = 1;
            memcpy (&Judge_RobotHurt, &ReceiveData[7], sizeof (ext_robot_hurt_t));
            break;
          }
          case 0x0207:
          {
            JudgeReceivedNewDataSignal[13] = 1;
            memcpy (&Judge_ShootData, &ReceiveData[7], sizeof (ext_shoot_data_t));
            break;
          }
          case 0x0208:
          {
            JudgeReceivedNewDataSignal[14] = 1;
            memcpy (&Judge_BulletRemaining, &ReceiveData[7], sizeof (ext_bullet_remaining_t));
            break;
          }
          case 0x0301:
          {
            JudgeReceivedNewDataSignal[15] = 1;
            memcpy (&Judge_StudentInfoHeader, &ReceiveData[7], sizeof (ext_student_interactive_header_data_t));
            memcpy (&Judge_RobotInterfaceData, &ReceiveData[7 + sizeof (ext_student_interactive_header_data_t)], sizeof (robot_interactive_data_t));
            break;
          }
          }
        }
      }

    }
  }
}
//赋值二字节数据到数组中
void Two_byte_send_id (uint8_t* arr, int data_lengh, int16_t cmd_id, int16_t content_id)
{
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  arr[0] = 0xA5;
  w2data.d = data_lengh; //数据帧中data的长度
  arr[1] = w2data.c[0];
  arr[2] = w2data.c[1];
  arr[3] = seq;
  Append_CRC8_Check_Sum (arr, 5);
  w2data.d = cmd_id; // cmd_id
  arr[5] = w2data.c[0];
  arr[6] = w2data.c[1];
  w2data.d = content_id; //数据内容id
  arr[7] = w2data.c[0];
  arr[8] = w2data.c[1];
  w2data.d = (int16_t) Robot_ID; //发送机器人的客户端
  arr[9] = w2data.c[0];
  arr[10] = w2data.c[1];
  w2data.d = (int16_t) Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  arr[11] = w2data.c[0];
  arr[12] = w2data.c[1];
}
//底盘模式
void CharacterCHASSISMODstateShow (uint32_t start_x, uint32_t start_y, uint8_t operate_tpye)
{
  Two_byte_send_id (UpData6, 51, 0x0301, 0x0110);
//    static uint8_t seq;
//    if (seq == 255)
//        seq = 0;
//    seq++;
//    UpData6[0] = 0xA5;
//    w2data.d = 51; //数据帧中data的长度
//    UpData6[1] = w2data.c[0];
//    UpData6[2] = w2data.c[1];
//    UpData6[3] = seq;
//    Append_CRC8_Check_Sum(UpData6, 5);
//    w2data.d = 0x0301; // cmd_id
//    UpData6[5] = w2data.c[0];
//    UpData6[6] = w2data.c[1];
//    w2data.d = 0x0110; //数据内容id
//    UpData6[7] = w2data.c[0];
//    UpData6[8] = w2data.c[1];
//    w2data.d = (int16_t)Robot_ID; //发送机器人的客户端
//    UpData6[9] = w2data.c[0];
//    UpData6[10] = w2data.c[1];
//    w2data.d = (int16_t)Robot_ID | 0x0100; //发送者对应的客户端id  0x100
//    UpData6[11] = w2data.c[0];
//    UpData6[12] = w2data.c[1];
  Judge_ClientCharacter.grapic_data_struct.graphic_name[0] = 13;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[1] = 8;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[2] = 6;
  Judge_ClientCharacter.grapic_data_struct.operate_tpye = operate_tpye;
  Judge_ClientCharacter.grapic_data_struct.graphic_tpye = 7;
  Judge_ClientCharacter.grapic_data_struct.layer = CHASSIS_MODE_LAYER;
  if (uiRx.chasMode != 0) //如果底盘模式标志位不为0的话，字符显示为橙色
    Judge_ClientCharacter.grapic_data_struct.color = 3;
  else//如果底盘模式标志位为0，字符显示黑色
    Judge_ClientCharacter.grapic_data_struct.color = 7;
  Judge_ClientCharacter.grapic_data_struct.start_angle = 20;
  Judge_ClientCharacter.grapic_data_struct.end_angle = 5;
  Judge_ClientCharacter.grapic_data_struct.width = 2;
  Judge_ClientCharacter.grapic_data_struct.start_x = start_x;
  Judge_ClientCharacter.grapic_data_struct.start_y = start_y;

  memcpy (UpData6 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));

  switch (uiRx.chasMode)
  {
  case 0x00:
  {
    memcpy (UpData6 + 28, &*cha[5], 9);
    break;
  }
  case 0x01:
  {
    memcpy (UpData6 + 28, &*cha[6], 9);
    break;
  }
  case 0x02:
  {
    memcpy (UpData6 + 28, &*cha[7], 9);
    break;
  }
  case 0x03:
  {
    memcpy (UpData6 + 28, &*cha[8], 9);
    break;
  }
  case 0x04:
  {
    memcpy (UpData6 + 28, &*cha[9], 9);
    break;
  }
  }
  Append_CRC16_Check_Sum (UpData6, 60);
  HAL_UART_Transmit_IT (&huart6, UpData6, 60);
}
//云台模式
void CharacterGIMBALstateShow (uint32_t start_x, uint32_t start_y, uint8_t operate_tpye)
{
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData9[0] = 0xA5;
  w2data.d = 51; //数据帧中data的长度
  UpData9[1] = w2data.c[0];
  UpData9[2] = w2data.c[1];
  UpData9[3] = seq;
  Append_CRC8_Check_Sum (UpData9, 5);
  w2data.d = 0x0301; // cmd_id
  UpData9[5] = w2data.c[0];
  UpData9[6] = w2data.c[1];
  w2data.d = 0x0110; //数据内容id
  UpData9[7] = w2data.c[0];
  UpData9[8] = w2data.c[1];
  w2data.d = (int16_t) Robot_ID; //发送机器人的客户端
  UpData9[9] = w2data.c[0];
  UpData9[10] = w2data.c[1];
  w2data.d = (int16_t) Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  UpData9[11] = w2data.c[0];
  UpData9[12] = w2data.c[1];
  Judge_ClientCharacter.grapic_data_struct.graphic_name[0] = 9;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[1] = 1;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[2] = 1;
  Judge_ClientCharacter.grapic_data_struct.operate_tpye = operate_tpye;
  Judge_ClientCharacter.grapic_data_struct.graphic_tpye = 7;
  Judge_ClientCharacter.grapic_data_struct.layer = GIMBAL_MODE_LAYER;
  if (uiRx.gimbalMode != 0) //如果云台模式标志位不为0的话，字符显示橙色
    Judge_ClientCharacter.grapic_data_struct.color = 3;
  else//如果云台模式标志位为0，字符显示黑色
    Judge_ClientCharacter.grapic_data_struct.color = 7;
  Judge_ClientCharacter.grapic_data_struct.start_angle = 20;
  Judge_ClientCharacter.grapic_data_struct.end_angle = 5;
  Judge_ClientCharacter.grapic_data_struct.width = 2;
  Judge_ClientCharacter.grapic_data_struct.start_x = start_x;
  Judge_ClientCharacter.grapic_data_struct.start_y = start_y;

  switch (uiRx.gimbalMode)
  {
  case 0x00:
  {
    memcpy (UpData9 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
    memcpy (UpData9 + 28, &*cha[10], 12); //NULL
    Append_CRC16_Check_Sum (UpData9, 60);
    break;
  }
  case 0x01:
  {
    memcpy (UpData9 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
    memcpy (UpData9 + 28, &*cha[11], 12); //MANUAL
    Append_CRC16_Check_Sum (UpData9, 60);
    break;
  }
  case 0x03:
  {
    memcpy (UpData9 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
    memcpy (UpData9 + 28, &*cha[12], 12); //ENCODE
    Append_CRC16_Check_Sum (UpData9, 60);
    break;
  }
  case 0x04:
  {
    memcpy (UpData9 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
    memcpy (UpData9 + 28, &*cha[13], 12); //FULL_VIS
    Append_CRC16_Check_Sum (UpData9, 60);
    break;
  }
  case 0x05:
  {
    memcpy (UpData9 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
    memcpy (UpData9 + 28, &*cha[14], 12); //HALF_VIS
    Append_CRC16_Check_Sum (UpData9, 60);
    break;
  }
  default:
    break;
  }
  HAL_UART_Transmit_IT (&huart6, UpData9, 60);
}
//摩擦轮状态
void CharacteFRIC_stateShow (uint32_t start_x, uint32_t start_y, uint8_t operate_tpye)
{
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData3[0] = 0xA5;
  w2data.d = 51; //数据帧中data的长度
  UpData3[1] = w2data.c[0];
  UpData3[2] = w2data.c[1];
  UpData3[3] = seq;
  Append_CRC8_Check_Sum (UpData3, 5);
  w2data.d = 0x0301; // cmd_id
  UpData3[5] = w2data.c[0];
  UpData3[6] = w2data.c[1];
  w2data.d = 0x0110; //数据内容id
  UpData3[7] = w2data.c[0];
  UpData3[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData3[9] = w2data.c[0];
  UpData3[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  UpData3[11] = w2data.c[0];
  UpData3[12] = w2data.c[1];
  Judge_ClientCharacter.grapic_data_struct.graphic_name[0] = 4;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[1] = 1;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[2] = 0;
  Judge_ClientCharacter.grapic_data_struct.layer = FRIC_MODE_LAYER;
  Judge_ClientCharacter.grapic_data_struct.operate_tpye = operate_tpye;
  Judge_ClientCharacter.grapic_data_struct.graphic_tpye = 7;
  if (uiRx.fricMode == 0x02) //如果摩擦轮状态标志位为2，则是开启状态
  {
    if (uiRx.firc_error == 0) //如果摩擦轮开启错误状态标志位为0，则显示橙色的字符
      Judge_ClientCharacter.grapic_data_struct.color = 3;
    else//如果摩擦轮开启错误状态标志位为1，则显示紫红的字符
      Judge_ClientCharacter.grapic_data_struct.color = 4;
  }
  else//如果摩擦轮状态标志位为2，则是关闭状态，显示黑色
  {
    Judge_ClientCharacter.grapic_data_struct.color = 7;
  }
  Judge_ClientCharacter.grapic_data_struct.start_angle = 20;
  Judge_ClientCharacter.grapic_data_struct.end_angle = 6;
  Judge_ClientCharacter.grapic_data_struct.width = 2;
  Judge_ClientCharacter.grapic_data_struct.start_x = start_x;
  Judge_ClientCharacter.grapic_data_struct.start_y = start_y;

  if (uiRx.fricMode == 0x02) //如果摩擦轮状态标志位为2，则是开启状态
  {
    if (uiRx.firc_error == 0) //如果摩擦轮开启错误状态标志位为0，则显示ERROR字符串
    {
      memcpy (UpData3 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
      memcpy (UpData3 + 28, &*cha[15], 7);
      Append_CRC16_Check_Sum (UpData3, 60);
    }
    else//如果摩擦轮开启错误状态标志位为1，则显示OPEN!!字符串
    {
      memcpy (UpData3 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
      memcpy (UpData3 + 28, &*cha[17], 7);
      Append_CRC16_Check_Sum (UpData3, 60);
    }
  }
  else//如果摩擦轮状态标志位不为2，则是关闭状态
  {
    memcpy (UpData3 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
    memcpy (UpData3 + 28, &*cha[16], 7);
    Append_CRC16_Check_Sum (UpData3, 60);
  }
  HAL_UART_Transmit_IT (&huart6, UpData3, 60);
}

//你是否开启陀螺模式,任务中没使用
void Character_DO_YOU_SPIN (uint32_t start_x, uint32_t start_y, uint8_t operate_tpye)
{
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData10[0] = 0xA5;
  w2data.d = 51; //数据帧中data的长度
  UpData10[1] = w2data.c[0];
  UpData10[2] = w2data.c[1];
  UpData10[3] = seq;
  Append_CRC8_Check_Sum (UpData10, 5);
  w2data.d = 0x0301; // cmd_id
  UpData10[5] = w2data.c[0];
  UpData10[6] = w2data.c[1];
  w2data.d = 0x0110; //数据内容id
  UpData10[7] = w2data.c[0];
  UpData10[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData10[9] = w2data.c[0];
  UpData10[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x0100
  UpData10[11] = w2data.c[0];
  UpData10[12] = w2data.c[1];
  Judge_ClientCharacter.grapic_data_struct.graphic_name[0] = 10;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[1] = 8;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[2] = 5;
  Judge_ClientCharacter.grapic_data_struct.layer = 2; // layer;
  Judge_ClientCharacter.grapic_data_struct.operate_tpye = operate_tpye;
  Judge_ClientCharacter.grapic_data_struct.graphic_tpye = 7;  // graphic_tpye;
  Judge_ClientCharacter.grapic_data_struct.color = 1;         // color;
  Judge_ClientCharacter.grapic_data_struct.start_angle = 25;  // 字号
  Judge_ClientCharacter.grapic_data_struct.end_angle = 6;     // end_angle;
  Judge_ClientCharacter.grapic_data_struct.width = 2;         // width;
  Judge_ClientCharacter.grapic_data_struct.start_x = start_x; // start_x;
  Judge_ClientCharacter.grapic_data_struct.start_y = start_y; // start_y;

  memcpy (UpData10 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));

  if (uiRx.chasMode == 0x00 || uiRx.chasMode == 0x01)
  {
    memcpy (UpData10 + 28, &*cha[17], 14);
  }
  else
  {
    memcpy (UpData10 + 28, &*cha[17], 14);
  }

  Append_CRC16_Check_Sum (UpData10, 60);
  HAL_UART_Transmit_IT (&huart6, UpData10, 60);
}

void AddPicture (uint16_t name, \
                 uint8_t i, \
                 uint8_t operate_type, \
                 uint32_t graphic_tpye,
                 uint8_t layer, \
                 uint32_t color, \
                 uint32_t start_angle, \
                 uint32_t end_angle, \
                 uint32_t width, \
                 uint32_t start_x, \
                 uint32_t start_y, \
                 uint32_t radius, \
                 uint32_t end_x, \
                 uint32_t end_y)
{
  //将三位数字拆分，百位数、十位数、个位数挨个赋值到name图姓名数组中
  Judge_ClientDrawSeven.grapic_data_struct[i].graphic_name[0] = name / 100;
  Judge_ClientDrawSeven.grapic_data_struct[i].graphic_name[1] = name / 10 % 10;
  Judge_ClientDrawSeven.grapic_data_struct[i].graphic_name[2] = name % 10;
  Judge_ClientDrawSeven.grapic_data_struct[i].operate_tpye = operate_type; //增加一个图形
  Judge_ClientDrawSeven.grapic_data_struct[i].graphic_tpye = graphic_tpye; //图形类型直线
  Judge_ClientDrawSeven.grapic_data_struct[i].layer = layer; //图层0
  Judge_ClientDrawSeven.grapic_data_struct[i].color = color; //橙色
  Judge_ClientDrawSeven.grapic_data_struct[i].start_angle = start_angle; //起始角度
  Judge_ClientDrawSeven.grapic_data_struct[i].end_angle = end_angle; //终止角度
  Judge_ClientDrawSeven.grapic_data_struct[i].width = width; //线宽
  Judge_ClientDrawSeven.grapic_data_struct[i].start_x = start_x; //起点x坐标
  Judge_ClientDrawSeven.grapic_data_struct[i].start_y = start_y; //起点y坐标
  Judge_ClientDrawSeven.grapic_data_struct[i].radius = radius; //字体大小或者半径
  Judge_ClientDrawSeven.grapic_data_struct[i].end_x = end_x; //终点x坐标
  Judge_ClientDrawSeven.grapic_data_struct[i].end_y = end_y; //终点y坐标
}

void AddPictureFunc (uint8_t i, \
                     uint8_t operate_type, \
                     uint32_t graphic_tpye,
                     uint8_t layer, \
                     uint32_t color, uint32_t start_angle,
                     uint32_t end_angle, \
                     uint32_t width, \
                     uint32_t start_x, \
                     uint32_t start_y, \
                     uint32_t radius, \
                     uint32_t end_x, \
                     uint32_t end_y, \
                     uint8_t name[3])
{
  Judge_ClientDrawFive.grapic_data_struct[i].graphic_name[0] = name[0];
  Judge_ClientDrawFive.grapic_data_struct[i].graphic_name[1] = name[1];
  Judge_ClientDrawFive.grapic_data_struct[i].graphic_name[2] = name[2];
  Judge_ClientDrawFive.grapic_data_struct[i].operate_tpye = operate_type; //增加一个图形
  Judge_ClientDrawFive.grapic_data_struct[i].graphic_tpye = graphic_tpye; //图形类型直线
  Judge_ClientDrawFive.grapic_data_struct[i].layer = layer; //图层0
  Judge_ClientDrawFive.grapic_data_struct[i].color = color; //橙色
  Judge_ClientDrawFive.grapic_data_struct[i].start_angle = start_angle; //起始角度
  Judge_ClientDrawFive.grapic_data_struct[i].end_angle = end_angle; //终止角度
  Judge_ClientDrawFive.grapic_data_struct[i].width = width; //线宽
  Judge_ClientDrawFive.grapic_data_struct[i].start_x = start_x; //起点x坐标
  Judge_ClientDrawFive.grapic_data_struct[i].start_y = start_y; //起点y坐标
  Judge_ClientDrawFive.grapic_data_struct[i].radius = radius; //字体大小或者半径
  Judge_ClientDrawFive.grapic_data_struct[i].end_x = end_x; //终点x坐标
  Judge_ClientDrawFive.grapic_data_struct[i].end_y = end_y; //终点y坐标
}

void AddFloatdata (uint16_t name, \
                   uint8_t i, \
                   uint8_t operate_type, \
                   uint32_t graphic_tpye, \
                   uint8_t layer, \
                   uint32_t color, \
                   uint32_t start_angle,
                   uint32_t end_angle, \
                   uint32_t width, \
                   uint32_t start_x, \
                   uint32_t start_y)
{
  Judge_ClientDrawFive.grapic_data_struct[i].graphic_name[0] = name / 100;
  Judge_ClientDrawFive.grapic_data_struct[i].graphic_name[1] = name / 10 % 10;
  Judge_ClientDrawFive.grapic_data_struct[i].graphic_name[2] = name % 10;
  Judge_ClientDrawFive.grapic_data_struct[i].operate_tpye = operate_type; //增加一个图形
  Judge_ClientDrawFive.grapic_data_struct[i].graphic_tpye = graphic_tpye; //图形类型直线
  Judge_ClientDrawFive.grapic_data_struct[i].layer = layer; //图层0
  Judge_ClientDrawFive.grapic_data_struct[i].color = color; //橙色
  Judge_ClientDrawFive.grapic_data_struct[i].start_angle = start_angle; //起始角度
  Judge_ClientDrawFive.grapic_data_struct[i].end_angle = end_angle; //终止角度
  Judge_ClientDrawFive.grapic_data_struct[i].width = width; //线宽
  Judge_ClientDrawFive.grapic_data_struct[i].start_x = start_x; //起点x坐标
  Judge_ClientDrawFive.grapic_data_struct[i].start_y = start_y; //起点y坐标
}
//准星，弹道下坠
void Crosshair (uint8_t operate_type)
{
  static uint16_t name_Crosshair = 100;
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData0[0] = 0xA5;
  w2data.d = 111; //数据帧中data的长度
  UpData0[1] = w2data.c[0];
  UpData0[2] = w2data.c[1];
  UpData0[3] = seq;
  Append_CRC8_Check_Sum (UpData0, 5);
  w2data.d = 0x0301; // cmd_id
  UpData0[5] = w2data.c[0];
  UpData0[6] = w2data.c[1];
  w2data.d = 0x0104; //数据内容id
  UpData0[7] = w2data.c[0];
  UpData0[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData0[9] = w2data.c[0];
  UpData0[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  UpData0[11] = w2data.c[0];
  UpData0[12] = w2data.c[1];

  if (uiRx.visMode == 0x00)
    AddPicture (name_Crosshair + 1, 0, operate_type, 2, CROSS_HAIR_LAYER, 3, 0, 0, 2, 960, 540, 30, 0, 0);
  else if (uiRx.visMode == 0x01)
    AddPicture (name_Crosshair + 1, 0, operate_type, 2, CROSS_HAIR_LAYER, 0, 0, 0, 2, 960, 540, 30, 0, 0);

  AddPicture (name_Crosshair + 2, 1, 1, 0, CROSS_HAIR_LAYER, 3, 0, 0, 2, 960, 540, 1, 960, 300);
  AddPicture (name_Crosshair + 3, 2, 1, 0, CROSS_HAIR_LAYER, 2, 0, 0, 2, 840, 460, 0, 1080, 460);
  AddPicture (name_Crosshair + 4, 3, 1, 0, CROSS_HAIR_LAYER, 3, 0, 0, 2, 870, 420, 0, 1050, 420);
  AddPicture (name_Crosshair + 5, 4, 1, 0, CROSS_HAIR_LAYER, 4, 0, 0, 2, 900, 380, 0, 1020, 380);
  AddPicture (name_Crosshair + 6, 5, 1, 0, CROSS_HAIR_LAYER, 5, 0, 0, 2, 930, 340, 0, 990, 340);
  AddPicture (name_Crosshair + 7, 6, 1, 0, CROSS_HAIR_LAYER, 6, 0, 0, 2, 930, 300, 0, 990, 300);

  memcpy (UpData0 + 13, &Judge_ClientDrawSeven.grapic_data_struct[0], sizeof (Judge_ClientDrawSeven.grapic_data_struct[0]));
  memcpy (UpData0 + 28, &Judge_ClientDrawSeven.grapic_data_struct[1], sizeof (Judge_ClientDrawSeven.grapic_data_struct[0]));
  memcpy (UpData0 + 43, &Judge_ClientDrawSeven.grapic_data_struct[2], sizeof (Judge_ClientDrawSeven.grapic_data_struct[0]));
  memcpy (UpData0 + 58, &Judge_ClientDrawSeven.grapic_data_struct[3], sizeof (Judge_ClientDrawSeven.grapic_data_struct[0]));
  memcpy (UpData0 + 73, &Judge_ClientDrawSeven.grapic_data_struct[4], sizeof (Judge_ClientDrawSeven.grapic_data_struct[0]));
  memcpy (UpData0 + 88, &Judge_ClientDrawSeven.grapic_data_struct[5], sizeof (Judge_ClientDrawSeven.grapic_data_struct[0]));
  memcpy (UpData0 + 103, &Judge_ClientDrawSeven.grapic_data_struct[6], sizeof (Judge_ClientDrawSeven.grapic_data_struct[0]));
  Append_CRC16_Check_Sum (UpData0, 120);
  HAL_UART_Transmit_IT (&huart6, UpData0, 120);
}

void Crosshair1 (uint8_t operate_type)
{
  static uint16_t name_Crosshair1 = 200;
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData15[0] = 0xA5;
  w2data.d = 111; //数据帧中data的长度
  UpData15[1] = w2data.c[0];
  UpData15[2] = w2data.c[1];
  UpData15[3] = seq;
  Append_CRC8_Check_Sum (UpData15, 5);
  w2data.d = 0x0301; // cmd_id
  UpData15[5] = w2data.c[0];
  UpData15[6] = w2data.c[1];
  w2data.d = 0x0104; //数据内容id
  UpData15[7] = w2data.c[0];
  UpData15[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData15[9] = w2data.c[0];
  UpData15[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  UpData15[11] = w2data.c[0];
  UpData15[12] = w2data.c[1];

  AddPicture (name_Crosshair1 + 1, 0, operate_type, 0, CROSS1_HAIR_LAYER, 8, 0, 0, 2, 700, 150, 1, 450, 50);
  AddPicture (name_Crosshair1 + 2, 1, operate_type, 0, CROSS1_HAIR_LAYER, 8, 0, 0, 2, 1220, 150, 0, 1470, 60);
  //AddPicture(5, 1, 0, 9, 8, 0, 0, 2, 700, 150, 0, 900, 150);
//    AddPicture(1, 1, 0, 9, 8, 0, 0, 2, 1020, 150, 0, 1220, 150);
//    AddPicture(3, 1, 0, 0, 3, 0, 0, 2, 870, 420, 0, 1050, 420);
//    AddPicture(4, 1, 0, 0, 4, 0, 0, 2, 900, 380, 0, 1020, 380);
//  AddPicture(10, 1, 0, 0, 5, 0, 0, 2, 930, 340, 0, 990, 340);
//  AddPicture(11, 1, 0, 0, 6, 0, 0, 2, 930, 300, 0, 990, 300);

  memcpy (UpData15 + 13, &Judge_ClientDrawSeven.grapic_data_struct[0], sizeof (Judge_ClientDrawSeven1.grapic_data_struct[0]));
  memcpy (UpData15 + 28, &Judge_ClientDrawSeven.grapic_data_struct[1], sizeof (Judge_ClientDrawSeven1.grapic_data_struct[0]));
//    memcpy(UpData15 + 28, &Judge_ClientDrawSeven1.grapic_data_struct[6],
//           sizeof(Judge_ClientDrawSeven1.grapic_data_struct[6]));
//    memcpy(UpData15 + 43, &Judge_ClientDrawSeven1.grapic_data_struct[7],
//           sizeof(Judge_ClientDrawSeven1.grapic_data_struct[7]));
//    memcpy(UpData15 + 58, &Judge_ClientDrawSeven1.grapic_data_struct[3],
//           sizeof(Judge_ClientDrawSeven1.grapic_data_struct[3]));
//    memcpy(UpData15 + 73, &Judge_ClientDrawSeven1.grapic_data_struct[4],
//           sizeof(Judge_ClientDrawSeven1.grapic_data_struct[4]));
//    memcpy(UpData15 + 88, &Judge_ClientDrawSeven1.grapic_data_struct[10],
//           sizeof(Judge_ClientDrawSeven1.grapic_data_struct[10]));
//  memcpy(UpData15 + 103, &Judge_ClientDrawSeven1.grapic_data_struct[11],
//           sizeof(Judge_ClientDrawSeven1.grapic_data_struct[11]));
  Append_CRC16_Check_Sum (UpData15, 120);
  HAL_UART_Transmit_IT (&huart6, UpData15, 120);
}

void Boarder()
{
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData[0] = 0xA5;
  w2data.d = 81; //数据帧中data的长度
  UpData[1] = w2data.c[0];
  UpData[2] = w2data.c[1];
  UpData[3] = seq;
  Append_CRC8_Check_Sum (UpData, 5);
  w2data.d = 0x0301; // cmd_id
  UpData[5] = w2data.c[0];
  UpData[6] = w2data.c[1];
  w2data.d = 0x0103; //数据内容id
  UpData[7] = w2data.c[0];
  UpData[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData[9] = w2data.c[0];
  UpData[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  UpData[11] = w2data.c[0];
  UpData[12] = w2data.c[1];

  AddPictureFunc (0, 1, 0, BOARDER_LAYER, 8, 0, 0, 2, 930, 300, 1, 930, 200, "blf"); //左竖线
  AddPictureFunc (1, 1, 0, BOARDER_LAYER, 8, 0, 0, 2, 990, 300, 0, 990, 200, "bld"); //右竖线
  AddPictureFunc (2, 1, 0, BOARDER_LAYER, 8, 0, 0, 2, 990, 275, 0, 1010, 275, "brf"); //左横线
  AddPictureFunc (3, 1, 0, BOARDER_LAYER, 8, 0, 0, 2, 930, 275, 0, 910, 275, "brd"); //右横线


  memcpy (UpData + 13, &Judge_ClientDrawFive.grapic_data_struct[0], sizeof (Judge_ClientDrawFive.grapic_data_struct[0]));
  memcpy (UpData + 28, &Judge_ClientDrawFive.grapic_data_struct[1], sizeof (Judge_ClientDrawFive.grapic_data_struct[1]));
  memcpy (UpData + 43, &Judge_ClientDrawFive.grapic_data_struct[2], sizeof (Judge_ClientDrawFive.grapic_data_struct[2]));
  memcpy (UpData + 58, &Judge_ClientDrawFive.grapic_data_struct[3], sizeof (Judge_ClientDrawFive.grapic_data_struct[3]));

  Append_CRC16_Check_Sum (UpData, 90);
  HAL_UART_Transmit_IT (&huart6, UpData, 90);
}

//动态字符显示，5为画浮点数，6为画整型数
void FloatDataShow (int32_t data1, int32_t data2, int32_t data3, int32_t data4, int32_t data5, uint8_t operate_type1, uint8_t operate_type2)
{
  static uint16_t name_FloatDataShow = 333;
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData7[0] = 0xA5;  //SOF
  w2data.d = 81; //数据帧中data的长度(6+15*2)
  UpData7[1] = w2data.c[0];
  UpData7[2] = w2data.c[1];
  UpData7[3] = seq;
  Append_CRC8_Check_Sum (UpData7, 5);
  w2data.d = 0x0301; // cmd_id
  UpData7[5] = w2data.c[0];
  UpData7[6] = w2data.c[1];
  w2data.d = 0x0103; //数据内容id
  UpData7[7] = w2data.c[0];
  UpData7[8] = w2data.c[1];
  w2data.d = (int16_t) Robot_ID; //发送机器人的客户端
  UpData7[9] = w2data.c[0];
  UpData7[10] = w2data.c[1];
  w2data.d = (int16_t) Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  UpData7[11] = w2data.c[0];
  UpData7[12] = w2data.c[1];

  AddFloatdata (name_FloatDataShow + 1, 0, operate_type1, 5, DYNAMIC_NUMBER_LAYER, 1, 20, 2, 3, 1080, 550); // data1，pit轴角度
  //变色，电容电压
  if (data2 > 30)
  {
    //AddFloatdata(6, operate_type2, 6, 1, 2, 20, 2, 3, 1600, 750); // data2，电容电压
    //AddFloatdata(name_FloatDataShow+2,1, operate_type2, 6, 1, 2, 20, 2, 3, 1030, 750); // data2，电容电压,黄色
    AddFloatdata (name_FloatDataShow + 2, 1, operate_type2, 6, DYNAMIC_NUMBER_LAYER, 2, 15, 2, 3, 940, 820); // data2，电容电压,黄色
  }
  else
  {
    //AddFloatdata(6, operate_type2, 6, 1, 4, 20, 2, 3, 1600, 750); // data2，电容电压
    //AddFloatdata(name_FloatDataShow+2,1, operate_type2, 6, 1, 4, 20, 2, 3, 1030, 750); // data2，电容电压，紫红色
    AddFloatdata (name_FloatDataShow + 2, 1, operate_type2, 6, DYNAMIC_NUMBER_LAYER, 4, 20, 2, 3, 940, 820); // data2，电容电压，紫红色
  }
  AddFloatdata (name_FloatDataShow + 3, 2, operate_type2, 6, DYNAMIC_NUMBER_LAYER, 2, 20, 2, 3, 945,  100); // data3，yaw轴编码器刻度
  AddFloatdata (name_FloatDataShow + 4, 3, operate_type2, 6, DYNAMIC_NUMBER_LAYER, 2, 20, 2, 3, 1600,  650); // data4，摩擦轮目标速度
  if (data5 < 90) //温度
    AddFloatdata (name_FloatDataShow + 5, 4, operate_type2, 6, DYNAMIC_NUMBER_LAYER, 2, 20, 2, 3, 1600,  550); // data5，摩擦轮返回温度
  else
    AddFloatdata (name_FloatDataShow + 5, 4, operate_type2, 6, DYNAMIC_NUMBER_LAYER, 4, 20, 2, 3, 1600,  550); // data5，摩擦轮返回温度
  /****************************************************************************************************/
  memcpy (UpData7 + 13, &Judge_ClientDrawFive.grapic_data_struct[0], sizeof (Judge_ClientDrawFive.grapic_data_struct[0]));
#if _IS_DJI_STILL_SB_ == 0
  w4data.d = (int32_t) (data1 * 10);

#else
  w4data.d = data1;
#endif /* IS_DJI_STILL_SB */

  UpData7[24] = w4data.c[0];
  UpData7[25] = w4data.c[1];
  UpData7[26] = w4data.c[2];
  UpData7[27] = w4data.c[3]; //存放浮点数b
  /****************************************************************************************************/
  //uiRx.realPowData，电容电压
  memcpy (UpData7 + 28, &Judge_ClientDrawFive.grapic_data_struct[1], sizeof (Judge_ClientDrawFive.grapic_data_struct[0]));

#if _IS_DJI_STILL_SB_ == 1
  w4data.d = (int32_t) ( (float) data2 / 1000.0f);
#else
  w4data.d = data2;
#endif /* IS_DJI_STILL_SB */

  UpData7[39] = w4data.c[0];
  UpData7[40] = w4data.c[1];
  UpData7[41] = w4data.c[2];
  UpData7[42] = w4data.c[3]; //存放浮点数c

  /****************************************************************************************************/
  //uiRx.realYawData，Yaw轴编码器刻度
  memcpy (UpData7 + 43, &Judge_ClientDrawFive.grapic_data_struct[2], sizeof (Judge_ClientDrawFive.grapic_data_struct[0]));

#if _IS_DJI_STILL_SB_ == 1
  w4data.d = (int32_t) ( (float) data2 / 1000.0f);
#else
  w4data.d = data3;
#endif /* IS_DJI_STILL_SB */

  UpData7[54] = w4data.c[0];
  UpData7[55] = w4data.c[1];
  UpData7[56] = w4data.c[2];
  UpData7[57] = w4data.c[3]; //存放浮点数c

  /****************************************************************************************************/
  //uiRx.realfricSpdData，摩擦轮目标速度
  memcpy (UpData7 + 58, &Judge_ClientDrawFive.grapic_data_struct[3], sizeof (Judge_ClientDrawFive.grapic_data_struct[0]));

#if _IS_DJI_STILL_SB_ == 1
  w4data.d = (int32_t) ( (float) data2 / 1000.0f);
#else
  w4data.d = data4;
#endif /* IS_DJI_STILL_SB */

  UpData7[69] = w4data.c[0];
  UpData7[70] = w4data.c[1];
  UpData7[71] = w4data.c[2];
  UpData7[72] = w4data.c[3]; //存放浮点数c
  /****************************************************************************************************/
  //uiRx.realfricTempData,摩擦轮温度
  memcpy (UpData7 + 73, &Judge_ClientDrawFive.grapic_data_struct[4], sizeof (Judge_ClientDrawFive.grapic_data_struct[0]));

#if _IS_DJI_STILL_SB_ == 1
  w4data.d = (int32_t) ( (float) data2 / 1000.0f);
#else
  w4data.d = data5;
#endif /* IS_DJI_STILL_SB */

  UpData7[84] = w4data.c[0];
  UpData7[85] = w4data.c[1];
  UpData7[86] = w4data.c[2];
  UpData7[87] = w4data.c[3]; //存放浮点数c
  /****************************************************************************************************/
  Append_CRC16_Check_Sum (UpData7, 90);
  HAL_UART_Transmit_IT (&huart6, UpData7, 90);
}

//显示Capvol字符
void CharacterCapvoltShow (uint32_t start_x, uint32_t start_y,
                           uint8_t operate_tpye)
{
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData5[0] = 0xA5;
  w2data.d = 51; //数据帧中data的长度
  UpData5[1] = w2data.c[0];
  UpData5[2] = w2data.c[1];
  UpData5[3] = seq;
  Append_CRC8_Check_Sum (UpData5, 5);
  w2data.d = 0x0301; // cmd_id
  UpData5[5] = w2data.c[0];
  UpData5[6] = w2data.c[1];
  w2data.d = 0x0110; //数据内容id
  UpData5[7] = w2data.c[0];
  UpData5[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData5[9] = w2data.c[0];
  UpData5[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  UpData5[11] = w2data.c[0];
  UpData5[12] = w2data.c[1];
  Judge_ClientCharacter.grapic_data_struct.graphic_name[0] = 6;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[1] = 7;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[2] = 7;
  Judge_ClientCharacter.grapic_data_struct.operate_tpye = operate_tpye;
  Judge_ClientCharacter.grapic_data_struct.graphic_tpye = 7;
  Judge_ClientCharacter.grapic_data_struct.layer = STATIC_CHARACTER_LAYER;//8号图层
  Judge_ClientCharacter.grapic_data_struct.color = 3;
  Judge_ClientCharacter.grapic_data_struct.start_angle = 20;
  Judge_ClientCharacter.grapic_data_struct.end_angle = 5;
  Judge_ClientCharacter.grapic_data_struct.width = 2;
  Judge_ClientCharacter.grapic_data_struct.start_x = start_x;
  Judge_ClientCharacter.grapic_data_struct.start_y = start_y;
  memcpy (UpData5 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
  memcpy (UpData5 + 28, &*cha[1], 9);
  Append_CRC16_Check_Sum (UpData5, 60);
  HAL_UART_Transmit_IT (&huart6, UpData5, 60);
}
//显示PITCH字符
void CharacterPitchShow (uint32_t start_x, uint32_t start_y, uint8_t operate_tpye)
{
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData1[0] = 0xA5;
  w2data.d = 51; //数据帧中data的长度
  UpData1[1] = w2data.c[0];
  UpData1[2] = w2data.c[1];
  UpData1[3] = seq;
  Append_CRC8_Check_Sum (UpData1, 5);
  w2data.d = 0x0301; // cmd_id
  UpData1[5] = w2data.c[0];
  UpData1[6] = w2data.c[1];
  w2data.d = 0x0110; //数据内容id
  UpData1[7] = w2data.c[0];
  UpData1[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData1[9] = w2data.c[0];
  UpData1[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  UpData1[11] = w2data.c[0];
  UpData1[12] = w2data.c[1];
  Judge_ClientCharacter.grapic_data_struct.graphic_name[0] = 6;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[1] = 2;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[2] = 8;
  Judge_ClientCharacter.grapic_data_struct.layer = STATIC_CHARACTER_LAYER;//8号图层
  Judge_ClientCharacter.grapic_data_struct.operate_tpye = operate_tpye;
  Judge_ClientCharacter.grapic_data_struct.graphic_tpye = 7;
  Judge_ClientCharacter.grapic_data_struct.color = 4;
  Judge_ClientCharacter.grapic_data_struct.start_angle = 20;
  Judge_ClientCharacter.grapic_data_struct.end_angle = 5;
  Judge_ClientCharacter.grapic_data_struct.width = 2;
  Judge_ClientCharacter.grapic_data_struct.start_x = start_x;
  Judge_ClientCharacter.grapic_data_struct.start_y = start_y;
  //  Judge_ClientCharacter.grapic_data_struct.radius=0;//字体大小或者半径
  //  Judge_ClientCharacter.grapic_data_struct.end_x=0;//终点x坐标
  //  Judge_ClientCharacter.grapic_data_struct.end_y=0;//终点y坐标
  memcpy (UpData1 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
  memcpy (UpData1 + 28, &*cha[0], 7);
  Append_CRC16_Check_Sum (UpData1, 60);
  HAL_UART_Transmit_IT (&huart6, UpData1, 60);
}
//显示FRIC字符
void CharacterFRIC_MODEShow (uint32_t start_x, uint32_t start_y, uint8_t operate_tpye)
{
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData2[0] = 0xA5;
  w2data.d = 51; //数据帧中data的长度 (6 + 15 + 30)
  UpData2[1] = w2data.c[0];
  UpData2[2] = w2data.c[1];
  UpData2[3] = seq;
  Append_CRC8_Check_Sum (UpData2, 5);
  w2data.d = 0x0301; // cmd_id
  UpData2[5] = w2data.c[0];
  UpData2[6] = w2data.c[1];
  w2data.d = 0x0110; //数据内容id
  UpData2[7] = w2data.c[0];
  UpData2[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData2[9] = w2data.c[0];
  UpData2[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  UpData2[11] = w2data.c[0];
  UpData2[12] = w2data.c[1];
  Judge_ClientCharacter.grapic_data_struct.graphic_name[0] = 4;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[1] = 6;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[2] = 9;
  Judge_ClientCharacter.grapic_data_struct.layer = STATIC_CHARACTER_LAYER; // layer;
  Judge_ClientCharacter.grapic_data_struct.operate_tpye = operate_tpye;
  Judge_ClientCharacter.grapic_data_struct.graphic_tpye = 7;  // graphic_tpye;
  Judge_ClientCharacter.grapic_data_struct.color = 5;         // color;
  Judge_ClientCharacter.grapic_data_struct.start_angle = 20;  // start_angle;
  Judge_ClientCharacter.grapic_data_struct.end_angle = 6;     // end_angle;
  Judge_ClientCharacter.grapic_data_struct.width = 2;         // width;
  Judge_ClientCharacter.grapic_data_struct.start_x = start_x; // start_x;
  Judge_ClientCharacter.grapic_data_struct.start_y = start_y; // start_y;
  memcpy (UpData2 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
  memcpy (UpData2 + 28, &*cha[4], 10);
  Append_CRC16_Check_Sum (UpData2, 60);
  HAL_UART_Transmit_IT (&huart6, UpData2, 60);
}
//显示CHASSIC_MODE字符
void CharacterCHASSIS_MODE (uint32_t start_x, uint32_t start_y, uint8_t operate_tpye)
{
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData4[0] = 0xA5;
  w2data.d = 51; //数据帧中data的长度
  UpData4[1] = w2data.c[0];
  UpData4[2] = w2data.c[1];
  UpData4[3] = seq;
  Append_CRC8_Check_Sum (UpData4, 5);
  w2data.d = 0x0301; // cmd_id
  UpData4[5] = w2data.c[0];
  UpData4[6] = w2data.c[1];
  w2data.d = 0x0110; //数据内容id
  UpData4[7] = w2data.c[0];
  UpData4[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData4[9] = w2data.c[0];
  UpData4[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x0100
  UpData4[11] = w2data.c[0];
  UpData4[12] = w2data.c[1];
  Judge_ClientCharacter.grapic_data_struct.graphic_name[0] = 6;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[1] = 6;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[2] = 1;
  Judge_ClientCharacter.grapic_data_struct.layer = STATIC_CHARACTER_LAYER; // layer;
  Judge_ClientCharacter.grapic_data_struct.operate_tpye = operate_tpye;
  Judge_ClientCharacter.grapic_data_struct.graphic_tpye = 7;  // graphic_tpye;
  Judge_ClientCharacter.grapic_data_struct.color = 8;         // color;
  Judge_ClientCharacter.grapic_data_struct.start_angle = 20;  // start_angle;
  Judge_ClientCharacter.grapic_data_struct.end_angle = 6;     // end_angle;
  Judge_ClientCharacter.grapic_data_struct.width = 2;         // width;
  Judge_ClientCharacter.grapic_data_struct.start_x = start_x; // start_x;
  Judge_ClientCharacter.grapic_data_struct.start_y = start_y; // start_y;
  memcpy (UpData4 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
  memcpy (UpData4 + 28, &*cha[2], 13);
  Append_CRC16_Check_Sum (UpData4, 60);
  HAL_UART_Transmit_IT (&huart6, UpData4, 60);
}

void CharacterGIMBAL_MODE (uint32_t start_x, uint32_t start_y, uint8_t operate_tpye)
{
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData8[0] = 0xA5;
  w2data.d = 51; //数据帧中data的长度
  UpData8[1] = w2data.c[0];
  UpData8[2] = w2data.c[1];
  UpData8[3] = seq;
  Append_CRC8_Check_Sum (UpData8, 5);
  w2data.d = 0x0301; // cmd_id
  UpData8[5] = w2data.c[0];
  UpData8[6] = w2data.c[1];
  w2data.d = 0x0110; //数据内容id
  UpData8[7] = w2data.c[0];
  UpData8[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData8[9] = w2data.c[0];
  UpData8[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x0100
  UpData8[11] = w2data.c[0];
  UpData8[12] = w2data.c[1];
  Judge_ClientCharacter.grapic_data_struct.graphic_name[0] = 7;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[1] = 9;
  Judge_ClientCharacter.grapic_data_struct.graphic_name[2] = 9;
  Judge_ClientCharacter.grapic_data_struct.layer = STATIC_CHARACTER_LAYER; //8号图层
  Judge_ClientCharacter.grapic_data_struct.operate_tpye = operate_tpye;
  Judge_ClientCharacter.grapic_data_struct.graphic_tpye = 7;  // graphic_tpye;
  Judge_ClientCharacter.grapic_data_struct.color = 8;         // color;
  Judge_ClientCharacter.grapic_data_struct.start_angle = 20;  // start_angle;
  Judge_ClientCharacter.grapic_data_struct.end_angle = 6;     // end_angle;
  Judge_ClientCharacter.grapic_data_struct.width = 2;         // width;
  Judge_ClientCharacter.grapic_data_struct.start_x = start_x; // start_x;
  Judge_ClientCharacter.grapic_data_struct.start_y = start_y; // start_y;
  memcpy (UpData8 + 13, &Judge_ClientCharacter.grapic_data_struct, sizeof (Judge_ClientCharacter.grapic_data_struct));
  memcpy (UpData8 + 28, &*cha[3], 13);
  Append_CRC16_Check_Sum (UpData8, 60);
  HAL_UART_Transmit_IT (&huart6, UpData8, 60);
}
//电容电压滑动条
uint8_t UpData16[128];
ext_client_custom_graphic_five_t Judge_ClientDrawFive_STATE;
void DrawCapVolRectangle (int32_t capVol, uint8_t operate_tpye, int16_t yaw_data)
{

  // yaw轴头朝前的电机编码器转角度值
#if ROBOT_ID == SUN

  int yaw_mid = 310-45;
	
#endif

#if ROBOT_ID == MOON

  int yaw_mid = 15;

#endif

  // 如果刚才那个角度值大于180度，那么减45度，因为圆弧的角度是0-180，超过180就取最短角度了
  static uint16_t name_Rectangle = 422;//图姓名，随意，一个图层下不重复就行哈
  static uint8_t seq;
  if (seq == 255)
    seq = 0;
  seq++;
  UpData16[0] = 0xA5;
  w2data.d = 81; //数据帧中data的长度
  UpData16[1] = w2data.c[0];
  UpData16[2] = w2data.c[1];
  UpData16[3] = seq;
  Append_CRC8_Check_Sum (UpData16, 5);
  w2data.d = 0x0301; // cmd_id
  UpData16[5] = w2data.c[0];
  UpData16[6] = w2data.c[1];
  w2data.d = 0x0103; //数据内容id,画5个图层
  UpData16[7] = w2data.c[0];
  UpData16[8] = w2data.c[1];
  w2data.d = Robot_ID; //发送机器人的客户端
  UpData16[9] = w2data.c[0];
  UpData16[10] = w2data.c[1];
  w2data.d = Robot_ID | 0x0100; //发送者对应的客户端id  0x100
  UpData16[11] = w2data.c[0];
  UpData16[12] = w2data.c[1];
  //画矩形
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].graphic_name[0] = name_Rectangle / 100;
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].graphic_name[1] = name_Rectangle / 10 % 10;
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].graphic_name[2] = name_Rectangle % 10;
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].operate_tpye = operate_tpye; //增加一个图形
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].graphic_tpye = 1 ; //图形类型矩形
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].layer = CAP_V_RECTANGLE_LAYHER; //图层4
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].color = 6; //橙色
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].start_angle = 0; //起始角度
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].end_angle = 0; //终止角度
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].width = 1; //线宽
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].start_x = 859; //起点x坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].start_y = 760; //起点y坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].radius = 0; //空
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].end_x = 1061; //终点x坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[0].end_y = 780; //终点y坐标
  memcpy (UpData16 + 13, &Judge_ClientDrawFive_STATE.grapic_data_struct[0], sizeof (Judge_ClientDrawFive_STATE.grapic_data_struct[0]));
  //画线
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].graphic_name[0] = (name_Rectangle + 1) / 100;
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].graphic_name[1] = (name_Rectangle + 1) / 10 % 10;
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].graphic_name[2] = (name_Rectangle + 1) % 10;
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].operate_tpye = operate_tpye; //增加一个图形
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].graphic_tpye = 0 ; //图形类型直线
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].layer = CAP_V_LINE_LAYHER; //图层5
  if (capVol > 30) //如果电容电压大于30的话（转化单位后），线条颜色为黄色
    Judge_ClientDrawFive_STATE.grapic_data_struct[1].color = 1; //黄色
  else//如果电容电压小于30的话（转化单位后），线条颜色为紫红色
    Judge_ClientDrawFive_STATE.grapic_data_struct[1].color = 4; //紫红色
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].start_angle = 0; //起始角度
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].end_angle = 0; //终止角度
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].width = 17; //线宽
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].start_x = 860; //起点x坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].start_y = 770; //起点y坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].radius = 0; //空
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].end_x = 860 + capVol * 2; //终点x坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[1].end_y = 770 ; //终点y坐标
  memcpy (UpData16 + 28, &Judge_ClientDrawFive_STATE.grapic_data_struct[1], sizeof (Judge_ClientDrawFive_STATE.grapic_data_struct[0]));
  //画圆
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].graphic_name[0] = (name_Rectangle + 2) / 100;
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].graphic_name[1] = (name_Rectangle + 2) / 10 % 10;
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].graphic_name[2] = (name_Rectangle + 2) % 10;
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].operate_tpye = operate_tpye; //增加一个图形
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].graphic_tpye = 2 ; //图形类型直线
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].layer = CAP_V_RECTANGLE_LAYHER; //图层4
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].color = 1; //黄色
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].start_angle = 0; //起始角度
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].end_angle = 0; //终止角度
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].width = 9; //线宽
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].start_x = 1600; //起点x坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].start_y = 805; //起点y坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].radius = 40; //半径
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].end_x = 0; //终点x坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[2].end_y = 0; //终点y坐标
  memcpy (UpData16 + 43, &Judge_ClientDrawFive_STATE.grapic_data_struct[2], sizeof (Judge_ClientDrawFive_STATE.grapic_data_struct[0]));
  //画线
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].graphic_name[0] = (name_Rectangle + 3) / 100;
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].graphic_name[1] = (name_Rectangle + 3) / 10 % 10;
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].graphic_name[2] = (name_Rectangle + 3) % 10;
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].operate_tpye = operate_tpye; //增加一个图形
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].graphic_tpye = 0 ; //图形类型直线
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].layer = CAP_V_LINE_LAYHER; //图层5
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].color = 1; //黄色
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].start_angle = 0; //起始角度
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].end_angle = 0; //终止角度
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].width = 7; //线宽
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].start_x = 1600; //起点x坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].start_y = 805; //起点y坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].radius = 0; //空
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].end_x = 1600; //终点x坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[3].end_y = 855; //终点y坐标
  memcpy (UpData16 + 58, &Judge_ClientDrawFive_STATE.grapic_data_struct[3], sizeof (Judge_ClientDrawFive_STATE.grapic_data_struct[0]));
//  //画圆弧
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].graphic_name[0] = (name_Rectangle + 5) / 100;
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].graphic_name[1] = (name_Rectangle + 5) / 10 % 10;
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].graphic_name[2] = (name_Rectangle + 5) % 10;
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].operate_tpye = operate_tpye; //增加一个图形
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].graphic_tpye = 4 ; //图形类型圆弧
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].layer = 7; //图层7
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].color = 4; //紫红色
  if ( (int32_t) yaw_data - yaw_mid - 90 > 0)
    Judge_ClientDrawFive_STATE.grapic_data_struct[4].start_angle = (int32_t) yaw_data - yaw_mid - 90; //起始角度
  else
    Judge_ClientDrawFive_STATE.grapic_data_struct[4].start_angle = (int32_t) yaw_data - yaw_mid - 90 + 360; //起始角度
  if ( (int32_t) yaw_data - yaw_mid > 0)
    Judge_ClientDrawFive_STATE.grapic_data_struct[4].end_angle = (int32_t) yaw_data - yaw_mid; //终止角度
  else
    Judge_ClientDrawFive_STATE.grapic_data_struct[4].end_angle = (int32_t) yaw_data - yaw_mid + 360; //终止角度
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].width = 9; //线宽
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].start_x = 1600; //起点x坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].start_y = 805; //起点y坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].radius = 0; //空
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].end_x = 40; //终点x坐标
  Judge_ClientDrawFive_STATE.grapic_data_struct[4].end_y = 40; //终点y坐标
  memcpy (UpData16 + 73, &Judge_ClientDrawFive_STATE.grapic_data_struct[4], sizeof (Judge_ClientDrawFive_STATE.grapic_data_struct[0]));

  Append_CRC16_Check_Sum (UpData16, 90);
  HAL_UART_Transmit_IT (&huart6, UpData16, 90);
}
void RobotSendMsgToClient (float data1, float data2, float data3, uint8_t mask)
{
  static uint8_t seq;
  if (seq == 255) seq = 0;
  seq++;
  UpData[0]   = 0xA5;
  w2data.d  = 19;
  UpData[1]   = w2data.c[0];
  UpData[2]   = w2data.c[1];
  UpData[3]   = seq;
  Append_CRC8_Check_Sum (UpData, 5);
  /* cmd_id */
  w2data.d  = 0x0301;
  UpData[5]   = w2data.c[0];
  UpData[6]   = w2data.c[1];
  /* content id */
  w2data.d  = 0xD180;
  UpData[7]   = w2data.c[0];
  UpData[8]   = w2data.c[1];
  /* sender id */
  w2data.d  = Judge_GameRobotState.robot_id;
  UpData[9]   = w2data.c[0];
  UpData[10]  = w2data.c[1];
  /* receiver id */
  if (Judge_GameRobotState.robot_id < 9)
    w2data.d  = Judge_GameRobotState.robot_id | 0x0100;
  else if (Judge_GameRobotState.robot_id > 9 && Judge_GameRobotState.robot_id < 16)
    w2data.d  = (Judge_GameRobotState.robot_id + 6) | 0x0100;

  //printf("send_ig = %x\r\n", w2data.d);
  UpData[11]  = w2data.c[0];
  UpData[12]  = w2data.c[1];

  /* data send */
  w4data.f  = data1;
  UpData[13]  = w4data.c[0];
  UpData[14]  = w4data.c[1];
  UpData[15]  = w4data.c[2];
  UpData[16]  = w4data.c[3];
  w4data.f  = data2;
  UpData[17]  = w4data.c[0];
  UpData[18]  = w4data.c[1];
  UpData[19]  = w4data.c[2];
  UpData[20]  = w4data.c[3];
  w4data.f  = data3;
  UpData[21]  = w4data.c[0];
  UpData[22]  = w4data.c[1];
  UpData[23]  = w4data.c[2];
  UpData[24]  = w4data.c[3];
  UpData[25]  = mask;
  /* CRC-check */
  Append_CRC16_Check_Sum (UpData, 28);
  //HAL_UART_Transmit_IT(&huart1,UpData,28);
}


#endif


