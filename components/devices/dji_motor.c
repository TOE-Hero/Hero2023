#include "main.h"
#include "dji_motor.h"

/**
* @brief  处理电机编码器数据令其连续，并将其转化成角度。
          该函数在每个can设备接收只能使用一次，假设定义了多个电机信息结构体（这是有必要的），也仅能使用一次该函数对电机编码器值处理成连续。
          若在一个can设备接收处理信息时使用两个，则其target_pos将为固定的电机上电时的编码器值
* @param  s_motor_data_t *s_motor
* @retval none
  */
void continue_motor_pos(s_motor_data_t *s_motor)
{   if(s_motor->is_pos_ready == 1)//如果电机第一次上电后记录了那时的电机编码器值并将预备标志位置一了的话，进入此判断
    {
        //如果（当前电机返回值-上一次电机返回值）值大于4096，因为电机不可能在几毫秒内转过半圈
        if(s_motor->back_position - s_motor->back_pos_last > 4096)
        {
            s_motor->circle_num--;//圈数--
        }
        else if(s_motor->back_position - s_motor->back_pos_last < -4096)//同上，只不过方向是反的
        {
            s_motor->circle_num++;//圈数++
        }
    }
    else
    {
        s_motor->target_pos = s_motor->back_position ;//如果电机预备标志位不为1，也就是电机第一次上电
        s_motor->is_pos_ready = 1;//电机预备标志位赋值为一，也就是说电机已经准备好
    }
    s_motor->back_pos_last		=	s_motor->back_position;//将上一次进入该函数的电机返回值赋值，方便计算连续值
    s_motor->serial_position	=	s_motor->back_position + s_motor->circle_num * 8191;//返回的电机连续编码器值
	s_motor->back_motor_ang		=	s_motor->back_position / 8191.0f * 360.0f;//返回的电机绝对角度
	s_motor->serial_motor_ang	= 	s_motor->serial_position / 8191.0f * 360.0f;//返回的电机连续角度

}

