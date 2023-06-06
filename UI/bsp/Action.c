#include "Action.h"
#include "Judge.h"

/***************************************
  * @brief  :底盘运动速度设定限制
  * @param  :
****************************************/
LOGIC Logic;
void Chassis_SpeedSet(float YSpeed)
{
  // s_trans_motor.LastV = Chassis.TargetV;
    YSpeed = YSpeed > MAXMOTORSPEED ? MAXMOTORSPEED : YSpeed;/*MAXMOTORSPEED = 3000*/
    YSpeed = YSpeed < -MAXMOTORSPEED ? -MAXMOTORSPEED : YSpeed;
	  s_chassis_motor.target_speed = YSpeed;
}

/**
  * @brief 正常移动，基恩士检测到边缘后开始变向并且重置编码器里程
  * @param  None
  * @retval void
  * @usage
  */
void Normal_Move(void){
	static int once;
//	if(sensor_left == 1 && sensor_right == 1 ){//1为未检测到边缘
//		once++;
//		if(once==200)//每1秒随机一次
//		{
//	  Logic.turn_flag = rand()/10%2+1 ;
//		once = 0;
//		}//真坑，随机数第一位的奇偶是一次奇一次偶。。。。
//	}
	if(sensor_left == 0 && sensor_right == 1)
		Logic.turn_flag = 1;
	if(sensor_left == 1 && sensor_right == 0)
		Logic.turn_flag = 2;
	else if(sensor_left == 0 && sensor_right == 0)
		Logic.turn_flag = 0;
	 if( Logic.last_flag != Logic.turn_flag)
	 {Logic.turn = 1;
		 printf("Logic.turn = 1\r\n");
	 }
	 else Logic.turn = 0;
	  Logic.last_flag = Logic.turn_flag;
}
/**
  * @brief 
  * @param  None
  * @retval void
  * @usage
 */
//void Logic_Move(void){
//	 static int speed;
//		if(Logic.turn_flag == 1){
//			speed += 30;
//			if(speed > NORMALSPEED)
//				speed = NORMALSPEED;
//		}
//		else if(Logic.turn_flag == 2){
//			speed -= 30;
//			if(speed < -NORMALSPEED)
//		 	speed = -NORMALSPEED;
//		}
//		else if(Logic.turn_flag == 0){
//			speed = 0;
//		}
//		s_chassis_motor.target_speed = speed;
//}
void Logic_Move(void){
	 static int speed;
		if(Logic.turn_flag == 1){
//			speed += 150;/*-6000到+6000的速度400ms*/
//			if(speed > Logic.goal_speed)
				speed = Logic.goal_speed;
		}
		else if(Logic.turn_flag == 2){
//			speed -= 150;
//			if(speed < -Logic.goal_speed)
		 	speed = -Logic.goal_speed;
		}
		else if(Logic.turn_flag == 0){
			speed = 0;
		}
		s_chassis_motor.target_speed = speed;
}

/**
  * @brief  Detection of blood volume changes
  * @param  None
  * @retval Is it harmed
  * @usage
  */
uint8_t Monitor_HP(void){
	static int16_t last_HP;
	static bool first;
	static uint16_t safe_delay;
	static uint16_t safe_count;
	static bool warning;
	if(!first++)
		last_HP = Logic.remain_HP;
	if(last_HP > Logic.remain_HP){
		safe_delay = 0;
		safe_count = 0;
		Logic.hurt.hurt++;
		warning = true;
	}
	else{
		safe_delay++;
		if(safe_delay > 200){
			safe_count++;//安全后每200*5=1s，safe_count++
			safe_delay = 0;
		}
		if(safe_count > 20)//20后被打击警报解除
			warning = false;
	}
	last_HP = Logic.remain_HP;
	return warning;
}
/**
  * @brief  Detection of blood volume changes
  * @param  None
  * @retval Is it harmed
  * @usage
  */
void Judge_FPS(void){
	static int32_t last_fps;
	if(fps - last_fps > 0){//两次检测之间fps所在位置被调用
		Logic.judge_fps = Logic.fps;
		fps = 0;
		Logic.fps = 0;
		last_fps = 0;
	}
	Logic.fps++;//若fps所在位置没被调用则每次加1,感觉应该是Logic.judge_fps++，写错了？
	last_fps = fps;
	if(Logic.judge_fps > 30)//加到30裁判系统错误警告
		Logic.judge_error = 1;
	else
		Logic.judge_error = 0;
}

void speed_buffer(void)
{  static int choose = 1;
	if((choose == 1)&&(Logic.power_buffer > chassis_buffer_waring))
	{
		Logic.goal_speed = 6500;
		choose = 1;
		return;
	}
	if((choose == 1)&&(Logic.power_buffer <= chassis_buffer_waring))
	{
		//Logic.goal_speed -= chassis_speed_change;/*斜坡函数*/
//		Logic.goal_speed = Logic.goal_speed < 4200 ? 4200 :  Logic.goal_speed;
		Logic.goal_speed = 4200;
    choose = 0;
		return;
	}
//	if((choose == 0)&&(Logic.power_buffer < 200))
//	{
//		Logic.goal_speed -= chassis_speed_change;
//		Logic.goal_speed = Logic.goal_speed < 4200 ? 4200 :  Logic.goal_speed;
//    choose = 0;
//		return;
//	}
		if((choose == 0)&&(Logic.power_buffer == 200))
		{
					Logic.goal_speed = 6500;
          choose = 1;
			    return;
    }
}