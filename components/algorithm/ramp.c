/**
 **********************************************************************************
 * @FilePath: \Hero2022-vscode\components\algorithm\ramp.c
 * @Brief: 
 * @Date: 2021-04-20 08:39:21
 * @Author: Rio
 **********************************************************************************
 */
#include "ramp.h"
/**************************** global variable ***********************************/
ramp_t ramp_cFllow = RAMP_SLOW_INIT;
ramp_t ramp_c = RAMP_SLOW_INIT;
ramp_t ramp_w = RAMP_DEFAULT_INIT;
ramp_t ramp_s = RAMP_DEFAULT_INIT;
ramp_t ramp_a = RAMP_DEFAULT_INIT;
ramp_t ramp_d = RAMP_DEFAULT_INIT;
/************************** Function declaration *********************************/
void ramp_init1(ramp_t *ramp);
float ramp_cal(ramp_t *ramp);
/*********************************************************************************/
/**
 * @brief 套娃初始化斜坡，放到main.c里
 * @param None
 * @return None
 * @attention None
 */
void ramp_All_init(void)
{
	ramp_init1(&ramp_w);
	ramp_init1(&ramp_s);
	ramp_init1(&ramp_a);
	ramp_init1(&ramp_d);
	ramp_init1(&ramp_c);
	ramp_init1(&ramp_cFllow);
}

/**
 * @brief 初始化斜坡
 * @param None
 * @return None
 * @attention None
 */
void ramp_init1(ramp_t *ramp){
	ramp->count = 0;
	ramp->out = 0;
}
/**
 * @brief 计算斜坡，从0开始数count的值，累加，每次加一次与设定的scale值（定值）做除法并赋值给函数输出值，
		  当count数到scale值时始终等于scale的值，最终的除数等于1，以此达到一次函数的效果
 * @param None
 * @return ramp->out，范围[0,1]
 * @attention 
 */
float ramp_cal(ramp_t *ramp){
  if(ramp->scale <=0) return 0;
  if (++ramp->count >= ramp->scale)
    ramp->count = ramp->scale;

  ramp->out = ramp->count / ((float)ramp->scale);
  return ramp->out;
}

