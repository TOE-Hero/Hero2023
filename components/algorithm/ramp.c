/**
 **********************************************************************************
 * @FilePath: \Hero2022-vscode\components\algorithm\ramp.c
 * @Brief: 
 * @Date: 2021-04-20 08:39:21
 * @Author: Rio
 **********************************************************************************
 */
#include "ramp.h"
/************************** Function declaration *********************************/
void ramp_init(ramp_t *ramp);
float ramp_cal(ramp_t *ramp);
/*********************************************************************************/

/**
 * @brief 初始化斜坡
 * @param None
 * @return None
 * @attention None
 */
void ramp_init(ramp_t *ramp){
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

