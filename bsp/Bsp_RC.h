#ifndef __Bsp_RC_H
#define __Bsp_RC_H

#include "stm32f4xx_hal.h"
#include "struct_typedef.h"
#include "Monitor.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
#define LS_DOWN					(rc_ctrl.rc.s[1]==RC_SW_DOWN)
#define LS_MID					(rc_ctrl.rc.s[1]==RC_SW_MID)
#define LS_UP					(rc_ctrl.rc.s[1]==RC_SW_UP)
#define RS_DOWN					(rc_ctrl.rc.s[0]==RC_SW_DOWN)
#define RS_MID					(rc_ctrl.rc.s[0]==RC_SW_MID)
#define RS_UP					(rc_ctrl.rc.s[0]==RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

#define CHASSIS_FRONT_KEY	KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY	KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY	KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY	KEY_PRESSED_OFFSET_D

#define PRESS_FRONT			(rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)
#define PRESS_BACK			(rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)
#define PRESS_LEFT			(rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)
#define PRESS_RIGHT			(rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)

#define PRESS_MOUSE_L		rc_ctrl.mouse.press_l
#define PRESS_MOUSE_R		rc_ctrl.mouse.press_r
#define PRESS_W 			  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)
#define PRESS_S				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)
#define PRESS_A				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)
#define PRESS_D				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)
#define PRESS_SHIFT	  	(rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT)
#define PRESS_CTRL		  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL)
#define PRESS_Q				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
#define PRESS_E				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
#define PRESS_R				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)
#define PRESS_F				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)
#define PRESS_G			  	(rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)
#define PRESS_Z				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)
#define PRESS_X				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)
#define PRESS_C			  	(rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)
#define PRESS_V				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)
#define PRESS_B				  (rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)
/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
        struct
        {
                int16_t ch[5];
                char s[2];
        }__packed rc;
        struct
        {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
        }__packed mouse;
        struct
        {
                uint16_t v;
        }__packed key;

}__packed RC_ctrl_t;



extern RC_ctrl_t rc_ctrl;


/* ----------------------- Internal Data ----------------------------------- */
extern void  RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
extern const RC_ctrl_t *get_remote_control_point(void);

extern void RC_unable(void);

extern void RC_restart(uint16_t dma_buf_num);
#endif

