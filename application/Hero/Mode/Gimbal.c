#include <cmsis_os.h>
#include <main.h>
#include <string.h>
#include <math.h>
#include "bsp_can.h"
#include "pid.h"
#include "STMGood.h"
#include "bsp_usart.h"
#include "dji_motor.h"
#include "bsp_remote.h"
#include "INS_task.h"
#include "nuc_interface.h"

#include "Hero_control.h"
/*********************** Debug ***********************/
#define  G_GYRO_PIT_DEBUG               0	//pid_abs_evaluation(&PIT_motor_pid_pos_imu, P1, 0, 0, 0, S1);
//pid_abs_evaluation(&PIT_motor_pid_speed_imu, P2, I2, D2, 2000 ,28000);

#define  G_GYRO_YAW_DEBUG               0	//pid_abs_evaluation(&YAW_motor_pid_pos_imu, P3 , I3, D3, 0, S3);
//pid_abs_evaluation(&YAW_motor_pid_speed_imu, P4 , I4 , D4, 20000, 28000);

#define  G_HALF_GYRO_PIT_DEBUG          0	//pid_abs_evaluation(&PIT_motor_pid_pos, P1, 0, 0, 0, S1);
//pid_abs_evaluation(&PIT_motor_pid_speed, P2, I2, 0, 2000 ,28000);

#define  G_HALF_GYRO_YAW_DEBUG          0	//pid_abs_evaluation(&YAW_motor_pid_pos_imu, P3 , I3, D3, 0, S3);
//pid_abs_evaluation(&YAW_motor_pid_speed_imu, P4 , I4 , D4, 20000, 28000);

#define  G_ENCODE_PIT_DEBUG             0	//pid_abs_evaluation(&PIT_motor_pid_pos, P1, 0, 0, 0, 300);
//pid_abs_evaluation(&PIT_motor_pid_speed, P2, I2, D2, 2000 ,28000);

#define  G_ENCODE_YAW_DEBUG             0	//pid_abs_param_init(&YAW_motor_pid_pos_encode, P3, 0, 0, 0, S3);
//pid_abs_param_init(&YAW_motor_pid_speed_encode, P4 , I4 , 0, 0, 28000);

#define  G_HALF_VISION_PIT_DEBUG        0	//pid_abs_evaluation(&PIT_motor_pid_pos_imu_vis, P1, 0, 0, 0, 1000);
//pid_abs_evaluation(&PIT_motor_pid_speed_imu_vis, P2, I2, 0, 2000, 20000);

#define  G_FULL_VISION_PIT_DEBUG        0	//pid_abs_evaluation(&PIT_motor_pid_pos_imu_vis, P1, 0, 0, 0, S1);
//pid_abs_evaluation(&PIT_motor_pid_speed_imu_vis, P2, I2, D2, 2000 ,28000);

#define  G_FULL_VISION_YAW_DEBUG        0	//pid_abs_evaluation(&YAW_motor_pid_pos_imu_vis, P3, 0, 0, 0, 1000);
//pid_abs_evaluation(&YAW_motor_pid_speed_imu_vis, P4, I4, 0, 2000, 28000);

/********************** SUN ***********************/
#if	ROBOT_ID == SUN
#define PIT_ENCODE_UP_LIM       6000//PIT抬头时电机编码器最大值
#define PIT_ENCODE_DOWN_LIM     1050//PIT低头时电机编码器最小值,2715
#define PIT_ENCODE_MID_R        ((PIT_ENCODE_UP_LIM+PIT_ENCODE_DOWN_LIM)/2)//PIT枪口水平时电机编码器值
#define PIT_ENCODE_HORIZONTAL	2815//底盘水平的情况下，PIT轴陀螺仪为0度时，PIT轴电机的码盘值
#define PIT_GYRO_UP_LIM         39.0f//PIT用陀螺仪控制时抬头最大角度
#define PIT_GYRO_DOWN_LIM       -19.0f//PIT用陀螺仪控制时低头最大角度
#define PIT_GYRO_MID_R        	((PIT_GYRO_UP_LIM+PIT_GYRO_DOWN_LIM)/2)
#define VISION_X_CENTRE         610.0f//YAW轴自瞄中值
#define VISION_Y_CENTRE         210.0f//PIT轴自瞄中值
#define RC_accumulate_ch_1_MIN  -(abs((PIT_ENCODE_UP_LIM-PIT_ENCODE_DOWN_LIM)/2))//遥控器递增最小值
#define RC_accumulate_ch_1_MAX  (abs((PIT_ENCODE_UP_LIM-PIT_ENCODE_DOWN_LIM)/2))//遥控器递增最大值
#define RC_accumulate_ch_1_GYRO_MIN  -(fabs((PIT_GYRO_UP_LIM-PIT_GYRO_DOWN_LIM)/2))//陀螺仪模式下遥控器递增最小值
#define RC_accumulate_ch_1_GYRO_MAX  (fabs((PIT_GYRO_UP_LIM-PIT_GYRO_DOWN_LIM)/2))//陀螺仪模式下遥控器递增最大值
#define MOUSE_X					(rc_ctrl.mouse.x*0.1*0.080)//鼠标x轴DPI，也就是YAW轴DPI
#define MOUSE_Y					(rc_ctrl.mouse.y*0.5)//鼠标y轴DPI，也就是PIT轴DPI
#define RC_X					-((rc_ctrl.rc.ch[0]*0.05)/66)//遥控器YAW轴DPI
#define RC_Y					(rc_ctrl.rc.ch[1] * 0.04f)//遥控器PIT轴DPI
#define REDUCTION_RATIO_2006    36//2006电机的减速比
#define OFFSET_ANGLE_2006       120.0f//2006电机放下与抬起的角度差值
#define INIT_CURRENT_2006       1300//2006电机初始化需要的电流值正负需要自己判断，注意2006电机电流-10000~10000
#define PC_FPS                  200
#endif
/********************** MOON ************************/
#if	ROBOT_ID == MOON
#define PIT_ENCODE_UP_LIM       6700//PIT抬头时电机编码器最大值
#define PIT_ENCODE_DOWN_LIM     1900//PIT低头时电机编码器最小值
#define PIT_ENCODE_HORIZONTAL	3415//底盘水平的情况下，PIT轴陀螺仪为0度时，PIT轴电机的码盘值
#define PIT_ENCODE_MID_R        ((PIT_ENCODE_UP_LIM+PIT_ENCODE_DOWN_LIM)/2)//3885.0f//PIT枪口水平时电机编码器值
#define PIT_GYRO_UP_LIM         35.0f//PIT用陀螺仪控制时抬头最大角度
#define PIT_GYRO_DOWN_LIM       -18.0f//PIT用陀螺仪控制时低头最大角度
#define PIT_GYRO_MID_R        	((PIT_GYRO_UP_LIM+PIT_GYRO_DOWN_LIM)/2)
#define VISION_X_CENTRE         530.0f//YAW轴自瞄中值
#define VISION_Y_CENTRE         210.0f//PIT轴自瞄中值
#define RC_accumulate_ch_1_MIN  -(abs((PIT_ENCODE_UP_LIM-PIT_ENCODE_DOWN_LIM)/2))//遥控器递增最小值
#define RC_accumulate_ch_1_MAX  (abs((PIT_ENCODE_UP_LIM-PIT_ENCODE_DOWN_LIM)/2))//遥控器递增最大值
#define MOUSE_X					(rc_ctrl.mouse.x*0.1*0.080)//鼠标x轴DPI，也就是YAW轴DPI
#define MOUSE_Y					(rc_ctrl.mouse.y*0.5)//鼠标y轴DPI，也就是PIT轴DPI
#define RC_X					-((rc_ctrl.rc.ch[0]*0.05)/66)//遥控器YAW轴DPI
#define RC_Y					(rc_ctrl.rc.ch[1] * 0.04f)//遥控器PIT轴DPI
#endif

#define RADIAN_TO_DEGREE            180.0f/3.14159265f//弧度-->度
/**************************** global variable ************************************/
float	 	              	RC_accumulate_ch_1=0.0f;//遥控器递增值的中间变量
float                     	RC_accumulate_ch_1_gyro = 0.0f;//遥控器递增值的中间变量，pit陀螺仪角度作外环模式
visInf_t 	              	s_visionInform = {0};//自瞄结构体
float                     	pit_offset_ang = 0.0f;//pit轴云台在使用陀螺仪角度作外环的时候，相对底盘的角度
float 						Gyro_up_lim = 0.0f;//pit轴云台在使用陀螺仪角度作外环抬头最大值
float 						Gyro_down_lim = 0.0f;//pit轴云台在使用陀螺仪角度作外环低头最大值
float 						Gyro_mid = 0.0f;//pit轴云台在使用陀螺仪角度作外环，角度中值
int32_t                     Vedio_transmission_limit_up = 0;
int32_t                     Vedio_transmission_limit_down = 0;
uint8_t                     Vedio_transmission_init_flag = 0;
/**************************** static variable ************************************/

/***************************** extern declaration ******************************/

/*******************************************************************************/

/**
 * @brief 图传上的2006电机上电位置自校准，都是对全局变量操作，放到云台初始化函数里，这个函数只可以上电运行一次
 * 
 */
void Vedio_Transmission_Positon_Init(void)
{
    #if ROBOT_ID == SUN
    uint16_t count_ = 500;
    Camera_motor.out_current = INIT_CURRENT_2006;
    while (--count_)
    {
        HAL_Delay(10);//10*500=5000，初始化5s，很蠢的写法，好用就行
        CANTx_SendCurrent(&hcan1,0x1FF, 0, Camera_motor.out_current, 0, 0 );
    }
    Vedio_transmission_init_flag = 1;//置位标志位为1
    #endif
}


#if ROBOT_ID == SUN
/**
 * @brief 图传2006运动控制，只有抬起和放下两种位置模式
 * 
 */
void Camera_Move(s_robo_Mode_Setting RoboMode, int32_t camera_down, int32_t camera_up)
{
    static motor_2006_toggle_flag = 0;
    static uint8_t keyLock_B;
    static uint32_t ChangeCameraStateCount = 0;
    //初始化图传位置
    if(Vedio_transmission_init_flag)//判断图传初始化标志位，为1则说明已经运行结束了图传初始化函数
    {
        if(abs(Camera_motor.back_motor_speed) < 20)//判断电机当前速度小于20，说明2006为静止状态
            Vedio_transmission_limit_down = Camera_motor.back_position;//如果2006为静止状态，赋值
        
        Vedio_transmission_limit_up   = Vedio_transmission_limit_down - (int32_t)(OFFSET_ANGLE_2006/360.0f*REDUCTION_RATIO_2006*8191);
        Vedio_transmission_init_flag = 0;//清空标志位
    }
    switch(robot_Mode.chassisMode)
    {
    /**************************************************** C_NULL ***********************************************************************/
    case C_NULL:
    {
        Camera_motor.target_pos = camera_down;
        motor_2006_toggle_flag = 0;
        ChangeCameraStateCount = 0;
        break;
    }
    /**************************************************** C_APART ***********************************************************************/
    case C_APART:
    {
        Camera_motor.target_pos = camera_down;
        motor_2006_toggle_flag = 0;
        ChangeCameraStateCount = 0;
        break;
    }
    /**************************************************** C_FOLLOW ***********************************************************************/
    case C_FOLLOW:
    {
        Camera_motor.target_pos = camera_down;
        motor_2006_toggle_flag = 0;
        ChangeCameraStateCount = 0;
        break;
    }
    /******************************************************* C_TOP ***********************************************************************/
    case C_TOP:
    {
        Camera_motor.target_pos = camera_down;
        motor_2006_toggle_flag = 0;
        ChangeCameraStateCount = 0;
        break;
    }
    /******************************************************* C_STOP *******************************************************************/
    case C_STOP:
    {
        if(PRESS_B) {
            keyLock_B=1;
        }
        if((!PRESS_B) && keyLock_B==1)
        {
            if(motor_2006_toggle_flag)
            {
                Camera_motor.target_pos = camera_down;
            }
            else
            {
                Camera_motor.target_pos = camera_up;
            }
            motor_2006_toggle_flag = !motor_2006_toggle_flag;

            keyLock_B = 0;
        }
        if(rc_ctrl.rc.ch[4]<=-600)//遥控器控制
        {
            ChangeCameraStateCount++;
            if(ChangeCameraStateCount > 300)
            {
                if(motor_2006_toggle_flag)
                {
                    Camera_motor.target_pos = camera_down;
                }
                else
                {
                    Camera_motor.target_pos = camera_up;
                }
                motor_2006_toggle_flag = !motor_2006_toggle_flag;

                ChangeCameraStateCount = 0;
            }
        }
        break;
    }
    }

    if(robot_Mode.chassisMode == C_NULL)
    {
        Camera_motor.out_current = 0;//零电流
    }
    else
    {
    Camera_motor.out_current = motor_double_loop_PID(&Camera_motor_pid_pos, &Camera_motor_pid_speed, \
                               (float)Camera_motor.serial_position, \
                               (float)Camera_motor.target_pos, \
                               (float)Camera_motor.back_motor_speed);
    }

}
#endif
/**
  * @brief          云台初始化
  * @param[in]      none
  * @retval         none
  */
void Gimbal_Init()
{
    Vedio_Transmission_Positon_Init();
    Camera_motor.target_pos = Vedio_transmission_limit_down;
    PIT_motor.target_pos                = PIT_ENCODE_MID_R;//让初始上电时目标值为中值
    YAW_motor_encode.target_motor_ang	= YAW_motor.serial_motor_ang;//上电时的编码器值为ENCODE模式下的目标值
    YAW_motor_encode.target_pos         = YAW_motor.serial_position;//上电时的编码器值为ENCODE模式下的目标值
    YAW_motor_imu.target_ang_imu	    = IMU_All_Value.yaw.yawAng;//上电时的角度为FLLOW模式下的目标值
    PIT_motor_imu.target_ang_imu	    = 0;//0度为GYRO模式下上电时的目标值
    memset(&YAW_motor, 0, sizeof(YAW_motor)); //初始化YAW_motor结构体为0
    memset(&PIT_motor, 0, sizeof(PIT_motor)); //初始化PIT_motor结构体为0
    memset(&Camera_motor, 0, sizeof(Camera_motor));
#if	ROBOT_ID == SUN//21国赛英雄
    pid_abs_param_init(&YAW_motor_pid_pos_imu, 35.0f, 0, 0, 0, 1000.0f);//YAW陀螺仪位置PID
    pid_abs_param_init(&YAW_motor_pid_speed_imu, -150.0f, -0.07f, 0, 3000.0f, 28000.0f);  //YAW陀螺仪速度PID
    pid_abs_param_init(&YAW_motor_pid_pos_encode, 0.0169f, 0, 0, 0, 1200.0f);
    pid_abs_param_init(&YAW_motor_pid_speed_encode, 6000.0f, 0.5f, 0, 0, 28000.0f);
    pid_abs_param_init(&YAW_motor_pid_pos_imu_vis, -0.4f, 0, 0, 0, 1000);//YAW自瞄位置PID
    pid_abs_param_init(&YAW_motor_pid_speed_imu_vis, -80.0f, -3.0f, 0, 150000, 28000);//YAW自瞄速度PID
    pid_abs_param_init(&PIT_motor_pid_pos, 0.3f, 0, 0, 0, 2000.0f);
    pid_abs_param_init(&PIT_motor_pid_speed, 100.0f, 3.0f, 0.0f, 2000.0f,28000.0f);
    pid_abs_param_init(&PIT_motor_pid_pos_imu, 21.0f, 0, 0, 0, 3000.0f);//PIT陀螺仪位置PID
    pid_abs_param_init(&PIT_motor_pid_speed_imu, 140.0f, 6, 0, 30000.0f,28000.0f); //PIT陀螺仪速度PID
    pid_abs_param_init(&PIT_motor_pid_pos_imu_vis, -5.0f, 0, 0, 0, 2000.0f);//PIT自瞄位置PID
    pid_abs_param_init(&PIT_motor_pid_speed_imu_vis, 150.0f, 4.0f, 0, 30000.0f, 28000.0f);//PIT自瞄速度PID

    pid_abs_param_init(&Camera_motor_pid_pos, 0.2, 0, 0, 0, 5000.0f);//2006相机电机PID
    pid_abs_param_init(&Camera_motor_pid_speed, 20, 0.02, 0, 0, 10000.0f);//2006相机电机PID

    s_visionInform.yawCenter   = VISION_X_CENTRE;//自瞄YAW轴画幅中心
    visTarget_t.visYawTarget		   = VISION_X_CENTRE;//令自瞄的输入数据初始化为画幅中心
    s_visionInform.pitchCenter = VISION_Y_CENTRE;//自瞄PIT画幅中心

#endif
#if ROBOT_ID == MOON//22分区赛英雄
    // pid_abs_param_init(&YAW_motor_pid_pos_imu, 27.0f, 0, 0, 0, 2000.0f);
    // pid_abs_param_init(&YAW_motor_pid_speed_imu, -230.0f , -2.0f , 0, 3000.0f, 28000.0f);
    pid_abs_param_init(&YAW_motor_pid_pos_imu, 21.8f, 0, 0, 0, 3000.0f);//YAW陀螺仪位置PID
    pid_abs_param_init(&YAW_motor_pid_speed_imu, -260.0f, -20.0f, 0, 30000.0f, 28000.0f);  //YAW陀螺仪速度PID
    pid_abs_param_init(&YAW_motor_pid_pos_encode, 0.02f, 0, 0, 0, 5000.0f);//YAW吊射位置PID
    pid_abs_param_init(&YAW_motor_pid_speed_encode, 20000.0f, 30.0f, 0, 3000, 28000.0f);  //YAW吊射速度PID
    pid_abs_param_init(&YAW_motor_pid_pos_imu_vis, -0.4f, 0, 0, 0, 1000);//YAW自瞄位置PID
    pid_abs_param_init(&YAW_motor_pid_speed_imu_vis, -80.0f, -3.0f, 0, 15000, 28000);//YAW自瞄速度PID
    // pid_abs_param_init(&PIT_motor_pid_pos, 0.3f, 0, 0, 0, 2000.0f);
    // pid_abs_param_init(&PIT_motor_pid_speed, 90.0f, 2.6f, 0.0f, 2000.0f ,28000.0f);
    pid_abs_param_init(&PIT_motor_pid_pos, 0.28f, 0, 0, 0, 3000.0f);
    pid_abs_param_init(&PIT_motor_pid_speed, 110.0f, 4.0f, 0.0f, 3000.0f,28000.0f);
    pid_abs_param_init(&PIT_motor_pid_pos_imu, 20.0f, 0, 0, 0, 3000.0f);
    pid_abs_param_init(&PIT_motor_pid_speed_imu, 140.0f, 5.0f, 0, 30000.0f,28000.0f);
    pid_abs_param_init(&PIT_motor_pid_pos_imu_vis, -4.8f, 0, 0, 0, 2000.0f);//PIT自瞄位置PID
    pid_abs_param_init(&PIT_motor_pid_speed_imu_vis, 130.0f, 3.5f, 0, 30000.0f, 28000.0f);//PIT自瞄速度PID
    s_visionInform.yawCenter   = VISION_X_CENTRE;//自瞄YAW轴画幅中心
    visTarget_t.visYawTarget		   = VISION_X_CENTRE;//令自瞄的输入数据初始化为画幅中心
    s_visionInform.pitchCenter = VISION_Y_CENTRE;//自瞄PIT画幅中心
#endif
}

/**
  * @brief          云台解算函数，放到Task里
  * @param[in]      none
  * @retval         none
  */
//云台开始计算PID输出电流
void Gimbal_Move()
{
    // PIT轴电机遥控器偏移量，pit_offset_ang是云台相对底盘的变化角度，将pit轴电机的刻度作为底盘角度映射，用云台imu与其相减，为了保护云台不超过限位
    pit_offset_ang = IMU_All_Value.pit.pitAng - (((float)PIT_motor.back_position-PIT_ENCODE_HORIZONTAL)*360)/8192/3.4;
    Gyro_up_lim = PIT_GYRO_UP_LIM + pit_offset_ang;
    Gyro_down_lim = PIT_GYRO_DOWN_LIM  + pit_offset_ang;
    Gyro_mid = PIT_GYRO_MID_R + pit_offset_ang;

#if	ROBOT_ID == SUN
    Camera_Move(robot_Mode, Vedio_transmission_limit_down, Vedio_transmission_limit_up);
#endif

    switch(robot_Mode.gimbalMode)
    {
    case G_NULL://发送电流为0
    {
        pcData.target.isExist = 0x00;//在非自瞄模式，清除识别到目标标志位
        YAW_motor_encode.target_pos       = YAW_motor.serial_position;
        YAW_motor_encode.target_motor_ang = YAW_motor.serial_motor_ang;//上电时的编码器值为ENCODE模式下的目标值
        YAW_motor_imu.target_ang_imu	  = IMU_All_Value.yaw.yawAng;//上电时的角度为FLLOW模式下的目标值
        PIT_motor.out_current=0;
        YAW_motor.out_current=0;
        CANTx_SendCurrent(&hcan1,0x1FF, 0, 0, 0, 0 );
#if	ROBOT_ID == SUN
        CANTx_SendCurrent(&hcan2,0x1FF, 0, 0, 0, 0 );
#endif
#if	ROBOT_ID == MOON
        CANTx_SendCurrent(&hcan2,0x1FF, 0, 0, 0, 0 );
#endif
        break;
    }
    /******************************************************** G_GYRO ***********************************************************************/
    case G_GYRO://YAW位置环陀螺仪角度，PIT位置环陀螺仪角度，二者速度环都是陀螺仪角速度
    {

#if G_GYRO_PIT_DEBUG == 1
        pid_abs_evaluation(&PIT_motor_pid_pos_imu, P1, 0, 0, 0, S1);
        pid_abs_evaluation(&PIT_motor_pid_speed_imu, P2, I2, D2, S2,28000);
#endif
#if G_GYRO_YAW_DEBUG == 1
        pid_abs_evaluation(&YAW_motor_pid_pos_imu, P3, I3, D3, 0, S3);
        pid_abs_evaluation(&YAW_motor_pid_speed_imu, P4, I4, D4, 20000, 28000);
#endif
        pcData.target.isExist = 0x00;//在非自瞄模式，清除识别到目标标志位
        /********************** PIT_motor_IMU************************/
        // 用遥控器值作为pit目标值，以作pid运算（右手云台）
        RC_accumulate_ch_1_gyro  += (RC_Y - MOUSE_Y)*0.01;

        if(RC_accumulate_ch_1_gyro >= Gyro_up_lim) RC_accumulate_ch_1_gyro = Gyro_up_lim;
        else if(RC_accumulate_ch_1_gyro <= Gyro_down_lim) RC_accumulate_ch_1_gyro = Gyro_down_lim;

        PIT_motor_imu.target_ang_imu = RC_accumulate_ch_1_gyro;//Gyro_mid + RC_accumulate_ch_1_gyro;
        PIT_motor.out_current = motor_double_loop_PID(&PIT_motor_pid_pos_imu, &PIT_motor_pid_speed_imu, \
                                IMU_All_Value.pit.pitAng, \
                                (float)PIT_motor_imu.target_ang_imu, \
                                IMU_All_Value.pit.pitAngV * RADIAN_TO_DEGREE);
        RC_accumulate_ch_1=PIT_motor.back_position-PIT_ENCODE_MID_R;
        /********************** YAW_motor IMU PID**********************/
        YAW_motor_imu.target_ang_imu += RC_X - MOUSE_X;

        YAW_motor_imu.target_ang_speed_imu = motor_single_loop_PID(&YAW_motor_pid_pos_imu, YAW_motor_imu.target_ang_imu, \
                                             IMU_All_Value.yaw.yawAng );
        YAW_motor.out_current = motor_single_loop_PID(&YAW_motor_pid_speed_imu, YAW_motor_imu.target_ang_speed_imu, \
                                IMU_All_Value.yaw.yawAngV*150);//+s_chassisMove.W*0.6;
        YAW_motor_encode.target_motor_ang = YAW_motor.serial_motor_ang;//防止半陀螺仪模式转到ENCODE模式时编码器还在刚进入FALLOW模式时的码盘值
        YAW_motor_encode.target_pos       = YAW_motor.serial_position;
        break;
    }
    /**************************************************** G_HALF_GYRO ***********************************************************************/
    case G_HALF_GYRO://YAW位置环陀螺仪角度，PIT位置环编码器刻度，二者速度环都是陀螺仪角速度
    {
#if G_HALF_GYRO_PIT_DEBUG == 1
        pid_abs_evaluation(&PIT_motor_pid_pos, P1, 0, 0, 0, S1);
        pid_abs_evaluation(&PIT_motor_pid_speed, P2, I2, 0, S2,28000);
#endif
#if G_HALF_GYRO_YAW_DEBUG == 1
        // 27.0f, 0, 0, 0, 2000.0f
        // -230.0f , -3f , 0, 3000.0f, 28000.0f
        pid_abs_evaluation(&YAW_motor_pid_pos_imu, P3, I3, D3, 0, S3);
        pid_abs_evaluation(&YAW_motor_pid_speed_imu, P4, I4, D4, S4, 28000);
#endif
        pcData.target.isExist = 0x00;//在非自瞄模式，清除识别到目标标志位
        /********************** PIT_motor*************************/
#if ROBOT_ID == SUN
        RC_accumulate_ch_1  += RC_Y - MOUSE_Y;

        if(RC_accumulate_ch_1 >= RC_accumulate_ch_1_MAX) RC_accumulate_ch_1 = RC_accumulate_ch_1_MAX;
        else if(RC_accumulate_ch_1 <= RC_accumulate_ch_1_MIN) RC_accumulate_ch_1 = RC_accumulate_ch_1_MIN;
        PIT_motor.target_pos = PIT_ENCODE_MID_R + RC_accumulate_ch_1;
        PIT_motor.out_current = motor_double_loop_PID(&PIT_motor_pid_pos, &PIT_motor_pid_speed, \
                                (float)PIT_motor.back_position, \
                                (float)PIT_motor.target_pos, \
                                IMU_All_Value.pit.pitAngV * RADIAN_TO_DEGREE);
        RC_accumulate_ch_1_gyro = IMU_All_Value.pit.pitAng;//防止从HALF_GYRO模式退出时，其目标值还是刚进该模式的值
#endif
#if ROBOT_ID == MOON
        // 用遥控器值作为pit目标值，以作pid运算（右手云台）
        RC_accumulate_ch_1  += RC_Y - MOUSE_Y;//

        if(RC_accumulate_ch_1 >= RC_accumulate_ch_1_MAX) RC_accumulate_ch_1 = RC_accumulate_ch_1_MAX;
        else if(RC_accumulate_ch_1 <= RC_accumulate_ch_1_MIN) RC_accumulate_ch_1 = RC_accumulate_ch_1_MIN;
        // RC_accumulate_ch_1其实是相对PIT枪口水平时电机编码器值的偏移量，做完运算后赋值给pid目标位置
        PIT_motor.target_pos = PIT_ENCODE_MID_R + RC_accumulate_ch_1;
        PIT_motor.out_current = motor_double_loop_PID(&PIT_motor_pid_pos, &PIT_motor_pid_speed, \
                                (float)PIT_motor.back_position, \
                                (float)PIT_motor.target_pos, \
                                IMU_All_Value.pit.pitAngV * RADIAN_TO_DEGREE);

        RC_accumulate_ch_1_gyro = IMU_All_Value.pit.pitAng;//防止从GYRO模式退出时，其目标值还是刚进该模式的值
#endif
        /********************* YAW_motor IMU PID************************/
        YAW_motor_imu.target_ang_imu += RC_X - MOUSE_X;
        YAW_motor.out_current = motor_double_loop_PID_integral_apart(&YAW_motor_pid_pos_imu, &YAW_motor_pid_speed_imu, \
                                IMU_All_Value.yaw.yawAng, \
                                YAW_motor_imu.target_ang_imu, \
                                IMU_All_Value.yaw.yawAngV*150, \
                                90)+s_chassisMove.W*0.6;
        YAW_motor_encode.target_motor_ang = YAW_motor.serial_motor_ang;//防止G_HALF_GYRO模式转到ENCODE模式时编码器还在刚进入FALLOW模式时的码盘值
        YAW_motor_encode.target_pos       = YAW_motor.serial_position;//防止G_HALF_GYRO模式转到ENCODE模式时编码器还在刚进入FALLOW模式时的码盘值
        break;
    }
    /******************************************************* G_ENCODE ***********************************************************************/
    case G_ENCODE://YAW位置环编码器刻度，PIT位置环编码器刻度，二者速度环都是陀螺仪角速度
    {
#if G_ENCODE_PIT_DEBUG == 1
        pid_abs_evaluation(&PIT_motor_pid_pos, P1, 0, 0, 0, 300);
        pid_abs_evaluation(&PIT_motor_pid_speed, P2, I2, D2, 2000,28000);
#endif
#if G_ENCODE_YAW_DEBUG == 1
        pid_abs_evaluation(&YAW_motor_pid_pos_encode, P3, 0, 0, 0, S3);
        pid_abs_evaluation(&YAW_motor_pid_speed_encode, P4, I4, 0, 3000, 28000);
#endif
        pcData.target.isExist = 0x00;//在非自瞄模式，清除识别到目标标志位
        /**************************** key_crl ******************************/
        static uint8_t keyLock_W,keyLock_A,keyLock_S,keyLock_D = 0;//按键标志位锁，置1后要清零
        //在吊射模式下，wasd用作控制云台，按住shift让云台PIT轴转的更快，mutiple是倍数的意思
        static float mutiple_pit = 0;//pit轴按键倍数值（W,S）
        static float mutiple_yaw = 0;//yaw轴按键倍数值（A，D）
        static float pit_key_crl = 0;//pit轴按键控制值
        static float yaw_key_crl = 0;//yaw轴按键控制值
        static uint16_t w_continuous_move = 0;//w键长按计数值
        static uint16_t s_continuous_move = 0;//s键长按计数值
        static uint16_t a_continuous_move = 0;//a键长按计数值
        static uint16_t d_continuous_move = 0;//d键长按计数值

        if(PRESS_SHIFT)
        {
            mutiple_pit = 1.6;//按住A/D后pit增加的速度倍数
            mutiple_yaw = 0.17;//按住W/S后yaw增加的速度倍数
            yaw_key_crl = PRESS_A*mutiple_yaw - PRESS_D*mutiple_yaw;
            pit_key_crl = PRESS_W*mutiple_pit - PRESS_S*mutiple_pit;
        }
        else
        {
            mutiple_pit = 2;//0.1;
            mutiple_yaw = 2;//0.001;
            /*********pit_key********/
            if(PRESS_W)//按下W键
            {
                keyLock_W=1;
                ++w_continuous_move;
                if(w_continuous_move>200)
                {
                    mutiple_pit = 0.1;
                    pit_key_crl = PRESS_W*mutiple_pit;
                }
            }
            if((!PRESS_W) && keyLock_W==1) {
                RC_accumulate_ch_1+=(1*mutiple_pit);
                keyLock_W=0;
                pit_key_crl=0;
                w_continuous_move=0;
            }
            if(PRESS_S)//按下S键
            {
                keyLock_S=1;
                ++s_continuous_move;
                if(s_continuous_move>200)
                {
                    mutiple_pit = 0.1;
                    pit_key_crl = -PRESS_S*mutiple_pit;
                }
            }
            if((!PRESS_S) && keyLock_S==1) {
                RC_accumulate_ch_1+=(-1*mutiple_pit);
                keyLock_S=0;
                pit_key_crl=0;
                s_continuous_move=0;
            }
            /*********yaw_key********/
            if(PRESS_A)//按下A键
            {
                keyLock_A=1;
                ++a_continuous_move;
                if(a_continuous_move>200)
                {
                    mutiple_yaw = 0.01;
                    yaw_key_crl = PRESS_A*mutiple_yaw;
                }
            }
            if((!PRESS_A) && keyLock_A==1) {
                YAW_motor_encode.target_pos-=(1*mutiple_yaw);
                keyLock_A=0;
                yaw_key_crl=0;
                a_continuous_move=0;
            }
            if(PRESS_D)//按下D键
            {
                keyLock_D=1;
                ++d_continuous_move;
                if(d_continuous_move>200)
                {
                    mutiple_yaw = 0.01;
                    yaw_key_crl = -PRESS_D*mutiple_yaw;
                }
            }
            if((!PRESS_D) && keyLock_D==1) {
                YAW_motor_encode.target_pos-=(-1*mutiple_yaw);
                keyLock_D=0;
                yaw_key_crl=0;
                d_continuous_move=0;
            }
            // yaw_key_crl = PRESS_A*mutiple_yaw - PRESS_D*mutiple_yaw;
            // pit_key_crl = PRESS_W*mutiple_pit - PRESS_S*mutiple_pit;
        }
        if((!PRESS_SHIFT)&&(!PRESS_W)&&(!PRESS_S)) pit_key_crl=0;//如果同时按下shift+w+s，则令pit_key_crl值为0
        if((!PRESS_SHIFT)&&(!PRESS_A)&&(!PRESS_D)) yaw_key_crl=0;//如果同时按下shift+a+d，则令pit_key_crl值为0
        //if((!PRESS_SHIFT)&&(!PRESS_W)&&(!PRESS_A)&&(!PRESS_S)&&(!PRESS_D)) {pit_key_crl=0;yaw_key_crl=0;}
#if ROBOT_ID == SUN
        /**************************** PIT_motor *****************************/
        RC_accumulate_ch_1  += rc_ctrl.rc.ch[1]*0.01f - rc_ctrl.mouse.y*0.3*PRESS_MOUSE_R+ pit_key_crl;//PRESS_W*mutiple_pit - PRESS_S*mutiple_pit;

        if(RC_accumulate_ch_1 >= RC_accumulate_ch_1_MAX) RC_accumulate_ch_1 = RC_accumulate_ch_1_MAX;
        else if(RC_accumulate_ch_1 <= RC_accumulate_ch_1_MIN) RC_accumulate_ch_1 = RC_accumulate_ch_1_MIN;

        PIT_motor.target_pos = PIT_ENCODE_MID_R + RC_accumulate_ch_1;
        PIT_motor.out_current = motor_double_loop_PID(&PIT_motor_pid_pos, &PIT_motor_pid_speed, \
                                (float)PIT_motor.back_position, \
                                (float)PIT_motor.target_pos, \
                                IMU_All_Value.pit.pitAngV * RADIAN_TO_DEGREE);
        RC_accumulate_ch_1_gyro = IMU_All_Value.pit.pitAng;
        PIT_motor_imu.target_ang_imu = IMU_All_Value.pit.pitAng;//防止从HALF_GYRO模式退出时，PIT轴陀螺仪角度的值还是刚进该模式的值
        /************************ YAW_motor ENCODE PID***********************/
        YAW_motor_encode.target_pos -= -(rc_ctrl.rc.ch[0]*0.05)/66 - rc_ctrl.mouse.x*0.105*PRESS_MOUSE_R + yaw_key_crl;
        //PRESS_A*mutiple_yaw - PRESS_D*mutiple_yaw;//键盘A\D控制云台YAW左右

        YAW_motor.out_current = motor_double_loop_PID(&YAW_motor_pid_pos_encode, &YAW_motor_pid_speed_encode, \
                                YAW_motor.serial_position,	\
                                /*(double)YAW_motor_encode.target_motor_ang,	\*/
                                YAW_motor_encode.target_pos, \
                                -IMU_All_Value.yaw.yawAngV * 2);
        //YAW_motor.back_motor_speed);
        YAW_motor_imu.target_ang_imu = IMU_All_Value.yaw.yawAng;//防止ENCODE模式转到其它模式时陀螺仪还在刚进入ENCODE模式时的角度
#endif
#if ROBOT_ID == MOON
        /**************************** PIT_motor *****************************/
        // 用遥控器值作为pit目标值，以作pid运算（右手云台），pit外环为6020编码器值，只有按住鼠标右键的情况下才允许鼠标控制
        RC_accumulate_ch_1  += rc_ctrl.rc.ch[1]*0.01f - rc_ctrl.mouse.y*0.3*PRESS_MOUSE_R+ pit_key_crl;

        if(RC_accumulate_ch_1 >= RC_accumulate_ch_1_MAX) RC_accumulate_ch_1 = RC_accumulate_ch_1_MAX;
        else if(RC_accumulate_ch_1 <= RC_accumulate_ch_1_MIN) RC_accumulate_ch_1 = RC_accumulate_ch_1_MIN;

        PIT_motor.target_pos = PIT_ENCODE_MID_R + RC_accumulate_ch_1;
        PIT_motor.out_current = motor_double_loop_PID(&PIT_motor_pid_pos, &PIT_motor_pid_speed, \
                                (float)PIT_motor.back_position, \
                                (float)PIT_motor.target_pos, \
                                IMU_All_Value.pit.pitAngV * RADIAN_TO_DEGREE);
        RC_accumulate_ch_1_gyro = IMU_All_Value.pit.pitAng;//防止从GYRO模式退出时，其目标值还是刚进该模式的值
        PIT_motor_imu.target_ang_imu = IMU_All_Value.pit.pitAng;//防止从HALF_GYRO模式退出时，PIT轴陀螺仪角度的值还是刚进该模式的值
        /************************ YAW_motor ENCODE PID***********************/
        YAW_motor_encode.target_pos -= -(rc_ctrl.rc.ch[0]*0.05)/66 - rc_ctrl.mouse.x*0.105*PRESS_MOUSE_R + yaw_key_crl;
        //PRESS_A*mutiple_yaw - PRESS_D*mutiple_yaw;//键盘A\D控制云台YAW左右
        YAW_motor.out_current = motor_double_loop_PID_integral_apart(&YAW_motor_pid_pos_encode, &YAW_motor_pid_speed_encode, \
                                YAW_motor.serial_position,	\
                                /*(double)YAW_motor_encode.target_motor_ang,	\*/
                                YAW_motor_encode.target_pos, \
                                -IMU_All_Value.yaw.yawAngV * 2, \
                                4);
        //YAW_motor.back_motor_speed);
        YAW_motor_imu.target_ang_imu = IMU_All_Value.yaw.yawAng;//防止ENCODE模式转到其它模式时陀螺仪还在刚进入ENCODE模式时的角度
#endif
        break;
    }
    /******************************************************* G_HALF_VISION *******************************************************************/
    case G_HALF_VISION://半自动自瞄，YAW轴手动，PIT轴自动
    {
#if G_HALF_VISION_PIT_DEBUG == 1
        pid_abs_evaluation(&PIT_motor_pid_pos_imu_vis, P1, 0, 0, 0, S1);//PIT自瞄位置PID
        pid_abs_evaluation(&PIT_motor_pid_speed_imu_vis, P2, I2, 0, S2, 28000);//PIT自瞄速度PID
#endif
        /********************* YAW_motor IMU PID************************/
        YAW_motor_imu.target_ang_imu += RC_X - MOUSE_X ;

        YAW_motor.out_current = motor_double_loop_PID_integral_apart(&YAW_motor_pid_pos_imu, &YAW_motor_pid_speed_imu, \
                                IMU_All_Value.yaw.yawAng, \
                                YAW_motor_imu.target_ang_imu, \
                                IMU_All_Value.yaw.yawAngV*150, \
                                90)+s_chassisMove.W*0.6;
        YAW_motor_encode.target_motor_ang = YAW_motor.serial_motor_ang;
        YAW_motor_encode.target_pos       = YAW_motor.serial_position;//防止FULL_VIS模式转到ENCODE模式时编码器还是刚进该模式时的值
        /********************* PIT_motor VIS PID************************/
        /********************************************* SUN *****************************************************/
#if ROBOT_ID==SUN
        //如果识别到的话
        if(s_visionInform.yawPos > 0.0f && s_visionInform.distance > 1.0f && finalFps.pc > 0.95*PC_FPS)
        {
            pcData.target.isExist = 0x01;//是否识别到目标标志位置1，这个值会发送给底盘C板用作UI显示
            PIT_motor.out_current=motor_double_loop_PID(&PIT_motor_pid_pos_imu_vis, &PIT_motor_pid_speed_imu_vis, \
                                  0.0f, \
                                  -visTarget_t.visPitTarget, \
                                  IMU_All_Value.pit.pitAngV*RADIAN_TO_DEGREE);
#if PIT_MOTOR == GM6020
            RC_accumulate_ch_1=PIT_motor.back_position-PIT_ENCODE_MID_R;
            RC_accumulate_ch_1_gyro = IMU_All_Value.pit.pitAng;
#endif
        }
        //如果没识别到，进入PIT轴编码器手动控制模式
        else
        {
            pcData.target.isExist = 0x00;
            RC_accumulate_ch_1_gyro  += (RC_Y - MOUSE_Y)*0.05;

            if(RC_accumulate_ch_1_gyro >= Gyro_up_lim) RC_accumulate_ch_1_gyro = Gyro_up_lim;
            else if(RC_accumulate_ch_1_gyro <= Gyro_down_lim) RC_accumulate_ch_1_gyro = Gyro_down_lim;

            PIT_motor_imu.target_ang_imu = RC_accumulate_ch_1_gyro;//Gyro_mid + RC_accumulate_ch_1_gyro;
            PIT_motor.out_current = motor_double_loop_PID(&PIT_motor_pid_pos_imu, &PIT_motor_pid_speed_imu, \
                                    IMU_All_Value.pit.pitAng, \
                                    (float)PIT_motor_imu.target_ang_imu, \
                                    IMU_All_Value.pit.pitAngV * RADIAN_TO_DEGREE);
            RC_accumulate_ch_1=PIT_motor.back_position-PIT_ENCODE_MID_R;
            break;
        }
#endif
        /******************************************** MOON *****************************************************/
#if ROBOT_ID==MOON
        //如果识别到并且pc的帧率在190帧以上，认为识别到了，为什么是190呢，因为pc帧率有波动，如果掉到200以下认为识别不到肯定不行啊
        if(s_visionInform.distance > 1.0f && finalFps.pc > 190)
        {
            pcData.target.isExist = 0x01;//是否识别到目标标志位置1，这个值会发送给底盘C板用作UI显示
            PIT_motor.out_current=motor_double_loop_PID(&PIT_motor_pid_pos_imu_vis, &PIT_motor_pid_speed_imu_vis, \
                                  0.0f, \
                                  -visTarget_t.visPitTarget, \
                                  IMU_All_Value.pit.pitAngV*RADIAN_TO_DEGREE);
#if PIT_MOTOR == GM6020
            RC_accumulate_ch_1=PIT_motor.back_position-PIT_ENCODE_MID_R;//防止从HALF_GYRO模式退出时，其目标值还是刚进该模式的值
            RC_accumulate_ch_1_gyro = IMU_All_Value.pit.pitAng;//防止从GYRO模式退出时，其目标值还是刚进该模式的值
#endif
        }
        //如果没识别到，进入PIT轴编码器手动控制模式
        else
        {
            pcData.target.isExist = 0x00;
            // 用遥控器值作为pit目标值，以作pid运算（右手云台），陀螺仪角度作外环，相当于GYRO模式
            RC_accumulate_ch_1_gyro  += (RC_Y - MOUSE_Y)*0.05;

            if(RC_accumulate_ch_1_gyro >= Gyro_up_lim) RC_accumulate_ch_1_gyro = Gyro_up_lim;
            else if(RC_accumulate_ch_1_gyro <= Gyro_down_lim) RC_accumulate_ch_1_gyro = Gyro_down_lim;

            PIT_motor_imu.target_ang_imu = RC_accumulate_ch_1_gyro;//Gyro_mid + RC_accumulate_ch_1_gyro;
            PIT_motor.out_current = motor_double_loop_PID(&PIT_motor_pid_pos_imu, &PIT_motor_pid_speed_imu, \
                                    IMU_All_Value.pit.pitAng, \
                                    (float)PIT_motor_imu.target_ang_imu, \
                                    IMU_All_Value.pit.pitAngV * RADIAN_TO_DEGREE);
            RC_accumulate_ch_1=PIT_motor.back_position-PIT_ENCODE_MID_R;//防止从HALF_GYRO模式退出时，其目标值还是刚进该模式的值
            break;
        }
#endif
        break;
    }

    /******************************************************* G_FULL_VISION *******************************************************************/
    case G_FULL_VISION://全自动自瞄，YAW、PIT轴全自动
    {
        static float omega_offset = 0;
#if G_FULL_VISION_PIT_DEBUG == 1
        /*-5.0f		0		0		0			2000.0f*/
        /*180.0f	6.0f	0		30000.0f	28000.0f*/
        pid_abs_evaluation(&PIT_motor_pid_pos_imu_vis, P1, 0, 0, 0, S1);
        pid_abs_evaluation(&PIT_motor_pid_speed_imu_vis, P2, I2, D2, S2,28000);
#endif
#if G_FULL_VISION_YAW_DEBUG == 1
        //-0.4f, 0, 0, 0, 1000
        // -80.0f, -3.0f, 0, 15000, 28000
        pid_abs_evaluation(&YAW_motor_pid_pos_imu_vis, P3, 0, 0, 0, S3);
        pid_abs_evaluation(&YAW_motor_pid_speed_imu_vis, P4, I4, 0, S4, 28000);
#endif
        /********************************************* SUN *****************************************************/
#if ROBOT_ID == SUN
        //如果识别到的话
        if(s_visionInform.yawPos > 0.0f && s_visionInform.distance > 1.0f && finalFps.pc > 0.95*PC_FPS)
        {

            pcData.target.isExist = 0x01;//是否识别到目标标志位置1，这个值会发送给底盘C板用作UI显示
            /********************* PIT_motor VIS PID************************/
            PIT_motor.out_current=motor_double_loop_PID(&PIT_motor_pid_pos_imu_vis, &PIT_motor_pid_speed_imu_vis, \
                                  0.0f, \
                                  -visTarget_t.visPitTarget, \
                                  IMU_All_Value.pit.pitAngV*RADIAN_TO_DEGREE);
#if PIT_MOTOR == GM6020
            RC_accumulate_ch_1=PIT_motor.back_position-PIT_ENCODE_MID_R;
            RC_accumulate_ch_1_gyro = IMU_All_Value.pit.pitAng;
#endif
            /********************* YAW_motor VIS PID************************/

            if(s_chassisMove.W == 3000) omega_offset = 2.1;
            else if(s_chassisMove.W == 4000) omega_offset = 1.54;
            else if(s_chassisMove.W == 5000) omega_offset = 1.685;
            else omega_offset = 1.8;

            YAW_motor.out_current = motor_double_loop_PID_integral_apart(&YAW_motor_pid_pos_imu_vis, &YAW_motor_pid_speed_imu_vis, \
                                    /*P9,	*/
                                    s_visionInform.yawCenter, \
                                    visTarget_t.visYawTarget, \
                                    IMU_All_Value.yaw.yawAngV*150, \
                                    100)-s_chassisMove.W*omega_offset;
            YAW_motor_imu.target_ang_imu = IMU_All_Value.yaw.yawAng;//防止FULL_VIS模式转到其它模式时陀螺仪还在刚进该模式时的角度
            YAW_motor_encode.target_motor_ang = YAW_motor.serial_motor_ang;//防止FULL_VIS模式转到ENCODE模式时编码器还在刚进该模式时的值
            YAW_motor_encode.target_pos       = YAW_motor.serial_position;
        }
        //如果未识别到的话,进入G_HALF_GYRO模式（单纯把调好的G_HALF_GYRO程序拷贝到这里）
        else
        {
            pcData.target.isExist = 0x00;
            /********************* PIT_motor ENCODE PID*********************/
            RC_accumulate_ch_1_gyro  += (RC_Y - MOUSE_Y)*0.01;

            if(RC_accumulate_ch_1_gyro >= Gyro_up_lim) RC_accumulate_ch_1_gyro = Gyro_up_lim;
            else if(RC_accumulate_ch_1_gyro <= Gyro_down_lim) RC_accumulate_ch_1_gyro = Gyro_down_lim;

            PIT_motor_imu.target_ang_imu = RC_accumulate_ch_1_gyro;
            PIT_motor.out_current = motor_double_loop_PID(&PIT_motor_pid_pos_imu, &PIT_motor_pid_speed_imu, \
                                    IMU_All_Value.pit.pitAng, \
                                    (float)PIT_motor_imu.target_ang_imu, \
                                    IMU_All_Value.pit.pitAngV * RADIAN_TO_DEGREE);
            RC_accumulate_ch_1=PIT_motor.back_position-PIT_ENCODE_MID_R;
            /********************* YAW_motor IMU PID************************/
            YAW_motor_imu.target_ang_imu += RC_X - MOUSE_X ;

            YAW_motor_imu.target_ang_speed_imu = motor_single_loop_PID(&YAW_motor_pid_pos_imu, YAW_motor_imu.target_ang_imu, \
                                                 IMU_All_Value.yaw.yawAng );
            YAW_motor.out_current = motor_single_loop_PID(&YAW_motor_pid_speed_imu, YAW_motor_imu.target_ang_speed_imu, \
                                    IMU_All_Value.yaw.yawAngV*150 )+s_chassisMove.W*0.8;
            YAW_motor_encode.target_motor_ang = YAW_motor.serial_motor_ang;//防止FULL_VIS模式转到ENCODE模式时编码器还是刚进该模式时的值
            YAW_motor_encode.target_pos       = YAW_motor.serial_position;
        }
#endif
        /******************************************** MOON *****************************************************/
#if ROBOT_ID == MOON
        //如果识别到的话
        if(s_visionInform.yawPos > 0.0f && s_visionInform.distance > 1.0f && finalFps.pc > 190)
        {
            pcData.target.isExist = 0x01;//是否识别到目标标志位置1，这个值会发送给底盘C板用作UI显示
            /********************* PIT_motor VIS PID************************/
            PIT_motor.out_current=motor_double_loop_PID(&PIT_motor_pid_pos_imu_vis, &PIT_motor_pid_speed_imu_vis, \
                                  0.0f, \
                                  -visTarget_t.visPitTarget, \
                                  IMU_All_Value.pit.pitAngV*RADIAN_TO_DEGREE);
#if PIT_MOTOR == GM6020
            RC_accumulate_ch_1=PIT_motor.back_position-PIT_ENCODE_MID_R;//防止从HALF_GYRO模式退出时，其目标值还是刚进该模式的值
            RC_accumulate_ch_1_gyro = IMU_All_Value.pit.pitAng;//防止从GYRO模式退出时，其目标值还是刚进该模式的值
#endif
            /********************* YAW_motor VIS PID************************/
#if ROBOT_ID == SUN
            if(s_chassisMove.W == 3000) omega_offset = 2.1;
            else if(s_chassisMove.W == 4000) omega_offset = 1.54;
            else if(s_chassisMove.W == 5000) omega_offset = 1.685;
            else omega_offset = 1.8;
#endif
#if ROBOT_ID == MOON
            //前馈
            if(s_chassisMove.W == 3000) omega_offset = 2.05;
            else if(s_chassisMove.W == 4000) omega_offset = 1.87;
            else if(s_chassisMove.W == 5000) omega_offset = 1.74;
            else omega_offset = 1.8;
#endif
            YAW_motor.out_current = motor_double_loop_PID_integral_apart(&YAW_motor_pid_pos_imu_vis, &YAW_motor_pid_speed_imu_vis, \
                                    /*P9, \*/
                                    s_visionInform.yawCenter, \
                                    visTarget_t.visYawTarget, \
                                    IMU_All_Value.yaw.yawAngV*150, \
                                    100)-s_chassisMove.W*omega_offset;
            YAW_motor_imu.target_ang_imu = IMU_All_Value.yaw.yawAng;//防止FULL_VIS模式转到其它模式时陀螺仪还在刚进该模式时的角度
            YAW_motor_encode.target_motor_ang = YAW_motor.serial_motor_ang;//防止FULL_VIS模式转到ENCODE模式时编码器还在刚进该模式时的值
            YAW_motor_encode.target_pos       = YAW_motor.serial_position;//防止FULL_VIS模式转到ENCODE模式时编码器还在刚进该模式时的值
        }
        //如果未识别到的话,进入G_HALF_GYRO模式（单纯把调好的G_HALF_GYRO程序拷贝到这里）
        else
        {
            pcData.target.isExist = 0x00;
            /********************* PIT_motor ENCODE PID*********************/
            // 用遥控器值作为pit目标值，以作pid运算（右手云台），陀螺仪角度作外环，相当于GYRO模式
            RC_accumulate_ch_1_gyro  += (RC_Y - MOUSE_Y)*0.01;

            if(RC_accumulate_ch_1_gyro >= Gyro_up_lim) RC_accumulate_ch_1_gyro = Gyro_up_lim;
            else if(RC_accumulate_ch_1_gyro <= Gyro_down_lim) RC_accumulate_ch_1_gyro = Gyro_down_lim;

            PIT_motor_imu.target_ang_imu = RC_accumulate_ch_1_gyro;
            PIT_motor.out_current = motor_double_loop_PID(&PIT_motor_pid_pos_imu, &PIT_motor_pid_speed_imu, \
                                    IMU_All_Value.pit.pitAng, \
                                    (float)PIT_motor_imu.target_ang_imu, \
                                    IMU_All_Value.pit.pitAngV * RADIAN_TO_DEGREE);
            RC_accumulate_ch_1=PIT_motor.back_position-PIT_ENCODE_MID_R;//防止从HALF_GYRO模式退出时，其目标值还是刚进该模式的值
            /********************* YAW_motor IMU PID************************/
            YAW_motor_imu.target_ang_imu += RC_X - MOUSE_X ;

            YAW_motor.out_current = motor_double_loop_PID_integral_apart(&YAW_motor_pid_pos_imu, &YAW_motor_pid_speed_imu, \
                                    IMU_All_Value.yaw.yawAng, \
                                    YAW_motor_imu.target_ang_imu, \
                                    IMU_All_Value.yaw.yawAngV*150, \
                                    90)+s_chassisMove.W*0.6;
            YAW_motor_encode.target_motor_ang = YAW_motor.serial_motor_ang;//防止FULL_VIS模式转到ENCODE模式时编码器还是刚进该模式时的值
            YAW_motor_encode.target_pos       = YAW_motor.serial_position;//防止FULL_VIS模式转到ENCODE模式时编码器还是刚进该模式时的值
        }
#endif
        break;
    }
    }
    /********************************************************* Output current ****************************************************************/\
#if	ROBOT_ID == SUN
    can_send_state.yaw = (&hcan1,0x1FF, YAW_motor.out_current, Camera_motor.out_current, 0, 0 );   //YAW轴发送电流
#endif
#if	ROBOT_ID == MOON
    can_send_state.yaw = CANTx_SendCurrent(&hcan1,0x1FF, YAW_motor.out_current, 0, 0, 0 );   //YAW轴发送电流
#endif
#if	ROBOT_ID == SUN   //PIT轴发送电流,Camera电流
    can_send_state.pit = (&hcan2,0x1FF, PIT_motor.out_current, 0, 0, 0 );
#endif
#if	ROBOT_ID == MOON
#if PIT_MOTOR==GM6020
    can_send_state.pit = CANTx_SendCurrent(&hcan2,0x1FF, PIT_motor.out_current, 0, 0, 0 );
#endif
#endif

}

