#include "main.h"
#include "dma.h"
#include "usart.h"
#include "Bsp_RC.h"
#include "string.h"
#include "Monitor.h"

/**************************** global variable ************************************/
RC_ctrl_t rc_ctrl;//遥控器控制变量
/************************** extern declaration ***********************************/
extern s_FPS_monitor	Fps;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
/*********************************************************************************/
/**
  * @brief          官方例程的遥控器初始化函数
  * @param[in]      uint8_t *rx1_buf
  * @param[in]      uint8_t *rx2_buf
  * @param[in]      uint16_t dma_buf_num
  * @retval         none
  */
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

/**
  * @brief          遥控器协议解析
  * @param[in]      uint8_t *sbus_buf: 原生数据指针
  * @param[out]     RC_ctrl_t *rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);


//receive data, 18 bytes one frame, but set 36 bytes 
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          遥控器初始化，放到main.c里
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


//UART3串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);//清除中断标志位
    }
    else if(USART3->SR & UART_FLAG_IDLE)//发生空闲中断时
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);
		/*DMA_SxCR_CT对应DMA数据流x配置寄存器（DMA_SxCR）位19：CT：当前目标（仅双缓冲模式），由硬件/软件置1/0，EN位为0时该位才可写入，
		  用来指示第一次传输的目标存储区。在使能数据流后，该位相当于一个状态标志，用于指示作为当前目标存储区。*/
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)//判断当前目标存储器是否为存储器0
        {
            /* Current memory buffer used is Memory 0 */
    
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            /*get receive data length, length = set_data_length - remain_length
            获取接收数据长度,长度 = 设定长度 - 剩余长度
			      DMA数据项数寄存器（NDTR）禁止数据流时才能写入，值0~65535，使能数据流后，改寄存器只读，指示要传输的剩余数据数目，每次DMA传输后，
			      该寄存器递减。当传输完成后，该寄存器保持为0（数据流正常时），或是在以下情况自动以先前写入的值重载
			      1.以循环模式配置数据流时
			      2.将DMA数据流x配置寄存器（DMA_SxCR）EN位（D[0]）重新使能为1时（重新使能数据流时自动重载）
			      NDTR寄存器值为0，即使使能了数据流，也无法完成任何事务*/
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);//CT值此时为1.非运算成0，CR再与其相与，将缓冲区位置设置为memery0
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}


/**
  * @brief          遥控器协议解析
  * @param[in]      uint8_t *sbus_buf: 原生数据指针
  * @param[out]     RC_ctrl_t *rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  		//!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL
    /*令遥控器油门中值为0*/
    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	Fps.dbus++;
	isInit_Ok();
}

/**
  * @brief          官方例程的遥控器初始化函数
  * @param[in]      uint8_t *rx1_buf
  * @param[in]      uint8_t *rx2_buf
  * @param[in]      uint16_t dma_buf_num
  * @retval         none
  */
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

/**
  * @brief          官方例程的遥控器失能函数
  * @param[in]      none
  * @retval         none
  */
void RC_unable(void)
{
    __HAL_UART_DISABLE(&huart3);
}
/**
  * @brief          官方例程的遥控器重新使能函数
  * @param[in]      none
  * @retval         none
  */
void RC_restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart3);
    __HAL_DMA_DISABLE(&hdma_usart3_rx);

    hdma_usart3_rx.Instance->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_UART_ENABLE(&huart3);

}

