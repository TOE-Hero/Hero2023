#include "cmsis_os.h"
#include "main.h"
#include "adc_task.h"
#include "bsp_adc.h"

fp32    chip_voltage;//电池电压
fp32    chip_temperature;//片内温度

/**
 * @brief adc检测电池电压和片内温度任务
 * 
 * @param argument 
 */
void adc_task(void const * argument)
{
    init_vrefint_reciprocal();
    while(1)
    {
        // 获取电池电压
        chip_voltage = get_battery_voltage();
        //get chip temperate
        // 获取片内温度
        chip_temperature = get_temprate();
        // get handware version
        osDelay(100);
    }
}


