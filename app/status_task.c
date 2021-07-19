#include "status_task.h"
#include "tim.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "bsp_T_imu.h"
#include "stdbool.h"
#include "remote_msg.h"

status_t status;

static void status_deinit(void);
static void status_restore(void);

/**
  * @brief status_task
  * @param
  * @attention
	* @note
  */
void status_task(void const *argu)
{
    for(;;)
    {
        static uint16_t cnt = 0;
        static uint8_t led_status = 0;
        
        taskENTER_CRITICAL();
        /* 遥控器通信状态检查 */
        //遥控通信周期大约14ms，此任务，每100ms检查一次中断标志
        cnt++;
        if(rc_normal_flag)  //当通信正常时
        {
            status.rc_status = 1;  //系统状态标志置1
            rc_normal_flag = 0;  //清除串口中断标志
            
            /* LED 状态显示 */
            if( cnt == 1 )
            {
                /* 陀螺仪状态 LED4 */
                if( status.gyro_status[0] && status.gyro_status[1] &&\
                        (imu_data.wz * imu_data.pitch * imu_data.yaw) != 0)
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
            }
            if( cnt == 3 )
            {
                /* 底盘状态 LED3 */
                if( status.chassis_status[0] && status.chassis_status[1] && \
                        status.chassis_status[2] && status.chassis_status[3] )
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
            }
            if( cnt == 5 )
            {
                /* 云台状态 LED2 */
                if( status.gimbal_status[0] && status.gimbal_status[1] && \
                        status.gimbal_status[2] )
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
            }
            if( cnt == 7 )
            {
                /* 视觉或功率控制状态 LED1 */
                if( status.power_control )  //status.vision_status
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, (GPIO_PinState)led_status);
                else
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
                
                /* 改变亮灭状态 开始下一个循环周期 */
                led_status = !led_status;
                cnt = 0;
            }
        }
        else  //遥控器失联
        {
            if(cnt >= 2)
            {
                led_status = !led_status;
                cnt = 0;
                
                if( led_status )
                    status_init();  //LED全部熄灭
                else
                    status_deinit();  //全部亮起
            }
            status.rc_status = 0;  //系统状态标志清0
        }
        
        /* 复位 */
        status_restore();
        taskEXIT_CRITICAL();
        osDelay(100);
    }
}

void status_init(void)  //灭灯
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_SET);  //LED_D
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_SET);  //LED_C
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);  //LED_B
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);  //LED_A
}
static void status_deinit(void)  //亮灯
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  GPIO_PIN_RESET);  //LED_D
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  GPIO_PIN_RESET);  //LED_C
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);  //LED_B
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);  //LED_A
}

static void status_restore(void)  //清除标志位
{
    status.dbus_status = 0;
    status.power_control = 0;
    for(int i = 0; i<2; i++)
        status.gyro_status[i] = 0;
    for(int i = 0; i<4; i++)
        status.chassis_status[i] = 0;
    for(int i = 0; i<3; i++)
        status.gimbal_status[i] = 0;
}
