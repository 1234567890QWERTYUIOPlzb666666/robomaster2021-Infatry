#include "comm_task.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "pid.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "bsp_usart.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "string.h"
#include "bsp_can.h"
#include "bsp_judge.h"
#include "bsp_T_imu.h"
#include "control_def.h"

motor_current_t motor_cur;
vision_tx_msg_t		vision_tx_msg = {0};

/**
  * @brief can_msg_send_task
  * @param
  * @attention
  * @note
  */
void can_msg_send_task(void const *argu)
{
    osEvent event;
    for(;;)
    {
        event = osSignalWait(GIMBAL_MOTOR_MSG_SEND  | \
                             CHASSIS_MOTOR_MSG_SEND | \
                             SHOOT_MOTOR_MSG_SEND, osWaitForever);
        if( event.status == osEventSignal )
        {
            if( ctrl_mode==PROTECT_MODE || !lock_flag )
            {
                for(int i=0; i<4; i++)		motor_cur.chassis_cur[i]= 0;
                for(int i=0; i<2; i++)		motor_cur.gimbal_cur[i] = 0;
                motor_cur.trigger_cur = 0;
                can1_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
                can2_send_message(GIMBAL_CAN_TX_ID, 0, 0, 0, 0);
                can1_send_message(CHASSIS_CAN_TX_ID,0, 0, 0, 0);
            }
            else if( lock_flag )  //�����������ݲŸ����� && (imu_data.wz * imu_data.pitch * imu_data.yaw) != 0
            {
                if( event.value.signals & GIMBAL_MOTOR_MSG_SEND )
                {
                    can1_send_message(GIMBAL_CAN_TX_ID, motor_cur.gimbal_cur[0], 0, 0, 0);
                    can2_send_message(GIMBAL_CAN_TX_ID, 0, motor_cur.gimbal_cur[1], motor_cur.trigger_cur, 0);
                }
                if( event.value.signals & CHASSIS_MOTOR_MSG_SEND )
                {
                    can1_send_message(CHASSIS_CAN_TX_ID,motor_cur.chassis_cur[0], motor_cur.chassis_cur[1], motor_cur.chassis_cur[2], motor_cur.chassis_cur[3]);
                }
                can1_send_supercap();
            }
        }
    }
}

/**
  * @brief usart_msg_send_task
  */
float pit_angle_error_buf = 0;
void usart_msg_send_task(void const *argu)
{
    uint8_t vision_tx_buf[8];
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        taskENTER_CRITICAL();
        /* ֡ͷ */
        vision_tx_msg.SOF = 0x11;
        /* ��ȡPIT��ˮƽ�ǶȲ� */
        pit_angle_error_buf = ((moto_pit.ecd - GIMBAL_PIT_CENTER_OFFSET) / (22.752f));
        memcpy(vision_tx_msg.pit_angle_error, &pit_angle_error_buf, 4);
        /* ��ȡ������Ϣ */
        vision_tx_msg.mode_msg.shooter_speed = shoot.shoot_speed_vision;
        /* (������/����)ģʽ���� */
        keyboard_scanf(KEY_VISION_ANTIROTATE);
        keyboard_scanf(KEY_VISION_bENERGY);
        if(FLAG_VISION_ANTIROTATE == -1)
            vision_tx_msg.mode_msg.aiming_status = 1;	//������
        else if(FLAG_VISION_bENERGY == -1)
            vision_tx_msg.mode_msg.aiming_status = 4;	//����
        else
            vision_tx_msg.mode_msg.aiming_status = 0;	//����
        if(Game_Robot_Status.robot_id>100)		vision_tx_msg.mode_msg.camp=1;
        else 																	vision_tx_msg.mode_msg.camp=0;
        /* ֡β */
        vision_tx_msg.EOF1 = 0x22;
        vision_tx_msg.EOF2 = 0x33;

        /* ����Ӿ�������Ϣ */
        memcpy(vision_tx_buf, &vision_tx_msg, 8);
        HAL_UART_Transmit_DMA(&huart6, vision_tx_buf, 8);

        taskEXIT_CRITICAL();
        osDelayUntil(&mode_wake_time, USART_SEND_PERIOD);
    }
}
