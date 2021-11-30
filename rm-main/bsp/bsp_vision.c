/**
  * @file bsp_vision.c
  * @version 1.0
  * @date Feb,23th 2021
	*
  * @brief  �Ӿ���Ϣ����
	*
  *	@author
  *
  */
#include "bsp_vision.h"
#include "bsp_can.h"
#include "string.h"
#include "gimbal_task.h"
#include "math.h"
#include "bsp_T_imu.h"
#include "KalmanFilter.h"
#include "math_calcu.h"
#include "modeswitch_task.h"

vision_msg_t vision;
float test_abs_speed_kp = 11.4f;
/**
  * @brief �Ӿ���Ϣ����
  * @param
  * @attention
	* @note
  */
void vision_data_handler(uint8_t *Vision_Data)
{
    /* �ֲ�ȫ�ֱ��� */
    static uint32_t vision_wake_time, last_vision_wake_time;

    /* �����Ӿ���Ϣ */
    memcpy(&vision.yaw.angle_error[1], Vision_Data, 4);
    memcpy(&vision.pit.angle_error[1],(Vision_Data+4), 4);
    memcpy(&vision.distance, (Vision_Data+8), 4);

    memcpy(&vision.tof, (Vision_Data+12), 4);
    memcpy(&vision.cnt, (Vision_Data+16), 1);

    memcpy(&vision.eof, (Vision_Data+17), 1);

    /* ����ͳһ��������ʱ��Ϊ�� */
    vision.yaw.angle_error[1] = -vision.yaw.angle_error[1];

    /* ��ȡ�Ӿ��������� */
    vision_wake_time = HAL_GetTick();
    vision.period = vision_wake_time - last_vision_wake_time;
    last_vision_wake_time = vision_wake_time;

    /* YAW����з��ٶ� */
    if(vision.yaw.angle_error[1] && vision.yaw.angle_error[0] && vision.distance)
    {
        vision.yaw.aim_speed = (vision.yaw.angle_error[1] - vision.yaw.angle_error[0]) / vision.period * 1000.0f;
        vision.yaw.kal.aim_speed = Kalman1Filter_calc(&kalman_yaw_aim_speed, vision.yaw.aim_speed);
        vision.yaw.kal.imu_speed = Kalman1Filter_calc(&kalman_yaw_imu_speed, imu_data.wz);
        vision.yaw.abs_speed = vision.yaw.kal.aim_speed + vision.yaw.kal.imu_speed/test_abs_speed_kp;  //ȷ��Ŀ�꾲ֹʱ�������ٶȱƽ�0

        vision.yaw.kal.abs_speed = Kalman1Filter_calc(&kalman_yaw_abs_speed, vision.yaw.abs_speed);
    }
    else
    {
        vision.yaw.aim_speed = 0;
        vision.yaw.kal.imu_speed = 0;
        vision.yaw.kal.aim_speed = 0;
        vision.yaw.abs_speed = 0;
        vision.yaw.kal.abs_speed=0;
    }

    /* �滻��ʷ��Ϣ */
    vision.pit.angle_error[0] = vision.pit.angle_error[1];
    vision.yaw.angle_error[0] = vision.yaw.angle_error[1];
}
