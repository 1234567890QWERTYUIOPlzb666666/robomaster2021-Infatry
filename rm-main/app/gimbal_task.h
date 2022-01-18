/*
 * @Author: your name
 * @Date: 2022-01-18 14:22:48
 * @LastEditTime: 2022-01-18 17:32:12
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \MDK-ARMd:\RM\Infantry-up\robomaster2021-Infatry\rm-main\app\gimbal_task.h
 */
#ifdef  __GIMBAL_TASK_GLOBALS
#define __GIMBAL_TASK_EXT
#else
#define __GIMBAL_TASK_EXT extern
#endif

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "stm32f4xx_hal.h"
#include "pid.h"

typedef struct
{
    /* ------------------------------- PIT ------------------------------- */
    /* ------------- position ------------- */
    /* pit ecd pid param */
    float pit_ecd_ref;
    float pit_ecd_fdb;
    float pit_ecd_err;
    /* -------------   speed  ------------- */
    /* pit spd pid param */
    float pit_spd_ref;
    float pit_spd_fdb;

    /* ------------------------------- YAW ------------------------------- */
    /* ------------- position ------------- */
    /* yaw angle pid param */
    float yaw_angle_ref;
    float yaw_angle_fdb;
    float yaw_angle_err;
    /* yaw motor ecd pid param */
    float yaw_mecd_ref;
    float yaw_mecd_fdb;
    float yaw_mecd_err;
    /* -------------   speed  ------------- */
    /* yaw speed pid param */
    float yaw_spd_ref;
    float yaw_spd_fdb;
    /* yaw motor ecd pid param */
    float yaw_mspd_ref;
    float yaw_mspd_fdb;
} gim_pid_t;

typedef struct
{
    /* gimbal ctrl parameter */
    gim_pid_t     pid;

    /* read from flash */
    int32_t       pit_center_offset;
    int32_t       yaw_center_offset;
    int16_t       current[2];  //yaw 0  pit  1

} gimbal_t;

extern gimbal_t gimbal;

void gimbal_task(void const *argu);
void vision_calcu(void);
void gimbal_param_init(void);
void gimbal_pid_calcu(void);
 void vision_data_calcu(void);
#endif
