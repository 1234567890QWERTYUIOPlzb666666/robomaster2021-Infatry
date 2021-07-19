#include "bsp_TriggerMotor.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "pid.h"
#include "gimbal_task.h"
#include "remote_msg.h"
#include "modeswitch_task.h"
#include "control_def.h"
#include "shoot_task.h"
#include "math_calcu.h"
#include "string.h"
#include "bsp_judge.h"
#include "remote_msg.h"

static void ShootParam_Update(void);
static void TriggerMotor_pidcal(void);

triger_test_t test_trigger;

extern TaskHandle_t can_msg_send_task_t;

void TriggerMotor_init(void)
{
    PID_struct_init(&pid_trigger_ecd, POSITION_PID, 6800, 0,
                    PID_TRIGGER_ECD_P, PID_TRIGGER_ECD_I, PID_TRIGGER_ECD_D);
    PID_struct_init(&pid_trigger_spd, POSITION_PID, 10000, 5500,
                    PID_TRIGGER_SPD_P, PID_TRIGGER_SPD_I, PID_TRIGGER_SPD_D);

    test_trigger.shoot_cnt = 0;
    test_trigger.shoot_wave_single = (uint32_t)TRIGGER_MOTOR_ECD;
    test_trigger.shoot_wave_circle = (uint32_t)TRIGGER_MOTOR_ECD;
    test_trigger.shoot_wave_single_dir = 1;
    test_trigger.shoot_wave_circle_dir = 1;
    test_trigger.shoot_wave_offset = 0;
    test_trigger.shoot_wave_range = 700;
}

/* 发射器裁判系统数据更新 */
static void ShootParam_Update(void)
{
    if( Game_Robot_Status.shooter_id1_17mm_speed_limit != 0 )
    {
        shoot.shoot_speed = Game_Robot_Status.shooter_id1_17mm_speed_limit;
        barrel.heat_max = Game_Robot_Status.shooter_id1_17mm_cooling_limit;
        barrel.cooling_rate = Game_Robot_Status.shooter_id1_17mm_cooling_rate;
    }

    barrel.heat -= barrel.cooling_rate * SHOOT_PERIOD * 0.001f;
    if( barrel.heat < 0 )	  barrel.heat = 0;
    barrel.heat_remain = barrel.heat_max - barrel.heat;
}

static void TriggerMotor_pidcal(void)
{
    barrel.pid.trigger_ecd_fdb = motor_trigger.total_ecd;
    pid_calc(&pid_trigger_ecd, barrel.pid.trigger_ecd_fdb, barrel.pid.trigger_ecd_ref);
    barrel.pid.trigger_spd_ref = pid_trigger_ecd.pos_out;  //位置环

    barrel.pid.trigger_spd_fdb = motor_trigger.speed_rpm;
    pid_calc(&pid_trigger_spd, barrel.pid.trigger_spd_fdb, barrel.pid.trigger_spd_ref);
    barrel.current = pid_trigger_spd.pos_out;  //速度环

    motor_cur.trigger_cur = barrel.current;
}

void TriggerMotor_control(void)
{
    /* 局部全局变量 */
    static uint16_t frequency_cnt=0;	//射频计算
    static uint8_t shoot_enable = 1;    //打能量机关单发使能标志
    static uint32_t shoot_time, shoot_last_time;//计算射击周期
    ShootParam_Update();  //更新裁判系统参数

    /* 拨盘 与 热量控制 */
//    if( FRIC_FIRE_MODE == shoot.firc_mode && SHOOT_FIRE_MODE == shoot.shoot_mode )
//    {
//        frequency_cnt++;
//        barrel.pid.trigger_ecd_error = barrel.pid.trigger_ecd_ref - barrel.pid.trigger_ecd_fdb;
//        if( ENERGY_MODE == ctrl_mode )  //击打能量机关模式
//        {
//            if( rc.mouse.l == 0 )   shoot_enable = 1;
//            if( shoot_enable \
//                    && rc.mouse.l \
//                    && ABS(barrel.pid.trigger_ecd_error) < 0.8f * TRIGGER_MOTOR_ECD \
//                    && barrel.heat_remain >= MIN_HEAT )  //热量控制
//            {
//                shoot_enable = 0;
//                barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
//                barrel.heat += 10;
//            }
//        }
//        else if( barrel.shoot_mode \
//                 && frequency_cnt * SHOOT_PERIOD >= shoot.trigger_period \
//                 && ABS(barrel.pid.trigger_ecd_error) < 0.8f * TRIGGER_MOTOR_ECD \
//                 && barrel.heat_remain >= MIN_HEAT )//一个周期打一颗  射频控制 //== 0  TRIGGER_PERIOD
//        {
//            frequency_cnt = 0;
//            /* 拨盘测试数据显示 */
//            test_trigger.last_ecd_ref = barrel.pid.trigger_ecd_ref;
//            test_trigger.last_ecd_fdb = barrel.pid.trigger_ecd_fdb;

//            test_trigger.shoot_cnt += 1;
//            test_trigger.shoot_wave_single_dir = -test_trigger.shoot_wave_single_dir;
//            if(test_trigger.shoot_cnt % 8 == 0)
//                test_trigger.shoot_wave_circle_dir = -test_trigger.shoot_wave_circle_dir;

//            test_trigger.shoot_wave_single = test_trigger.shoot_wave_single_dir * test_trigger.shoot_wave_range;
//            test_trigger.shoot_wave_circle = test_trigger.shoot_wave_circle_dir * test_trigger.shoot_wave_range;
//            /* 拨一颗子弹 */
//            barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
//            barrel.heat += 10;
//            /* 获取射击周期 */
//            shoot_time = osKernelSysTick();
//            barrel.shoot_period = shoot_time - shoot_last_time;
//            shoot_last_time = shoot_time;
//        }
//    }
//    else//禁止拨盘开火
//    {
//        frequency_cnt = 0;
//        pid_trigger_spd.iout = 0;
//        barrel.pid.trigger_ecd_ref = barrel.pid.trigger_ecd_fdb;
//        barrel.shoot_period = 0;
//    }
    
    
    /* 发射器测试 单发模式 */
    if( FRIC_FIRE_MODE == shoot.firc_mode && SHOOT_FIRE_MODE == shoot.shoot_mode )
    {
        frequency_cnt++;
        barrel.pid.trigger_ecd_error = barrel.pid.trigger_ecd_ref - barrel.pid.trigger_ecd_fdb;
        
        if( barrel.heat_max == 180 )  //裁判系统设置成热量120，则启动单发模式
        {
            static uint8_t single_shoot_flag = 1;
            if( rc.ch5 == 0 ) single_shoot_flag = 1;
            if( rc.ch5 == 660 && single_shoot_flag )
            {
                single_shoot_flag = 0;
                /* 拨一颗子弹 */
                barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
                barrel.heat += 10;
            }
        }
        else if( ENERGY_MODE == ctrl_mode )  //击打能量机关模式
        {
            if( rc.mouse.l == 0 )   shoot_enable = 1;
            if( shoot_enable \
                && rc.mouse.l \
                && ABS(barrel.pid.trigger_ecd_error) < 0.8f * TRIGGER_MOTOR_ECD \
                && barrel.heat_remain >= MIN_HEAT )  //热量控制
            {
                shoot_enable = 0;
                barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
                barrel.heat += 10;
            }
        }
        else if( barrel.shoot_mode \
                 && frequency_cnt * SHOOT_PERIOD >= shoot.trigger_period \
                 && ABS(barrel.pid.trigger_ecd_error) < 0.8f * TRIGGER_MOTOR_ECD \
                 && barrel.heat_remain >= MIN_HEAT )//一个周期打一颗  射频控制 //== 0  TRIGGER_PERIOD
        {
            frequency_cnt = 0;
            /* 拨盘测试数据显示 */
            test_trigger.last_ecd_ref = barrel.pid.trigger_ecd_ref;
            test_trigger.last_ecd_fdb = barrel.pid.trigger_ecd_fdb;

            test_trigger.shoot_cnt += 1;
            test_trigger.shoot_wave_single_dir = -test_trigger.shoot_wave_single_dir;
            if(test_trigger.shoot_cnt % 8 == 0)
                test_trigger.shoot_wave_circle_dir = -test_trigger.shoot_wave_circle_dir;

            test_trigger.shoot_wave_single = test_trigger.shoot_wave_single_dir * test_trigger.shoot_wave_range;
            test_trigger.shoot_wave_circle = test_trigger.shoot_wave_circle_dir * test_trigger.shoot_wave_range;
            /* 拨一颗子弹 */
            barrel.pid.trigger_ecd_ref += TRIGGER_MOTOR_ECD;
            barrel.heat += 10;
            /* 获取射击周期 */
            shoot_time = osKernelSysTick();
            barrel.shoot_period = shoot_time - shoot_last_time;
            shoot_last_time = shoot_time;
        }
    }
    else//禁止拨盘开火
    {
        frequency_cnt = 0;
        pid_trigger_spd.iout = 0;
        barrel.pid.trigger_ecd_ref = barrel.pid.trigger_ecd_fdb;
        barrel.shoot_period = 0;
    }

    TriggerMotor_pidcal();
    osSignalSet(can_msg_send_task_t, SHOOT_MOTOR_MSG_SEND);
}



