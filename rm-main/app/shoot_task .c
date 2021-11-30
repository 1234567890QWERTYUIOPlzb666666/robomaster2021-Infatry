#define __SHOOT_TASK_GLOBALS

#include "shoot_task.h"
#include "control_def.h"
#include "usart.h"
#include "cmsis_os.h"
#include "remote_msg.h"
#include "bsp_CoverServo.h"
#include "bsp_FricMotor.h"
#include "bsp_TriggerMotor.h"
#include "bsp_judge.h"
#include "comm_task.h"
#include "modeswitch_task.h"

static void shoot_mode_sw(void);
static void house_switch_control(void);

void shoot_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        shoot_mode_sw();			//射击模式切换
        FricMotor_Control();		//摩擦轮电机控制
        TriggerMotor_control();		//拨弹电机控制
        house_switch_control();     //弹舱盖控制

        osDelayUntil(&mode_wake_time, SHOOT_PERIOD);
    }
}

void shoot_init(void)
{
    FricMotor_init();
    shoot.shoot_mode     = SHOOT_STOP_MODE;
    barrel.shoot_mode    = SHOOT_STOP_MODE;
    shoot.trigger_period = 90;//TRIGGER_PERIOD;
    barrel.cooling_rate  = 10;
    barrel.heat_max      = 50;
    shoot.shoot_speed    = 15;
    TriggerMotor_init();
}

static void shoot_mode_sw(void)
{
    /* 视觉射速档位处理 */
    if( shoot.shoot_speed == 15 )			shoot.shoot_speed_vision = 1;
    else if( shoot.shoot_speed == 18 )		shoot.shoot_speed_vision = 2;
    else if( shoot.shoot_speed == 30 )		shoot.shoot_speed_vision = 3;

    /* 发射器模式切换 */
    if( PROTECT_MODE == ctrl_mode )
    {
        LASER_UP;
        //LASER_DOWN;
        shoot.shoot_mode = SHOOT_STOP_MODE;
        shoot.firc_mode  = FRIC_STOP_MODE;
    }
    else if( KEYBOARD_MODE==ctrl_mode || VISION_MODE==ctrl_mode )  //键盘模式 或 视觉模式
    {
        if( rc.mouse.l )    shoot.shoot_mode = SHOOT_FIRE_MODE;  //按下左键，使能发射器开火
        else				shoot.shoot_mode = SHOOT_STOP_MODE;
    }
    else if( ENERGY_MODE == ctrl_mode )
    {
        shoot.shoot_mode = SHOOT_FIRE_MODE;  //键盘按下G键进入能量模式，使能发射器开火
    }
    /* 枪管模式 与 射频 控制 */
    if( SHOOT_FIRE_MODE == shoot.shoot_mode )  //开火
    {
        barrel.shoot_mode = SHOOT_FIRE_MODE;
        //shoot.trigger_period = TRIGGER_PERIOD;
    }
    else  //停火
    {
        barrel.shoot_mode = SHOOT_STOP_MODE;
    }
}

static void house_switch_control(void)
{
    if( PROTECT_MODE == ctrl_mode )
    {
        Magazine_PWM = COVER_START;  //保护模式下，关闭弹舱
        shoot.house_switch = 0;  //重置弹舱开关标志
    }
    else
    {
        if(shoot.house_switch==-1)	    Magazine_PWM = COVER_END;  //打开弹舱
        else	                        Magazine_PWM = COVER_START;
    }
}
