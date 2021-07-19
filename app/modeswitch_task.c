#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "supercap_task.h"
#include "control_def.h"
#include "bsp_TriggerMotor.h"
#include "visionfire_task.h"
#include "remote_msg.h"
#include "cmsis_os.h"
#include "bsp_vision.h"
#include "chassis_task.h"
#include "status_task.h"

static uint8_t rc_normal_check(void);
static void sw1_mode_handler(void);
static void sw2_mode_handler(void);
static void rc_abnormal_proess(void);

/* 全局控制状态枚举变量 */
ctrl_mode_e ctrl_mode;

/* 解锁标志 */
uint8_t lock_flag = 0;

void mode_switch_task(void const *argu)
{
    for(;;)
    {
        if( !lock_flag )
        {
            unlock_init();  //解锁操作
        }
        if( rc_normal_check() )
        {
            sw1_mode_handler();
            sw2_mode_handler();
        }
        else
        {
            rc_abnormal_proess();
        }
        osDelay(5);
    }
}

static uint8_t rc_normal_check(void)
{
    if( status.rc_status == 0 ) return 0;
    else                        return 1;
}

static void rc_abnormal_proess(void)
{
    lock_flag = 0;  //需要重新解锁
    ctrl_mode = PROTECT_MODE;
    rc.sw1 = RC_MI;
    rc.sw2 = RC_UP;
}

static void sw1_mode_handler(void)  //由拨杆1决定的模式切换，主要是云台和底盘
{
    switch( rc.sw1 )
    {
    case RC_UP:
    {
        ctrl_mode = REMOTER_MODE;
        /* 开关弹舱 */
        static uint8_t house_switch_enable = 1;
        if( rc.ch5 == 0 )   house_switch_enable = 1;
        if( house_switch_enable && rc.ch5 == -660)  //切换弹舱开关状态标志位
        {
            house_switch_enable = 0;
            shoot.house_switch = ~shoot.house_switch;
        }
        break;
    }
    case RC_MI:
    {
        ctrl_mode = PROTECT_MODE;
        break;
    }
    case RC_DN:
    {
        /* --------------------------------- vision and system mode --------------------------------- */
        if( rc.mouse.r == 1 )  //视觉模式，右键开启
        {
            keyboard_scanf(KEY_VISION_bENERGY);
            if( kb_status[KB_G] == -1 )		ctrl_mode = ENERGY_MODE;  //能量机关模式，G键开启
            else 							ctrl_mode = VISION_MODE;  //视觉索敌模式，仅右键开启
        }
        else
        {
            ctrl_mode = KEYBOARD_MODE;  //键盘模式，非视觉模式下
            kb_status[KB_G] = 0;        //重置 能量机关模式使能标志（按键状态）
        }

        /* --------------------------------- chassis status --------------------------------- */
        /* E键开迎敌模式 */
        keyboard_scanf(KEY_CHASSIS_FIGHT);
        if( FLAG_CHASSIS_FIGHT == -1 )     chassis.fight_status = -1;
        else                            chassis.fight_status = 0;
        /* F键小陀螺 */
        keyboard_scanf(KEY_CHASSIS_ROTATE);
        if( FLAG_CHASSIS_ROTATE == -1 )     chassis.spin_status = -1;
        else                            chassis.spin_status = 0;
        /* --------------------------------- shoot status --------------------------------- */
        /* V键开关摩擦轮 */
        keyboard_scanf(KEY_SHOOT_FRIC);
        if( FLAG_SHOOT_FRIC == -1 )
        {
            shoot.firc_mode = FRIC_FIRE_MODE;
            LASER_UP;
        }
        else
        {
            shoot.firc_mode = FRIC_STOP_MODE;
            LASER_DOWN;
        }
        /* B键控制开关弹舱盖 */
        keyboard_scanf(KEY_SHOOT_HOUSE);
        if( FLAG_SHOOT_HOUSE == -1 )     shoot.house_switch = -1;  //打开状态
        else                            shoot.house_switch = 0;//关闭状态
        break;
    }
    default:
    {
        break;
    }
    }
}

static void sw2_mode_handler(void)		//由拨杆2决定的模式切换，主要是发射器
{
    if( ctrl_mode == REMOTER_MODE )
    {
        switch( rc.sw2 )
        {
        case RC_UP:
        {
            LASER_UP;  ////
            //LASER_DOWN;
            shoot.firc_mode  = FRIC_STOP_MODE;
            shoot.shoot_mode = SHOOT_STOP_MODE;
            break;
        }
        case RC_MI:
        {
            LASER_UP;
            shoot.firc_mode  = FRIC_FIRE_MODE;
            shoot.shoot_mode = SHOOT_STOP_MODE;
            break;
        }
        case RC_DN:
        {
            shoot.firc_mode  = FRIC_FIRE_MODE;
            shoot.shoot_mode = SHOOT_FIRE_MODE;
            break;
        }
        }
    }
}

void unlock_init(void)  //解锁函数
{
    if( rc.sw1 == RC_MI && rc.sw2 == RC_UP )//左拨杆居中，右拨杆置上
    {
        if( rc.ch4 == -660 )
        {
            if( rc.ch3 == 660 )
            {
                lock_flag = 1;  //左控制杆拨至右下
            }
        }
    }
}
