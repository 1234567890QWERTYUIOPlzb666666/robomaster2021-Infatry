#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#ifdef  __SHOOT_TASK_GLOBALS
#define __SHOOT_TASK_EXT
#else
#define __SHOOT_TASK_EXT extern
#endif

#include "stm32f4xx_hal.h"
#include "bsp_FricMotor.h"

/* 红外激光 */
#define LASER_UP		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET)
#define LASER_DOWN	    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET)

/* 拨盘闭环控制变量结构 */
typedef struct
{
    /* position ecd loop */
    float trigger_ecd_ref;
    float trigger_ecd_fdb;
    float trigger_ecd_error;
    /* speed loop */
    float trigger_spd_ref;
    float trigger_spd_fdb;
} trigger_pid_t;

/* 枪管控制中心结构 */
typedef struct
{
    uint8_t     shoot_mode;
    trigger_pid_t pid;
    int16_t     current;            //发射器电流值？？
    float       heat;               //当前热量？？
    int32_t     heat_remain;        //剩余热量？？
    uint16_t    heat_max;			//最大热量	裁判系统读出
    uint16_t    cooling_rate;	    //冷却速率  裁判系统读出
    uint32_t    shoot_clock;        //射击时钟
    uint32_t    shoot_period;       //射击周期
} barrel_t;

/* 发射器工作模式枚举 */
typedef enum
{
    SHOOT_STOP_MODE,  //停火
    SHOOT_FIRE_MODE,  //开火
} shoot_mode_e;

/* 发射器控制中心结构 */
typedef struct
{
    uint8_t  shoot_mode;		//允许射击模式  右键开
    uint8_t  cover_mode;        //弹仓盖
    uint8_t  firc_mode;			//摩擦轮模式
    int      house_switch;		//弹仓盖
    uint16_t shoot_speed;		//射速
    float    trigger_period;	//拨盘拨出一颗子弹的周期，体现射频
    uint8_t  shoot_speed_vision;//发给视觉的射速档位
} shoot_t;

void shoot_task(void const *argu);
void shoot_init(void);

__SHOOT_TASK_EXT barrel_t barrel;		//枪管控制中心结构
__SHOOT_TASK_EXT shoot_t shoot;    //发射器控制中心结构

#endif
