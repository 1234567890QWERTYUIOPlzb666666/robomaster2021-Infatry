#include "stdint.h"

#ifndef __BSP_TRIIGERMOTOR_H
#define __BSP_TRIIGERMOTOR_H

#define TRIGGER_MOTOR_ECD   36859.0f  //拨盘一颗子弹转过的编码值 8191 * 36 / 8 = 36859.5f
#define MIN_HEAT		    20

typedef struct
{
    float       last_ecd_ref;
    float       last_ecd_fdb;
    uint32_t    shoot_cnt;
    float       shoot_wave_single;
    float       shoot_wave_circle;
    int8_t      shoot_wave_single_dir;
    int8_t      shoot_wave_circle_dir;
    int32_t     shoot_wave_offset;
    int16_t     shoot_wave_range;
} triger_test_t;

extern triger_test_t test_trigger;

void TriggerMotor_init(void);
void TriggerMotor_control(void);

#endif

