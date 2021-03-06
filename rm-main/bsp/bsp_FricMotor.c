#include "bsp_FricMotor.h"
#include "tim.h"
#include "pid.h"
#include "math_calcu.h"
#include "remote_msg.h"
#include "shoot_task.h"
#include "modeswitch_task.h"
#include "control_def.h"

static void FricGunControl(uint16_t pwm, int16_t err_1, int16_t err_2);

Slope_Struct shoot_Fric_pwm;

void FricMotor_init(void)
{
    //900-2000
    //????ʱ?????Ŵ???????
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);

    FricMotor_PWM1 = Init_PWM;
    FricMotor_PWM2 = Init_PWM;
    /* ??ʼ??Ħ????б?º??? */
    shoot_Fric_pwm.limit_target = Init_PWM;
    shoot_Fric_pwm.real_target  = Init_PWM;
    shoot_Fric_pwm.change_scale = 0.5;

    /* ??ʼ?????? */
    shoot.shoot_speed = 15;
}

/**
  * @func   		void FricGunControl(uint8_t Control)
  * @bref			Ħ??????ͣ
  * @param[in]      Control??0Ϊֹͣ??1Ϊ????
  * @retval         void
  * @note			900??????ת
  */
static void FricGunControl(uint16_t pwm, int16_t err_1, int16_t err_2)
{
    shoot_Fric_pwm.limit_target = Init_PWM+pwm;

    if( lock_flag )			//?????????ܹ?????PWM
    {
        Slope_On(&shoot_Fric_pwm); //??Ħ????б??????

        FricMotor_PWM1 = shoot_Fric_pwm.real_target + err_1;
        FricMotor_PWM2 = shoot_Fric_pwm.real_target + err_2;
    }
}

/* 1????--14m/s */
//0    ?ң?5055 ????5075 ???ࣺ-20
//30   ?ң?5385 ????4760 ???ࣺ625
//-10  ?ң?4955 ????5195 ???ࣺ-240
//-5   ?ң?5010 ????5135 ???ࣺ-125
//-2   ?ң?5040 ????5112 ???ࣺ-72
//-1   ?ң?5052 ????5103 ???ࣺ-51
//2    ?ң?5084 ????5070 ???ࣺ14
//1    ?ң?5073 ????5074 ???ࣺ2

/* 28m/s */
//1    ?ң?8965 ????8970 ???ࣺ5

/* 1????--14m/s */
//1    ?ң?5290 ????4875 ???ࣺ415
//5    ?ң?5114 ????5030 ???ࣺ84
//6    ?ң?5125 ????5020 ???ࣺ105
//7    ?ң?5135 ????5010 ???ࣺ125

//????0   5055 ?ң?0   5085
//????10  5160 
//????5   5108
//????3   5095 ?ң?0   5091


int16_t pwm_1 = 3, pwm_2 = 0;
//uint16_t test_pwm = 550;
//????

/**
  * @name     FricMotor_Control
  * @brief    Ħ???ֿ???
  * @param    None
  * @retval   None
  * @note     ???۵?˫ǹ?ܣ?һ??????ģʽ??Ħ???ֶ??򿪣????̻?????ǹ??ģʽ??ͬ????????
  */
void FricMotor_Control(void)
{
    switch( shoot.firc_mode )
    {
    case FRIC_STOP_MODE:
    {
        FricGunControl(0, 0, 0);		//Ħ????ͣת
        break;
    }
    case FRIC_FIRE_MODE:
    {
        if( shoot.shoot_speed == 15 )
        {
            FricGunControl(LOW_SPEED, pwm_1, pwm_2);// LOW_SPEED
        }
        else if( shoot.shoot_speed == 18 )
        {
            FricGunControl(MID_SPEED, pwm_1, pwm_2);  //MID_SPEED
        }
        else if( shoot.shoot_speed == 30 )
        {
            FricGunControl(HIGH_SPEED, pwm_1, pwm_2);  //HIGH_SPEED
        }
        break;
    }
    default:
    {
        break;
    }
    }
}
