#include "chassis_task.h"
#include "gimbal_task.h"
#include "cmsis_os.h"
#include "comm_task.h"
#include "string.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "math_calcu.h"
#include "math.h"
#include "control_def.h"
#include "bsp_powerlimit.h"
#include "usart.h"
#include "DataScope_DP.h"

#define SIGN_DIR(cnt) ( (cnt%2)? 1: -1 )

extern TaskHandle_t can_msg_send_task_t;

chassis_t chassis;


static void chassis_mode_switch(void)
{
    /* 单次触发标志 */
    static uint8_t spin_flag = 0;
    static uint8_t fight_flag = 0;
    
    /* 底盘状态转换 */
    switch( ctrl_mode )
    {
        case PROTECT_MODE:
        {
            chassis.mode = CHASSIS_MODE_PROTECT;
            key_status_clear(KEY_CHASSIS_ROTATE);
            key_status_clear(KEY_CHASSIS_FIGHT);
        }
        break;
        case REMOTER_MODE:
        {
            /* 底盘跟随模式 */
            chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
            /* 底盘小陀螺模式 */
            if( rc.ch5 == 0 )   spin_flag = 1;
            if( rc.ch5 == 660 && spin_flag )
            {
                if( CHASSIS_MODE_REMOTER_ROTATE == chassis.mode )
                    chassis.mode = CHASSIS_MODE_REMOTER_FOLLOW;
                else if( CHASSIS_MODE_REMOTER_FOLLOW == chassis.mode )
                    chassis.mode = CHASSIS_MODE_REMOTER_ROTATE;
            }
        }
        break;
        case KEYBOARD_MODE:
        {
            keyboard_scanf(KEY_CHASSIS_ROTATE);
            if( FLAG_CHASSIS_ROTATE )
                chassis.mode = CHASSIS_MODE_KEYBOARD_ROTATE;
            else if( FLAG_CHASSIS_FIGHT )
                chassis.mode = CHASSIS_MODE_KEYBOARD_FIGHT;
        }
        break;
        default: break;
    }
}


/**
  * @brief chassis_task
  * @param
  * @attention
	* @note
  */
void chassis_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        /* 迎敌或小陀螺方向 */
        static int8_t  spin_dir = 1;
        static int8_t fight_dir = 1;
        
        taskENTER_CRITICAL();
        switch( chassis.mode )
        {
            case CHASSIS_MODE_PROTECT:
            {
                chassis.spd_input.vx = 0;
                chassis.spd_input.vy = 0;
                chassis.spd_input.vw = 0;
            } 
            break;
            case CHASSIS_MODE_REMOTER_FOLLOW:
            case CHASSIS_MODE_REMOTER_ROTATE:
            {
                chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
                chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
                chassis.angle_error  =  chassis.position_error * (2.0f*PI/8191.0f);
                chassis.spd_input.vx = 1.0f*(float)(rc.ch4*scale.ch4 * cos(chassis.angle_error) + (-1.0f)*rc.ch3*scale.ch3 * sin(chassis.angle_error));
                chassis.spd_input.vy = 1.0f*(float)(rc.ch4*scale.ch4 * sin(chassis.angle_error) - (-1.0f)*rc.ch3*scale.ch3 * cos(chassis.angle_error));
                if( CHASSIS_MODE_REMOTER_FOLLOW == chassis.mode )
                    chassis.spd_input.vw = -1.0f*pid_calc(&pid_chassis_angle,chassis.position_ref,chassis.position_ref + chassis.position_error);
                else if( CHASSIS_MODE_REMOTER_ROTATE == chassis.mode )
                {
                    if( chassis.wheel_max<=6000 )
                    chassis.spd_input.vw = chassis.wheel_max;
                    else    chassis.spd_input.vw = chassis.wheel_max;
                }   
            }
            break;
            case CHASSIS_MODE_KEYBOARD_FOLLOW:
            case CHASSIS_MODE_KEYBOARD_ROTATE:
            case CHASSIS_MODE_KEYBOARD_FIGHT:
            {
                /* 键盘输入斜坡处理 */
                chassis_ramp();
                /* 底盘平移速度设定计算 */
                if( CHASSIS_MODE_KEYBOARD_FIGHT == chassis.mode )
                {
                    chassis.position_ref = (GIMBAL_YAW_CENTER_OFFSET + fight_dir * GIMBAL_YAW_BETWEEN_ECD) % 8191;
                    chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
                    chassis.angle_error =  chassis.position_error * (2.0f*PI/8191.0f);
                    chassis.spd_input.vx = 1.0f *(float)(chassis_x_ramp.out * cos(chassis.angle_error + fight_dir * FIGHT_OFFSET_ERR) + (-1.0f)*chassis_y_ramp.out * sin(chassis.angle_error + fight_dir * FIGHT_OFFSET_ERR));
                    chassis.spd_input.vy = 1.0f *(float)(chassis_x_ramp.out * sin(chassis.angle_error + fight_dir * FIGHT_OFFSET_ERR) - (-1.0f)*chassis_y_ramp.out * cos(chassis.angle_error + fight_dir * FIGHT_OFFSET_ERR));
                }
                else if( CHASSIS_MODE_KEYBOARD_FOLLOW == chassis.mode || CHASSIS_MODE_KEYBOARD_ROTATE == chassis.mode )
                {
                    chassis.position_ref = GIMBAL_YAW_CENTER_OFFSET;
                    chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
                    chassis.angle_error =  chassis.position_error * (2.0f*PI/8191.0f);
                    chassis.spd_input.vx = 1.0f *(float)(chassis_x_ramp.out * cos(chassis.angle_error) + (-1.0f)*chassis_y_ramp.out * sin(chassis.angle_error));
                    chassis.spd_input.vy = 1.0f *(float)(chassis_x_ramp.out * sin(chassis.angle_error) - (-1.0f)*chassis_y_ramp.out * cos(chassis.angle_error));
                }
                /* 底盘自旋速度设定计算 */
                if( CHASSIS_MODE_KEYBOARD_FOLLOW == chassis.mode || CHASSIS_MODE_KEYBOARD_FIGHT == chassis.mode )
                    chassis.spd_input.vw = -1.0f*pid_calc(&pid_chassis_angle,chassis.position_ref, chassis.position_ref + chassis.position_error);
                else if( CHASSIS_MODE_KEYBOARD_ROTATE == chassis.mode )
                {
                    if( chassis.wheel_max <= 6000 )
                    chassis.spd_input.vw = chassis.wheel_max;
                    else    chassis.spd_input.vw = chassis.wheel_max;
                }
            }
            break;
            default: break;
        }
        /* 底盘速度分配 */
        chassis_spd_distribution();
        /* 电机电流 PID 计算 */
        for (int i = 0; i < 4; i++)
        {
            chassis.wheel_spd_fdb[i] = moto_chassis[i].speed_rpm;
            chassis.current[i] = (int16_t)pid_calc(&pid_chassis_spd[i], chassis.wheel_spd_fdb[i], chassis.wheel_spd_ref[i]);
        }
        /* 功率及超级电容控制 */
        PowerParam_Update();
        SuperCap_Control();
        Power_Control(chassis.current);
        /* 波形显示 */
        static uint8_t debug_i;
        debug_i++;
        if(debug_i == 15)
        {
            DataWave(&huart3);
            debug_i=0;
        }
        /* 发送电流，控制电机 */
        //memcpy(motor_cur.chassis_cur,chassis.current, sizeof(chassis.current));
        osSignalSet(can_msg_send_task_t, CHASSIS_MOTOR_MSG_SEND);
        taskEXIT_CRITICAL();
        
        osDelayUntil(&mode_wake_time, CHASSIS_PERIOD);
    }
}

void chassis_init()
{
    for(uint8_t i=0; i<4; i++)
    {
        PID_struct_init(&pid_chassis_spd[i], POSITION_PID, 13000, 5000,
                        7.0f, 0.0f, 0.0f);
    }
    PID_struct_init(&pid_chassis_angle, POSITION_PID, 6000, 0,
                    7.0f, 0.0f, 0.0f);
    scale.ch3 = RC_CH3_SCALE;
    scale.ch4 = RC_CH4_SCALE;
    chassis.mode = CHASSIS_MODE_PROTECT;
    //chassis.wheel_max = 8500;  /* 关掉功率控制时需要初始化 */
}


/**
	* @brief 麦轮解算函数
	* @param input : ?=+vx(mm/s)  ?=+vy(mm/s)  ccw=+vw(deg/s)
	*        output: every wheel speed(rpm)
	* @note  1=FL 2=FR 3=BL 4=BR
	*/
void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
    int16_t wheel_rpm[4];
    wheel_rpm[0] =  vx + vy - vw;
    wheel_rpm[1] = -vx + vy - vw;
    wheel_rpm[2] =  vx - vy - vw;
    wheel_rpm[3] = -vx - vy - vw;
    
    memcpy(speed, wheel_rpm, 4*sizeof(int16_t));
}

/**
	* @brief          扭腰模式计算底盘角度设定值
	* @author
	* @param[in]      void
	* @retval         void
	*/
void sparate_move(void)
{
    if(ABS(chassis.position_error) <= 300 )
        chassis.position_ref = moto_yaw.ecd;
    if(chassis.position_error > 300 )
    {
        chassis.position_ref = gimbal.yaw_center_offset - 300;
        if(chassis.position_ref > 8191)
            chassis.position_ref = chassis.position_ref - 8191;
    }
    if(chassis.position_error < -300 )
    {
        chassis.position_ref = gimbal.yaw_center_offset + 300;
        if(chassis.position_ref < 0)
            chassis.position_ref = chassis.position_ref + 8191;
    }
    chassis.position_error = circle_error(chassis.position_ref,moto_yaw.ecd,8191);
}

/**
	* @brief          底盘速度分配函数，输入速度超过最大轮速时，将输入的速度按比例分配到三个轴向上
	* @author
	* @param[in]      void
	* @retval         void
	*/
void chassis_spd_distribution(void)
{
    float  wheel_spd_input_buf[4];
    float  wheel_spd_total=0;  //总轮速
    float  distribution_temp=1.0f;	//限制比例

    /* 麦轮逆运动学原设定速度解算 */
    mecanum_calc(chassis.spd_input.vx,chassis.spd_input.vy, chassis.spd_input.vw, chassis.wheel_spd_input);

    /* 计算总速度 */
    for(int i=0; i<4; i++)
    {
        wheel_spd_input_buf[i]=ABS(chassis.wheel_spd_input[i]);
        wheel_spd_total += wheel_spd_input_buf[i];
    }
    /* 计算速度减小比例系数 */
    if(wheel_spd_total > (chassis.wheel_max * 4.0f))  //判断最大速度是否超额
    {
        distribution_temp = wheel_spd_total / (chassis.wheel_max * 4.0f); //超额速度除最大速度得分配比例
    }
    /* 速度重分配 并 限幅 */
    for(uint8_t j=0; j<4; j++)
    {
        chassis.wheel_spd_ref[j] = chassis.wheel_spd_input[j] / distribution_temp ;
        chassis.wheel_spd_ref[j] = data_limit(chassis.wheel_spd_ref[j], 8500, -8500);  //电机转速最高到8900
    }
    /* 麦轮正运动学处理后设定速度解算 */
    chassis.spd_ref.vx = (+chassis.wheel_spd_ref[0] - chassis.wheel_spd_ref[1] + chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;
    chassis.spd_ref.vy = (+chassis.wheel_spd_ref[0] + chassis.wheel_spd_ref[1] - chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;
    chassis.spd_ref.vw = (-chassis.wheel_spd_ref[0] - chassis.wheel_spd_ref[1] - chassis.wheel_spd_ref[2] - chassis.wheel_spd_ref[3] )* 0.25f;

    chassis.spd_error = chassis.spd_ref.vx + chassis.spd_ref.vy + chassis.spd_ref.vw
                        - chassis.spd_fdb.vx - chassis.spd_fdb.vy - chassis.spd_fdb.vw;

    /* 麦轮正运动学反馈速度解算 */
    chassis.spd_fdb.vx = (+moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm + moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;
    chassis.spd_fdb.vy = (+moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;
    chassis.spd_fdb.vw = (-moto_chassis[0].speed_rpm - moto_chassis[1].speed_rpm - moto_chassis[2].speed_rpm - moto_chassis[3].speed_rpm )* 0.25f;

    //计算目标速度和反馈速度差值，大于一定值用超级电容放电

//    mecanum_calc(chassis.spd_input.vx,chassis.spd_input.vy, chassis.spd_input.vw, chassis.wheel_spd_input);
//    for(int j=0; j<4; j++)
//    {
//        chassis.wheel_spd_ref[j] = chassis.wheel_spd_input[j];
//        chassis.wheel_spd_ref[j] = data_limit(chassis.wheel_spd_ref[j],8500,-8500);  //电机转速最高到8900
//    }

}

/**
  * @brief          底盘斜坡启动函数，通过斜坡函数映射目标速度
  * @author
  * @param[in]      void
  * @retval         void
  */
void chassis_ramp(void)
{
    if(rc.kb.bit.W)
    {
        ramp_calc(&chassis_x_ramp,1.0f,chassis.keyboard_input, chassis.wheel_max, 0.0f);
    }
    else if(rc.kb.bit.S)
    {
        ramp_calc(&chassis_x_ramp,1.0f,-chassis.keyboard_input, 0.0f, -chassis.wheel_max);
    }
    else
    {
        if(chassis_x_ramp.out > 0)
        {
            ramp_calc(&chassis_x_ramp,1.0f,-chassis.keyboard_input,chassis.wheel_max, 0.0f);
        }
        else if(chassis_x_ramp.out < 0)
        {
            ramp_calc(&chassis_x_ramp,1.0f,chassis.keyboard_input, 0.0f, -chassis.wheel_max);
        }
    }
    if(rc.kb.bit.D)
    {
        ramp_calc(&chassis_y_ramp,1.0f,chassis.keyboard_input, chassis.wheel_max, 0.0f);
    }
    else if(rc.kb.bit.A)
    {
        ramp_calc(&chassis_y_ramp,1.0f,-chassis.keyboard_input, 0.0f, -chassis.wheel_max);
    }
    else
    {
        if(chassis_y_ramp.out > 0)
        {
            ramp_calc(&chassis_y_ramp,1.0f,-chassis.keyboard_input, chassis.wheel_max, 0.0f);
        }
        else if(chassis_y_ramp.out < 0)
        {
            ramp_calc(&chassis_y_ramp,1.0f,chassis.keyboard_input, 0.0f, -chassis.wheel_max);
        }
    }
}

/**
  * @brief          底盘S型启动函数，通过S型函数映射目标速度
  * @author
  * @param[in]      void
  * @retval         void
  */
void chassis_sigmoid(void)
{
    static float x_speed_sigmoid,y_speed_sigmoid;
    static float x_input,y_input;
    static float x_sigmoid,y_sigmoid;
    if(rc.kb.bit.W) 					x_input += SIGMOID_PERIOD;
    else if(rc.kb.bit.S)			x_input -= SIGMOID_PERIOD;
    else
    {
        if(x_input > 0)					x_input -= SIGMOID_PERIOD;
        else if(x_input < 0)		x_input += SIGMOID_PERIOD;
    }
    if(rc.kb.bit.D) 					y_input += SIGMOID_PERIOD;
    else if(rc.kb.bit.A)			y_input -= SIGMOID_PERIOD;
    else
    {
        if(y_input > 0)					y_input -= SIGMOID_PERIOD;
        else if(y_input < 0)		y_input += SIGMOID_PERIOD;
    }

    if(x_input >= (2*SIGMOID_MAX))					x_input=  (2*SIGMOID_MAX);
    else if(x_input <= -(2*SIGMOID_MAX))		x_input= -(2*SIGMOID_MAX);

    if(y_input >= (2*SIGMOID_MAX))					y_input=  (2*SIGMOID_MAX);
    else if(y_input <= -(2*SIGMOID_MAX))		y_input= -(2*SIGMOID_MAX);

    if(x_input <= ABS(SIGMOID_PERIOD) && x_input >= -ABS(SIGMOID_PERIOD))
        x_sigmoid = 0;
    else if(x_input >= ABS(SIGMOID_PERIOD))
        x_sigmoid = Sigmoid_function(x_input);
    else if(x_input <= -ABS(SIGMOID_PERIOD))
        x_sigmoid = -Sigmoid_function(x_input);

    if(y_input <= ABS(SIGMOID_PERIOD) && y_input >= -ABS(SIGMOID_PERIOD))
        y_sigmoid = 0;
    else if(y_input >= ABS(SIGMOID_PERIOD))
        y_sigmoid = Sigmoid_function(y_input);
    else if(y_input <= -ABS(SIGMOID_PERIOD))
        y_sigmoid = -Sigmoid_function(y_input);

    x_speed_sigmoid = x_sigmoid * chassis.wheel_max;
    y_speed_sigmoid = y_sigmoid * chassis.wheel_max;

    chassis.spd_input.vx = (float)(x_speed_sigmoid * cos(chassis.angle_error) + (-1.0f)*y_speed_sigmoid * sin(chassis.angle_error));
    chassis.spd_input.vy = (float)(x_speed_sigmoid * sin(chassis.angle_error) - (-1.0f)*y_speed_sigmoid * cos(chassis.angle_error));
}

