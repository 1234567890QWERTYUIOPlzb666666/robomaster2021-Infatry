#define __GIMBAL_TASK_GLOBALS
#include "chassis_task.h"
#include "gimbal_task.h"
#include "status_task.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "remote_msg.h"
#include "pid.h"
#include "bsp_can.h"
#include "control_def.h"
#include "bsp_TriggerMotor.h"
#include "remote_msg.h"
#include "math_calcu.h"
#include "bsp_T_imu.h"
#include "bsp_vision.h"
#include "KalmanFilter.h"
#include "usart.h"
#include "func_generator.h"

extern TaskHandle_t can_msg_send_task_t;
gimbal_t gimbal;

static void vision_energy_calcu(void);
static void vision_data_calcu(void);
void gimbal_param_init(void)
{
    memset(&gimbal, 0, sizeof(gimbal_t));
    /* pit 轴 */
    PID_struct_init(&pid_pit_ecd, POSITION_PID, 5500, 0,
                    pid_pit_ecd_P, pid_pit_ecd_I, pid_pit_ecd_D);
    PID_struct_init(&pid_pit_spd, POSITION_PID, 28000,20000,
                    pid_pit_spd_P, pid_pit_spd_I, pid_pit_spd_D);
    /* YAW 轴 */
    PID_struct_init(&pid_yaw_angle, POSITION_PID, 5000, 0,
                    pid_yaw_angle_P, pid_yaw_angle_I, pid_yaw_angle_D);
    PID_struct_init(&pid_yaw_spd, POSITION_PID, 28000, 20000,
                    pid_yaw_spd_P, pid_yaw_spd_I, pid_yaw_spd_D);

    /* 测试用 */
    PID_struct_init(&pid_yaw_mecd, POSITION_PID, 5000, 0,
                    pid_yaw_mecd_P, pid_yaw_mecd_I, pid_yaw_mecd_D);
    PID_struct_init(&pid_yaw_mspd, POSITION_PID, 28000, 20000,
                    pid_yaw_mspd_P,pid_yaw_mspd_I, pid_yaw_mspd_D);

    scale.ch1 = RC_CH1_SCALE;
    scale.ch2 = RC_CH2_SCALE;
}


//FGT_sin_t test_s = 
//{
//    .Td = 1,
//    .time = 0,
//    .max = GIMBAL_YAW_CENTER_OFFSET + 800,
//    .min = -1000,
//    .dc = GIMBAL_YAW_CENTER_OFFSET - 800,
//    .T = 600,
//    .A = 400,
//    .phi = 0,
//    .out = 0
//};


/**
  * @brief gimbal_task
  * @param
  * @attention
	* @note
  */
void gimbal_task(void const *argu)
{
    uint32_t mode_wake_time = osKernelSysTick();
    for(;;)
    {
        taskENTER_CRITICAL();
        switch(ctrl_mode)
        {
        case PROTECT_MODE:
        {
            gimbal.pid.pit_ecd_ref   = GIMBAL_PIT_CENTER_OFFSET;
            gimbal.pid.yaw_angle_ref = imu_data.yaw;
            gimbal.pid.yaw_mecd_ref  = GIMBAL_YAW_CENTER_OFFSET;
            for(uint8_t i=0; i<2; i++)	gimbal.current[i]=0;
            break;
        }

        case REMOTER_MODE:
        {
            //FGT_sin_cal(&test_s);
            //gimbal.pid.pit_ecd_ref = test_pit.out;
            gimbal.pid.pit_ecd_ref   += rc.ch2 * scale.ch2;
            gimbal.pid.yaw_mecd_ref  += rc.ch1 * (-0.008f);  //发射器测试时用
            //gimbal.pid.yaw_mecd_ref  = test_s.out;  //发射器测试时用
            gimbal.pid.yaw_angle_ref += rc.ch1 * scale.ch1;
            gimbal_pid_calcu();
            break;
        }

        case KEYBOARD_MODE:
        {
            gimbal.pid.pit_ecd_ref   += rc.mouse.y *  KEYBOARD_SCALE_PIT;
            gimbal.pid.yaw_angle_ref += rc.mouse.x *  KEYBOARD_SCALE_YAW;
            gimbal_pid_calcu();
            break;
        }

        case VISION_MODE:
        case ENERGY_MODE:
        {
            if(vision.distance)		//视觉模式下捕获到目标
            {
                vision_energy_calcu();
                //vision_calcu();
                vision.aim_flag=1;
            }
            else if(vision.aim_flag==1)	//丢失目标后的第一帧  重置目标值 防止云台疯
            {
                gimbal.pid.pit_ecd_ref   = gimbal.pid.pit_ecd_fdb;
                gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb;
                vision.aim_flag=0;
            }
            else
            {
                gimbal.pid.pit_ecd_ref   += rc.mouse.y * KEYBOARD_SCALE_PIT;
                gimbal.pid.yaw_angle_ref += rc.mouse.x * KEYBOARD_SCALE_YAW;
                gimbal_pid_calcu();
            }
            break;
        }
        default:
        {
            break;
        }
        }

        memcpy(motor_cur.gimbal_cur, gimbal.current, sizeof(gimbal.current));
        osSignalSet(can_msg_send_task_t, GIMBAL_MOTOR_MSG_SEND);
        taskEXIT_CRITICAL();

        osDelayUntil(&mode_wake_time, GIMBAL_PERIOD);
    }
}

float test_vision_kp = 0.45f;
float test_vision_angle_error_pit_kp = 1.3f;
float test_vision_angle_error_yaw_kp = 1.5f;
void vision_calcu()
{
    /* pit轴视觉目标值计算 */
    if(ABS(vision.pit.angle_error[1]) <= 45)     //重力下坠有解时才预测
    {
        vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_angle_error, vision.pit.angle_error[1] * 22.75f);
        gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb + vision.pit.kal.angle_error;  //正负号注意pit电机安装正方向
    }
    else//重力下坠无解，操作手控制pit轴
    {
        gimbal.pid.pit_ecd_ref += rc.mouse.y *  KEYBOARD_SCALE_PIT;
    }


    /* yaw轴视觉目标值计算 */
    if(kb_status[KB_R] == -1)//小陀螺模式 视觉10帧内一帧为有效信息 追随敌人  其他为无效信息 保持中心瞄准
    {
        if(vision.yaw.angle_error[1]!=0)
            gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb + vision.yaw.angle_error[1]/test_vision_angle_error_pit_kp;
    }
    else  //正常自瞄模式
    {
        /* 计算预测量 */
        vision.tof = Kalman1Filter_calc(&kalman_bullet_time,vision.tof);
        vision.yaw.predict = test_vision_kp * vision.yaw.kal.abs_speed * vision.tof;

        /* 预测限幅 */
        if( vision.yaw.predict >= 10.0f )		    vision.yaw.predict = 10.0f;
        else if( vision.yaw.predict <= -10.0f )		vision.yaw.predict = -10.0f;

        vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_angle_error,
                                     vision.yaw.angle_error[1]/1.5f + vision.yaw.predict);  //目标绝对角速度 * 子弹飞行时间
//        vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_angle_error,
//                                     vision.yaw.angle_error[1]/test_vision_angle_error_yaw_kp);
        gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb + vision.yaw.kal.angle_error;
//        }
    }
    gimbal_pid_calcu();
}

void gimbal_pid_calcu(void)
{
    /*------------------------pit轴串级pid计算------------------------*/
    //位置反馈：编码器位置
    //速度反馈：陀螺仪速度
    gimbal.pid.pit_ecd_ref  = data_limit(gimbal.pid.pit_ecd_ref,GIMBAL_PIT_MAX,GIMBAL_PIT_MIN);	//目标值限幅
    gimbal.pid.pit_ecd_fdb = moto_pit.ecd;
    gimbal.pid.pit_ecd_err = circle_error(gimbal.pid.pit_ecd_ref,gimbal.pid.pit_ecd_fdb,8191);
    pid_calc(&pid_pit_ecd, gimbal.pid.pit_ecd_fdb, gimbal.pid.pit_ecd_fdb + gimbal.pid.pit_ecd_err);

    gimbal.pid.pit_spd_ref =  -pid_pit_ecd.pos_out;   //PID外环目标值
    gimbal.pid.pit_spd_fdb =  imu_data.wy;			 //pit角速度反馈传进PID结构体
    pid_calc(&pid_pit_spd, gimbal.pid.pit_spd_fdb, gimbal.pid.pit_spd_ref);

    gimbal.current[1] = -1.0f * pid_pit_spd.pos_out;
    /*------------------------yaw轴串级pid计算------------------------*/
    //位置反馈：陀螺仪角度
    //速度反馈：陀螺仪WZ
//    if(gimbal.pid.yaw_angle_ref<0)            gimbal.pid.yaw_angle_ref += 360;
//    else if(gimbal.pid.yaw_angle_ref>360)     gimbal.pid.yaw_angle_ref -= 360;	//目标值限幅
//    gimbal.pid.yaw_angle_fdb = imu_data.yaw;  //陀螺仪角度反馈
//    gimbal.pid.yaw_angle_err = circle_error(gimbal.pid.yaw_angle_ref,gimbal.pid.yaw_angle_fdb,360);
//    pid_calc(&pid_yaw_angle, gimbal.pid.yaw_angle_fdb, gimbal.pid.yaw_angle_fdb + gimbal.pid.yaw_angle_err);

//    gimbal.pid.yaw_spd_ref = pid_yaw_angle.pos_out;
//    gimbal.pid.yaw_spd_fdb = imu_data.wz;  //陀螺仪速度反馈
//    pid_calc(&pid_yaw_spd, gimbal.pid.yaw_spd_fdb, gimbal.pid.yaw_spd_ref);

//    gimbal.current[0] = pid_yaw_spd.pos_out;

    //位置反馈：编码器位置
    //速度反馈：陀螺仪WZ
    //注意：测试发射器时用。使用时，需要注释掉底盘，保证编码值绝对，下方电流来源切换
    if( gimbal.pid.yaw_mecd_ref >8191 )
        gimbal.pid.yaw_mecd_ref -= 8191;
    else if( gimbal.pid.yaw_mecd_ref < 0 )
        gimbal.pid.yaw_mecd_ref += 8191;
    gimbal.pid.yaw_mecd_fdb = moto_yaw.ecd;
    gimbal.pid.yaw_mecd_err = circle_error(gimbal.pid.yaw_mecd_ref,gimbal.pid.yaw_mecd_fdb,8191);
    pid_calc(&pid_yaw_mecd, gimbal.pid.yaw_mecd_fdb, gimbal.pid.yaw_mecd_fdb + gimbal.pid.yaw_mecd_err);

    gimbal.pid.yaw_mspd_ref = pid_yaw_mecd.pos_out;
    gimbal.pid.yaw_mspd_fdb = imu_data.wz;  //陀螺仪速度反馈
    pid_calc(&pid_yaw_mspd, gimbal.pid.yaw_mspd_fdb, gimbal.pid.yaw_mspd_ref);
    gimbal.current[0] = pid_yaw_mspd.pos_out;
}


float test_energy_th1 = 55.0f;
float test_energy_k1  = 0.30f;
float test_energy_k2  = 0.20f;

float test_energy_th2 = 55.0f;
float test_energy_k3  = 0.020f;
float test_energy_k4  = 0.014f;

static void vision_energy_calcu(void)
{
    /*------------------------pit轴视觉目标值计算------------------------*/
    if( ABS(vision.pit.angle_error[1]) >= test_energy_th1)
        vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_energy, test_energy_k1 * vision.pit.angle_error[1]);
    else if( ABS(vision.pit.angle_error[1]) < test_energy_th1 )
        vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_energy, test_energy_k2 * vision.pit.angle_error[1]);
    gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb - vision.pit.kal.angle_error;

    /*------------------------yaw轴视觉目标值计算------------------------*/
    if(ABS(vision.yaw.angle_error[1]) >= test_energy_th2)
        vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_energy, test_energy_k3 * vision.yaw.angle_error[1]);
    else if(ABS(vision.yaw.angle_error[1]) < test_energy_th2)
        vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_energy, test_energy_k4 * vision.yaw.angle_error[1]);
    gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb + vision.yaw.kal.angle_error;

    gimbal_pid_calcu();
    
    /*------------------------pit轴视觉目标值计算------------------------*/
//    vision.pit.kal.angle_error = Kalman1Filter_calc(&kalman_pit_energy, 0.5f * vision.pit.angle_error[1]);
//    gimbal.pid.pit_ecd_ref = gimbal.pid.pit_ecd_fdb - vision.pit.kal.angle_error;

//    /*------------------------yaw轴视觉目标值计算------------------------*/
//    vision.yaw.kal.angle_error = Kalman1Filter_calc(&kalman_yaw_energy, 0.025f * vision.yaw.angle_error[1]);
//    gimbal.pid.yaw_angle_ref = gimbal.pid.yaw_angle_fdb + vision.yaw.kal.angle_error;

//    gimbal_pid_calcu();
}
static void static_data_calcu(void)
{
    vision.yaw.angle_error[1] = -vision.yaw.angle_error[1];
    if(vision.yaw.angle_error[1] && vision.yaw.angle_error[0] && vision.distance)
    {
        vision.yaw.aim_speed = (vision.yaw.angle_error[1] - vision.yaw.angle_error[0]) / vision.period * 1000.0f;
        vision.yaw.kal.aim_speed = Kalman1Filter_calc(&kalman_yaw_aim_speed, vision.yaw.aim_speed);
        vision.yaw.kal.imu_speed = Kalman1Filter_calc(&kalman_yaw_imu_speed, imu_data.wz);
       // vision.yaw.abs_speed = vision.yaw.kal.aim_speed + vision.yaw.kal.imu_speed/test_abs_speed_kp;  //ȷ��Ŀ�꾲ֹʱ�������ٶȱƽ�0

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
