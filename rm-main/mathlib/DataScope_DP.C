//Channel：选择通道（1-10）
//传入单精度浮点数
//在函数的最下方的DataWave函数修改发送的数据
//其他函数无需修改
#include "DataScope_DP.h"
#include "mytype.h"
#include "math.h"
#include "main.h"
#include "usart.h"
#include "bsp_powerlimit.h"
#include "gimbal_task.h"
#include "pid.h"
#include "chassis_task.h"
#include "bsp_T_imu.h"
#include "bsp_judge.h"
#include "bsp_vision.h"

#include "shoot_task.h"
#include "func_generator.h"
#include "KalmanFilter.h"
#include "bsp_TriggerMotor.h"
#define ABS(x)		((x>0)? (x): (-x))


DataTypedfef CK;	//传输数据用到的结构体

extern float vision_yaw_perdict;
//函数说明：将单精度浮点数据转成4字节数据并存入指定地址
//附加说明：用户无需直接操作此函数
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
//函数无返回
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}


//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
//Channel：选择通道（1-10）
//函数无返回
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
    if ( (Channel > 10) || (Channel == 0) ) return;  //通道个数大于10或等于0，直接跳出，不执行函数
    else
    {
        switch (Channel)
        {
        case 1:
            Float2Byte(&Data,CK.DataScope_OutPut_Buffer,1);
            break;
        case 2:
            Float2Byte(&Data,CK.DataScope_OutPut_Buffer,5);
            break;
        case 3:
            Float2Byte(&Data,CK.DataScope_OutPut_Buffer,9);
            break;
        case 4:
            Float2Byte(&Data,CK.DataScope_OutPut_Buffer,13);
            break;
        case 5:
            Float2Byte(&Data,CK.DataScope_OutPut_Buffer,17);
            break;
        case 6:
            Float2Byte(&Data,CK.DataScope_OutPut_Buffer,21);
            break;
        case 7:
            Float2Byte(&Data,CK.DataScope_OutPut_Buffer,25);
            break;
        case 8:
            Float2Byte(&Data,CK.DataScope_OutPut_Buffer,29);
            break;
        case 9:
            Float2Byte(&Data,CK.DataScope_OutPut_Buffer,33);
            break;
        case 10:
            Float2Byte(&Data,CK.DataScope_OutPut_Buffer,37);
            break;
        }
    }
}


//函数说明：生成 DataScopeV1.0 能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
    if ( (Channel_Number > 10) || (Channel_Number == 0) ) {
        return 0;    //通道个数大于10或等于0，直接跳出，不执行函数
    }
    else
    {
        CK.DataScope_OutPut_Buffer[0] = '$';  //帧头

        switch(Channel_Number)
        {
        case 1:
            CK.DataScope_OutPut_Buffer[5]  =  5;
            return  6;
        case 2:
            CK.DataScope_OutPut_Buffer[9]  =  9;
            return 10;
        case 3:
            CK.DataScope_OutPut_Buffer[13] = 13;
            return 14;
        case 4:
            CK.DataScope_OutPut_Buffer[17] = 17;
            return 18;
        case 5:
            CK.DataScope_OutPut_Buffer[21] = 21;
            return 22;
        case 6:
            CK.DataScope_OutPut_Buffer[25] = 25;
            return 26;
        case 7:
            CK.DataScope_OutPut_Buffer[29] = 29;
            return 30;
        case 8:
            CK.DataScope_OutPut_Buffer[33] = 33;
            return 34;
        case 9:
            CK.DataScope_OutPut_Buffer[37] = 37;
            return 38;
        case 10:
            CK.DataScope_OutPut_Buffer[41] = 41;
            return 42;
        }
    }
    return 0;
}


extern float test_abs_speed_kp;
//函数说明：MiniBalance上位机通过串口打印数据波形
//附加说明：直接在主函数中调用此函数（注意延时减少打印量）
//函数无返回
void DataWave(UART_HandleTypeDef* huart)
{
    //Channel：选择通道（1-10）
    //传入单精度浮点数
    /* 拨盘 */
//    DataScope_Get_Channel_Data((float) (test_trigger.shoot_wave_single), 1 );
//    DataScope_Get_Channel_Data((float) (test_trigger.shoot_wave_circle), 2 );
    
//    DataScope_Get_Channel_Data((float) (pid_trigger_ecd.pos_out), 1 );
//    DataScope_Get_Channel_Data((float) (pid_trigger_spd.pos_out), 2 );
//    DataScope_Get_Channel_Data((float) (pid_trigger_spd.iout), 3 );
    
//    DataScope_Get_Channel_Data((float) (pid_trigger_ecd.err[0]), 1 );
//    DataScope_Get_Channel_Data((float) (pid_trigger_spd.err[0]), 2 );
    
//    DataScope_Get_Channel_Data((float) (motor_trigger.given_current), 5 );
//    DataScope_Get_Channel_Data((float) (test_current), 1 );
//    DataScope_Get_Channel_Data((float) (motor_trigger.speed_rpm), 2 );
//    DataScope_Get_Channel_Data((float) (motor_trigger.given_current), 3 );
//    
    
    /* 云台 PID调参 */
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_ecd_ref, 1 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_ecd_fdb, 2 );
//    DataScope_Get_Channel_Data((float) pid_pit_ecd.pout, 3 );
//    DataScope_Get_Channel_Data((float) pid_pit_ecd.dout, 4);
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_spd_ref, 5 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_spd_fdb, 6 );
//    DataScope_Get_Channel_Data((float) pid_pit_spd.pout, 7 );
//    DataScope_Get_Channel_Data((float) pid_pit_spd.iout, 8 );
    
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_ecd_ref, 1 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_ecd_fdb, 2 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_spd_ref, 3 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.pit_spd_fdb, 4 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mecd_ref, 1 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mecd_fdb, 2 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mspd_ref, 3 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_mspd_fdb, 4 );    
    
    /* 陀螺仪 */
//    DataScope_Get_Channel_Data((float) imu_data.wy, 1 );
//    DataScope_Get_Channel_Data((float) imu_data.wz, 2 );
//    DataScope_Get_Channel_Data((float) imu_data.pitch, 3 );
//    DataScope_Get_Channel_Data((float) imu_data.yaw, 4 );
    
    /* 波形生成测试 */
//    DataScope_Get_Channel_Data((float) test_sin.out, 1 );
//    DataScope_Get_Channel_Data((float) test_cos.out, 2 );
//    DataScope_Get_Channel_Data((float) test_sqr.out, 3 );
    /* 视觉 */
//    DataScope_Get_Channel_Data((float) vision.yaw.kal.imu_speed/test_abs_speed_kp, 1 );  // +-350范围，俯视逆时针为正
//    DataScope_Get_Channel_Data((float) vision.yaw.kal.aim_speed, 2 );
//    DataScope_Get_Channel_Data((float) vision.yaw.kal.abs_speed,3 );
    
      DataScope_Get_Channel_Data((float) vision.tof,1 );
    
//    DataScope_Get_Channel_Data((float) vision.yaw.angle_error[1],1 );
    DataScope_Get_Channel_Data((float) vision.yaw.predict,2 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_angle_ref,3 );
//    DataScope_Get_Channel_Data((float) gimbal.pid.yaw_angle_fdb,4 );

    /* 调底盘功率控制 */
//    DataScope_Get_Channel_Data((float) powercontrol.power_buffer,1 );
//    DataScope_Get_Channel_Data((float) powercontrol.judge_power_buffer,2 );
//    DataScope_Get_Channel_Data((float) supercap.volage,3 );
//    DataScope_Get_Channel_Data((float) powercontrol.max_power,3 );
//    DataScope_Get_Channel_Data((float) powercontrol.judge_power,4 );
//    DataScope_Get_Channel_Data((float) powercontrol.judge_power_buffer,5 );
    /* 调试底盘速度 */
//    DataScope_Get_Channel_Data((float) (chassis.wheel_spd_fdb[0]),1 );
//    DataScope_Get_Channel_Data((float) (-chassis.wheel_spd_fdb[1]),2 );
//    DataScope_Get_Channel_Data((float) (-chassis.wheel_spd_fdb[2]),3 );
//    DataScope_Get_Channel_Data((float) (chassis.wheel_spd_fdb[3]),4 );

    
    CK.Send_Count = DataScope_Data_Generate(2);//串口需要发送的数据个数

    if(huart == &huart2)
    {
        for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++)
        {
            while((USART2->SR&0X40)==0);
            USART2->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt];
        }
    }
    else if(huart == &huart3)
    {
        for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++)
        {
            while((USART3->SR&0X40)==0);
            USART3->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt];
        }
    }
    else if(huart == &huart6)
    {
        for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++)
        {
            while((USART6->SR&0X40)==0);
            USART6->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt];
        }
    }
    else if(huart == &huart5)
    {
        for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++)
        {
            while((UART5->SR&0X40)==0);
            UART5->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt];
        }
    }
}

