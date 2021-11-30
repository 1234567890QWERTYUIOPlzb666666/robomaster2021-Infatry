#ifndef _CONTROL_DEF_H_
#define _CONTROL_DEF_H_

#include "stm32f4xx_hal.h"
#include "math_calcu.h"

//�������
//С����   4
//С���ڸ� 5
#define Robot_Number 5

/*----------------------------- the whole system ----------------------------- */
// task period
#define GIMBAL_PERIOD	  1
#define CHASSIS_PERIOD    2
#define SHOOT_PERIOD      2
#define MODESWITCH_PERIOD 6
#define STATUS_PERIOD     500

// gimbal test pid param
#define pid_yaw_mecd_P 10.0f
#define pid_yaw_mecd_I 0.0f
#define pid_yaw_mecd_D 0.0f

#define pid_yaw_mspd_P 15.0f
#define pid_yaw_mspd_I 0.1f
#define pid_yaw_mspd_D 0.0f

/*----------------------------- manipulator preference ----------------------------- */
/* special function key and key status definition */
// chassis control key (status) define
#define KEY_CHASSIS_FIGHT       KB_E
#define FLAG_CHASSIS_FIGHT      kb_status[KEY_CHASSIS_FIGHT]

#define KEY_CHASSIS_ROTATE      KB_F
#define FLAG_CHASSIS_ROTATE     kb_status[KEY_CHASSIS_ROTATE]

// shoot control key (status) define
#define KEY_SHOOT_FRIC          KB_V
#define FLAG_SHOOT_FRIC         kb_status[KEY_SHOOT_FRIC]

#define KEY_SHOOT_MODE          KB_Q
#define FLAG_SHOOT_MODE         kb_status[KEY_SHOOT_FRIC]

#define KEY_SHOOT_HOUSE         KB_B
#define FLAG_SHOOT_HOUSE        kb_status[KEY_SHOOT_HOUSE]

// vision control key status define
#define KEY_VISION_sENERGY      KB_Z
#define FLAG_VISION_sENERGY     kb_status[KEY_VISION_sENERGY]

#define KEY_VISION_bENERGY      KB_X
#define FLAG_VISION_bENERGY     kb_status[KEY_VISION_bENERGY]

#define KEY_VISION_ANTIROTATE   KB_R
#define FLAG_VISION_ANTIROTATE  kb_status[KEY_VISION_ANTIROTATE]



#if ( Robot_Number == 4 )

/*-----------------------------shoot-----------------------------*/
//���ٲ���
#define SPEED_PWM_14        550     //14m/s����  ��������Ϊ15m/s  ����ʱ��13.8-14.6 ����ʱ��14.3-14.6
#define SPEED_PWM_16        590     //16m/s����  ��������Ϊ18m/s  ����ʱ��15.8-16.7 ����ʱ��16.9-17.2
#define SPEED_PWM_28        900     //28m/s����  ��������Ϊ30m/s  ����ʱ��25.0-28.2 ����ʱ��28.0-28.5

#define LOW_SPEED           SPEED_PWM_14   //У׼ʱ�ĳ�1100 (�ճ���ʱ������) ���Բ���ʱ���������ó�200
#define MID_SPEED	        SPEED_PWM_16	
#define HIGH_SPEED	        SPEED_PWM_28

/* PWM������ */
#define PWM_SEND_CH1    TIM12->CCR1 //PB14
#define PWM_SEND_CH2    TIM12->CCR2 //PB15
#define PWM_SEND_CH3    TIM3->CCR3  //PB0
#define PWM_SEND_CH4    TIM3->CCR4  //PB1
/* ���ո� */
#define COVER_START     1600
#define COVER_END       580
#define Magazine_PWM    PWM_SEND_CH2   
/* ǹ��PWM IO�� */
#define FricMotor_PWM1  PWM_SEND_CH1
#define FricMotor_PWM2  PWM_SEND_CH4

/* ���� PID ���� */
#define PID_TRIGGER_ECD_P 0.3f
#define PID_TRIGGER_ECD_I 0.0f
#define PID_TRIGGER_ECD_D 0.3f

#define PID_TRIGGER_SPD_P 5.0f
#define PID_TRIGGER_SPD_I 0.008f
#define PID_TRIGGER_SPD_D 0.0f

/* ����Ƶ�� */
#define TRIGGER_PERIOD    90  //������ڣ�ms��

/*-----------------------------chassis---------------------------*/
#define RC_CH4_SCALE    12     
#define RC_CH3_SCALE    12

#define SPEED_45W  		3400.0f
#define SPEED_50W		3600.0f
#define SPEED_55W		3900.0f
#define SPEED_60W		4200.0f
#define SPEED_80W		5800.0f
#define SPEED_100W      6400.0f
#define SPEED_120W      7400.0f
#define SPEED_SUPERCAP  8000.0f

#define POWER_OFFSET    -2.0f		    //���ƹ��ʿ��ư��3.0W
#define SUPERCAP_MAX_VOLAGE	25.5f		//������������ѹ
/*-----------------------------gimbal----------------------------*/
#define Reduction_ratio			    1.0f	//pit����ٱ�
#define RC_CH2_SCALE                0.008f
#define RC_CH1_SCALE                -0.0003f

#define KEYBOARD_SCALE_PIT          -0.06f
#define KEYBOARD_SCALE_YAW	        -0.005f

#define GIMBAL_PIT_CENTER_OFFSET    2775
#define GIMBAL_PIT_MAX              3670
#define GIMBAL_PIT_MIN              2400

#define GIMBAL_YAW_CENTER_OFFSET    6828
#define GIMBAL_YAW_BETWEEN_ECD      (8191 / 8)
#define GIMBAL_YAW_FIGHT_OFFSET     ( (GIMBAL_YAW_CENTER_OFFSET + GIMBAL_YAW_BETWEEN_ECD) % 8191 )
#define FIGHT_OFFSET_ERR            (-1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI)

/* YAW��PIDϵ�� */
#define pid_yaw_angle_P 180.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_spd_P 25.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

/* PIT��PIDϵ�� */
#define pid_pit_ecd_P 10.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 16.0f
#define pid_pit_spd_I 0.25f
#define pid_pit_spd_D 0.0f

#elif ( Robot_Number == 5 )

/*-----------------------------shoot-----------------------------*/
#define SPEED_PWM_14        540     //14m/s����  ��������Ϊ15m/s  ����ʱ��13.8-14.6 ����ʱ��14.3-14.6
#define SPEED_PWM_16        605     //16m/s����  ��������Ϊ18m/s  ����ʱ��15.8-16.7 ����ʱ��16.9-17.2
#define SPEED_PWM_28        1050     //28m/s����  ��������Ϊ30m/s  ����ʱ��25.0-28.2 ����ʱ��28.0-28.5

#define LOW_SPEED           SPEED_PWM_14   //У׼ʱ�ĳ�1100 (�ճ���ʱ������) ���Բ���ʱ���������ó�200
#define MID_SPEED	        SPEED_PWM_16	
#define HIGH_SPEED	        SPEED_PWM_28

/* ������PWM�� */
#define PWM_SEND_CH1    TIM12->CCR1 //PB14
#define PWM_SEND_CH2    TIM12->CCR2 //PB15
#define PWM_SEND_CH3    TIM3->CCR3  //PB0
#define PWM_SEND_CH4    TIM3->CCR4  //PB1
/* ���ո� */
#define COVER_START     1600
#define COVER_END       580
#define Magazine_PWM    PWM_SEND_CH2   
/* ǹ��PWM IO�� */
#define FricMotor_PWM1  PWM_SEND_CH1  //��Ħ����
#define FricMotor_PWM2  PWM_SEND_CH4  //��Ħ����

/* ���� PID ���� */
#define PID_TRIGGER_ECD_P 0.3f
#define PID_TRIGGER_ECD_I 0.0f
#define PID_TRIGGER_ECD_D 0.3f

#define PID_TRIGGER_SPD_P 5.0f
#define PID_TRIGGER_SPD_I 0.008f
#define PID_TRIGGER_SPD_D 0.0f

//#define PID_TRIGGER_ECD_P 0.34f
//#define PID_TRIGGER_ECD_I 0.0f
//#define PID_TRIGGER_ECD_D 0.0f

//#define PID_TRIGGER_SPD_P 15.0f
//#define PID_TRIGGER_SPD_I 0.2f
//#define PID_TRIGGER_SPD_D 0.0f

//90 66 60 50
//11.1 15.15 16.67 

/* ����Ƶ�� */
#define TRIGGER_PERIOD    90  //������ڣ�ms��

/*-----------------------------chassis---------------------------*/
#define RC_CH4_SCALE    12     
#define RC_CH3_SCALE    12

#define SPEED_45W  		3400.0f
#define SPEED_50W		4600.0f
#define SPEED_55W		4900.0f
#define SPEED_60W		5000.0f
#define SPEED_80W		6000.0f
#define SPEED_100W      6200.0f
#define SPEED_120W      7600.0f
#define SPEED_SUPERCAP  8000.0f

#define POWER_OFFSET    -2.0f		    //���ƹ��ʿ��ư��3.0W
#define SUPERCAP_MAX_VOLAGE	23.9f		//������������ѹ
/*-----------------------------gimbal----------------------------*/
#define Reduction_ratio			    1.0f	//pit����ٱ�
#define RC_CH2_SCALE                0.008f
#define RC_CH1_SCALE                -0.0003f

#define KEYBOARD_SCALE_PIT          -0.06f
#define KEYBOARD_SCALE_YAW	        -0.005f

#define GIMBAL_PIT_CENTER_OFFSET    2760
#define GIMBAL_PIT_MAX              3555
#define GIMBAL_PIT_MIN              2460

#define GIMBAL_YAW_CENTER_OFFSET    2700
#define GIMBAL_YAW_BETWEEN_ECD      (8191 / 8)
#define FIGHT_OFFSET_ERR            (-1.0f * GIMBAL_YAW_BETWEEN_ECD / 8191 * 2 * PI)

/* YAW��PIDϵ�� */
#define pid_yaw_angle_P 180.0f
#define pid_yaw_angle_I 0.0f
#define pid_yaw_angle_D 0.0f

#define pid_yaw_spd_P 20.0f
#define pid_yaw_spd_I 0.3f
#define pid_yaw_spd_D 0.0f

/* PIT��PIDϵ�� */
#define pid_pit_ecd_P 10.0f
#define pid_pit_ecd_I 0.0f
#define pid_pit_ecd_D 0.0f

#define pid_pit_spd_P 4.0f
#define pid_pit_spd_I 0.2f
#define pid_pit_spd_D 0.0f


//���飺0 ��������1 С������3
#endif

#endif
