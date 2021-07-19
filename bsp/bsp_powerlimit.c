/**
  ******************************************************************************
  * FileName    : bsp_powerlimit.c
  * Version     : v1.0
  * Author      : BZW
  * Date        : 2021-04-24
  * modification��
  * Functions   :
  * Description :
  *         ���ʿ����߼���
  *         Ŀ�꣺����ȫ�̣�����ʵ�ʹ��ʽӽ���ǰ����ʣ���ǰʣ�໺������һֱ�޷��ָ�����������״̬��
  *         ������
  *               1. ����������������Ϊ30J���Ƚϰ�ȫ�������������Ʒſ����������Ʊ������ͣ�Ҳ�бȽϳ���Ŀռ�����ס����
  *               2. �������Ʊ���������ʵ�ֹ��ܵ�ǰ���£�ԽСԽ�á�ʵ��Ч�����޹���ʱ��ʻ�Ƚ�˳��
  *               3. �������ƣ�����ʵ�ֹ��ܵ�ǰ���£�Խ��Խ�á���������Ϊ������ɲ��ʱ��������ټ��ɡ�
  *                            ��ʹ���Ʒſ����ܴﵽ���ٶ�Ҳ�����޵ģ�ֻҪ���������㹻�����ϵ������ƣ��������ƻ�����
  *
  *
  ******************************************************************************
  */
#include "bsp_powerlimit.h"
#include "chassis_task.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "math_calcu.h"
#include "math.h"
#include "remote_msg.h"
#include "string.h"
#include "bsp_can.h"
#include "can.h"
#include "modeswitch_task.h"
#include "pid.h"
#include "bsp_judge.h"
#include "control_def.h"

/* ----------------------------------------------------------------------------------------------------------------------- */
/* ���ʿ���Ҫ���Ĳ�������С��������   ����� */
#define 	BUFFER_MIN	   30.0f	//Ԥ�⻺������С�ڸ�ֵ����е�������  30
uint8_t   MAX_POWER_JUDGE = 80;		//����ϵͳ���Ļ���������� �ñ������ڸ���ֵ  ���յ�����ϵͳʱ�ᰴ����ϵͳ��Ϣ����

/* ----------------------------------------------------------------------------------------------------------------------- */
extern CAN_TxHeaderTypeDef Tx1Message;
extern CAN_RxHeaderTypeDef Rx1Message;

powercontrol_t powercontrol = {0};
supercap_t supercap;

void PowerControl_Init(void)
{
    powercontrol.max_power = MAX_POWER_JUDGE;
    powercontrol.limit_kp = 0.25f;  //�����Ȯ�� 0.4
    chassis.keyboard_input = 40.0f;
    powercontrol.power_buffer = 60.0f;
}

//float test_power = 5800.0f;  //5600 80W
//float test_input = 50.0f;
//float test_kp    = 0.2;
void PowerParam_Update(void)
{
    /* ����ϵͳ������ֵ */
    powercontrol.judge_power=Power_Heat_Data.chassis_power;  //�������ݶ�����
    powercontrol.judge_power_buffer=Power_Heat_Data.chassis_power_buffer;  //��������
    powercontrol.max_power=Game_Robot_Status.chassis_power_limit;

    powercontrol.power_buffer = powercontrol.judge_power_buffer;
    /* ���������޷� */
    if(powercontrol.power_buffer>=60)	 powercontrol.power_buffer = 60;

    /* �����������ˢ�� */
    if( rc.kb.bit.SHIFT && supercap.volage > 14.0f )  //�������ݵ�ѹ�ȵ����͹�����ѹ�ߣ��������������ٷŵ�ʱ
    {
        chassis.wheel_max = SPEED_SUPERCAP;
        chassis.keyboard_input = 50.0f;
    }
//    else
//    {
//        powercontrol.limit_kp = test_kp;
//        chassis.wheel_max = test_power;
//        chassis.keyboard_input = test_input;
//    }
    else if(powercontrol.max_power == 45 )
    {
        chassis.wheel_max = SPEED_45W;
        chassis.keyboard_input = 35.0f;
    }
    else if( powercontrol.max_power == 50 )
    {
        chassis.wheel_max = SPEED_50W;
        chassis.keyboard_input = 40.0f;
    }
    else if(powercontrol.max_power == 55 )
    {
        chassis.wheel_max = SPEED_55W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 60 )
    {
        chassis.wheel_max = SPEED_60W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 80 )
    {
        chassis.wheel_max = SPEED_80W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 100 )
    {
        chassis.wheel_max = SPEED_100W;
        chassis.keyboard_input = 40.0f;
    }
    else if( powercontrol.max_power == 120 || powercontrol.max_power == 65535 )
    {
        chassis.wheel_max = SPEED_120W;
        chassis.keyboard_input = 45.0f;
    }
}

/**
  * @name     Power_Control
  * @brief    ���ʿ���
  * @param    current:���̵�����������׵�ַ
  * @retval
  * @date     2021-04-24
  * @note
  */
void Power_Control(int16_t * current)
{
    static uint8_t powerlimit_cnt=0;  //���Ƽ���λ	ÿ�λ�����������5������100ms ������50�ι���
    if(supercap.mode!=2 || supercap.volage < 14.0f)  //�������ݷǷŵ�ģʽ�²Ž��й��ʿ���
    {
        if(powercontrol.power_buffer < BUFFER_MIN)
        {
            powercontrol.status=1;  //��������Сʱ����״̬λ ��1
            powerlimit_cnt = 0;
            powercontrol.limit_temp= powercontrol.limit_kp * (BUFFER_MIN - powercontrol.power_buffer);	//�������Ʊ���
        }

        if(powercontrol.status)
        {
            powerlimit_cnt++;
            for(uint8_t i=0; i<4; i++)
            {
                current[i] /= (ABS(powercontrol.limit_temp)+1.0f) ;
            }
        }

        if(powerlimit_cnt >= POWERLIMIT_CNT)
        {
            powercontrol.status = 0;  //����״̬λ����
            powerlimit_cnt = 0;  //���Ƽ���λ����
        }
    }
}


/**
  * @name     SuperCap_Mode_Update
  * @brief    ��������ģʽ�л�
  * @param    None
  * @retval
  * @date     2021-04-24
  * @note     0������ģʽ
  *           1��ֻ���ģʽ
  *           2����ŵ�ģʽ����Դ����ģ��ȫ�������ݳ�磬���ݸ����̷ŵ磬
  *                         ���ݵ�ѹ����ʱ�������ջ��������½���
  */
void SuperCap_Mode_Update(void)
{
    if( !lock_flag || ctrl_mode == PROTECT_MODE )
    {
        supercap.mode = 0; //������δ�������������ݽ��뱣��ģʽ
    }
    else if( supercap.volage >= 23.7f && !rc.kb.bit.SHIFT )
    {
        supercap.mode = 0;  //��ѹ����������ޣ���û��ʹ��SHIFT
    }
    else if( rc.kb.bit.SHIFT )
    {
        if( supercap.volage >= SUPERCAP_DISCHAGER_VOLAGE )
            supercap.mode = 2;  //����ѹ������10V���ɽ��г�ŵ����
        else if( powercontrol.power_buffer <= BUFFER_MIN )
            supercap.mode = 0;  //����Ҫ���й������ƣ�ֹͣ���
        else
            supercap.mode = 1;  //�ŵ��������͵�ѹС��12V���ҵ����õĹ��ʲ���ʱ���
    }
    else if( !rc.kb.bit.SHIFT )
    {
        if( powercontrol.power_buffer <= BUFFER_MIN )
            supercap.mode = 0;  //����Ҫ���й������ƣ�ֹͣ���
        else
            supercap.mode = 1;  //�ŵ��������͵�ѹС��12V���ҵ����õĹ��ʲ���ʱ���
    }
}

/**
  * @name     SuperCap_Control
  * @brief    �������ݿ���
  * @param    None
  * @retval
  * @note     ģʽ���£����ŵ繦�ʼ���������
  */
void SuperCap_Control(void)
{
    SuperCap_Mode_Update();
    if(supercap.mode==0)  //�������ݱ���ģʽ
    {
        supercap.charge_power_ref=0;
        supercap.charge_current_set=0;
    }
    else if(supercap.mode==1)  //��������ֻ���ģʽ
    {
        /* ������繦�� = ��Դ����-���̹��� */
        supercap.charge_power_fdb=powercontrol.supply_power-powercontrol.chassis_power-5.0f;  //�������ݰ����5W����
        if(supercap.charge_power_fdb<0)		supercap.charge_power_fdb=0;
        /* Ŀ���繦�� = �����-���̹���  ���ǿ����õĳ�繦�� */
        supercap.charge_power_ref=powercontrol.max_power-powercontrol.chassis_power-10.0f;  //��繦��Ҫ�Ե��������繦��
        if(supercap.charge_power_ref<0)	supercap.charge_power_ref=0;  //���Ѿ�������ʱ�������Ļ��������������̹��ʴ��������ù��ʣ�������
        /* Ŀ�����������=���й���/�������ݵ�ѹ */
        supercap.charge_current_set = supercap.charge_power_ref/(supercap.volage);
        if(supercap.charge_current_set<=0)				supercap.charge_current_set=0;
        else if(supercap.charge_current_set>=10)	supercap.charge_current_set=10;
    }
    else if(supercap.mode==2)  //�������ݱ߳��߷ŵ�ģʽ
    {

        /* �������ݷŵ��� ���й��� = ����ϵͳ����� */
        supercap.charge_power_ref=powercontrol.max_power-10;  //��繦��Ҫ�Ե��������繦��
        if(supercap.charge_power_ref<0)	supercap.charge_power_ref=0;
        supercap.charge_current_set = supercap.charge_power_ref/(supercap.volage);
        if(supercap.charge_current_set<=0)				supercap.charge_current_set=0;
        else if(supercap.charge_current_set>=10)	supercap.charge_current_set=10;
    }
}


/**
  * @name     can1_send_supercap
  * @brief
  * @param    None
  * @retval
  * @note
  */
void can1_send_supercap(void)
{
    uint8_t supercap_send_buff[8]= {0};
    uint8_t FreeTxNum = 0;

    Tx1Message.StdId = 0x010;
    Tx1Message.IDE 	 = CAN_ID_STD;
    Tx1Message.RTR   = CAN_RTR_DATA;
    Tx1Message.DLC   = 0x08;

    supercap_send_buff[0] = supercap.mode;  //��������ģʽ
    memcpy(supercap_send_buff+1,&supercap.charge_current_set,sizeof(supercap.charge_current_set));

    /* CAN���͵��ݿ���ģʽ */
    FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);  //��ѯ���������Ƿ�Ϊ��
    while(FreeTxNum == 0)
    {
        FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
    }

    HAL_CAN_AddTxMessage(&hcan1, &Tx1Message,supercap_send_buff,(uint32_t*)CAN_TX_MAILBOX1);
}

/**
  * @name   Power_data_handler
  * @brief
  * @param  can_id: ������ԴCAN
  *			CAN_Rx_data: CAN�������ݻ���
  * @retval
  * @note   ���ݿ��ư��õ��ݵ�ѹ���Դ����ģ���chassis�������
  *         ���ʿ��ư��õ���ʵ�ʹ���
  */
void Power_data_handler(uint32_t can_id,uint8_t * CAN_Rx_data)
{
    uint16_t supercap_voltage_buf = 0;
    uint16_t source_power_buf = 0;
    uint16_t chassis_power_buf = 0;

    memcpy(&supercap_voltage_buf, CAN_Rx_data, 2);
    memcpy(&source_power_buf, (CAN_Rx_data+2), 2);
    memcpy(&chassis_power_buf, (CAN_Rx_data+4), 2);

    /* ���ݵ�ѹ��Ϣ */
    supercap.volage = supercap_voltage_buf / 100.0f;
    supercap.volume_percent = (supercap.volage - SUPERCAP_DISCHAGER_VOLAGE) /
                              (SUPERCAP_MAX_VOLAGE - SUPERCAP_DISCHAGER_VOLAGE) * 100;
    /* ��Դ���� */
    powercontrol.supply_power = source_power_buf /100.0f;

    /* ����ʵʱ������Ϣ */
    powercontrol.chassis_power = chassis_power_buf / 100.0f;

}

