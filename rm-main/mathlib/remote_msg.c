/**
  * @file     remote_msg.c
  * @version  v2.0
  * @date     July,6th 2019
  *
  * @brief    ң�������ݽ����ļ�
  *
  *	@author   Fatmouse
  *
  */
#include "remote_msg.h"
#include "stdlib.h"
#include "string.h"
#include "gimbal_task.h"
#include "modeswitch_task.h"

/* ң�������� */
rc_info_t rc;

/* ң�������ݴ���״̬��־ */
uint8_t rc_normal_flag = 0;

/* ���������� */
scale_t scale;

/* ���̰���״̬��־λ */
uint8_t kb_status[11]= {0};


/**
  * @brief       handle received rc data
  * @param[out]  rc:   structure to save handled rc data
  * @param[in]   buff: the buff which saved raw rc data
  * @retval
  */
void rc_callback_handler(rc_info_t *rc, uint8_t *buff)
{
    /* ң����ͨ�ż�� */
    rc_normal_flag = 1;
    /* ���ݽ��� */
    rc->ch1 = (buff[0]      | buff[1]  << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2]  << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3]  << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5]  << 7) & 0x07FF;
    rc->ch4 -= 1024;
    rc->ch5=  (buff[16]     | buff[17] << 8) & 0x07FF;
    rc->ch5 -= 1024;
    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    if ((abs(rc->ch1) > 660) || \
        (abs(rc->ch2) > 660) || \
        (abs(rc->ch3) > 660) || \
        (abs(rc->ch4) > 660)    )
    {
        memset(rc, 0, sizeof(rc_info_t));
    }
    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];

    rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code

}


static uint16_t key_map(key_index_e key_index)
{
    uint16_t key;
    switch( key_index )
    {
        case KB_Q:  key = rc.kb.bit.Q; break;
        case KB_E:  key = rc.kb.bit.E; break;
        case KB_R:  key = rc.kb.bit.R; break;
        case KB_F:  key = rc.kb.bit.F; break;
        case KB_G:  key = rc.kb.bit.G; break;
        case KB_Z:  key = rc.kb.bit.Z; break;
        case KB_X:  key = rc.kb.bit.X; break;
        case KB_C:  key = rc.kb.bit.C; break;
        case KB_V:  key = rc.kb.bit.V; break;
        case KB_B:  key = rc.kb.bit.B; break;
        case KB_CTRL:
                    key = rc.kb.bit.CTRL; break;
        default:    break;
    }
    return key;
}

/**
  * @brief       ���̰���ɨ��
  * @param[out]  kb_status[key_index]: �������²������״̬��־λȡ��
  * @param[in]   key: 				   �����İ���
	*			 key_index��	       �����İ������
  * @note        ����Ҫ��
  * @author      ZZJ
  */
void keyboard_scanf(key_index_e key_index)
{
    static uint8_t key_press[11]= {0};
    uint16_t key = key_map(key_index);
    if(key && (key_press[key_index]==0))
    {
        key_press[key_index] = 1;
        kb_status[key_index] = !kb_status[key_index];
    }
    else if(key == 0)
    {
        key_press[key_index] = 0;
    }
}

void key_status_clear(key_index_e key_index)
{
    kb_status[key_index] = KEY_END;
}

