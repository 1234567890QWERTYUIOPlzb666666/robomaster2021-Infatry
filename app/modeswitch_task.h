#ifndef __MODESWITCH_TASK_H__
#define __MODESWITCH_TASK_H__

#include "stdint.h"

typedef enum
{
    PROTECT_MODE,
    LOCK_MODE,			//��̨�������е㣬���������˶�
    SEPARATE_MODE,      //������̨���룬���̸�����̨
    DANCE_MODE,         //ҡ��ģʽ
    REMOTER_MODE,
    KEYBOARD_MODE,      //����ģʽ
    VISION_MODE,        //�Ӿ�����ģʽ
    ENERGY_MODE,		//��������ģʽ
} ctrl_mode_e;

void mode_switch_task(void const *argu);
void unlock_init(void);  //����ģ�飬������ģ�����
extern ctrl_mode_e ctrl_mode;
extern uint8_t lock_flag;

#endif
