#ifndef __FUNC_GENERATOR_H__
#define __FUNC_GENERATOR_H__

#ifdef  __FUNC_GENERATOR_GLOBALS__
    #define __FUNC_GENERATOR_EXT
#else
    #define __FUNC_GENERATOR_EXT extern
#endif

#include "stdint.h"
#include "stdlib.h"
#include "math.h"
#include "math_calcu.h"


/*------------------------------------�����źŽ�ֱ����������-----------------------------------*/
typedef struct
{
    uint16_t Td;    //�ź���������
    uint32_t time;  //��ǰ����ʱ��

    float max;      //�ź�����ֵ
    float min;      //�ź���С��ֵ

    float dc;       //ֱ����

    float T;        //�����źŵ�����
    float A;        //�����źŵ����
    float phi;      //�����źŵĳ���

    float out;      //�����¼
} FGT_sin_t;


void FGT_sin_init(
    FGT_sin_t*  sin,
    uint16_t    Td,
    float       max,
    float       min,
    float       dc,
    float       T,
    float       A,
    float       phi
);

void FGT_sin_reinit(
    FGT_sin_t*  sin,
    float       max,
    float       min,
    float       dc,
    float       T,
    float       A
);

float FGT_sin_cal(FGT_sin_t* sin);

/*------------------------------------�����źŽ�ֱ����������-----------------------------------*/
typedef struct
{
    uint16_t Td;
    uint32_t time;

    float max;
    float min;

    float dc;

    float Th;   //�ߵ�ƽʱ��
    float Tl;   //�͵�ƽʱ��
    float high; //�ߵ�ƽ��ֵ
    float low;  //�͵�ƽ��ֵ

    float out;
} FGT_sqr_t;


void FGT_sqr_init(
    FGT_sqr_t* sqr,

    uint16_t Td,

    float max,
    float min,

    float dc,

    float Th,
    float Tl,
    float high,
    float low
);

float FGT_sqr_cal(FGT_sqr_t* sqr);


/*------------------------------------�ǲ��źŽ�ֱ����������-----------------------------------*/
typedef struct
{
    uint16_t Td;
    uint32_t time;

    float max;
    float min;

    float dc;

    float T1;   //����ʼ����һ���յ��ʱ��
    float T2;   //��һ�����յ���ʱ��
    float high;
    float low;
    float out;
} FGT_agl_t;


void FGT_agl_init(
    FGT_agl_t* agl,

    uint16_t Td,
    float max,
    float min,

    float dc,

    float T1,
    float T2,
    float high,
    float low
);

float FGT_agl_cal(FGT_agl_t* agl);


/*------------------------------------���ź���-----------------------------------*/
typedef struct
{
    uint16_t Td;
    uint32_t time;

    float T1;   //����ʼ����һ���յ��ʱ��
    float T2;   //��һ�����յ���ʱ��
    float T3;   //�ڶ����յ㵽����ĩ��ʱ��

    float out;
} FGT_npz_t;

void FGT_npz_init(
    FGT_npz_t* npz,
    uint16_t Td,
    float T1,
    float T2,
    float T3
);

float FGT_npz_cal(FGT_npz_t* npz);


/*------------------------------------�������������-----------------------------------*/
/*ʹ��Ӳ��RNGʱ����*/
//HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);
int RNG_Get_RandomRange(int min,int max);


/*------------------------------------��˹����-----------------------------------*/
float GaussGenerate(float mu, float sigma_f);
void Gauss(float gs[], int lengh, float mu, float sigma_f);


/*------------------------------------һ�㺯���źŷ�����-----------------------------------*/
typedef struct _FGT_f_t
{
    float (*f)(float x);
    float time;
    float T;
    float Td;
    float max;
    float min;
    float out;
} FGT_f_t;

float FGT_f_generator(FGT_f_t* pf);

/*------------------------------------�����������ܳ�ʼ��-----------------------------------*/
void FGT_init(void);

__FUNC_GENERATOR_EXT FGT_sin_t test_sin;
__FUNC_GENERATOR_EXT FGT_sin_t test_cos;
__FUNC_GENERATOR_EXT FGT_sqr_t test_sqr;
__FUNC_GENERATOR_EXT FGT_agl_t test_agl;
__FUNC_GENERATOR_EXT FGT_npz_t test_npz_101;
__FUNC_GENERATOR_EXT FGT_npz_t test_npz_10;
__FUNC_GENERATOR_EXT FGT_f_t test_f;

__FUNC_GENERATOR_EXT FGT_sqr_t test_pit;
__FUNC_GENERATOR_EXT FGT_sin_t chassis_spin_sin;


#endif
