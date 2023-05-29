#ifndef USER_LIB_H
#define USER_LIB_H
#include "struct_typedef.h"
#include <assert.h>
#include "stdlib.h"

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))


//��������
#define ElemType float
//���е����ռ�
#define MAXSIZE 10
//���еĹ���ṹ
typedef struct
{
ElemType *base; //ָ����пռ�Ļ�ַ
int front; //ͷָ��
int rear; //βָ��
}Queue;


#define QUEUE_SIZE 50
struct queue {
	float  data[QUEUE_SIZE];
	int front;
	int  tail;
	int empty;
};


typedef struct
{
    uint16_t nowLength;
    uint16_t queueLength;
    float queueTotal;
    //����
    float queue[100];
    uint8_t full_flag;
} QueueObj;

void Queueinit(struct queue *);
void enqueue(struct queue *, float);
int dequeue(struct queue *);



typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�������
    fp32 min_value;    //�޷���Сֵ
    fp32 max_value;    //�޷����ֵ
    fp32 frame_period; //ʱ����
} ramp_function_source_t;

typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�˲����������
    fp32 num[1];       //�˲�����
    fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
//���ٿ���
extern fp32 invSqrt(fp32 num);

//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
void ramp_calc_min(ramp_function_source_t *ramp_source_type, fp32 input);
//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//��������
extern void abs_limit(fp32 *num, fp32 Limit);
//�жϷ���λ
extern fp32 sign(fp32 value);
//��������
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26����
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//�޷�����
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern fp32 theta_format(fp32 Ang);
extern float Run_Rand_Get(int Min_Rand_Set,int Max_Rand_Set);
extern void VAL_LIMIT(float *Value, float minValue, float maxValue);

void InitQueue(Queue *Q);
void EnQueue(Queue *Q, ElemType x);
void ShowQueue(Queue *Q);
void DeQueue(Queue *Q);
void GetHdad(Queue *Q, ElemType *v);
int Length(Queue *Q);
void ClearQueue(Queue *Q);
void DestroyQueue(Queue *Q);

void Clear_Queue(QueueObj* queue);
void LoopQueueYaw(uint8_t queue_len,float Data);
void  LoopQueuePitch(uint8_t queue_len,float Data);

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
