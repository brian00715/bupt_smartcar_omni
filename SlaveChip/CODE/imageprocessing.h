/*
 * imageprocessing.h
 *
 *  Created on: 2021��4��12��
 *      Author: Z.yd
 */

#ifndef CODE_IMAGEPROCESSING_H_
#define CODE_IMAGEPROCESSING_H_

#include "headfile.h"

/*
 * ��������ṹ��
 */
typedef struct
{
    uint16 x;
    uint16 y;
} Site_t;

/*
 * ������δ�С�ṹ��
 */
typedef struct
{
    uint16 W;       //��
    uint16 H;       //��
} Size_t;


/*** �궨�� ****///״̬��
#define leftround 1
#define leftround_entrance_flag 2
#define leftround_entrance 3
#define leftround_in 4
#define leftround_export 5
#define leftround_out_flag 6
#define leftround_out 16

#define rightround  7
#define rightround_entrance_flag 8
#define rightround_entrance 9
#define rightround_in 10
#define rightround_export 11
#define rightround_out_flag 12
#define rightround_out 13



extern uint8 Image_Process_Flag;
extern uint8 Threshold;
extern uint16 Threshold_ChaHe;
extern uint8 mt9v03x_image_binary[MT9V03X_H][MT9V03X_W];
extern uint8 Image_Lline[51];     //�洢��߽������
extern uint8 Image_Rline[51];     //�洢�ұ߽������
extern uint8 Image_Mline[51];     //�洢���ߵ�����
extern uint8 Image_MTuBian_Sum;    //�洢����ͻ�������
extern uint8 Image_MTuBian[10];
extern uint8 Image_LBig_Curve_Flag; //������־
extern uint8 Image_RBig_Curve_Flag; //�Ҵ����־
extern float Image_XieLv_float[2];
extern int16 Image_XieLv_int;
extern uint8 Image_Show_Flag;
extern uint8 Image_MGuaiDian_Sum;
extern uint8 Image_MGuaiDian[10];
extern int32 count;

extern uint8 Image_LTuBian_Sum;      //��¼ͻ�����������
extern uint8 Image_LTuBian[10];    //��¼���߷���ͻ��ʱ�ĸ߶�
extern uint8 Image_LGuaiDian[10];  //��¼�����յ��λ��
extern uint8 Image_LGuaiDian_Sum;    //��¼�յ�����

extern uint8 Image_RTuBian_Sum;      //��¼ͻ�����������
extern uint8 Image_RTuBian[10];    //��¼���߷���ͻ��ʱ�ĸ߶�
extern uint8 Image_RGuaiDian[10];  //��¼�����յ��λ��
extern uint8 Image_RGuaiDian_Sum;    //��¼�յ�����

extern uint8 Image_Llost_Sum;       //��߶�������
extern uint8 Image_Rlost_Sum;       //�ұ߶�������
extern uint8 Image_Llost[50];      //��¼��߶���λ��
extern uint8 Image_Rlost[50];      //��¼�ұ߶���λ��

extern uint8 Image_LIsland_Meet_Flag ;   //�󻷵������־
extern uint8 Image_RIsland_Meet_Flag ;  //���󻷵���־
extern uint8 Image_LIsland_In_Flag ;   //�󻷵������־
extern uint8 Image_LIsland_Out_Flag ;  //���󻷵���־
extern uint8 Image_RIsland_In_Flag ;   //���һ�����־
extern uint8 Image_RIsland_Out_Flag ;  //���һ�����־

extern float Image_MAdd_float;
extern int16 Image_MAdd_int;
extern int16 Image_Error;

extern void Image_Binary(uint8 Threshold);
extern void Image_Processing(void);
extern float Image_Regression(int startline,int endline,int n);
extern float Image_MAdd(int startline, int endline);
extern uint8 Image_Border_Judge(uint8 Image_1,uint8 Image_2);

extern int   MidInit ;

extern void state_machine();
extern void right_circle();
extern void left_circle();
extern int16 position_error();
extern void FindBorder(void);
extern void Img_Prc(void);

#endif /* CODE_IMAGEPROCESSING_H_ */
