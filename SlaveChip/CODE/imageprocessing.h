/*
 * imageprocessing.h
 *
 *  Created on: 2021年4月12日
 *      Author: Z.yd
 */

#ifndef CODE_IMAGEPROCESSING_H_
#define CODE_IMAGEPROCESSING_H_

#include "headfile.h"

/*
 * 定义坐标结构体
 */
typedef struct
{
    uint16 x;
    uint16 y;
} Site_t;

/*
 * 定义矩形大小结构体
 */
typedef struct
{
    uint16 W;       //宽
    uint16 H;       //高
} Size_t;


/*** 宏定义 ****///状态机
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
extern uint8 Image_Lline[51];     //存储左边界的数组
extern uint8 Image_Rline[51];     //存储右边界的数组
extern uint8 Image_Mline[51];     //存储中线的数组
extern uint8 Image_MTuBian_Sum;    //存储产生突变的总数
extern uint8 Image_MTuBian[10];
extern uint8 Image_LBig_Curve_Flag; //左大弯标志
extern uint8 Image_RBig_Curve_Flag; //右大弯标志
extern float Image_XieLv_float[2];
extern int16 Image_XieLv_int;
extern uint8 Image_Show_Flag;
extern uint8 Image_MGuaiDian_Sum;
extern uint8 Image_MGuaiDian[10];
extern int32 count;

extern uint8 Image_LTuBian_Sum;      //记录突变产生的总数
extern uint8 Image_LTuBian[10];    //记录中线发生突变时的高度
extern uint8 Image_LGuaiDian[10];  //记录产生拐点的位置
extern uint8 Image_LGuaiDian_Sum;    //记录拐点数量

extern uint8 Image_RTuBian_Sum;      //记录突变产生的总数
extern uint8 Image_RTuBian[10];    //记录中线发生突变时的高度
extern uint8 Image_RGuaiDian[10];  //记录产生拐点的位置
extern uint8 Image_RGuaiDian_Sum;    //记录拐点数量

extern uint8 Image_Llost_Sum;       //左边丢线数量
extern uint8 Image_Rlost_Sum;       //右边丢线数量
extern uint8 Image_Llost[50];      //记录左边丢线位置
extern uint8 Image_Rlost[50];      //记录右边丢线位置

extern uint8 Image_LIsland_Meet_Flag ;   //左环岛进入标志
extern uint8 Image_RIsland_Meet_Flag ;  //出左环岛标志
extern uint8 Image_LIsland_In_Flag ;   //左环岛进入标志
extern uint8 Image_LIsland_Out_Flag ;  //出左环岛标志
extern uint8 Image_RIsland_In_Flag ;   //进右环岛标志
extern uint8 Image_RIsland_Out_Flag ;  //出右环岛标志

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
