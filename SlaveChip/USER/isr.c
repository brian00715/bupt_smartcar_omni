/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：一群：179029047(已满)  二群：244861897(已满)  三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file            isr
 * @company         成都逐飞科技有限公司
 * @author          逐飞科技(QQ790875685)
 * @version         查看doc内version文件 版本说明
 * @Software        MounRiver Studio V1.3
 * @Target core     CH32V103R8T6
 * @Taobao          https://seekfree.taobao.com/
 * @date            2020-12-04
 ********************************************************************************************************************/

#include "headfile.h"

uint8 Gpio_Sup_Up[3] = { 0 };
uint8 Gpio_Sup_Down[3] = { 0 };
uint16 Pwm_Count = 0;
uint8 Pwm_Up_Down = 0;

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel7_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void ADC1_2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI9_5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_BRK_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_TRG_COM_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_CC_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void I2C1_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void I2C1_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void I2C2_EV_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void I2C2_ER_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void SPI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void SPI2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_10_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void RTCAlarm_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void USBWakeUp_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void USBHD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

//外部中断
void EXTI0_IRQHandler(void) {

}

void EXTI1_IRQHandler(void) {
    if (SET == EXTI_GetITStatus(EXTI_Line1)) {
        if (camera_type == CAMERA_BIN_UART)
            ov7725_uart_vsync();
        else if (camera_type == CAMERA_GRAYSCALE)
            mt9v03x_vsync();
        EXTI_ClearITPendingBit(EXTI_Line1);
    }

}

void EXTI2_IRQHandler(void) {
    EXTI_ClearITPendingBit(EXTI_Line2);
}

void EXTI3_IRQHandler(void) {

}

void EXTI4_IRQHandler(void) {

}

void EXTI9_5_IRQHandler(void) {

}

void EXTI15_10_IRQHandler(void) {

}

void ADC1_2_IRQHandler(void) {

}

//定时中断
void TIM1_BRK_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Break) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Break);

    }
}

void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

    }
}

void TIM1_TRG_COM_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Trigger) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Trigger);

    }
    if (TIM_GetITStatus(TIM1, TIM_IT_COM) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_COM);

    }
}

void TIM1_CC_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);

    }
    if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);

    }
    if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);

    }
    if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);

    }
}
//int32 count=0;
void TIM2_IRQHandler(void) {

    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    }
}

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}

//uint8 Uart_SendData[20]={0};
void TIM4_IRQHandler(void) {
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
        //        if(mt9v03x_finish_flag==1)
        //        {
        //            mt9v03x_finish_flag=0;
        //            ips114_showint32(0,13,count++,10);
        //        }
        /********************************若按下B2则重新获取adc最大值,控制屏幕图像显示和从机数据发送**********************************************/
        if (gpio_get(B2) == 0)
            Gpio_Sup_Up[0]++;
        if (gpio_get(B2) == 1 && Gpio_Sup_Up[0] > 5)
            Gpio_Sup_Down[0]++;
        if (gpio_get(B2) == 1 && Gpio_Sup_Up[0] > 5 && Gpio_Sup_Down[0] > 5) {
            InducerMax_Get_Start_Flag = 1;
            Image_Show_Flag = (Image_Show_Flag + 1) % 2;        //是否在屏幕上显示图像标志
            Gpio_Sup_Up[0] = 0;
            Gpio_Sup_Down[0] = 0;
        }
        /************************按下B5//C9减小二值化阈值**********************************/
        if (gpio_get(C9) == 0)
            Gpio_Sup_Up[1]++;
        if (gpio_get(C9) == 1 && Gpio_Sup_Up[1] > 5)
            Gpio_Sup_Down[1]++;
        if (gpio_get(C9) == 1 && Gpio_Sup_Up[1] > 5 && Gpio_Sup_Down[1] > 5) {
            if (Threshold - 5 > 0) {
                Threshold--;
                Threshold_ChaHe -= 5;
            }
            Gpio_Sup_Up[1] = 0;
            Gpio_Sup_Down[1] = 0;
        }
        /************************按下B2//C8增加二值化阈值*********************************/
        if (gpio_get(C8) == 0)
            Gpio_Sup_Up[2]++;
        if (gpio_get(C8) == 1 && Gpio_Sup_Up[2] > 5)
            Gpio_Sup_Down[2]++;
        if (gpio_get(C8) == 1 && Gpio_Sup_Up[2] > 5 && Gpio_Sup_Down[2] > 5) {
            if (Threshold < 255) {
                Threshold++;
            }
            Threshold_ChaHe += 5;
            Gpio_Sup_Up[2] = 0;
            Gpio_Sup_Down[2] = 0;
        }
        /******************屏幕显示图像与从机发送数据不同时进行********************/
        if(Image_Show_Flag==0)
        {
//            Image_Processing();
            encoder_data[0] = timer_quad_get(TIMER_2); //编码器取值
            encoder_data[1] = timer_quad_get(TIMER_3); //编码器取值
            timer_quad_clear(TIMER_2);                 //清空计数器
            timer_quad_clear(TIMER_3);                 //清空计数器


//            Uart_SendData[0]=0x00;Uart_SendData[1]=0xff;
//            Uart_SendData[2]=(encoder_data[0]>>8)&0xff;Uart_SendData[3]=encoder_data[0]&0xff;
//            Uart_SendData[4]=(encoder_data[1]>>8)&0xff;Uart_SendData[5]=encoder_data[1]&0xff;
//            Uart_SendData[6]=Image_Process_Flag;
//            Uart_SendData[7]=(Image_XieLv_int>>8)&0xff;Uart_SendData[8]=Image_XieLv_int&0xff;
//            Uart_SendData[9]=Image_LBig_Curve_Flag;Uart_SendData[10]=Image_RBig_Curve_Flag;
//            Uart_SendData[11]=0xee;Uart_SendData[12]=0x11;
//            uart_putbuff(UART_3, Uart_SendData, 13);

            /*************************给主片定时发送数据*****************************/
            uart_putchar(UART_3, 0x00);uart_putchar(UART_3, 0xff);      //帧头
//            uart_putchar(UART_3,0x20);
            uart_putdoublechar(UART_3,encoder_data[0]);uart_putdoublechar(UART_3,encoder_data[1]); //发送编码器数据
//            uart_putchar(UART_3,0x21);
//            uart_putchar(UART_3,Image_Process_Flag);   //发送是否进行图像处理标志，1为已进行图像处理，0为未进行图像处理
            if(Image_Process_Flag ==1)
            {
                uart_putchar(UART_3,1);   //发送是否进行图像处理标志，1为已进行图像处理，0为未进行图像处理
                Image_Process_Flag = 0;
            }
            else uart_putchar(UART_3,0);
//            uart_putchar(UART_3,0x22);
            uart_putdoublechar(UART_3, Image_Error);
            // uart_putdoublechar(UART_3, Image_MAdd_int);    //发送整形中线差和
//            uart_putdoublechar(UART_3,Image_XieLv_int);   //发送整形斜率（原斜率*1000）
            uart_putchar(UART_3,Image_Mline[0]); //发送中点坐标
//            uart_putchar(UART_3,0x23);
//            uart_putchar(UART_3,Image_LBig_Curve_Flag);uart_putchar(UART_3,Image_RBig_Curve_Flag); //发送左大弯和右大弯标志位
            if(Image_LBig_Curve_Flag) uart_putchar(UART_3,8);
            else if(Image_RBig_Curve_Flag) uart_putchar(UART_3,9);
            else uart_putchar(UART_3,1);

            uart_putchar(UART_3, 0xee);uart_putchar(UART_3, 0x11);      //帧尾
        }
    }
}

//串口中断
void USART1_IRQHandler(void) {

}

void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        if (camera_type == CAMERA_BIN_UART)
            ov7725_cof_uart_interrupt();
        else if (camera_type == CAMERA_GRAYSCALE)
            mt9v03x_uart_callback();
    }
}

void USART3_IRQHandler(void) {

}

void DMA1_Channel4_IRQHandler(void) {
    if (SET == DMA_GetFlagStatus(DMA1_FLAG_TC4)) {
        DMA_ClearFlag(DMA1_FLAG_TC4);
        if (camera_type == CAMERA_BIN_UART)
            ov7725_uart_dma();
        else if (camera_type == CAMERA_GRAYSCALE)
            mt9v03x_dma();
    }
}

/*******************************************************************************
 * Function Name  : NMI_Handler
 * Description    : This function handles NMI exception.
 * Input          : None
 * Return         : None
 *******************************************************************************/
void NMI_Handler(void) {

}

/*******************************************************************************
 * Function Name  : HardFault_Handler
 * Description    : This function handles Hard Fault exception.
 * Input          : None
 * Return         : None
 *******************************************************************************/
void HardFault_Handler(void) {

    while (1) {
    }
}

