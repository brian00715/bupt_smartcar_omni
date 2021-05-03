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
#include "string.h"
#include "isr.h"
#include "config.h"
#include "cmd.h"
#include "sci_compute.h"
#include "mecanum_chassis.h"
#include "encoder.h"
#include "slave_comm.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
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
//void TIM1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
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

void EXTI0_IRQHandler(void)
{
}

void EXTI1_IRQHandler(void)
{
	if (SET == EXTI_GetITStatus(EXTI_Line1))
	{
		//		if (camera_type == CAMERA_BIN_UART)
		//			ov7725_uart_vsync();
		//		else if (camera_type == CAMERA_GRAYSCALE)
		//		mt9v03x_vsync();
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void EXTI2_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line2);
}

void EXTI3_IRQHandler(void)
{
}

void EXTI4_IRQHandler(void)
{
}

void EXTI9_5_IRQHandler(void)
{
}

void EXTI15_10_IRQHandler(void)
{
}

void ADC1_2_IRQHandler(void)
{
}

void TIM1_BRK_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Break) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
	}
}

uint8 tim1_5ms_cnt = 0;
char TIM1_10ms_Flag = 0;
char TIM1_20ms_Flag = 0;
char TIM1_100ms_Flag = 0;
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		tim1_5ms_cnt++;
		if (tim1_5ms_cnt % 2 == 0)
		{
			TIM1_10ms_Flag = 1;
			MecanumChassis.send_ctrl_msg_flag = 1;
		}
		else
		{
			TIM1_10ms_Flag = 0;
		}
		if (tim1_5ms_cnt % 4 == 0)
		{
			TIM1_20ms_Flag = 1;
		}
		else
		{
			TIM1_20ms_Flag = 0;
		}
		if (tim1_5ms_cnt % 20 == 0)
		{
			TIM1_100ms_Flag = 1;
		}
		else
		{
			TIM1_100ms_Flag = 0;
		}
		//硬件SPI采集
		//		get_icm20602_accdata_spi();
		//		get_icm20602_gyro_spi();
		//		if (icm_gyro_z > 2000)
		//		{
		//			icm_gyro_z = 2000;
		//		}
		//		else if (icm_gyro_z < -2000)
		//		{
		//			icm_gyro_z = -2000;
		//		}
		//		MecanumChassis.posture_status.yaw = KalmanFilter(icm_acc_z * 1.0, icm_gyro_z * 1.0);
		//		MecanumChassis.posture_status.yaw += RAD2ANGLE(icm_gyro_z * 0.005); // 偏航角积分

		if (MecanumChassis.motor_self_check_ok)
		{
			encoder_data[0] = encoder_coff[0] * timer_quad_get(TIMER_2); //编码器取值
			encoder_data[1] = encoder_coff[1] * timer_quad_get(TIMER_3); //编码器取值
			MecanumChassis.motor[0].now_rpm = encoder_data[0];
			MecanumChassis.motor[1].now_rpm = encoder_data[1];
			timer_quad_clear(TIMER_2); //清空计数器
			timer_quad_clear(TIMER_3); //清空计数器
		}
		else
		{
			encoder_data[0] = timer_quad_get(TIMER_2); //编码器取值
			encoder_data[1] = timer_quad_get(TIMER_3); //编码器取值
			timer_quad_clear(TIMER_2);				   //清空计数器
			timer_quad_clear(TIMER_3);				   //清空计数器
		}
	}
}

void TIM1_TRG_COM_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Trigger) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Trigger);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_COM) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_COM);
	}
}

void TIM1_CC_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
	}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
	}
}

void TIM2_IRQHandler(void)
{

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

uint8_t UART1_RxBuffer[RX_BUFFER_SIZE] =
	{0};
uint8_t UART1_RxBufferCnt = 0;
uint8_t UART1_RxComplete = 0;
uint8_t UART1_RxIDLEFlag = 0;		// 闲时中断标志位
uint8_t UART1_RxBufferOverflow = 0; // 缓冲数组溢出标志
void USART1_IRQHandler(void)
{
	//>>>中断方式接收数据<<<
	//	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	//	{
	//		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	//
	//		UART1_RxBuffer[UART1_RxBufferCnt++] = USART_ReceiveData(USART1);
	//
	//		if (UART1_RxBuffer[UART1_RxBufferCnt - 1] == 0x0D) // XCOM 的发送新行只有0X0D
	//		{
	//			UART1_RxBuffer[UART1_RxBufferCnt - 1] ='\0';
	//			UART1_RxComplete = 1;
	//			UART1_RxBufferCnt = 0;
	//		}
	//		if (UART1_RxBufferCnt == RX_BUFFER_SIZE - 1)
	//		{
	//			UART1_RxBufferCnt = 0;
	//			UART1_RxBuffer[UART1_RxBufferCnt] ='\0';
	//			UART_RxBufferOverflow  = 1;
	////			memset(UART_RxBuffer, 0, sizeof(uint8_t) * RX_BUFFER_SIZE);
	//		}
	//	}

	// >>>DMA方式接收数据<<<
	if (USART_GetFlagStatus(USART1, USART_FLAG_IDLE) != RESET)
	{
		USART_ClearFlag(USART1, USART_FLAG_IDLE);
		UART1_RxIDLEFlag = 1;
		uint16_t tmp;
		UNUSED(tmp); // 避免GCC编译器警告
		tmp = USART1->STATR;
		tmp = USART1->DATAR;			 // 根据应用手册，必须要有这两步，否则清标志位的操作其实并不生效
		DMA_Cmd(DMA1_Channel5, DISABLE); //关闭本次DMA

		uint8_t num = RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5); //得到真正接收数据个数
		DMA1_Channel5->CNTR = RX_BUFFER_SIZE;
		UART1_RxBuffer[num] = ' '; // 末尾加空格，否则无法正常解析最后一个参数
		UART1_RxBuffer[num + 1] = '\0';
		CMD_UARTCallback();

		DMA_Cmd(DMA1_Channel5, ENABLE); //开启下一次DMA
	}
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}

uint8_t UART3_RxBuffer[20] =
	{0};
uint8_t UART3_RxBufferCnt = 0;
uint8_t UART3_RxOK = 0;
void USART3_IRQHandler(void)
{
	// >>>DMA方式接收数据<<<
	if (USART_GetFlagStatus(USART3, USART_FLAG_IDLE) != RESET)
	{
		uint16_t tmp;
		UNUSED(tmp); // 避免GCC编译器警告
		tmp = USART3->STATR;
		tmp = USART3->DATAR;			 // 根据应用手册，必须要有这两步，否则清标志位的操作其实并不生效
		DMA_Cmd(DMA1_Channel3, DISABLE); //关闭本次DMA

//		uint8_t num = 20 - DMA_GetCurrDataCounter(DMA1_Channel3); //得到真正接收数据个数
		DMA1_Channel3->CNTR = 20;
		SlaveComm_UARTCallback();

		DMA_Cmd(DMA1_Channel3, ENABLE); //开启下一次DMA
		USART_ClearFlag(USART3, USART_FLAG_IDLE);
	}
}

void DMA1_Channel3_IRQHandler(void)
{
	if (SET == DMA_GetFlagStatus(DMA1_FLAG_TC3))
	{
		DMA_ClearFlag(DMA1_FLAG_TC3);
	}
}

void DMA1_Channel4_IRQHandler(void)
{
	if (SET == DMA_GetFlagStatus(DMA1_FLAG_TC4))
	{
		DMA_ClearFlag(DMA1_FLAG_TC4);
		//		if (camera_type == CAMERA_BIN_UART)
		//			ov7725_uart_dma();
		//		else if (camera_type == CAMERA_GRAYSCALE)
		//		mt9v03x_dma();
	}
}

void DMA1_Channel5_IRQHandler(void)
{
	if (SET == DMA_GetFlagStatus(DMA1_FLAG_TC5)) // 传输完成标志位
	{
		DMA_ClearFlag(DMA1_FLAG_TC5);
	}
}

/*******************************************************************************
 * Function Name  : NMI_Handler
 * Description    : This function handles NMI exception.
 * Input          : None
 * Return         : None
 *******************************************************************************/
void NMI_Handler(void)
{
}

/*******************************************************************************
 * Function Name  : HardFault_Handler
 * Description    : This function handles Hard Fault exception.
 * Input          : None
 * Return         : None
 *******************************************************************************/
void HardFault_Handler(void)
{

	while (1)
	{
	}
}
