#include "headfile.h"
#include "string.h"
#include "isr.h"
#include "config.h"
#include "cmd.h"
#include "sci_compute.h"
#include "mecanum_chassis.h"
#include "encoder.h"
#include "slave_comm.h"
#include "handle.h"

int32 time_count = 0;
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
//void DMA1_Channel4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void DMA1_Channel5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel6_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
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
//void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
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
uint8 TIM1_500ms_Flag = 0;
int stop_count = 0;
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		time_count++;
		tim1_5ms_cnt++;

		if (stop_flag == 1)
		{
			stop_count++;
			if (stop_count <= 10)
			{
				MecanumChassis.target_speed = 0.52;
				MecanumChassis.target_omega = 0;
				MecanumChassis.target_dir = 1.5708;
			}
			else if (stop_count <= 100)
			{
				MecanumChassis.target_speed = 0.52;
				MecanumChassis.target_omega = -1.61;
				MecanumChassis.target_dir = 1.5708;
			}
			else if (stop_count <= 125)
			{
				MecanumChassis.target_speed = 0.52;
				MecanumChassis.target_omega = 0;
				MecanumChassis.target_dir = 1.5708;
			}

			else
			{
				MecanumChassis.target_speed = 0;
				MecanumChassis.target_omega = 0;
				MecanumChassis.target_dir = 1.5708;
			}
		}

		if (tim1_5ms_cnt % 2 == 0)
		{
			TIM1_10ms_Flag = 1;
			MecanumChassis.send_ctrl_msg_flag = 1; // 限制电机控制命令频率
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

		if (tim1_5ms_cnt % 200 == 0) // 1s
		{
			//			if (UART3_RxOK)
			// uprintf("Slave Send OK.\r\n");
		}

		if (tim1_5ms_cnt % 100 == 0)
		{
			TIM1_500ms_Flag = 1;
		}
		else
		{
			TIM1_500ms_Flag = 0;
		}

		//硬件SPI采集陀螺仪数据
		get_icm20602_accdata_spi();
		get_icm20602_gyro_spi();
		icm_acc_z = ((float) icm_acc_raw_z / 4096);				// ±8g
		icm_gyro_z = __ANGLE2RAD((float )icm_gyro_raw_z / 16.4); // ±2000dps,16 bit adc
		icm_gyro_y = __ANGLE2RAD((float )icm_gyro_raw_y / 16.4);
		// MecanumChassis.PostureStatus.yaw = KalmanFilter(icm_acc_raw_z * 1.0, icm_gyro_raw_z * 1.0);
		if (fabs(icm_gyro_z) > 0.023) // 过滤零漂
		{
			MecanumChassis.PostureStatus.yaw += (icm_gyro_z - 0.019) * 0.005; // 偏航角积分
		}

		// >>>采集编码器12的数据<<<
		encoder_data[0] = timer_quad_get(TIMER_2); //编码器取值
		encoder_data[1] = timer_quad_get(TIMER_3); //编码器取值
		if (MecanumChassis.motor_self_check_ok)
		{
			encoder_data[0] *= encoder_coff[0];
			encoder_data[1] *= encoder_coff[1];
		}
		timer_quad_clear(TIMER_2); //清空计数器
		timer_quad_clear(TIMER_3); //清空计数器
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

// void TIM4_IRQHandler(void)
// {
// 	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
// 	{
// 		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
// 	}
// }

uint8_t UART1_RxBuffer[UART1_RX_BUFFER_SIZE] ={ 0 };
uint8_t UART1_RxLen = 0;

uint8_t UART2_RxBuffer[UART2_RX_BUFFER_SIZE] ={ 0 };
uint8_t UART2_RxArray[UART2_RX_BUFFER_SIZE] ={ 0 };
uint8_t UART2_TxBuffer[UART2_TX_BUFFER_SIZE] ={ 0 };
uint8_t UART2_RxBufferCnt = 0;
uint8_t UART2_RxComplete = 0;
uint8_t UART2_RxIDLEFlag = 0;		// 闲时中断标志位
uint8_t UART2_RxBufferOverflow = 0; // 缓冲数组溢出标志
uint8_t UART2_TxDMAOK = 0;
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		DMA_Cmd(DMA1_Channel5, DISABLE); //关闭本次DMA
		USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);

		UNUSED(UART1_RxBuffer);
		UART1_RxLen = UART1_RX_BUFFER_SIZE- DMA_GetCurrDataCounter(DMA1_Channel5); //得到真正接收数据个数
		Handle_UARTCallback();
		memset(UART1_RxBuffer, 0, sizeof(uint8_t) * UART1_RX_BUFFER_SIZE);


		DMA1_Channel5->CNTR = UART1_RX_BUFFER_SIZE;
		DMA_Cmd(DMA1_Channel5, ENABLE); //开启下一次DMA

		uint16_t tmp;
		UNUSED(tmp); // 避免GCC编译器警告
		tmp = USART1->STATR;
		tmp = USART1->DATAR;			 // 根据应用手册，必须要有这两步，否则清标志位的操作其实并不生效
		USART_ClearITPendingBit(USART1, USART_IT_IDLE);
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

	}
}

void USART2_IRQHandler(void)
{
//>>>中断方式接收数据<<<
#ifdef CMD_RX_USE_IT
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		memset(UART1_RxBuffer, 0, sizeof(uint8_t) * UART1_RX_BUFFER_SIZE);
		UART2_RxBuffer[UART2_RxBufferCnt++] = USART_ReceiveData(USART2);

		if (UART2_RxBuffer[UART2_RxBufferCnt - 1] == 0x0D) // XCOM 的发送新行只有0X0D
		{
			UART2_RxBuffer[UART2_RxBufferCnt - 1] = '\0';
			UART2_RxComplete = 1;
			UART2_RxBufferCnt = 0;
			CMD_UARTCallback();
		}
		if (UART2_RxBufferCnt == UART2_RX_BUFFER_SIZE - 1)
		{
			UART2_RxBufferCnt = 0;
			UART2_RxBuffer[UART2_RxBufferCnt] = '\0';
			UART2_RxBufferOverflow = 1;
			memset(UART2_RxBuffer, 0, sizeof(uint8_t) * UART2_RX_BUFFER_SIZE);
		}
	}
#endif

#ifdef CMD_RX_USE_DMA
	// >>>DMA方式接收数据<<<
	if (USART_GetFlagStatus(USART2, USART_FLAG_IDLE) != RESET)
	{
		USART_ClearFlag(USART2, USART_FLAG_IDLE);
		UART2_RxIDLEFlag = 1;
		uint16_t tmp;
		UNUSED(tmp); // 避免GCC编译器警告
		tmp = USART2->STATR;
		tmp = USART2->DATAR;// 根据应用手册，必须要有这两步，否则清标志位的操作其实并不生效
		DMA_Cmd(DMA1_Channel7, DISABLE);//关闭本次DMA

		uint8_t num = UART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel7);//得到真正接收数据个数
		DMA1_Channel7->CNTR = UART2_RX_BUFFER_SIZE;
		UART2_RxBuffer[num] = ' ';// 末尾加空格，否则无法正常解析最后一个参数
		UART2_RxBuffer[num + 1] = '\0';

		CMD_UARTCallback();

		DMA_Cmd(DMA1_Channel7, ENABLE);//开启下一次DMA
	}
#endif

	// >>>DMA发送<<<
//	if (USART_GetITStatus(USART2, USART_IT_TC) != RESET) // 全部数据发送完成，产生该标记
//	{
//		USART_ClearITPendingBit(USART2, USART_IT_TC); // 清除完成标记
//		DMA_Cmd(DMA1_Channel6, DISABLE);			  // 关闭DMA
//		memset(UART2_TxBuffer, 0, sizeof(uint8) * UART2_TX_BUFFER_SIZE);
//		DMA1_Channel6->CNTR = 0;					  // 清除数据长度
//	}
}

uint8_t UART3_RxBuffer[UART3_RX_BUFFER_SIZE] =
{ 0 };
uint8_t UART3_RxBufferCnt = 0;
uint8_t UART3_RxOK = 0;
uint8_t UART3_RxLen = 0;
uint8_t UART3_StartTrans = 0; // 从机开始发送数据
void USART3_IRQHandler(void)
{
	// >>>DMA方式接收数据<<<
	if (USART_GetFlagStatus(USART3, USART_FLAG_IDLE) != RESET)
	{
		USART_ClearFlag(USART3, USART_FLAG_IDLE);
		uint16_t tmp;
		UNUSED(tmp); // 避免GCC编译器警告
		tmp = USART3->STATR;
		tmp = USART3->DATAR;					//根据应用手册，必须要有这两步，否则清标志位的操作其实并不生效
		DMA_Cmd(DMA1_Channel3, DISABLE);							//关闭本次DMA
		UART3_RxLen = UART3_RX_BUFFER_SIZE
				- DMA_GetCurrDataCounter(DMA1_Channel3); //得到真正接收数据个数
		DMA1_Channel3->CNTR = UART3_RX_BUFFER_SIZE;

		SlaveComm_UARTCallback(); // 中断回调函数

		DMA_Cmd(DMA1_Channel3, ENABLE); //开启下一次DMA
	}
}

// void DMA1_Channel3_IRQHandler(void)
// {
// 	if (SET == DMA_GetITStatus(DMA1_FLAG_TC3))
// 	{
// 		 DMA_ClearFlag(DMA1_FLAG_TC3);
// 	}
// }

// void DMA1_Channel5_IRQHandler(void)
// {
// 	if (SET == DMA_GetITStatus(DMA1_FLAG_TC5))
// 	{
// 		// DMA_ClearFlag(DMA1_FLAG_TC5);
// 	}
// }

void DMA1_Channel6_IRQHandler(void)
{
	if (SET == DMA_GetITStatus(DMA1_FLAG_TC6))
	{
		DMA_ClearFlag(DMA1_FLAG_TC6);
		DMA_ClearFlag(DMA1_FLAG_GL6);

		DMA_Cmd(DMA1_Channel6, DISABLE);			  // 关闭DMA
		memset(UART2_TxBuffer, 0, sizeof(uint8) * UART2_TX_BUFFER_SIZE);
	}
}

// void DMA1_Channel7_IRQHandler(void)
// {
// 	if (SET == DMA_GetITStatus(DMA1_FLAG_TC7)) // 传输完成标志位
// 	{
// 		DMA_ClearFlag(DMA1_FLAG_TC7);
// 	}
// }

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
