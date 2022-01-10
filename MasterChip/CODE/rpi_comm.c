/**
 * @file raspi_comm.c
 * @author Simon Kenneth (smkk715@163.com)
 * @brief 树莓派通信相关
 * @version 0.1
 * @date 2022-01-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "rpi_comm.h"
#include "mecanum_chassis.h"
#include "isr.h"
#include "cmd.h"
#include "encoder.h"
#include "sci_compute.h"

RPiMsg_s RPiMsg;
SmartcarMsg_s SmartcarMsg;
uint8 RPiComm_UARTRxOK = 0;
uint8_t RPiComm_UARTRxData[UART1_RX_BUFFER_SIZE] = {0};

void RPiComm_Init(void)
{
    RPiMsg.cam_servo_pwm = 0;
    RPiMsg.ctrl_mode = CTRL_MODE_NONE;
    RPiMsg.speed = 0;
    RPiMsg.dir = 1.57;
    RPiMsg.omega = 0;

    SmartcarMsg.vx = 0;
    SmartcarMsg.vy = 0;
    SmartcarMsg.vz = 0;
    SmartcarMsg.yaw = 0;

    uart_init(UART_1, 115200, UART1_TX_A9, UART1_RX_A10);
    nvic_init(USART1_IRQn, 1, 2, ENABLE);          // 配置UART NVIC
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); // 开启闲时中断
    UART_DMA_ReceiveInit(USART1, DMA1_Channel5, (u32)(&USART1->DATAR),
                         (u32)UART1_RxBuffer, UART1_RX_BUFFER_SIZE); // USART1 DMA初始化
    nvic_init(DMA1_Channel5_IRQn, 1, 3, ENABLE);                     // 配置DMA NVIC
}

/**
 * @brief 向树莓派发送数据
 * 
 */
void RPiComm_Exe(void)
{

    uart_putchar(UART_1, 0x00);
    uart_putchar(UART_1, 0xff);

    // IMU数据
    uint8 transfer_buffer[4];
    float2buffer(icm_acc_x, transfer_buffer);
    uart_putbuff(UART_1, transfer_buffer, 4);
    float2buffer(icm_acc_y, transfer_buffer);
    uart_putbuff(UART_1, transfer_buffer, 4);
    float2buffer(icm_acc_z, transfer_buffer);
    uart_putbuff(UART_1, transfer_buffer, 4);
    float2buffer(MecanumChassis.PostureStatus.yaw, transfer_buffer);
    uart_putbuff(UART_1, transfer_buffer, 4);
    // 编码器数据
//    uart_putchar(UART_1, (encoder_data[0] >> 8) & 0x00ff);
//    uart_putchar(UART_1, encoder_data[0] & 0x00ff);
//    uart_putchar(UART_1, (encoder_data[1] >> 8) & 0x00ff);
//    uart_putchar(UART_1, encoder_data[1] & 0x00ff);
//    uart_putchar(UART_1, (encoder_data[2] >> 8) & 0x00ff);
//    uart_putchar(UART_1, encoder_data[2] & 0x00ff);
//    uart_putchar(UART_1, (encoder_data[3] >> 8) & 0x00ff);
//    uart_putchar(UART_1, encoder_data[3] & 0x00ff);
    // 车轮转速数据
    for(int i=0;i<4;i++)
    {
        float2buffer(MecanumChassis.motor[i].now_rpm, transfer_buffer);
        uart_putbuff(UART_1, transfer_buffer, 4);
    }

    uart_putchar(UART_1, 0xee);
    uart_putchar(UART_1, 0x11);
}

/**
 * @brief 读取缓冲数组，处理串口接收数据
 * 
 */
void RPiComm_UARTCallback(void)
{
    if(UART1_RxBuffer[0]==0x00 && UART1_RxBuffer[1]==0xff)
    {
//        uint8 ctrl_mode = UART1_RxBuffer[2];

        uint8 sub_buffer[4];
        substring(sub_buffer, UART1_RxBuffer, 3, 4);
        MecanumChassis.target_speed = buffer2float(sub_buffer);
        substring(sub_buffer, UART1_RxBuffer, 7, 4);
        MecanumChassis.target_dir = buffer2float(sub_buffer);
        substring(sub_buffer, UART1_RxBuffer, 11, 4);
        MecanumChassis.target_omega = buffer2float(sub_buffer);

//        int16 cam_pwm =  UART1_RxBuffer[15]<<8 | UART1_RxBuffer[16];


    }
}
