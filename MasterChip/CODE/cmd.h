#ifndef CMD_H_
#define CMD_H_

#include "isr.h"

#define CMD_RX_USE_IT // ʹ���жϷ�ʽ����
// #define CMD_RX_USE_DMA // ʹ��DMA��ʽ����
#define CMD_TX_USE_BLOCK // ������ʽ����
//#define CMD_TX_USE_DMA // DMA��ʽ����

#define CMD_SIZE_X 10  // һ��ָ�����10������
#define CMD_SIZE_Y 100 // ÿ���������100���ַ�

extern uint8_t *CMD_Buffer[CMD_SIZE_X]; // ָ�����飬ÿ��Ԫ�ض�ָ��ָ���Ԫ�ַ���
extern uint8_t CMD_BufferCnt;
extern uint8_t UART2_RxBuffer[UART2_RX_BUFFER_SIZE];
extern uint8_t UART2_RxComplete;
extern uint8_t UART2_RxIDLEFlag;
extern int wave_index;
extern char CMD_RxOK;

extern float CMD_TargetYaw;

void CMD_Init(void);
void CMD_Exe(void);
void UART_DMA_ReceiveInit(USART_TypeDef *usart, DMA_Channel_TypeDef *dma_ch, uint32 src_addr,
						  uint32 des_addr, uint32 size);
void UART_DMA_SendInit(USART_TypeDef *usart, DMA_Channel_TypeDef *dma_ch, uint32 src_addr,
					   uint32 des_addr);
void UART_DMA_EnableSendData(DMA_Channel_TypeDef *dman, uint8 *data, uint8 len);
void CMD_UARTCallback(void);
int CMD_CommandParse(char *cmd_line, uint8_t *argc, char *argv[]);
int CMD_CommandExe(int argc, char **argv);
void ToolBox_Scope(float *dataArray, int dataNum);


#endif
