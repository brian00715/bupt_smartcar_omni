/**
 * @file valuepack.c
 * @author 
 * @brief 虚拟示波器，配合手机app“蓝牙调试器”使用
 * @version 0.1
 * @date 
 * @copyright Copyright (c) 2021
 * 
 */
#include "valuepack.h"
#include "zf_uart.h"

unsigned char bits[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

// const unsigned int VALUEPACK_INDEX_RANGE=VALUEPACK_BUFFER_SIZE<<3;
// const unsigned short  TXPACK_BYTE_SIZE = ((TX_BOOL_NUM+7)>>3)+TX_BYTE_NUM+(TX_SHORT_NUM<<1)+(TX_INT_NUM<<2)+(TX_FLOAT_NUM<<2);
// const unsigned short  RXPACK_BYTE_SIZE = ((RX_BOOL_NUM+7)>>3)+RX_BYTE_NUM+(RX_SHORT_NUM<<1)+(RX_INT_NUM<<2)+(RX_FLOAT_NUM<<2);
// unsigned short rx_pack_length = RXPACK_BYTE_SIZE+3;

long rxIndex = 0;
long rdIndex = 0;

// unsigned char *vp_rxbuff = (unsigned char*)malloc(sizeof(unsigned char)*VALUEPACK_BUFFER_SIZE);
// unsigned char *vp_txbuff = (unsigned char*)malloc(sizeof(unsigned char)*(TXPACK_BYTE_SIZE+3));

unsigned char vp_rxbuff[VALUEPACK_BUFFER_SIZE];
unsigned char vp_txbuff[TXPACK_BYTE_SIZE + 3];



struct TxPack txpack;
struct RxPack rxpack;

unsigned short this_index = 0;
unsigned short last_index = 0;
unsigned short rdi, rdii, idl, idi;
uint32_t idc;
unsigned int err = 0;
unsigned char sum = 0;
unsigned char isok;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//procValuePack 用来处理手机传来的数据
//

// unsigned char readValuePack(struct RxPack *rx_pack_ptr)
// {
// 	isok = 0;
// 	this_index = VALUEPACK_BUFFER_SIZE - hdma_usart2_rx.Instance->NDTR;
// 	if (this_index < last_index)
// 		rxIndex += VALUEPACK_BUFFER_SIZE + this_index - last_index;
// 	else
// 		rxIndex += this_index - last_index;
// 	while (rdIndex < (rxIndex - ((rx_pack_length))))
// 		rdIndex += rx_pack_length;
// 	while (rdIndex <= (rxIndex - rx_pack_length))
// 	{

// 		rdi = rdIndex % VALUEPACK_BUFFER_SIZE;
// 		rdii = rdi + 1;
// 		if (vp_rxbuff[rdi] == PACK_HEAD)
// 		{
// 			if (vp_rxbuff[(rdi + RXPACK_BYTE_SIZE + 2) % VALUEPACK_BUFFER_SIZE] == PACK_TAIL)
// 			{
// 				//  计算校验和
// 				sum = 0;
// 				for (short s = 0; s < RXPACK_BYTE_SIZE; s++)
// 				{
// 					rdi++;
// 					if (rdi >= VALUEPACK_BUFFER_SIZE)
// 						rdi -= VALUEPACK_BUFFER_SIZE;
// 					sum += vp_rxbuff[rdi];
// 				}
// 				rdi++;
// 				if (rdi >= VALUEPACK_BUFFER_SIZE)
// 					rdi -= VALUEPACK_BUFFER_SIZE;

// 				if (sum == vp_rxbuff[rdi])
// 				{
// //  提取数据包数据 一共有五步， bool byte short int float

// // 1. bool
// #if RX_BOOL_NUM > 0

// 					idc = (uint32_t)rx_pack_ptr->bools;
// 					idl = (RX_BOOL_NUM + 7) >> 3;
// 					for (idi = 0; idi < idl; idi++)
// 					{
// 						if (rdii >= VALUEPACK_BUFFER_SIZE)
// 							rdii -= VALUEPACK_BUFFER_SIZE;
// 						(*((unsigned char *)idc)) = vp_rxbuff[rdii];
// 						rdii++;
// 						idc++;
// 					}
// #endif
// // 2.byte
// #if RX_BYTE_NUM > 0
// 					idc = (uint32_t)(rx_pack_ptr->bytes);
// 					idl = RX_BYTE_NUM;
// 					for (idi = 0; idi < idl; idi++)
// 					{
// 						if (rdii >= VALUEPACK_BUFFER_SIZE)
// 							rdii -= VALUEPACK_BUFFER_SIZE;
// 						(*((unsigned char *)idc)) = vp_rxbuff[rdii];
// 						rdii++;
// 						idc++;
// 					}
// #endif
// // 3.short
// #if RX_SHORT_NUM > 0
// 					idc = (uint32_t)(rx_pack_ptr->shorts);
// 					idl = RX_SHORT_NUM << 1;
// 					for (idi = 0; idi < idl; idi++)
// 					{
// 						if (rdii >= VALUEPACK_BUFFER_SIZE)
// 							rdii -= VALUEPACK_BUFFER_SIZE;
// 						(*((unsigned char *)idc)) = vp_rxbuff[rdii];
// 						rdii++;
// 						idc++;
// 					}
// #endif
// // 4.int
// #if RX_INT_NUM > 0
// 					idc = (uint32_t)(&(rx_pack_ptr->integers[0]));
// 					idl = RX_INT_NUM << 2;
// 					for (idi = 0; idi < idl; idi++)
// 					{
// 						if (rdii >= VALUEPACK_BUFFER_SIZE)
// 							rdii -= VALUEPACK_BUFFER_SIZE;
// 						(*((unsigned char *)idc)) = vp_rxbuff[rdii];
// 						rdii++;
// 						idc++;
// 					}
// #endif
// // 5.float
// #if RX_FLOAT_NUM > 0
// 					idc = (uint32_t)(&(rx_pack_ptr->floats[0]));
// 					idl = RX_FLOAT_NUM << 2;
// 					for (idi = 0; idi < idl; idi++)
// 					{
// 						if (rdii >= VALUEPACK_BUFFER_SIZE)
// 							rdii -= VALUEPACK_BUFFER_SIZE;
// 						(*((unsigned char *)idc)) = vp_rxbuff[rdii];
// 						rdii++;
// 						idc++;
// 					}
// #endif
// 					err = rdii;
// 					rdIndex += rx_pack_length;
// 					isok = 1;
// 				}
// 				else
// 				{
// 					rdIndex++;
// 					err++;
// 				}
// 			}
// 			else
// 			{
// 				rdIndex++;
// 				err++;
// 			}
// 		}
// 		else
// 		{
// 			rdIndex++;
// 			err++;
// 		}
// 	}
// 	last_index = this_index;
// 	return isok;
// }

void sendBuffer(unsigned char *p, unsigned short length)
{
	dma_send(p, (unsigned int)length);
}

unsigned short loop;
unsigned char valuepack_tx_bit_index;
unsigned char valuepack_tx_index;

void sendValuePack(struct TxPack *tx_pack_ptr)
{
	int i;
	vp_txbuff[0] = 0xa5;
	sum = 0;
	//  由于结构体中不同类型的变量在内存空间的排布不是严格对齐的，中间嵌有无效字节，因此需要特殊处理

	valuepack_tx_bit_index = 0;
	valuepack_tx_index = 1;

#if TX_BOOL_NUM > 0

	for (loop = 0; loop < TX_BOOL_NUM; loop++)
	{
		if (tx_pack_ptr->bools[loop])
			vp_txbuff[valuepack_tx_index] |= 0x01 << valuepack_tx_bit_index;
		else
			vp_txbuff[valuepack_tx_index] &= ~(0x01 << valuepack_tx_bit_index);

		valuepack_tx_bit_index++;
		if (valuepack_tx_bit_index >= 8)
		{
			valuepack_tx_bit_index = 0;
			valuepack_tx_index++;
		}
	}
	if (valuepack_tx_bit_index != 0)
		valuepack_tx_index++;
#endif

#if TX_BYTE_NUM > 0

	for (loop = 0; loop < TX_BYTE_NUM; loop++)
	{
		vp_txbuff[valuepack_tx_index] = tx_pack_ptr->bytes[loop];
		valuepack_tx_index++;
	}

#endif

#if TX_SHORT_NUM > 0
	for (loop = 0; loop < TX_SHORT_NUM; loop++)
	{
		vp_txbuff[valuepack_tx_index] = tx_pack_ptr->shorts[loop] & 0xff;
		vp_txbuff[valuepack_tx_index + 1] = tx_pack_ptr->shorts[loop] >> 8;
		valuepack_tx_index += 2;
	}
#endif

#if TX_INT_NUM > 0
	for (loop = 0; loop < TX_INT_NUM; loop++)
	{
		i = tx_pack_ptr->integers[loop];

		vp_txbuff[valuepack_tx_index] = i & 0xff;
		vp_txbuff[valuepack_tx_index + 1] = (i >> 8) & 0xff;
		vp_txbuff[valuepack_tx_index + 2] = (i >> 16) & 0xff;
		vp_txbuff[valuepack_tx_index + 3] = (i >> 24) & 0xff;

		valuepack_tx_index += 4;
	}
#endif

#if TX_FLOAT_NUM > 0
	for (loop = 0; loop < TX_FLOAT_NUM; loop++)
	{
		i = *(int *)(&(tx_pack_ptr->floats[loop]));

		vp_txbuff[valuepack_tx_index] = i & 0xff;
		vp_txbuff[valuepack_tx_index + 1] = (i >> 8) & 0xff;
		vp_txbuff[valuepack_tx_index + 2] = (i >> 16) & 0xff;
		vp_txbuff[valuepack_tx_index + 3] = (i >> 24) & 0xff;

		valuepack_tx_index += 4;
	}
#endif

	for (unsigned short d = 1; d <= TXPACK_BYTE_SIZE; d++)
		sum += vp_txbuff[d];
	vp_txbuff[TXPACK_BYTE_SIZE + 1] = sum;
	vp_txbuff[TXPACK_BYTE_SIZE + 2] = 0x5a;
	sendBuffer(vp_txbuff, TXPACK_BYTE_SIZE + 3);
}


void dma_send(unsigned char *buffer, unsigned int length)
{
	uart_putbuff(UART_2,buffer,length);
}

