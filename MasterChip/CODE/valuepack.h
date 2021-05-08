#ifndef VALUEPACK_H
#define VALUEPACK_H


// ������ͨ��DMA��USART �������ݰ��Ľ��պͷ���
// ���յ������Զ�д�뵽buffer�У�ͨ����ʱ����readValuePack()��������������ʱ���������10ms���ڡ�
// ���ݷ���Ҳ����DMA

/// 1.ָ�����ջ������Ĵ�С ----------------------------------------------------------------------------------
//    һ����Ҫ512�ֽ����ϣ���Ҫ����ʵ�ʽ������ݵ��ٶȺ�proc������Ƶ�ʿ��ǡ�
#define VALUEPACK_BUFFER_SIZE 32

/// 2.ָ�����͵��ֻ������ݰ��Ľṹ--------------------------�ڷ���ʱ���Զ�������ǰ����ϰ�ͷ����β��У������ݣ���˻���3���ֽ�
///
//    ����ʵ����Ҫ�ı������������ݰ��� bool byte short int float �������͵���Ŀ

#define TX_BOOL_NUM 0
#define TX_BYTE_NUM 0
#define TX_SHORT_NUM 0
#define TX_INT_NUM 0
#define TX_FLOAT_NUM 2

/// 3.ָ���������ݰ��Ľṹ-----------------------------------------------------------------------------------
//    ����ʵ����Ҫ�ı������������ݰ��� bool byte short int float �������͵���Ŀ

#define RX_BOOL_NUM 4
#define RX_BYTE_NUM 0
#define RX_SHORT_NUM 0
#define RX_INT_NUM 4
#define RX_FLOAT_NUM 0

#define VALUEPACK_INDEX_RANGE VALUEPACK_BUFFER_SIZE << 3
#define TXPACK_BYTE_SIZE ((TX_BOOL_NUM + 7) >> 3) + TX_BYTE_NUM + (TX_SHORT_NUM << 1) + (TX_INT_NUM << 2) + (TX_FLOAT_NUM << 2)
#define RXPACK_BYTE_SIZE ((RX_BOOL_NUM + 7) >> 3) + RX_BYTE_NUM + (RX_SHORT_NUM << 1) + (RX_INT_NUM << 2) + (RX_FLOAT_NUM << 2)
#define rx_pack_length RXPACK_BYTE_SIZE + 3

extern unsigned char vp_rxbuff[VALUEPACK_BUFFER_SIZE];
extern unsigned char vp_txbuff[TXPACK_BYTE_SIZE + 3];

struct TxPack
{
#if TX_BOOL_NUM > 0
	unsigned char bools[TX_BOOL_NUM];
#endif

#if TX_BYTE_NUM > 0
	char bytes[TX_BYTE_NUM];
#endif

#if TX_SHORT_NUM > 0
	short shorts[TX_SHORT_NUM];
#endif

#if TX_INT_NUM > 0
	int integers[TX_INT_NUM];
#endif

#if TX_FLOAT_NUM > 0
	float floats[TX_FLOAT_NUM];
#endif

	char space; // �����壬ֻΪ�˲��ýṹ��Ϊ�գ��ṹ��Ϊ�ջᱨ��
};

struct RxPack
{
#if RX_BOOL_NUM > 0
	unsigned char bools[RX_BOOL_NUM];
#endif

#if RX_BYTE_NUM > 0
	char bytes[RX_BYTE_NUM];
#endif

#if RX_SHORT_NUM > 0
	short shorts[RX_SHORT_NUM];
#endif

#if RX_INT_NUM > 0
	int integers[RX_INT_NUM];
#endif

#if RX_FLOAT_NUM > 0
	float floats[RX_FLOAT_NUM];
#endif
	char space; // �����壬ֻΪ�˲��ýṹ��Ϊ�գ��ṹ��Ϊ�ջᱨ��
};
// ��ʼ�� valuepack ����һЩ��Ҫ��Ӳ����������

void initValuePack();

// ��Ҫ��֤����ÿ��ִ��10�θú���
// �ú�������Ҫ�������Ƚ������յĻ�������������յ�������RX���ݰ��������RX���ݰ��е����ݣ�Ȼ��ʼ���ڷ���TX���ݰ� ��
// ���յ����ݰ�ʱ ���� 1 �����򷵻� 0
unsigned char readValuePack(struct RxPack *rx_pack_ptr);
void sendBuffer(unsigned char *p, unsigned short length);

// �������ݰ�
void sendValuePack(struct TxPack *tx_pack_ptr);
void dma_send(unsigned char *buffer, unsigned int length);
void bluetooth_handle_exe();

extern struct TxPack txpack;
extern struct RxPack rxpack;

#define PACK_HEAD 0xa5
#define PACK_TAIL 0x5a

#endif // VALUEPACK_H
