#include "button.h"


//开关状态变量
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;

//上一次开关状态变量
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;

//开关标志位
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;

//开关信号量
//rt_sem_t key1_sem;
//rt_sem_t key2_sem;
//rt_sem_t key3_sem;

void button_entry(void *parameter)
{
	//保存按键状态
	key1_last_status = key1_status;
	key2_last_status = key2_status;
	key3_last_status = key3_status;

	//读取当前按键状态
	key1_status = gpio_get(KEY1);
	key2_status = gpio_get(KEY2);
	key3_status = gpio_get(KEY3);

	//检测到按键按下之后并放开 释放一次信号量
	if (key1_status && !key1_last_status)
	{
		//        rt_sem_release(key1_sem);
		//        rt_mb_send(buzzer_mailbox, 100);
	}
	if (key2_status && !key2_last_status)
	{
		//        rt_sem_release(key2_sem);
		//        rt_mb_send(buzzer_mailbox, 300);
	}
	if (key3_status && !key3_last_status)
	{
		//        rt_sem_release(key3_sem);
		//        rt_mb_send(buzzer_mailbox, 600);
	}
}

void button_init(void)
{
	//    rt_timer_t timer1;

	gpio_init(C8, GPI, 0, GPIO_INT_CONFIG);
	gpio_init(C9, GPI, 0, GPIO_INT_CONFIG);
	gpio_init(B2, GPI, 0, GPIO_INT_CONFIG);
	gpio_init(B15, GPO, 1, GPIO_PIN_CONFIG); // 核心板LED

	//    key1_sem = rt_sem_create("key1", 0, RT_IPC_FLAG_FIFO);  //创建按键的信号量，当按键按下就释放信号量，在需要使用按键的地方获取信号量即可
	//    key2_sem = rt_sem_create("key2", 0, RT_IPC_FLAG_FIFO);
	//    key3_sem = rt_sem_create("key3", 0, RT_IPC_FLAG_FIFO);

	//    timer1 = rt_timer_create("button", button_entry, RT_NULL, 20, RT_TIMER_FLAG_PERIODIC);

	//    if(RT_NULL != timer1)
	//    {
	//        rt_timer_start(timer1);
	//    }
}
