#include "key.h"
/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2024-07-019

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2024-07-019

All rights reserved
***********************************************/
uint8_t click(void)
{
	uint8_t key_num=0;
	if(DL_GPIO_readPins(KEY_PORT,KEY_PIN_21_PIN)==0)
	{
		delay_ms(50);
		while(DL_GPIO_readPins(KEY_PORT,KEY_PIN_21_PIN)==0);
		delay_ms(50);
		key_num=1;
	}
	return key_num;
}



