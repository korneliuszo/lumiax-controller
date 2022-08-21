#include "main_modules.hpp"
#include <zephyr/device.h>

K_KERNEL_STACK_DEFINE(tlay2_stack,300);


void process_packet(Tlay2<128>* obj, uint8_t*data,size_t len);

Tlay2<128> tlay2 = {DEVICE_DT_GET(DT_NODELABEL(uart0)),tlay2_stack,K_THREAD_STACK_SIZEOF(tlay2_stack),process_packet};

void process_packet(Tlay2<128>* obj, uint8_t*data,size_t len)
{
	switch(data[1])
	{
	case 0: //PING
		obj->tx_init_reply();
		for(size_t i=2;i<len;i++)
			obj->tx_byte(data[i]);
		obj->tx_end();
		break;
	default:
		break;
	}
}
