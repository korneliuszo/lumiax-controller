#ifndef TLAY2_TLAY2_H
#define TLAY2_TLAY2_H

#include <stdint.h>
#include <zephyr/zephyr.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/cbprintf.h>

#include "uart.hpp"


template<int MAX_PKT_SIZE>
class Tlay2
{
private:
	struct k_mutex tx_mutex;
	struct k_thread rx_thread;
	struct k_sem rx_sem_wait;
	struct k_sem tx_sem_wait;
	k_thread_stack_t *rx_stack;
	size_t rx_stack_size;
	typedef void (*process_packet_t)(Tlay2<MAX_PKT_SIZE>* obj, uint8_t*data,size_t len);
	UART<MAX_PKT_SIZE+10,MAX_PKT_SIZE+10> uart;
	process_packet_t process_packet;

	uint8_t tlay2_rx_buf[MAX_PKT_SIZE];
	uint8_t tlay2_rx_len;
	uint8_t tlay2_tx_crc;

	static bool rx_isr_c(void* user_data, uint8_t *byte)
	{
		Tlay2<MAX_PKT_SIZE> *obj=static_cast<Tlay2<MAX_PKT_SIZE>*>(user_data);
		return obj->rx_isr(byte);
	}
	bool rx_isr(uint8_t* byte)
	{
		if(*byte == '\n')
			k_sem_give(&rx_sem_wait);
		return true;
	}
	static void rx_thread_c(void* user_data, void* arg2, void* arg3)
	{
		Tlay2<MAX_PKT_SIZE> *obj=static_cast<Tlay2<MAX_PKT_SIZE>*>(user_data);
		obj->rx_thread_fn();
	}
	void rx_thread_fn()
	{
		while(true)
		{
			k_sem_take(&rx_sem_wait,K_FOREVER);
			retry:
			tlay2_rx_len=0;
			uint8_t c;
			do {
				uart.get(&c);
				if (c == '\n')
					break;
				if (c == 0xdc)
				{
					uart.get(&c);
					c ^= 0x80;
				}
				tlay2_rx_buf[tlay2_rx_len++]=c;
			}while(tlay2_rx_len < MAX_PKT_SIZE);
			tlay2_rx_len-=1;
			if(c != '\n')
				goto retry;
			if (tlay2_rx_len < 2)
				continue;
			if (crc8_ccitt(0x00,tlay2_rx_buf,tlay2_rx_len+1) != 0)
				continue;
			if(process_packet)
				process_packet(this,tlay2_rx_buf,tlay2_rx_len);
		}
	}
public:
	Tlay2(
			const struct device *_dev,
			k_thread_stack_t *_rx_stack,
			size_t _rx_stack_size,
			process_packet_t _process_packet=NULL)

	:rx_stack(_rx_stack),
	 rx_stack_size(_rx_stack_size),
	 uart(_dev,rx_isr_c,this),
	 process_packet(_process_packet)
	{
		k_mutex_init(&tx_mutex);
		k_sem_init(&rx_sem_wait, 0, MAX_PKT_SIZE);
		k_sem_init(&tx_sem_wait, 1, 1);
	}
	bool Init()
	{
		bool ret;

		ret = uart.Init();
		if(!ret)
			return ret;
		k_thread_create(&rx_thread,rx_stack, rx_stack_size,rx_thread_c,this,NULL,NULL,5,0,K_NO_WAIT);
		return true;
	}

	void tx_init_reply()
	{
		k_sem_take(&tx_sem_wait,K_FOREVER);
		tlay2_tx_crc=0x00;
		tx_byte(tlay2_rx_buf[0]); //remote channel
		tx_byte(tlay2_rx_buf[1]); //local function
	}
	void tx_init(uint8_t channel, uint8_t function)
	{
		k_sem_take(&tx_sem_wait,K_FOREVER);
		tlay2_tx_crc=0x00;
		tx_byte(channel); // remote channel
		tx_byte(function); // local function
	}
	void tx_end()
	{
		tx_byte(tlay2_tx_crc);
		uart.send('\n');
		k_sem_give(&tx_sem_wait);
	}
	void tx_byte(uint8_t byte)
	{
		tlay2_tx_crc=crc8_ccitt(tlay2_tx_crc,&byte,1);
		if (byte == '\n' || byte == 0xdc)
		{
			uart.send(0xdc);
			byte ^=0x80;
		}
		uart.send(byte);
	}
	void tx_u16(uint16_t byte)
	{
		tx_byte(byte&0xff);
		tx_byte(byte >> 8);
	}

	void tx_u32(uint32_t byte)
	{
		tx_byte(byte&0xff);
		tx_byte((byte >> 8)&0xff);
		tx_byte((byte >> 16)&0xff);
		tx_byte((byte >> 24)&0xff);
	}
	uint16_t rx_u16(uint8_t * buff)
	{
		return ((uint16_t)buff[1] << 8) | buff[0];
	}

	uint32_t rx_u32(uint8_t * buff)
	{
		return ((uint32_t)buff[3] << 24) | ((uint32_t)buff[2] << 16) | ((uint32_t)buff[1] << 8) | buff[0];

	}
	static int cwrap_tx_byte(int c, void *ctx)
	{
		if(ctx == NULL)
			return -1;
		uint8_t cr=c;
		static_cast<Tlay2<MAX_PKT_SIZE>*>(ctx)->tx_byte(cr);
		return cr;
	}

	int printvt(const char *format, va_list ap)
	{
		int rc;

		tx_init(0,0);
		rc = cbvprintf((cbprintf_cb)cwrap_tx_byte, this, format, ap);
		tx_end();
		//while(uart.fifo_tx.check());
		return rc;
	}

	int printt( const char *format, ...)
	{
		va_list ap;
		int rc;

		va_start(ap, format);
		rc = printvt(format,ap);
		va_end(ap);

		return rc;
	}
};

#endif
