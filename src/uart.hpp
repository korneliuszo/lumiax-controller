#ifndef TLAY2_UART_H
#define TLAY2_UART_H

#include <stdint.h>

#include "fifo.hpp"
#include <zephyr/drivers/uart.h>
#include <zephyr/zephyr.h>

template<int RX_SIZE,int TX_SIZE>
class UART
{
private:
	const struct device *dev;
	typedef bool (*rx_callback_t)(void* obj, uint8_t *byte);

	rx_callback_t rx_callback;
	void* rx_callback_obj;
	FIFO<RX_SIZE> fifo_rx;
	FIFO<TX_SIZE> fifo_tx;
private:
	struct k_sem tx_sem_wait;
	static void isr_c(const struct device *dev,
						      void *user_data)
	{
		UART<RX_SIZE,TX_SIZE> *obj=static_cast<UART<RX_SIZE,TX_SIZE>*>(user_data);
		obj->isr();
	}
	void isr()
	{
		if (!uart_irq_update(dev) || !uart_irq_is_pending(dev))
			return;

		if(uart_irq_rx_ready(dev))
		{
			uint8_t recvData;
			uart_fifo_read(dev, &recvData, 1);
			if(rx_callback)
			{
				if(rx_callback(rx_callback_obj,&recvData))
					fifo_rx.put(recvData);
			}
			else
			{
				fifo_rx.put(recvData);
			}
		}
		if(uart_irq_tx_ready(dev))
		{
			uint8_t val;
			intptr_t cnt = fifo_tx.get(&val);
			if(cnt)
			{
				uart_fifo_fill(dev, &val, 1);
				k_sem_give(&tx_sem_wait);
			}
			else
			{
				uart_irq_tx_disable(dev);
			}
		}
	}
public:
	UART(
			const struct device *_dev,
			rx_callback_t _rx_callback=NULL,
			void* _rx_callback_obj=NULL)
	:dev(_dev),
	 rx_callback(_rx_callback),
	 rx_callback_obj(_rx_callback_obj)
	{
		k_sem_init(&tx_sem_wait, 0, 1);
	};
	bool Init()
	{
		if (!device_is_ready(dev)) {
			printk("Device %s not ready\n", dev->name);
			return false;
		}
		uart_irq_callback_user_data_set(dev,isr_c,this);
		uart_irq_rx_enable(dev);
		return true;
	}
	intptr_t get(uint8_t *byte)
	{
		return fifo_rx.get(byte);
	}
	intptr_t send(uint8_t byte)
	{
		intptr_t ret;
		do {
			ret = fifo_tx.put(byte);
			uart_irq_tx_enable(dev);
			if(!ret)
			{
				k_sem_take(&tx_sem_wait,K_FOREVER);
			}
		} while(!ret);
		return ret;
	}
};

#endif
