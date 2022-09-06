#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/modbus/modbus.h>
#include <zephyr/drivers/gpio.h>

#include "main_modules.hpp"
#include "printt.h"
#include "stdio.h"

extern "C" void barrot_init();

struct Reg_data reg_data;

Display display;


int printt( const char *format, ...)
{
	va_list ap;

	int rc;

	va_start(ap, format);
	rc = tlay2.printvt(format,ap);
	va_end(ap);
	return rc;
}

const static struct modbus_iface_param client_param = {
	.mode = MODBUS_MODE_RTU,
	.rx_timeout = 750000,
	.serial = {
		.baud = 9600/25,
		.parity = UART_CFG_PARITY_NONE,
		.stop_bits_client = UART_CFG_STOP_BITS_2,
	},
};

#define MODBUS_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_modbus_serial)

int client_iface;

static int init_modbus_client(void)
{
	const char iface_name[] = {DEVICE_DT_NAME(MODBUS_NODE)};

	client_iface = modbus_iface_get_by_name(iface_name);

	return modbus_init_client(client_iface, client_param);
}

int d_print(int c)
{
	printt("%c",c);
	return 1;
}

void bt_enabled(int err)
{
	barrot_init();
}

#define DISPLAY_STACK_SIZE 1000
K_THREAD_STACK_DEFINE(display_stack_area, DISPLAY_STACK_SIZE);
struct k_thread display_thread_data;

void display_thread(void*,void*,void*)
{
	int spinner_idx=0;
	int spinner_data_idx=0;
	char spinner[] = "/-\\|";
	Reg_data::Data cached = {};
	while(true)
	{
		if(!k_sem_take(&reg_data.new_sample,K_NO_WAIT))
		{
			k_mutex_lock(&reg_data.mut, K_FOREVER);
			cached = reg_data.d;
			k_mutex_unlock(&reg_data.mut);
			spinner_data_idx++;
			if (!spinner[spinner_data_idx])
				spinner_data_idx=0;
		}
		{
			//auto l = display.lock();
			display.print_chr(0,0,spinner[spinner_idx++]);
			display.print_chr(8,0,spinner[spinner_data_idx]);
			display.print_str(32,0,cached.on ? " ON":"OFF");
			char buff[13];
			snprintf(buff,13,"Bat %6.2u%%",cached.b_soc);
			display.print_str(0,16,buff);
			snprintf(buff,13,"Bat %+.2d.%.2uV",cached.b_v/100,cached.b_v%100);
			display.print_str(0,32,buff);
			snprintf(buff,13,"Bat %+.2d.%.2uA",cached.b_a/100,cached.b_a%100);
			display.print_str(0,48,buff);
			snprintf(buff,13,"PV  %+.2d.%.2uV",cached.s_v/100,cached.s_v%100);
			display.print_str(0,64,buff);
			snprintf(buff,13,"PV  %+.2d.%.2uA",cached.s_a/100,cached.s_a%100);
			display.print_str(0,80,buff);
			snprintf(buff,13,"Out %+.2d.%.2uV",cached.l_v/100,cached.l_v%100);
			display.print_str(0,96,buff);
			snprintf(buff,13,"Out %+.2d.%.2uA",cached.l_a/100,cached.l_a%100);
			display.print_str(0,112,buff);

			display.update();
		}
		if (!spinner[spinner_idx])
			spinner_idx=0;
	}
}

void pb_expiry(struct k_timer *timer)
{
	atomic_set(&onoff,!atomic_get(&onoff));
}

K_TIMER_DEFINE(pb_timer, pb_expiry, NULL);

struct gpio_callback gpio_cb_str;

atomic_t onoff = ATOMIC_INIT(0);

void gpio_cb(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	printt("GPIO 0x%04x",pins);
	if(pins & (1<<24))
	{
		static uint64_t last_push = 0;
		enum class STATE{
			INIT,
			SECOND,
			THIRD,
		};
		static STATE state = STATE::INIT;
		uint64_t now = k_uptime_get();
		uint64_t diff = now - last_push;
		printt("GPIO pwrdown delta %lld",diff);

		switch(state)
		{
		case STATE::INIT:
		default:
			state = STATE::SECOND;
			break;
		case STATE::SECOND:
			if (diff<110 && diff>90)
				state = STATE::THIRD;
			else
				state = STATE::SECOND;
			break;
		case STATE::THIRD:
			if (diff<110 && diff>90)
			{
				state = STATE::SECOND;
				printt("Poweringoff");
				modbus_write_coil(client_iface, 1, 0x0000, 1);
			}
			else
				state = STATE::SECOND;
		}
		last_push = now;
	}
	if(pins & (1<<25))
	{
		gpio_port_value_t rd;
		gpio_port_get(port,&rd);
		printt("aa %d",rd &(1<<25));
		if(rd &(1<<25))
		{
			k_timer_stop(&pb_timer);
		}
		else
		{
			if(atomic_get(&onoff))
			{
				printt("SHUTDOWN EVENT");
				k_timer_start(&pb_timer,K_SECONDS(5),K_NO_WAIT);
			}
			else
			{
				k_timer_start(&pb_timer,K_SECONDS(1),K_NO_WAIT);
			}
		}
	}
}

int main(void)
{
	k_mutex_init(&reg_data.mut);

	{
		const device * dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
		gpio_pin_configure(dev, 23, GPIO_OUTPUT);
		gpio_pin_set(dev, 23, 1);
		gpio_init_callback(&gpio_cb_str,gpio_cb,(1<<25)|(1<<24));
		gpio_add_callback(dev,&gpio_cb_str);
		gpio_pin_configure(dev, 24, GPIO_INPUT | GPIO_PULL_UP);
		gpio_pin_configure(dev, 25, GPIO_INPUT | GPIO_PULL_UP);
		gpio_pin_interrupt_configure(dev, 24, GPIO_INT_EDGE_BOTH);
		gpio_pin_interrupt_configure(dev, 25, GPIO_INT_EDGE_BOTH);

	}

	if(!tlay2.Init())
		return 1;

	tlay2.printt("Hello World! %s", CONFIG_BOARD);

	int err;

	err = display.Init();
	if(err)
		return err;

	err = bt_enable(bt_enabled);
	if (err) {
		return 1;
	}

	k_sem_init(&reg_data.new_sample, 0, 1);

	k_thread_create(&display_thread_data, display_stack_area,
	                                 K_THREAD_STACK_SIZEOF(display_stack_area),
	                                 display_thread,
	                                 NULL, NULL, NULL,
	                                 10, 0, K_NO_WAIT);
	if (init_modbus_client()) {
		tlay2.printt("Modbus RTU client initialization failed");
		return 1;
	}


	while(1)
	{
		uint16_t holding_reg[11];
		if(modbus_read_input_regs(client_iface, 1, 0x3045, holding_reg,
			       ARRAY_SIZE(holding_reg))!=0)
		{
			printt("Read fail1");
			continue;
		}

		uint16_t onoff_rd;
		if(modbus_read_input_regs(client_iface, 1, 0x3035, &onoff_rd,1)!=0)
		{
			printt("Read fail3");
			continue;
		}

		if((!!(onoff_rd&1)) != (!!atomic_get(&onoff))){
			modbus_write_coil(client_iface, 1, 0x0000, !!atomic_get(&onoff));
		}


		k_mutex_lock(&reg_data.mut, K_FOREVER);
		reg_data.d.b_soc = holding_reg[0];
		reg_data.d.b_v = holding_reg[1];
		reg_data.d.b_a = holding_reg[2];
		reg_data.d.l_v = holding_reg[5];
		reg_data.d.l_a = holding_reg[6];
		reg_data.d.s_v = holding_reg[9];
		reg_data.d.s_a = holding_reg[10];
		reg_data.d.on = (onoff_rd&1) == 1;
		k_mutex_unlock(&reg_data.mut);
		k_sem_give(&reg_data.new_sample);
		printt("Read ok");

		//k_msleep(1000);
	}

	return 0;
}

