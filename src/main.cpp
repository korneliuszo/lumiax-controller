#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/modbus/modbus.h>

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

#define DISPLAY_STACK_SIZE 500
K_THREAD_STACK_DEFINE(display_stack_area, DISPLAY_STACK_SIZE);
struct k_thread display_thread_data;

void display_thread(void*,void*,void*)
{
	int spinner_idx=0;
	char spinner[] = "/-\\|";
	while(true)
	{
		k_sem_take(&reg_data.new_sample,K_FOREVER);
		k_mutex_lock(&reg_data.mut, K_FOREVER);
		Reg_data::Data cached = reg_data.d;
		k_mutex_unlock(&reg_data.mut);

		{
			auto l = display.lock();
			display.print_chr(0,0,spinner[spinner_idx++]);
			char buff[13];
			snprintf(buff,13,"Bat %6.2u%%",cached.b_soc);
			display.print_str(0,16,buff);
			snprintf(buff,13,"Bat %+.2d.%.2uV",cached.b_v/100,cached.b_v%100);
			display.print_str(0,32,buff);

			display.update();
		}
		if (!spinner[spinner_idx])
			spinner_idx=0;
	}
}

int main(void)
{
	k_mutex_init(&reg_data.mut);

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
	                                 5, 0, K_NO_WAIT);
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
			printt("Read fail");
			continue;
		}
		k_mutex_lock(&reg_data.mut, K_FOREVER);
		reg_data.d.b_soc = holding_reg[0];
		reg_data.d.b_v = holding_reg[1];
		reg_data.d.b_a = holding_reg[2];
		reg_data.d.l_v = holding_reg[5];
		reg_data.d.l_a = holding_reg[6];
		reg_data.d.s_v = holding_reg[9];
		reg_data.d.s_a = holding_reg[10];
		k_mutex_unlock(&reg_data.mut);
		k_sem_give(&reg_data.new_sample);
		printt("Read ok");

		//k_msleep(1000);
	}

	return 0;
}

