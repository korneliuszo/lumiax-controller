#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/modbus/modbus.h>

#include "main_modules.hpp"
#include "printt.h"
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

int main(void)
{
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


	k_mutex_init(&reg_data.mut);

	if (init_modbus_client()) {
		tlay2.printt("Modbus RTU client initialization failed");
		return 1;
	}


	display.blank_off();

	display.print_chr(32,0,'a');

	display.print_chr(32,32,'a');

	display.print_chr(48,48,'a');

	display.print_chr(48,128,'a');

	display.print_chr(32,120,'a');

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
		reg_data.b_soc = holding_reg[0];
		reg_data.b_v = holding_reg[1];
		reg_data.b_a = holding_reg[2];
		reg_data.l_v = holding_reg[5];
		reg_data.l_a = holding_reg[6];
		reg_data.s_v = holding_reg[9];
		reg_data.s_a = holding_reg[10];
		k_mutex_unlock(&reg_data.mut);
		printt("Read ok");

		//k_msleep(1000);
	}

	return 0;
}

