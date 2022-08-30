/*
 * bt.cpp
 *
 *  Created on: Aug 16, 2022
 *      Author: kosa
 */
#define DT_DRV_COMPAT barrot_serial

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "printt.h"


struct uart_barrot_device_config {
	bt_addr_t btaddr;
};

static const struct bt_uuid_16 ccc_uuid = BT_UUID_INIT_16(BT_UUID_GATT_CCC_VAL);

#define TX_TMP_BUFF_LEN 20

struct uart_barrot_data {
	uart_irq_callback_user_data_t irq_cb;
	void *irq_user_data;
	const struct device *dev;
	struct bt_conn *conn_connected;
	struct notifier_state {
		atomic_t state;
		struct bt_gatt_subscribe_params subscribe_params;
		struct bt_gatt_discover_params discover_params;
		struct bt_uuid_16 uuid;
		bt_gatt_notify_func_t notifier;
	}tx,rx;
	struct bt_gatt_discover_params tx_w_discover_params;
	uint16_t tx_w_handle;
	struct bt_uuid_16 tx_w_uuid;
	uint8_t initial_credit;
	uint8_t credit;
	const uint8_t* rxptr;
	uint8_t rxlen;
	struct k_mutex irq_mutex;
	uint8_t tx_tmp_buff[TX_TMP_BUFF_LEN];
	uint8_t tx_tmp_buff_cnt;
	bool tx_updated;
	struct bt_gatt_exchange_params mtu_exchange_params;
};

static struct uart_barrot_data uart_barrot_data_0 = {};

static const struct uart_barrot_device_config uart_barrot_dev_cfg_0 = {
	.btaddr	= {DT_INST_PROP(0, btaddr)},
};

static void run_tx()
{
	k_mutex_lock(&uart_barrot_data_0.irq_mutex, K_FOREVER);
	uart_barrot_data_0.tx_tmp_buff_cnt = 0;

	do
	{
		uart_barrot_data_0.tx_updated=0;
		uart_barrot_data_0.irq_cb(uart_barrot_data_0.dev,
			uart_barrot_data_0.irq_user_data);
	}while(uart_barrot_data_0.tx_updated);

	struct bt_conn *conn = NULL;

	struct bt_conn *conn_cache = uart_barrot_data_0.conn_connected;

	if (conn_cache) {
		/* Get a connection reference to ensure that a
		 * reference is maintained in case disconnected
		 * callback is called while we perform GATT Write
		 * command.
		 */
		conn = bt_conn_ref(conn_cache);
	}

	if (conn) {
		if(uart_barrot_data_0.tx_tmp_buff_cnt){
			uart_barrot_data_0.credit -= 1;
			bt_gatt_write_without_response(conn,uart_barrot_data_0.tx_w_handle,
				uart_barrot_data_0.tx_tmp_buff,
				uart_barrot_data_0.tx_tmp_buff_cnt,
				false);
			//printt("TXING %d",uart_barrot_data_0.tx_tmp_buff_cnt);
		}
		bt_conn_unref(conn);
	}
	uart_barrot_data_0.tx_tmp_buff_cnt = 0;
	k_mutex_unlock(&uart_barrot_data_0.irq_mutex);
	return;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{

	char dev[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));
	printt("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i",
	       dev, type, ad->len, rssi);

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	/* connect only to devices in close proximity */
	/*if (rssi < -70) {
		return;
	}*/
	bt_addr_le_t target_mac = {};
	target_mac.type = BT_ADDR_LE_PUBLIC;
	target_mac.a =  uart_barrot_dev_cfg_0.btaddr;
	if(memcmp(&target_mac,addr,sizeof(bt_addr_le_t))!=0)
		return;

	int err;

	err = bt_le_scan_stop();
	if (err) {
		printt("%s: Stop LE scan failed (err %d)\n", __func__, err);
		return;
	}

	struct bt_conn *conn;

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM_DEFAULT, &conn);
	if (err) {
		bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	} else {
		bt_conn_unref(conn);
	}

}


static uint8_t notify_tx_func(struct bt_conn *conn,
		   struct bt_gatt_subscribe_params *params,
		   const void *data, uint16_t length)
{
	struct notifier_state *state = CONTAINER_OF(params,struct notifier_state,subscribe_params);
	if (!data) {
		printt("[UNSUBSCRIBED] %u", params->ccc_handle);
		atomic_clear_bit(&state->state,0);
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}

	//printt("[NOTIFICATION_TX] data %p length %u", data, length);

	struct uart_barrot_data *uart_data = CONTAINER_OF(state,struct uart_barrot_data,tx);

	uint8_t * d = (uint8_t *)data;

	if(length == 2 && d[0] == 0x01)
	{
		if(!uart_data->initial_credit)
		{
			uart_data->initial_credit = d[1];
			uart_data->credit = d[1];
		}
		else
			uart_data->credit += d[1];
		//printt("New credit %d",uart_data->credit);
	}
	if(uart_data->credit)
	{
		run_tx();
	}

	return BT_GATT_ITER_CONTINUE;
}

static uint8_t notify_rx_func(struct bt_conn *conn,
		   struct bt_gatt_subscribe_params *params,
		   const void *data, uint16_t length)
{
	struct notifier_state *state = CONTAINER_OF(params,struct notifier_state,subscribe_params);
	if (!data) {
		printt("[UNSUBSCRIBED] %u", params->ccc_handle);
		atomic_clear_bit(&state->state,0);
		params->value_handle = 0U;
		params->ccc_handle = 0;
		return BT_GATT_ITER_STOP;
	}

	//printt("[NOTIFICATION_RX] data %p length %u", data, length);

	struct uart_barrot_data *uart_data = CONTAINER_OF(state,struct uart_barrot_data,rx);
	k_mutex_lock(&uart_data->irq_mutex, K_FOREVER);
	uart_data->rxptr = data;
	uart_data->rxlen = length;
	uart_barrot_data_0.irq_cb(uart_data->dev,uart_data->irq_user_data);
	uart_data->rxptr = NULL;
	k_mutex_unlock(&uart_data->irq_mutex);


	return BT_GATT_ITER_CONTINUE;
}

static uint8_t discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{

	struct notifier_state *state = CONTAINER_OF(params,struct notifier_state,discover_params);
	int err;

	if (!attr) {
		printt("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printt("[ATTRIBUTE] handle %u\n", attr->handle);
	printt("[ATTRIBUTE] perm %u\n", attr->perm);
	printt("[PARMMS] type %u\n", params->type);

	if(params->type == BT_GATT_DISCOVER_CHARACTERISTIC)
	{
		params->uuid = &ccc_uuid.uuid;
		params->start_handle = attr->handle + 2;
		params->type = BT_GATT_DISCOVER_DESCRIPTOR;
		state->subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
		err = bt_gatt_discover(conn, params);
		if (err) {
			printt("Discover failed (err %d)\n", err);
		}
	}
	else
	{
		state->subscribe_params.value = BT_GATT_CCC_NOTIFY;
		state->subscribe_params.ccc_handle = attr->handle;
		state->subscribe_params.notify = state->notifier;
		err = bt_gatt_subscribe(conn, &state->subscribe_params);

		if (err && err != -EALREADY) {
			printt("Subscribe failed (err %d)\n", err);
		} else {
			printt("[SUBSCRIBED]\n");
		}

	}


	return BT_GATT_ITER_STOP;
}

static uint8_t discover_tx_w_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{

	struct uart_barrot_data *data = CONTAINER_OF(params,struct uart_barrot_data,tx_w_discover_params);

	if (!attr) {
		printt("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}

	printt("[ATTRIBUTE] handle %u\n", attr->handle);
	printt("[ATTRIBUTE] perm %u\n", attr->perm);
	printt("[PARMMS] type %u\n", params->type);

	data->tx_w_handle = bt_gatt_attr_value_handle(attr);

	return BT_GATT_ITER_STOP;

}


static void discoverer_start(struct bt_conn *conn, struct notifier_state * state)
{
	state->discover_params.uuid = &state->uuid.uuid;

	state->discover_params.func = discover_func;
	state->discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	state->discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	state->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	int err = bt_gatt_discover(conn, &state->discover_params);
	if (err) {
		printt("Discover failed(err %d)\n", err);
		return;
	}
}

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	printt("%s: MTU exchange %s (%u)\n", __func__,
	       err == 0U ? "successful" : "failed",
	       bt_gatt_get_mtu(conn));

	struct uart_barrot_data *data = CONTAINER_OF(params,struct uart_barrot_data,mtu_exchange_params);
	discoverer_start(conn, &data->rx);
	discoverer_start(conn, &data->tx);
	data->tx_w_discover_params.uuid = &data->tx_w_uuid.uuid;
	data->tx_w_discover_params.func = discover_tx_w_func;
	data->tx_w_discover_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
	data->tx_w_discover_params.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE;
	data->tx_w_discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
	int err2 = bt_gatt_discover(conn, &data->tx_w_discover_params);
	if (err2) {
		printt("Discover failed(err %d)\n", err);
		return;
	}
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	struct bt_conn_info conn_info;
	int err;

	if (conn_err) {
		return;
	}

	uart_barrot_data_0.conn_connected = bt_conn_ref(conn);

	err = bt_conn_get_info(conn, &conn_info);
	if (err) {
		return;
	}

	printt("%s: Current MTU = %u\n", __func__, bt_gatt_get_mtu(conn));

	uart_barrot_data_0.mtu_exchange_params.func = mtu_exchange_cb;

	printt("%s: Exchange MTU...\n", __func__);
	err = bt_gatt_exchange_mtu(conn, &uart_barrot_data_0.mtu_exchange_params);
	if (err) {
		printt("%s: MTU exchange failed (err %d)", __func__, err);
	}

}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn_info conn_info;
	int err;

	err = bt_conn_get_info(conn, &conn_info);
	if (err) {
		return;
	}

	uart_barrot_data_0.conn_connected = NULL;

	bt_conn_unref(conn);

	uart_barrot_data_0.credit = 0;
	uart_barrot_data_0.tx_w_handle = 0;
	atomic_clear_bit(&uart_barrot_data_0.tx.state,0);
	atomic_clear_bit(&uart_barrot_data_0.rx.state,0);

	if (conn_info.role == BT_CONN_ROLE_CENTRAL) {
		bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};


void barrot_init()
{
	bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	return;
}

static int barrot_configure(const struct device *dev,
		 const struct uart_config *cfg)
{
	return 0;
}

static void barrot_irq_tx_enable(const struct device *dev)
{
	atomic_set_bit(&uart_barrot_data_0.tx.state,0);
	run_tx();
}

static void barrot_irq_tx_disable(const struct device *dev)
{
	atomic_clear_bit(&uart_barrot_data_0.tx.state,0);
}

static void barrot_irq_rx_enable(const struct device *dev)
{
	atomic_set_bit(&uart_barrot_data_0.rx.state,0);
}

static void barrot_irq_rx_disable(const struct device *dev)
{
	atomic_clear_bit(&uart_barrot_data_0.rx.state,0);
}

static void barrot_irq_callback_set(const struct device *dev,
			 uart_irq_callback_user_data_t cb,
			 void *user_data)
{
	struct uart_barrot_data *data;

	data = dev->data;
	data->irq_cb = cb;
	data->irq_user_data = user_data;
}

static int barrot_fifo_fill(const struct device *dev,
				   const uint8_t *tx_data, int size)
{
	if(!uart_barrot_data_0.tx_w_handle)
	{
		return 0;
	}

	if(!uart_barrot_data_0.tx.subscribe_params.ccc_handle)
	{
		return 0;
	}

	uint8_t i = MIN(size,
			TX_TMP_BUFF_LEN-uart_barrot_data_0.tx_tmp_buff_cnt);

	memcpy(&uart_barrot_data_0.tx_tmp_buff[uart_barrot_data_0.tx_tmp_buff_cnt],
			tx_data,i);
	uart_barrot_data_0.tx_tmp_buff_cnt+=i;
	uart_barrot_data_0.tx_updated = 1;
	return i;
}

static int barrot_fifo_read(const struct device *dev,
				   uint8_t *rx_data, const int size)
{
	int i=0;

	if (!uart_barrot_data_0.rxptr)
	{
		return 0;
	}


	while(i<size && uart_barrot_data_0.rxlen)
	{
		rx_data[i++] = *(uart_barrot_data_0.rxptr++);
		uart_barrot_data_0.rxlen--;
	}
	return i;
}

static int barrot_irq_tx_ready(const struct device *dev)
{
	return 	atomic_test_bit(&uart_barrot_data_0.tx.state,0) &&
			(uart_barrot_data_0.tx_w_handle) &&
			(uart_barrot_data_0.tx.subscribe_params.ccc_handle) &&
			(TX_TMP_BUFF_LEN > uart_barrot_data_0.tx_tmp_buff_cnt);
}

static int barrot_irq_rx_ready(const struct device *dev)
{
	return atomic_test_bit(&uart_barrot_data_0.rx.state,0) &&
			(uart_barrot_data_0.rxptr) &&
			(uart_barrot_data_0.rxlen);
}

int barrot_irq_tx_complete(const struct device *dev)
{
	return 1;
}


static int barrot_irq_is_pending(const struct device *dev)
{
	return 1;

}

static int barrot_irq_update(const struct device *dev)
{
	return 1;
}

static const struct uart_driver_api uart_barrot_driver_api = {
		.configure = barrot_configure,
		.fifo_fill		= barrot_fifo_fill,
		.fifo_read		= barrot_fifo_read,
		.irq_tx_enable		= barrot_irq_tx_enable,
		.irq_tx_disable		= barrot_irq_tx_disable,
		.irq_tx_ready		= barrot_irq_tx_ready,
		.irq_rx_enable		= barrot_irq_rx_enable,
		.irq_rx_disable		= barrot_irq_rx_disable,
		.irq_tx_complete	= barrot_irq_tx_complete,
		.irq_rx_ready		= barrot_irq_rx_ready,
		.irq_is_pending		= barrot_irq_is_pending,
		.irq_update		= barrot_irq_update,

		.irq_callback_set = barrot_irq_callback_set,
};


static int uart_barrot_init(const struct device *dev)
{
	struct uart_barrot_data *data;

	data = dev->data;
	data->dev = dev;
	struct bt_uuid_16 txuuid = BT_UUID_INIT_16(0xff03);
	data->tx.uuid = txuuid;
	data->tx.notifier = notify_tx_func;
	struct bt_uuid_16 txwuuid = BT_UUID_INIT_16(0xff02);
	data->tx_w_uuid = txwuuid;
	struct bt_uuid_16 rxuuid = BT_UUID_INIT_16(0xff01);
	data->rx.uuid = rxuuid;
	data->rx.notifier = notify_rx_func;
	k_mutex_init(&data->irq_mutex);
	return 0;

}
DEVICE_DT_INST_DEFINE(0,
		uart_barrot_init,
		NULL,
		&uart_barrot_data_0, &uart_barrot_dev_cfg_0,
		PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,
		(void *)&uart_barrot_driver_api);
