/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * LE Connection Subrating - central demo for two nRF54L15 DKs.
 *
 * Scans for the subrate_peripheral sample, connects at a fixed connection
 * interval, then each press of Button 1 cycles the subrate factor through
 * active / idle / dormant tiers (1 / 8 / 33) using a central-initiated
 * Connection Subrate Update (Core Spec, Vol 6, Part B, 5.1.19). Measure the
 * board current at each tier to observe the central-side power saving.
 *
 * The LED is left off while idle so it does not perturb a current measurement;
 * it blinks (tier-index times) only when a new factor is applied.
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

/* Must match the subrate_peripheral sample's CONFIG_BT_DEVICE_NAME. */
#define TARGET_NAME "subrate_demo"

#define CONN_INTERVAL 24U  /* 30 ms (1.25 ms units) */
#define CONN_TIMEOUT  200U /* 2 s (10 ms units) */

/* active / idle / dormant subrate factors (1 = subrating off). */
static const uint16_t tiers[] = {1U, 8U, 33U};
static size_t tier_idx;

static struct bt_conn *default_conn;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback button_cb_data;
static volatile bool button_pressed;
static volatile int pending_blink;
static int64_t last_press_ms;

static void blink(int times)
{
	for (int i = 0; i < times; i++) {
		(void)gpio_pin_set_dt(&led, 1);
		k_sleep(K_MSEC(120));
		(void)gpio_pin_set_dt(&led, 0);
		k_sleep(K_MSEC(120));
	}
}

static void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	button_pressed = true;
}

static bool name_match(struct bt_data *data, void *user_data)
{
	bool *found = user_data;

	if (data->type == BT_DATA_NAME_COMPLETE &&
	    data->data_len == sizeof(TARGET_NAME) - 1 &&
	    memcmp(data->data, TARGET_NAME, data->data_len) == 0) {
		*found = true;
		return false; /* stop parsing */
	}
	return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	bool found = false;
	int err;

	ARG_UNUSED(rssi);

	if (default_conn || (type != BT_GAP_ADV_TYPE_ADV_IND &&
			     type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND)) {
		return;
	}

	bt_data_parse(ad, name_match, &found);
	if (!found) {
		return;
	}

	(void)bt_le_scan_stop();

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
				BT_LE_CONN_PARAM(CONN_INTERVAL, CONN_INTERVAL, 0, CONN_TIMEOUT),
				&default_conn);
	if (err) {
		printk("Create connection failed (err %d)\n", err);
		(void)bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	}
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	ARG_UNUSED(conn);

	if (err) {
		printk("Failed to connect (err 0x%02x)\n", err);
		bt_conn_unref(default_conn);
		default_conn = NULL;
		(void)bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
		return;
	}

	tier_idx = 0U;
	printk("Connected. Press Button 1 to cycle subrate factor (1 / 8 / 33).\n");
	pending_blink = 1;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);

	printk("Disconnected (reason 0x%02x); scanning again\n", reason);
	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
	(void)bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
}

static void subrate_changed(struct bt_conn *conn,
			    const struct bt_conn_le_subrate_changed *params)
{
	ARG_UNUSED(conn);

	if (params->status != BT_HCI_ERR_SUCCESS) {
		printk("Subrate change failed (status 0x%02x)\n", params->status);
		return;
	}
	printk("Subrate factor now %u (continuation %u, peripheral latency %u)\n",
	       params->factor, params->continuation_number, params->peripheral_latency);
	pending_blink = (int)tier_idx + 1;
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
	.subrate_changed = subrate_changed,
};

int main(void)
{
	int err;

	if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&button)) {
		printk("GPIO not ready\n");
		return 0;
	}
	(void)gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	(void)gpio_pin_configure_dt(&button, GPIO_INPUT);
	(void)gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_data, button_isr, BIT(button.pin));
	(void)gpio_add_callback(button.port, &button_cb_data);

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("Subrating central started; scanning for \"%s\"\n", TARGET_NAME);

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
	if (err) {
		printk("Scan start failed (err %d)\n", err);
		return 0;
	}

	while (1) {
		if (pending_blink) {
			int n = pending_blink;

			pending_blink = 0;
			blink(n);
		}

		if (button_pressed) {
			int64_t now = k_uptime_get();

			button_pressed = false;

			if (now - last_press_ms < 300) {
				continue; /* debounce */
			}
			last_press_ms = now;

			if (!default_conn) {
				continue;
			}

			tier_idx = (tier_idx + 1U) % ARRAY_SIZE(tiers);

			struct bt_conn_le_subrate_param param = {
				.subrate_min = tiers[tier_idx],
				.subrate_max = tiers[tier_idx],
				.max_latency = 0U,
				.continuation_number = 0U,
				.supervision_timeout = CONN_TIMEOUT,
			};

			err = bt_conn_le_subrate_request(default_conn, &param);
			if (err) {
				printk("Subrate request (factor %u) failed (err %d)\n",
				       tiers[tier_idx], err);
			} else {
				printk("Requested subrate factor %u\n", tiers[tier_idx]);
			}
		}
		k_sleep(K_MSEC(50));
	}

	return 0;
}
