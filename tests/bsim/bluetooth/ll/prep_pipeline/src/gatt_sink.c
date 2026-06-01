/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <errno.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include "prep_pipeline.h"
#include "gatt_sink.h"

/* ---- Server side: every device exposes the sink characteristic ---------- */

static volatile uint32_t sink_rx_count;

static ssize_t sink_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(attr);
	ARG_UNUSED(buf);
	ARG_UNUSED(offset);
	ARG_UNUSED(flags);
	sink_rx_count++;
	return len;
}

BT_GATT_SERVICE_DEFINE(sink_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(SINK_SVC_UUID)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(SINK_CHR_UUID),
			       BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE,
			       NULL, sink_write_cb, NULL));

/* ---- Client side: discover the value handle, then blast max writes ------ */

static struct bt_uuid_128 sink_chr_uuid = BT_UUID_INIT_128(SINK_CHR_UUID);
static struct bt_gatt_discover_params disc_params;
static uint16_t disc_handle;
static K_SEM_DEFINE(disc_done, 0, 1);

static uint8_t disc_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
		       struct bt_gatt_discover_params *params)
{
	if (attr) {
		disc_handle = bt_gatt_attr_value_handle(attr);
	}
	k_sem_give(&disc_done);
	return BT_GATT_ITER_STOP;
}

int sink_discover(struct bt_conn *conn, uint16_t *handle)
{
	int err;

	disc_handle = 0U;
	k_sem_reset(&disc_done);

	disc_params.uuid = &sink_chr_uuid.uuid;
	disc_params.func = disc_cb;
	disc_params.start_handle = 0x0001U;
	disc_params.end_handle = 0xffffU;
	disc_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

	err = bt_gatt_discover(conn, &disc_params);
	if (err) {
		printk("sink discover start failed (err %d)\n", err);
		return err;
	}

	if (k_sem_take(&disc_done, K_SECONDS(5)) != 0 || disc_handle == 0U) {
		printk("sink discover did not complete\n");
		return -ETIMEDOUT;
	}

	*handle = disc_handle;
	printk("sink characteristic value handle = 0x%04x\n", disc_handle);
	return 0;
}

void sink_blast_one(struct bt_conn *conn, uint16_t handle)
{
	static uint8_t payload[SINK_WRITE_LEN];
	int err;

	/* Retry on tx-buffer exhaustion so events stay full; bail if the conn
	 * goes away (write returns -ENOTCONN).
	 */
	do {
		err = bt_gatt_write_without_response(conn, handle, payload,
						     sizeof(payload), false);
		if (err == -ENOMEM || err == -ENOBUFS || err == -EAGAIN) {
			k_sleep(K_MSEC(1));
			continue;
		}
		break;
	} while (true);

	if (err && err != -ENOTCONN) {
		/* Unexpected; don't spam, just yield. */
		k_sleep(K_MSEC(1));
	}
}
