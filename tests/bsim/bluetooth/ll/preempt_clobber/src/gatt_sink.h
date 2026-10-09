/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 *
 * Write-without-response "sink": server attribute + client blaster, shared by
 * the roles to saturate each link (long events -> forced overlap).
 */
#ifndef GATT_SINK_H_
#define GATT_SINK_H_

#include <zephyr/bluetooth/conn.h>

/* Discover the sink characteristic value handle on `conn`. Blocks (test thread
 * context only). Returns 0 and writes *handle on success, negative on error.
 */
int sink_discover(struct bt_conn *conn, uint16_t *handle);

/* Push one max-size write-without-response toward `handle` on `conn`, retrying
 * on tx-buffer exhaustion so every connection event stays full. Returns after a
 * write is queued (or the conn drops).
 */
void sink_blast_one(struct bt_conn *conn, uint16_t handle);

#endif /* GATT_SINK_H_ */
