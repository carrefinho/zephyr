/*
 * Copyright (c) 2018-2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Forward declarations for the aggregate types used only by pointer in the
 * prototypes below. This header is included by translation units (e.g.
 * ull_chan.c) that do not pull in ull_conn_types.h, so without these the
 * struct would be declared inside a parameter list (-Werror under some configs,
 * e.g. when BT_CTLR_ADVANCED_FEATURES changes the adv include path).
 */
struct ll_conn;

int ull_central_reset(void);
void ull_central_cleanup(struct node_rx_pdu *rx_free);
void ull_central_setup(struct node_rx_pdu *rx, struct node_rx_ftr *ftr,
		      struct lll_conn *lll);
void ull_central_ticker_cb(uint32_t ticks_at_expire, uint32_t ticks_drift,
			  uint32_t remainder, uint16_t lazy, uint8_t force,
			  void *param);
#if defined(CONFIG_BT_CTLR_SUBRATING)
void ull_central_latency_cancel(struct ll_conn *conn, uint16_t handle);
#endif /* CONFIG_BT_CTLR_SUBRATING */
uint8_t ull_central_chm_update(void);
