/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "util/util.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "hal/ccm.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"

#include "ull_conn_internal.h"

#include "ll_feat.h"
#include "ll_settings.h"

#include <zephyr/bluetooth/hci_types.h>

#include "hal/debug.h"

#if defined(CONFIG_BT_CTLR_SET_HOST_FEATURE)
static uint64_t host_features;
#if defined(CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS)
/* Host-controlled feature bits on page 1 (bits >= 64), stored page-1-relative
 * (bit 73 -> bit 9). The page-0 host_features uint64 above cannot hold them.
 */
static uint64_t host_features_page1;
#endif /* CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS */

/* True if the Controller has an established ACL; LE Set Host Feature is
 * disallowed while connected.
 */
static bool ll_feat_acl_established(void)
{
#if defined(CONFIG_BT_CONN)
	uint16_t conn_free_count = ll_conn_free_count_get();

	/* Check if any connection contexts where allocated */
	if (conn_free_count != CONFIG_BT_MAX_CONN) {
		for (uint16_t handle = 0U; handle < CONFIG_BT_MAX_CONN; handle++) {
			if (ll_connected_get(handle)) {
				return true;
			}
		}
	}
#endif /* CONFIG_BT_CONN */
	return false;
}

uint8_t ll_set_host_feature(uint8_t bit_number, uint8_t bit_value)
{
	uint64_t feature;

#if defined(CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS)
	/* Bit 73 (SCI Host Support) is on feature page 1; BIT64(73) would
	 * overflow the page-0 uint64 path below, so handle it separately.
	 */
	if (bit_number == BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS_HOST_SUPP) {
		if (ll_feat_acl_established()) {
			return BT_HCI_ERR_CMD_DISALLOWED;
		}

		if (bit_value) {
			host_features_page1 |= BIT64(bit_number - 64U);
		} else {
			host_features_page1 &= ~BIT64(bit_number - 64U);
		}

		return BT_HCI_ERR_SUCCESS;
	}
#endif /* CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS */

	/* Check if Bit_Number is not controlled by the Host */
	feature = BIT64(bit_number);
	if (!(feature & LL_FEAT_HOST_BIT_MASK)) {
		return BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL;
	}

	if (ll_feat_acl_established()) {
		return BT_HCI_ERR_CMD_DISALLOWED;
	}

	/* Set or Clear the Host feature bit */
	if (bit_value) {
		host_features |= feature;
	} else {
		host_features &= ~feature;
	}

	return BT_HCI_ERR_SUCCESS;
}

void ll_feat_reset(void)
{
	host_features = 0U;
#if defined(CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS)
	host_features_page1 = 0U;
#endif /* CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS */
}

uint64_t ll_feat_get(void)
{
	return LL_FEAT | (host_features & LL_FEAT_HOST_BIT_MASK);
}

#else /* !CONFIG_BT_CTLR_SET_HOST_FEATURE */
uint64_t ll_feat_get(void)
{
	return LL_FEAT;
}

#endif /* !CONFIG_BT_CTLR_SET_HOST_FEATURE */

#if defined(CONFIG_BT_CTLR_EXTENDED_FEAT_SET)
/* LL Extended Feature Set (Core 6.0): return the 24-octet feature page 'page'.
 * Page 0 (the legacy uint64) is handled by ll_feat_get() and not produced here.
 * 24 == BT_HCI_LE_BYTES_PER_FEATURE_PAGE; literal used to avoid include-order
 * dependency on <zephyr/bluetooth/hci_types.h>.
 */
void ll_feat_get_page(uint8_t page, uint8_t *out)
{
	memset(out, 0, 24);

	switch (page) {
	case 1:
#if defined(CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS)
		/* Bit 72: Shorter Connection Intervals (controller capability). */
		out[(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS - 64) / 8] |=
			BIT(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS & 7);
		/* Bit 73: SCI Host Support, advertised only if the local host
		 * enabled it via LE Set Host Feature.
		 */
		if (host_features_page1 &
		    BIT64(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS_HOST_SUPP - 64U)) {
			out[(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS_HOST_SUPP - 64) / 8] |=
				BIT(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS_HOST_SUPP & 7);
		}
#endif /* CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS */
#if defined(CONFIG_BT_CTLR_FRAME_SPACE_UPDATE)
		/* Bit 65: Frame Space Update (controller capability, not host-gated). */
		out[(BT_LE_FEAT_BIT_FRAME_SPACE_UPDATE - 64) / 8] |=
			BIT(BT_LE_FEAT_BIT_FRAME_SPACE_UPDATE & 7);
#endif /* CONFIG_BT_CTLR_FRAME_SPACE_UPDATE */
		break;
	default:
		/* Unsupported page, leave all-zero */
		break;
	}
}

uint8_t ll_feat_local_max_page(void)
{
	uint8_t features[24];
	uint8_t page;

	/* Return the highest local feature page (1..CONFIG_BT_CTLR_LOCAL_FEATURE_PAGE)
	 * that has any feature bit set; else 0 (only page 0 present).
	 */
	for (page = CONFIG_BT_CTLR_LOCAL_FEATURE_PAGE; page >= 1U; page--) {
		ll_feat_get_page(page, features);

		for (uint8_t i = 0U; i < 24U; i++) {
			if (features[i] != 0U) {
				return page;
			}
		}
	}

	return 0U;
}
#endif /* CONFIG_BT_CTLR_EXTENDED_FEAT_SET */

#if defined(CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS)
bool ll_feat_sci_host_supported(void)
{
	return (host_features_page1 &
		BIT64(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS_HOST_SUPP - 64U)) != 0U;
}
#endif /* CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS */
