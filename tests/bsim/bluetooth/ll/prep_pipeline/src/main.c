/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: Apache-2.0
 */
#include "bstests.h"

extern struct bst_test_list *test_split_install(struct bst_test_list *tests);
extern struct bst_test_list *test_dut_install(struct bst_test_list *tests);
extern struct bst_test_list *test_host_install(struct bst_test_list *tests);

bst_test_install_t test_installers[] = {
	test_split_install,
	test_dut_install,
	test_host_install,
	NULL
};

int main(void)
{
	bst_main();
	return 0;
}
