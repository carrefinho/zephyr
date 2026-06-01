# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bsim_test_prep_pipeline)

target_sources(app PRIVATE
  src/main.c
  src/gatt_sink.c
  src/split.c
  src/dut.c
  src/host.c
)

# bsim test framework headers (bstests.h, bs_types.h, bs_tracing.h,
# time_machine.h) live in the BabbleSim components.
zephyr_include_directories(
  ${BSIM_COMPONENTS_PATH}/libUtilv1/src/
  ${BSIM_COMPONENTS_PATH}/libPhyComv1/src/
)
