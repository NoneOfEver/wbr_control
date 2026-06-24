# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(cherryusb_cdc_acm_test)

target_sources(app PRIVATE
	src/main.c
	src/cdc_acm.c
)

set(CHERRYUSB_CONFIG_DIR ${HPM_ZEPHYR_DIR}/samples/cherryusb/config)
zephyr_include_directories(${CHERRYUSB_CONFIG_DIR})
