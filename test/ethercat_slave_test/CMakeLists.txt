# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

get_filename_component(WBR_CONTROL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(HPM_ZEPHYR_DIR ${WBR_CONTROL_ROOT}/../hpm_support)
list(APPEND BOARD_ROOT ${HPM_ZEPHYR_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hpm5e_ethercat_slave_test)

set(SSC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ssc_generated/src)

file(GLOB SSC_SOURCES CONFIGURE_DEPENDS ${SSC_DIR}/*.c)

target_sources(app PRIVATE
  src/main.c
  src/hpm5e_esc_hw.c
  ${SSC_SOURCES}
)

target_include_directories(app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${SSC_DIR}
)

# SSC 5.13 contains intentional fallthroughs and legacy declarations. Keep the
# generated source warnings separate from application warnings.
set_source_files_properties(${SSC_SOURCES} PROPERTIES
  COMPILE_OPTIONS "-Wno-unused-parameter;-Wno-sign-compare;-Wno-implicit-fallthrough"
)
