cmake_minimum_required(VERSION 3.20.0)

set(HPM_ZEPHYR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../hpm_support)
list(APPEND BOARD_ROOT ${HPM_ZEPHYR_DIR})

# HPM5E3Y board DTS already assigns UART0 to HDMA. The test overlay contains
# HPM6750-specific IRQ and XDMA assignments, so use the board DTS directly.
if(BOARD MATCHES "^dust-hpm5e3y")
  set(DTC_OVERLAY_FILE "")
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../common/diagnostics.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(uart0_async_tx_test)

target_sources(app PRIVATE src/main.c)
