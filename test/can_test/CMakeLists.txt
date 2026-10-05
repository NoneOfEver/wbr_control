cmake_minimum_required(VERSION 3.20.0)

include(${CMAKE_CURRENT_LIST_DIR}/../common/diagnostics.cmake)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(can_test)

option(CAN_TEST_LOOPBACK "Run CAN controller in internal loopback mode" OFF)
option(CAN_TEST_ALL_BUSES "Probe all four CAN controllers in one physical-bus image" OFF)
option(CAN_TEST_PIN_PROBE "Probe CAN0/CAN1 TXD to RXD through the transceivers" OFF)
if(CAN_TEST_LOOPBACK)
  target_compile_definitions(app PRIVATE CAN_TEST_LOOPBACK=1)
endif()

if(CAN_TEST_PIN_PROBE)
  target_sources(app PRIVATE src/pin_probe.c)
elseif(CAN_TEST_ALL_BUSES)
  target_sources(app PRIVATE src/all_buses.c)
else()
  target_sources(app PRIVATE src/main.c)
endif()
