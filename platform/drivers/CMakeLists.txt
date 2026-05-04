if(CONFIG_RM_TEST_RUNTIME_INIT_UART)
  file(GLOB_RECURSE RM_TEST_UART_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/communication/uart_dispatch.cpp
  )
  target_sources(app PRIVATE ${RM_TEST_UART_SOURCES})
endif()

if(CONFIG_RM_TEST_RUNTIME_INIT_CAN)
  file(GLOB_RECURSE RM_TEST_CAN_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/communication/can_dispatch.cpp
  )
  target_sources(app PRIVATE ${RM_TEST_CAN_SOURCES})
else()
  target_sources(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/communication/can_dispatch_stub.cpp
  )
endif()

if(CONFIG_RM_TEST_RUNTIME_INIT_USB)
  file(GLOB_RECURSE RM_TEST_USB_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/communication/usb_session.cpp
  )
  target_sources(app PRIVATE ${RM_TEST_USB_SOURCES})
endif()

file(GLOB_RECURSE RM_TEST_DEVICE_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/devices/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/devices/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/devices/*.cpp
)

target_sources(app PRIVATE ${RM_TEST_DEVICE_SOURCES})