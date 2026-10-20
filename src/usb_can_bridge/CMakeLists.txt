cmake_minimum_required(VERSION 3.20.0)
get_filename_component(WBR_CONTROL_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
get_filename_component(HPM_ZEPHYR_DIR "${WBR_CONTROL_ROOT}/../hpm_support" ABSOLUTE)
list(APPEND BOARD_ROOT ${HPM_ZEPHYR_DIR})
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_can_bridge)
set(chassis ${WBR_CONTROL_ROOT}/src/chassis_controller)
set(generated ${CMAKE_CURRENT_BINARY_DIR}/generated)

# Generate only transport value types used by this gateway. No chassis,
# oscilloscope, status publisher or test component is configured or linked.
set(message_names booster_state chassismotors_feedback_raw chassismotors_send_raw
 gimbal_state hi91_imu_sample pc_auto_aim_command remote_input_state usb_raw_frame_queue)
foreach(name IN LISTS message_names)
 list(APPEND message_inputs ${WBR_CONTROL_ROOT}/msg/${name}.msg)
 list(APPEND message_headers ${generated}/msg/${name}.hpp)
endforeach()
add_custom_command(OUTPUT ${generated}/msg/messages_generated.cpp ${message_headers}
 COMMAND ${PYTHON_EXECUTABLE} ${WBR_CONTROL_ROOT}/tools/generate_messages.py
 --output-dir ${generated}/msg --source-output ${generated}/msg/messages_generated.cpp ${message_inputs}
 DEPENDS ${WBR_CONTROL_ROOT}/tools/generate_messages.py ${message_inputs})
add_custom_command(OUTPUT ${generated}/params/motors_params_generated.h
 COMMAND ${PYTHON_EXECUTABLE} ${WBR_CONTROL_ROOT}/tools/generate_params.py
 --input ${chassis}/motors_params.yaml --schema ${chassis}/motors_params.schema.yaml
 --namespace platform::motors_params --output ${generated}/params/motors_params_generated.h
 DEPENDS ${WBR_CONTROL_ROOT}/tools/generate_params.py ${chassis}/motors_params.yaml ${chassis}/motors_params.schema.yaml)

target_sources(app PRIVATE main.cpp peripherals.cpp
 ${chassis}/communication/can_dispatch.cpp ${chassis}/communication/usb_session.cpp
 ${chassis}/modules/ahrs/onboard_imu_module.cpp
 ${WBR_CONTROL_ROOT}/src/protocols/motors/dm_motor_protocol.cpp
 ${generated}/msg/messages_generated.cpp ${message_headers}
 ${generated}/params/motors_params_generated.h)
target_include_directories(app PRIVATE ${WBR_CONTROL_ROOT} ${WBR_CONTROL_ROOT}/src
 ${generated} ${generated}/params)
target_compile_definitions(app PRIVATE WBR_USB_BULK_TRANSPORT=1 WBR_USB_RAW_OUT_BYTES=128 WBR_CAN_FORWARDING=1)
zephyr_include_directories(${HPM_ZEPHYR_DIR}/samples/cherryusb/config)
