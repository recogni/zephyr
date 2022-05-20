# Copyright (c) 2020 Linumiz
# SPDX-License-Identifier: Apache-2.0

#if(CONFIG_ERPC)  <- This isn't getting set...

  set(ERPC_DIR ${ZEPHYR_CURRENT_MODULE_DIR})
#  set(ERPC_C_DIR ${ERPC_DIR}/erpc_c})

  zephyr_library()

  zephyr_include_directories(${ERPC_DIR}/erpc_c/setup)
  zephyr_include_directories(${ERPC_DIR}/erpc_c/infra)
  zephyr_include_directories(${ERPC_DIR}/erpc_c/port)
  zephyr_include_directories(${ERPC_DIR}/erpc_c/config)
  zephyr_include_directories(${ERPC_DIR}/erpc_c/transports)

  zephyr_library_sources(
    ${ERPC_DIR}/erpc_c/infra/erpc_arbitrated_client_manager.cpp
    ${ERPC_DIR}/erpc_c/infra/erpc_basic_codec.cpp
    ${ERPC_DIR}/erpc_c/infra/erpc_client_manager.cpp
    ${ERPC_DIR}/erpc_c/infra/erpc_crc16.cpp
    ${ERPC_DIR}/erpc_c/infra/erpc_framed_transport.cpp
    ${ERPC_DIR}/erpc_c/infra/erpc_message_buffer.cpp
    ${ERPC_DIR}/erpc_c/infra/erpc_message_loggers.cpp
    ${ERPC_DIR}/erpc_c/infra/erpc_server.cpp
    ${ERPC_DIR}/erpc_c/infra/erpc_simple_server.cpp
    ${ERPC_DIR}/erpc_c/infra/erpc_transport_arbitrator.cpp
    ${ERPC_DIR}/erpc_c/infra/erpc_pre_post_action.cpp
    ${ERPC_DIR}/erpc_c/port/erpc_port_stdlib.cpp
    ${ERPC_DIR}/erpc_c/port/erpc_threading_zephyr.cpp
#    ${ERPC_DIR}/erpc_c/port/erpc_serial.cpp
    ${ERPC_DIR}/erpc_c/setup/erpc_arbitrated_client_setup.cpp
    ${ERPC_DIR}/erpc_c/setup/erpc_client_setup.cpp
    ${ERPC_DIR}/erpc_c/setup/erpc_setup_mbf_dynamic.cpp
    ${ERPC_DIR}/erpc_c/setup/erpc_setup_mbf_static.cpp
    ${ERPC_DIR}/erpc_c/setup/erpc_server_setup.cpp
#    ${ERPC_DIR}/erpc_c/setup/erpc_setup_serial.cpp
    ${ERPC_DIR}/erpc_c/setup/erpc_setup_tcp.cpp
    ${ERPC_DIR}/erpc_c/transports/erpc_inter_thread_buffer_transport.cpp
#    ${ERPC_DIR}/erpc_c/transports/erpc_serial_transport.cpp
    ${ERPC_DIR}/erpc_c/transports/erpc_tcp_transport.cpp
  )
#else()
#    XXX Hold on a sec
#endif()
