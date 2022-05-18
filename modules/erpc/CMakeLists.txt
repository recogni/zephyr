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
  zephyr_include_directories(${ERPC_DIR}/erpc_c/config)
  zephyr_include_directories(${ZEPHYR_BASE}/include/posix)


  zephyr_library_sources(
    ${ERPC_DIR}/erpc_c/transports/erpc_tcp_transport.cpp
  )
#else()
#    XXX Hold on a sec
#endif()
