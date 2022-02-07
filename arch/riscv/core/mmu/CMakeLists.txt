#
# Copyright (c) 2021 Recogni, Inc.
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_RISCV_MMU core_mmu.c)

zephyr_library_include_directories(
  .
  ../../include
)
