/*
 * Copyright (c) 2021 IoT.bzh
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <kernel.h>
#include <string.h>
#include <tracing_core.h>
#include <tracing_buffer.h>
#include <tracing_backend.h>

uint8_t ram_tracing[CONFIG_RAM_TRACING_BUFFER_SIZE];
static uint32_t pos;
static bool buffer_full;

static void tracing_backend_ram_output(
		const struct tracing_backend *backend,
		uint8_t *data, uint32_t length)
{
	if (buffer_full) {
		return;
	}

	if ((pos + length) > CONFIG_RAM_TRACING_BUFFER_SIZE) {
		buffer_full = true;
		return;
	}

	memcpy(ram_tracing + pos, data, length);
	pos += length;
}

static void tracing_backend_ram_init(void)
{
	memset(ram_tracing, 0, CONFIG_RAM_TRACING_BUFFER_SIZE);
}

const struct tracing_backend_api tracing_backend_ram_api = {
	.init = tracing_backend_ram_init,
	.output  = tracing_backend_ram_output
};

TRACING_BACKEND_DEFINE(tracing_backend_ram, tracing_backend_ram_api);

int tracing_ram_info_get(uint8_t **addr, int *len, uint32_t *offset)
{
	if (addr != NULL) {
		*addr = ram_tracing;
	}

	if (len != NULL) {
		*len = CONFIG_RAM_TRACING_BUFFER_SIZE;
	}

	if (offset != NULL) {
		*offset = pos;
	}

	return 0;
}
