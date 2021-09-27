/*
 * Copyright (c) 2020 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bootutil/bootutil_public.h"
#include <dfu/mcuboot.h>
#include <init.h>
#include <shell/shell.h>
#include <stdlib.h>

#include "mcuboot_priv.h"

#define UNFOUND_SLOT 99

struct area_desc {
	const char *name;
	unsigned int id;
};

static const struct area_desc areas[] = {
	{"primary", FLASH_AREA_IMAGE_PRIMARY},
#ifdef FLASH_AREA_IMAGE_SECONDARY
	{"secondary", FLASH_AREA_IMAGE_SECONDARY},
#endif
};

static const char *swap_state_magic_str(uint8_t magic)
{
	switch (magic) {
	case BOOT_MAGIC_GOOD:
		return "good";
	case BOOT_MAGIC_BAD:
		return "bad";
	case BOOT_MAGIC_UNSET:
		return "unset";
	case BOOT_MAGIC_ANY:
		return "any";
	case BOOT_MAGIC_NOTGOOD:
		return "notgood";
	}

	return "unknown";
}

static const char *swap_type_str(uint8_t type)
{
	switch (type) {
	case BOOT_SWAP_TYPE_NONE:
		return "none";
	case BOOT_SWAP_TYPE_TEST:
		return "test";
	case BOOT_SWAP_TYPE_PERM:
		return "perm";
	case BOOT_SWAP_TYPE_REVERT:
		return "revert";
	case BOOT_SWAP_TYPE_FAIL:
		return "fail";
	}

	return "unknown";
}

static const char *swap_state_flag_str(uint8_t flag)
{
	switch (flag) {
	case BOOT_FLAG_SET:
		return "set";
	case BOOT_FLAG_BAD:
		return "bad";
	case BOOT_FLAG_UNSET:
		return "unset";
	case BOOT_FLAG_ANY:
		return "any";
	}

	return "unknown";
}

static int cmd_mcuboot_erase(const struct shell *shell, size_t argc,
			     char **argv)
{
	unsigned int id = UNFOUND_SLOT;
	int err;
	struct mcuboot_img_header hdr[2];
	struct boot_swap_state swap_state[2];
	const struct area_desc *area;

	if (strcmp(argv[1], "unused") == 0)
	{
		/* Gather info for all slots */
		for (int i = 0; i < ARRAY_SIZE(areas); i++)
		{
			area = &areas[i];
			memset(&hdr[i], 0, sizeof(hdr[i]));
			memset(&swap_state[i], 0, sizeof(swap_state[i]));
			boot_read_bank_header(area->id, &hdr[i], sizeof(hdr[i]));
			boot_read_swap_state_by_id(area->id, &swap_state[i]);
		}

		/* Must have two valid images to delete one */
		if ((swap_state[0].magic == BOOT_MAGIC_GOOD) &&
			(swap_state[1].magic == BOOT_MAGIC_GOOD))
		{
			/* If deleting one, make sure the other is confirmed so we aren't stuck with
			 * a single, unconfirmed image.
			 */
			for (int i = 0; i < ARRAY_SIZE(areas); i++)
			{
				int other = !i;
				area = &areas[i];
				if (swap_state[i].copy_done != BOOT_FLAG_SET)
				{
					if (swap_state[other].image_ok != BOOT_FLAG_SET)
					{
						shell_error(shell, "Confirm %s before deleting %s", areas[other].name, areas[i].name);
					} else {
						shell_print(shell, "Erasing %s area (%u)", area->name, area->id);
						shell_print(shell, "  version: %u.%u.%u+%u",
							(unsigned int) hdr[i].h.v1.sem_ver.major,
							(unsigned int) hdr[i].h.v1.sem_ver.minor,
							(unsigned int) hdr[i].h.v1.sem_ver.revision,
							(unsigned int) hdr[i].h.v1.sem_ver.build_num);
						id = area->id;
					}
				}
			}
		} else {
			shell_error(shell, "Two images must be present to delete one");
		}
	} else {
		id = strtoul(argv[1], NULL, 0);
	}

	if (id != UNFOUND_SLOT) 
	{
		err = boot_erase_img_bank(id);
		if (err) {
			shell_error(shell, "failed to erase bank %u", id);
			return err;
		}
	}

	return 0;
}

static int cmd_mcuboot_confirm(const struct shell *shell, size_t argc,
			       char **argv)
{
	int err;

	err = boot_write_img_confirmed();
	if (err) {
		shell_error(shell, "failed to confirm: %d", err);
	}

	return err;
}

#ifndef CONFIG_BOARD_SCORPIO
/* Requesting upgrade is nosensical with equal slot, since 
 * the latest version is always chosen regardless of any flags.
 */
static int cmd_mcuboot_request_upgrade(const struct shell *shell, size_t argc,
				       char **argv)
{
	int permanent = 0;
	int err;

	if (argc > 1) {
		if (!strcmp(argv[1], "permanent")) {
			permanent = 1;
		} else {
			shell_warn(shell, "invalid argument!");
			return -EINVAL;
		}
	}

	err = boot_request_upgrade(permanent);
	if (err) {
		shell_error(shell, "failed to request upgrade: %d", err);
	}

	return err;
}
#endif

static int cmd_mcuboot_info_area(const struct shell *shell,
				 const struct area_desc *area)
{
	struct mcuboot_img_header hdr;
	struct boot_swap_state swap_state;
	int err;

	err = boot_read_bank_header(area->id, &hdr, sizeof(hdr));
	if (err) {
		return err;
	}

	err = boot_read_swap_state_by_id(area->id, &swap_state);
	if (err) {
		shell_error(shell, "failed to read %s area (%u) %s: %d",
			    area->name, area->id, "swap state", err);
		return err;
	}

	if (swap_state.copy_done == BOOT_FLAG_SET)
	{
		if (swap_state.image_ok == BOOT_FLAG_SET)
			shell_error(shell, "%s area (%u)  Booted/Confirmed", area->name, area->id);
		else
			shell_error(shell, "%s area (%u)  Booted/Unconfirmed (WARNING: image may be removed on next boot unless confirmed)", area->name, area->id);
	} else {
		shell_print(shell, "%s area (%u)  %s", area->name, area->id, swap_state.image_ok == BOOT_FLAG_SET ? "Confirmed" : "Unconfirmed");
	}

	shell_print(shell, "  version: %u.%u.%u+%u",
			(unsigned int) hdr.h.v1.sem_ver.major,
			(unsigned int) hdr.h.v1.sem_ver.minor,
			(unsigned int) hdr.h.v1.sem_ver.revision,
			(unsigned int) hdr.h.v1.sem_ver.build_num);
	shell_print(shell, "  image size: %u",
			(unsigned int) hdr.h.v1.image_size);

	shell_print(shell, "  magic: %s",
		    swap_state_magic_str(swap_state.magic));

	if (IS_ENABLED(CONFIG_MCUBOOT_TRAILER_SWAP_TYPE)) {
		shell_print(shell, "  swap type: %s",
			    swap_type_str(swap_state.swap_type));
	}

	shell_print(shell, "  copy done: %s",
		    swap_state_flag_str(swap_state.copy_done));
	shell_print(shell, "  image ok: %s",
		    swap_state_flag_str(swap_state.image_ok));

	return 0;
}

static int cmd_mcuboot_info(const struct shell *shell, size_t argc,
			    char **argv)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(areas); i++) {
		shell_print(shell, "");
		cmd_mcuboot_info_area(shell, &areas[i]);
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mcuboot_cmds,
	SHELL_CMD_ARG(confirm, NULL, "confirm", cmd_mcuboot_confirm, 1, 0),
	SHELL_CMD_ARG(erase, NULL, "erase <area_id> | [unused] (erases non-boot image)", cmd_mcuboot_erase, 2, 0),
#ifndef CONFIG_BOARD_SCORPIO
	SHELL_CMD_ARG(request_upgrade, NULL, "request_upgrade [permanent]",
		      cmd_mcuboot_request_upgrade, 1, 1),
#endif
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(mcuboot, &mcuboot_cmds, "MCUboot commands",
		   cmd_mcuboot_info);
