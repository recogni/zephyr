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
	{"Primary", FLASH_AREA_IMAGE_PRIMARY},
#ifdef FLASH_AREA_IMAGE_SECONDARY
	{"Secondary", FLASH_AREA_IMAGE_SECONDARY},
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

#ifndef CONFIG_BOARD_SCORPIO
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
#endif

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

static int flash_area_id_from_slot_id(const int slot_id)
{
	// The flash area ID for a given slot is one more than
	// the "slot_id" that is used by mcuboot etc.
	return slot_id + 1;
}

static int slot_id_from_flash_area_id(const int flash_area_id)
{
	// The flash area ID for a given slot is one more than
	// the "slot_id" that is used by mcuboot etc.
	return flash_area_id - 1;
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
						shell_print(shell, "Erasing %s area (slot=%u)", area->name, slot_id_from_flash_area_id(area->id));
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
		id = flash_area_id_from_slot_id(strtoul(argv[1], NULL, 0));
	}

	if (id != UNFOUND_SLOT) 
	{
		err = boot_erase_img_bank(id);
		if (err) {
			shell_error(shell, "failed to erase slot %u", slot_id_from_flash_area_id(id));
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

#if !defined(CONFIG_BOARD_SCORPIO)
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
#endif // !defined(CONFIG_BOARD_SCORPIO)

static int cmd_mcuboot_info_area(const struct shell *shell,
				 const struct area_desc *area, int verbose)
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
		shell_error(shell, "failed to read %s area (slot=%u) %s: %d",
			    area->name, slot_id_from_flash_area_id(area->id), "swap state", err);
		return err;
	}
    if (!verbose)
    {
        if (swap_state.magic == BOOT_MAGIC_GOOD)
        {
            shell_print(shell, "Slot %u: %u.%u.%u+%u %s %s %s",
                    slot_id_from_flash_area_id(area->id),
                    (unsigned int) hdr.h.v1.sem_ver.major,
                    (unsigned int) hdr.h.v1.sem_ver.minor,
                    (unsigned int) hdr.h.v1.sem_ver.revision,
                    (unsigned int) hdr.h.v1.sem_ver.build_num,
                    swap_state.image_ok == BOOT_FLAG_SET ? "Confirmed" : "Unconfirmed",
                    swap_state.copy_done == BOOT_FLAG_SET ? "Booted" : "      ",
                    (swap_state.copy_done == BOOT_FLAG_SET && swap_state.image_ok != BOOT_FLAG_SET) ? "WARNING!: This image may be removed on next reboot unless confirmed!" : "");
        }
    } else {
        if (swap_state.copy_done == BOOT_FLAG_SET)
        {
            if (swap_state.image_ok == BOOT_FLAG_SET)
                shell_error(shell, "Slot %u:  Booted/Confirmed", slot_id_from_flash_area_id(area->id));
            else
                shell_error(shell, "Slot %u:  Booted/Unconfirmed (WARNING: image may be removed on next boot unless confirmed)", slot_id_from_flash_area_id(area->id));
        } else {
            shell_print(shell, "Slot %u:  %s", slot_id_from_flash_area_id(area->id), swap_state.image_ok == BOOT_FLAG_SET ? "Confirmed" : "Unconfirmed");
        }

        shell_print(shell, "  Version: %u.%u.%u+%u",
                (unsigned int) hdr.h.v1.sem_ver.major,
                (unsigned int) hdr.h.v1.sem_ver.minor,
                (unsigned int) hdr.h.v1.sem_ver.revision,
                (unsigned int) hdr.h.v1.sem_ver.build_num);
        shell_print(shell, "  Image size: %uK",
                (unsigned int) hdr.h.v1.image_size/1024);

        shell_print(shell, "  Magic: %s",
                swap_state_magic_str(swap_state.magic));

    #ifndef CONFIG_BOARD_SCORPIO
        if (IS_ENABLED(CONFIG_MCUBOOT_TRAILER_SWAP_TYPE)) {
            shell_print(shell, "  swap type: %s",
                    swap_type_str(swap_state.swap_type));
        }
    #endif

        shell_print(shell, "  Booted: %s",
                swap_state_flag_str(swap_state.copy_done));
        shell_print(shell, "  Confirmed: %s",
                swap_state_flag_str(swap_state.image_ok));
    }
	return 0;
}

static int cmd_mcuboot_info(const struct shell *shell, size_t argc,
			    char **argv)
{
	int i;
    int verbose = 0;

    if ((argc > 1) && strncmp(argv[1], "-v", 2) == 0) 
    {
        verbose = 1;
    }
	for (i = 0; i < ARRAY_SIZE(areas); i++) {
		cmd_mcuboot_info_area(shell, &areas[i], verbose);
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(mcuboot_cmds,
	SHELL_CMD_ARG(confirm, NULL, "confirm", cmd_mcuboot_confirm, 1, 0),
	SHELL_CMD_ARG(erase, NULL, "erase <slot_id> | [unused] (erases non-boot image)", cmd_mcuboot_erase, 2, 0),
	SHELL_CMD_ARG(info, NULL, "info <-v> verbose", cmd_mcuboot_info, 1, 1),
#ifndef CONFIG_BOARD_SCORPIO
	SHELL_CMD_ARG(request_upgrade, NULL, "request_upgrade [permanent]",
		      cmd_mcuboot_request_upgrade, 1, 1),
#endif
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(mcuboot, &mcuboot_cmds, "MCUboot commands", NULL);
