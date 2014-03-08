/*
 * Firebee board - http://acp.atari.org/
 * based on dummy_68k.c by CodeSourcery,
 * see copyright below.
 *
 * Copyright (c) 2014 Gregory Estrade.
 *
 * This code is licensed under the GPL
 *
 */
/*
 * Dummy board with just RAM and CPU for use as an ISS.
 *
 * Copyright (c) 2007 CodeSourcery.
 *
 * This code is licensed under the GPL
 */

#include "hw/hw.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/block/flash.h"
#include "hw/devices.h"
#include "elf.h"
#include "exec/address-spaces.h"
#include "sysemu/qtest.h"

#define KERNEL_LOAD_ADDR 0x10000

#define RAM_SIZE 0x20000000

#define FLASH_BASE 0xe0000000
#define FLASH_SECTOR_LEN (128 * 1024)
#define FLASH_SIZE 0x20000000

/* Board init.  */

static void firebee_m68k_init(QEMUMachineInitArgs *args)
{
    ram_addr_t ram_size = RAM_SIZE;
    const char *cpu_model = args->cpu_model;
    const char *kernel_filename = args->kernel_filename;
    CPUM68KState *env;
    MemoryRegion *address_space_mem =  get_system_memory();
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    int kernel_size;
    uint64_t elf_entry;
    hwaddr entry;
    DriveInfo *dinfo;
    int be;

    if (!cpu_model)
        cpu_model = "cfv4e";
    env = cpu_init(cpu_model);
    if (!env) {
        fprintf(stderr, "Unable to find m68k CPU definition\n");
        exit(1);
    }

    /* Initialize CPU registers.  */
    env->vbr = 0;

    /* RAM at address zero */
    memory_region_init_ram(ram, NULL, "firebee_sdram.ram", ram_size);
    vmstate_register_ram_global(ram);
    memory_region_add_subregion(address_space_mem, 0, ram);

    /* Flash */
    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (!dinfo && !qtest_enabled()) {
        fprintf(stderr, "A flash image must be given with the "
                "'pflash' parameter\n");
        exit(1);
    }

    be = 0;
    if (!pflash_cfi01_register(FLASH_BASE, NULL, "firebee.rom", FLASH_SIZE,
                               dinfo ? dinfo->bdrv : NULL,
                               FLASH_SECTOR_LEN, FLASH_SIZE / FLASH_SECTOR_LEN,
                               2, 0, 0, 0, 0, be)) {
        fprintf(stderr, "qemu: Error registering flash memory.\n");
        exit(1);
    }

    /* Load kernel.  */
    if (kernel_filename) {
        kernel_size = load_elf(kernel_filename, NULL, NULL, &elf_entry,
                               NULL, NULL, 1, ELF_MACHINE, 0);
        entry = elf_entry;
        if (kernel_size < 0) {
            kernel_size = load_uimage(kernel_filename, &entry, NULL, NULL);
        }
        if (kernel_size < 0) {
            kernel_size = load_image_targphys(kernel_filename,
                                              KERNEL_LOAD_ADDR,
                                              ram_size - KERNEL_LOAD_ADDR);
            entry = KERNEL_LOAD_ADDR;
        }
        if (kernel_size < 0) {
            fprintf(stderr, "qemu: could not load kernel '%s'\n",
                    kernel_filename);
            exit(1);
        }
    } else {
        entry = FLASH_BASE;
    }
    env->pc = entry;
}

static QEMUMachine firebee_m68k_machine = {
    .name = "firebee",
    .desc = "Firebee",
    .init = firebee_m68k_init,
};

static void firebee_m68k_machine_init(void)
{
    qemu_register_machine(&firebee_m68k_machine);
}

machine_init(firebee_m68k_machine_init);
