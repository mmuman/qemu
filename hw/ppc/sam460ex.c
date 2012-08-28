/*
 * QEMU PowerPC SAM460ex board emulation
 *
 * Copyright (c) 2012 François Revol
 *
 * This file is derived from hw/ppc440_bamboo.c,
 * the copyright for that material belongs to the original owners.
 *
 * This work is licensed under the GNU GPL license version 2 or later.
 *
 */

#include "config.h"
#include "qemu-common.h"
#include "net.h"
#include "hw/hw.h"
#include "hw/pc.h"
#include "hw/pci.h"
#include "blockdev.h"
#include "hw/boards.h"
#include "kvm.h"
#include "kvm_ppc.h"
#include "hw/devices.h"
#include "device_tree.h"
#include "hw/loader.h"
#include "elf.h"
#include "exec-memory.h"
#include "hw/ppc.h"
#include "hw/ppc4xx.h"
#include "hw/ppc405.h"
#include "hw/flash.h"
#include "sysemu.h"
#include "hw/sysbus.h"

#define BINARY_DEVICE_TREE_FILE "sam460ex.dtb"
#define UBOOT_FILENAME "u-boot-sam460-20100605-rebuilt.bin"
/* to extract the official U-Boot bin from the updater: */
/* dd if=updater/updater-460 bs=1 skip=$(($(stat -c '%s' updater/updater-460) - 0x80000)) of=u-boot-sam460-20100605.bin */

/* FIXME: I/O addresses are U-Boot's virtual addresses for now */

/* from Sam460 U-Boot include/configs/Sam460ex.h */
#define FLASH_BASE             0xfff00000
#define FLASH_BASE_H           0x4
#define FLASH_SIZE             (1 << 20)
#define UBOOT_LOAD_BASE        0xfff80000
#define UBOOT_SIZE             0x00080000
#define UBOOT_ENTRY            0xfffffffc

#define SM501_VRAM_SIZE        0x800000

/* from U-Boot */
#define EPAPR_MAGIC           (0x45504150)
#define KERNEL_ADDR           0x1000000
#define FDT_ADDR              0x1800000
#define RAMDISK_ADDR          0x1900000

/* TODO: factor this out to a header */
#define PPC440EP_PCI_CONFIG   0xeec00000
#define PPC440EP_PCI_INTACK   0xeed00000
#define PPC440EP_PCI_SPECIAL  0xeed00000
#define PPC440EP_PCI_REGS     0xef400000
#define PPC440EP_PCI_IO       0xe8000000
#define PPC440EP_PCI_IOLEN    0x00010000

#define PPC440EP_SDRAM_NR_BANKS 4

static const unsigned int ppc440ep_sdram_bank_sizes[] = {
    256<<20, 128<<20, 64<<20, 32<<20, 16<<20, 8<<20, 0
};

struct boot_info
{
    uint32_t dt_base;
    uint32_t dt_size;
    uint32_t entry;
};

static int sam460ex_load_uboot(void)
{
    DriveInfo *dinfo;
    BlockDriverState *dstate = NULL;
    target_phys_addr_t base = FLASH_BASE | ((target_phys_addr_t)FLASH_BASE_H << 32);

    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
        dstate = dinfo->bdrv;
    }

    if (!pflash_cfi01_register(base, NULL, "sam460ex.flash", FLASH_SIZE,
                               dstate, (64 * 1024),
                               FLASH_SIZE >> 16,
                               1, 0x89, 0x18, 0x0000, 0x0, 1)) {
        fprintf(stderr, "qemu: Error registering flash memory.\n");
        /* XXX: return an eror instead? */
        exit(1);
    }

    if (!dstate) {
        /*fprintf(stderr, "No flash image given with the 'pflash' parameter,"
                " using default u-boot image\n");*/
        base = UBOOT_LOAD_BASE | ((target_phys_addr_t)FLASH_BASE_H << 32);
        rom_add_file_fixed(UBOOT_FILENAME, base, -1);
    }

    return 0;
}

static int sam460ex_load_device_tree(target_phys_addr_t addr,
                                     uint32_t ramsize,
                                     target_phys_addr_t initrd_base,
                                     target_phys_addr_t initrd_size,
                                     const char *kernel_cmdline)
{
    int ret = -1;
    uint32_t mem_reg_property[] = { 0, 0, ramsize };
    char *filename;
    int fdt_size;
    void *fdt;
    uint32_t tb_freq = 400000000;
    uint32_t clock_freq = 400000000;

    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, BINARY_DEVICE_TREE_FILE);
    if (!filename) {
        goto out;
    }
    fdt = load_device_tree(filename, &fdt_size);
    g_free(filename);
    if (fdt == NULL) {
        goto out;
    }

    /* Manipulate device tree in memory. */

    ret = qemu_devtree_setprop(fdt, "/memory", "reg", mem_reg_property,
                               sizeof(mem_reg_property));
    if (ret < 0)
        fprintf(stderr, "couldn't set /memory/reg\n");

    /* default FDT doesn't have a /chosen node... */
    qemu_devtree_add_subnode(fdt, "/chosen");

    ret = qemu_devtree_setprop_cell(fdt, "/chosen", "linux,initrd-start",
                                    initrd_base);
    if (ret < 0)
        fprintf(stderr, "couldn't set /chosen/linux,initrd-start\n");

    ret = qemu_devtree_setprop_cell(fdt, "/chosen", "linux,initrd-end",
                                    (initrd_base + initrd_size));
    if (ret < 0)
        fprintf(stderr, "couldn't set /chosen/linux,initrd-end\n");

    ret = qemu_devtree_setprop_string(fdt, "/chosen", "bootargs",
                                      kernel_cmdline);
    if (ret < 0)
        fprintf(stderr, "couldn't set /chosen/bootargs\n");

    /* Copy data from the host device tree into the guest. Since the guest can
     * directly access the timebase without host involvement, we must expose
     * the correct frequencies. */
    if (kvm_enabled()) {
        tb_freq = kvmppc_get_tbfreq();
        clock_freq = kvmppc_get_clockfreq();
    }

    qemu_devtree_setprop_cell(fdt, "/cpus/cpu@0", "clock-frequency",
                              clock_freq);
    qemu_devtree_setprop_cell(fdt, "/cpus/cpu@0", "timebase-frequency",
                              tb_freq);

    ret = rom_add_blob_fixed(BINARY_DEVICE_TREE_FILE, fdt, fdt_size, addr);
    if (ret < 0) {
        goto out;
    }
    g_free(fdt);
    ret = fdt_size;

out:

    return ret;
}

/* Create reset TLB entries for BookE, mapping only the flash memory.  */
static void mmubooke_create_initial_mapping_uboot(CPUPPCState *env)
{
    ppcemb_tlb_t *tlb = &env->tlb.tlbe[0];

    /* on reset the flash is mapped by a shadow TLB,
     * but since we don't implement them we need to use
     * the same values U-Boot will use to avoid a fault.
     */
    tlb->attr = 0;
    tlb->prot = PAGE_VALID | ((PAGE_READ | PAGE_WRITE | PAGE_EXEC) << 4);
    tlb->size = 0x10000000; /* up to 0xffffffff  */
    tlb->EPN = 0xf0000000 & TARGET_PAGE_MASK;
    tlb->RPN = (0xf0000000 & TARGET_PAGE_MASK) | 0x4;
    tlb->PID = 0;
}

/* Create reset TLB entries for BookE, spanning the 32bit addr space.  */
static void mmubooke_create_initial_mapping(CPUPPCState *env,
                                     target_ulong va,
                                     target_phys_addr_t pa)
{
    //struct boot_info *bi = env->load_info;
    ppcemb_tlb_t *tlb = &env->tlb.tlbe[0];

    tlb->attr = 0;
    tlb->prot = PAGE_VALID | ((PAGE_READ | PAGE_WRITE | PAGE_EXEC) << 4);
    tlb->size = 1 << 31; /* up to 0x80000000  */
    tlb->EPN = va & TARGET_PAGE_MASK;
    tlb->RPN = pa & TARGET_PAGE_MASK;
    tlb->PID = 0;

    tlb = &env->tlb.tlbe[1];
    tlb->attr = 0;
    tlb->prot = PAGE_VALID | ((PAGE_READ | PAGE_WRITE | PAGE_EXEC) << 4);
    tlb->size = 1 << 31; /* up to 0xffffffff  */
    tlb->EPN = 0x80000000 & TARGET_PAGE_MASK;
    tlb->RPN = 0x80000000 & TARGET_PAGE_MASK;
    tlb->PID = 0;
}

static void main_cpu_reset(void *opaque)
{
    PowerPCCPU *cpu = opaque;
    CPUPPCState *env = &cpu->env;
    struct boot_info *bi = env->load_info;

    cpu_reset(CPU(cpu));

	/* either we have a kernel to boot or we jump to U-Boot */
    if (bi->entry != UBOOT_ENTRY) {
        env->gpr[1] = (16<<20) - 8;
        env->gpr[3] = FDT_ADDR;

        fprintf(stderr, "cpu reset: kernel entry %x\n", bi->entry);
        env->nip = bi->entry;

        /* Create a mapping for the kernel.  */
        mmubooke_create_initial_mapping(env, 0, 0);
        env->gpr[6] = tswap32(EPAPR_MAGIC);
        env->gpr[7] = (16<<20) - 8;//bi->ima_size;

    } else {
        env->nip = UBOOT_ENTRY;
        mmubooke_create_initial_mapping_uboot(env);
        fprintf(stderr, "cpu reset: U-Boot entry\n");
    }
}

static void sam460ex_init(ram_addr_t ram_size,
                        const char *boot_device,
                        const char *kernel_filename,
                        const char *kernel_cmdline,
                        const char *initrd_filename,
                        const char *cpu_model)
{
    unsigned int pci_irq_nrs[4] = { 28, 27, 26, 25 };
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *ram_memories
        = g_malloc(PPC440EP_SDRAM_NR_BANKS * sizeof(*ram_memories));
    target_phys_addr_t ram_bases[PPC440EP_SDRAM_NR_BANKS];
    target_phys_addr_t ram_sizes[PPC440EP_SDRAM_NR_BANKS];
    qemu_irq *pic;
    qemu_irq *irqs;
    PCIBus *pcibus;
    PowerPCCPU *cpu;
    CPUPPCState *env;
    uint64_t elf_entry;
    uint64_t elf_lowaddr;
    target_phys_addr_t entry = UBOOT_ENTRY;
    target_phys_addr_t loadaddr = 0;
    target_long initrd_size = 0;
    DeviceState *dev;
    int success;
    int i;
    struct boot_info *boot_info;

    /* Setup CPU. */
    if (cpu_model == NULL) {
        cpu_model = "440EP";
    }
    cpu = cpu_ppc_init(cpu_model);
    if (cpu == NULL) {
        fprintf(stderr, "Unable to initialize CPU!\n");
        exit(1);
    }
    env = &cpu->env;

    qemu_register_reset(main_cpu_reset, cpu);
    boot_info = g_malloc0(sizeof(struct boot_info));
    env->load_info = boot_info;

    ppc_booke_timers_init(env, 400000000, 0);
    ppc_dcr_init(env, NULL, NULL);

    /* interrupt controller */
    irqs = g_malloc0(sizeof(qemu_irq) * PPCUIC_OUTPUT_NB);
    irqs[PPCUIC_OUTPUT_INT] = ((qemu_irq *)env->irq_inputs)[PPC40x_INPUT_INT];
    irqs[PPCUIC_OUTPUT_CINT] = ((qemu_irq *)env->irq_inputs)[PPC40x_INPUT_CINT];
    pic = ppcuic_init(env, irqs, 0x0C0, 0, 1);

    /* SDRAM controller */
    memset(ram_bases, 0, sizeof(ram_bases));
    memset(ram_sizes, 0, sizeof(ram_sizes));
    ram_size = ppc4xx_sdram_adjust(ram_size, PPC440EP_SDRAM_NR_BANKS,
                                   ram_memories,
                                   ram_bases, ram_sizes,
                                   ppc440ep_sdram_bank_sizes);
    /* XXX 440EP's ECC interrupts are on UIC1, but we've only created UIC0. */
    ppc4xx_sdram_init(env, pic[14], PPC440EP_SDRAM_NR_BANKS, ram_memories,
                      ram_bases, ram_sizes, 1);

    /* PCI */
    dev = sysbus_create_varargs("ppc4xx-pcihost", PPC440EP_PCI_CONFIG,
                                pic[pci_irq_nrs[0]], pic[pci_irq_nrs[1]],
                                pic[pci_irq_nrs[2]], pic[pci_irq_nrs[3]],
                                NULL);
    pcibus = (PCIBus *)qdev_get_child_bus(dev, "pci.0");
    if (!pcibus) {
        fprintf(stderr, "couldn't create PCI controller!\n");
        exit(1);
    }

    sm501_init(address_space_mem, 0x10000000, SM501_VRAM_SIZE,
               /*irq[SM501]FIXME*/pic[13], serial_hds[2]);


    isa_mmio_init(PPC440EP_PCI_IO, PPC440EP_PCI_IOLEN);

    /* Core has 4 UARTs but the board only has one wired */
    if (serial_hds[0] != NULL) {
        serial_mm_init(address_space_mem, 0xef600300, 0, pic[0],
                       PPC_SERIAL_MM_BAUDBASE, serial_hds[0],
                       DEVICE_BIG_ENDIAN);
    }

    if (pcibus) {
        /* Register network interfaces. */
        for (i = 0; i < nb_nics; i++) {
            /* There are no PCI NICs on the Bamboo board, but there are
             * PCI slots, so we can pick whatever default model we want. */
            pci_nic_init_nofail(&nd_table[i], "e1000", NULL);
        }
    }

    /* Load U-Boot image. */
    if (!kernel_filename) {
        success = sam460ex_load_uboot();
        if (success < 0) {
            fprintf(stderr, "qemu: could not load firmware\n");
            exit(1);
        }
    }

    /* Load kernel. */
    if (kernel_filename) {
        success = load_uimage(kernel_filename, &entry, &loadaddr, NULL);
        fprintf(stderr, "load_uimage: %d\n", success);
        if (success < 0) {
            success = load_elf(kernel_filename, NULL, NULL, &elf_entry,
                               &elf_lowaddr, NULL, 1, ELF_MACHINE, 0);
            entry = elf_entry;
            loadaddr = elf_lowaddr;
        }
        /* XXX try again as binary */
        if (success < 0) {
            fprintf(stderr, "qemu: could not load kernel '%s'\n",
                    kernel_filename);
            exit(1);
        }
    }

    /* Load initrd. */
    if (initrd_filename) {
        initrd_size = load_image_targphys(initrd_filename, RAMDISK_ADDR,
                                          ram_size - RAMDISK_ADDR);

        if (initrd_size < 0) {
            fprintf(stderr, "qemu: could not load ram disk '%s' at %x\n",
                    initrd_filename, RAMDISK_ADDR);
            exit(1);
        }
    }

    /* If we're loading a kernel directly, we must load the device tree too. */
    if (kernel_filename) {
        int dt_size;

        dt_size = sam460ex_load_device_tree(FDT_ADDR, ram_size, RAMDISK_ADDR,
                                    initrd_size, kernel_cmdline);
        if (dt_size < 0) {
            fprintf(stderr, "couldn't load device tree\n");
            exit(1);
        }

        boot_info->dt_base = FDT_ADDR;
        boot_info->dt_size = dt_size;
    }

    boot_info->entry = entry;

    if (kvm_enabled())
        kvmppc_init();
}

static QEMUMachine sam460ex_machine = {
    .name = "sam460ex",
    .desc = "aCube Sam460ex",
    .init = sam460ex_init,
};

static void sam460ex_machine_init(void)
{
    qemu_register_machine(&sam460ex_machine);
}

machine_init(sam460ex_machine_init);
