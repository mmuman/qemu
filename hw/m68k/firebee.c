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
#include "hw/m68k/mcf.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/block/flash.h"
#include "hw/devices.h"
#include "qemu/timer.h"
#include "hw/ptimer.h"
#include "sysemu/sysemu.h"
#include "elf.h"
#include "exec/address-spaces.h"
#include "sysemu/qtest.h"

#define SYS_FREQ 33000000

#define KERNEL_LOAD_ADDR 0x10000

#define RAM_SIZE 0x20000000

#define FLASH_BASE 0xe0000000
#define FLASH_SECTOR_SIZE (128 * 1024)
#define FLASH_SIZE (8 * 1024 * 1024)

/* Slice Timers (SLT) */

#define MCF_SLT_RUN (1 << 26)
#define MCF_SLT_IEN (1 << 25)
#define MCF_SLT_TEN (1 << 24)

#define MCF_SLT_BE (1 << 25)
#define MCF_SLT_ST (1 << 24)

typedef struct {
    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state *timer;
    uint32_t tcnt;
    uint32_t cr;
    uint32_t sr;
} mcf_slt_state;

static void mcf_slt_reset(mcf_slt_state *s)
{
    s->tcnt = 0;
    s->cr = 0;
    s->sr = 0;
}

static void mcf_slt_update(mcf_slt_state *s)
{
    if ((s->cr & MCF_SLT_IEN) && (s->sr & MCF_SLT_ST))
        qemu_irq_raise(s->irq);
    else
        qemu_irq_lower(s->irq);
}

static void mcf_slt_write(void *opaque, hwaddr addr,
                          uint64_t value, unsigned size)
{
    mcf_slt_state *s = (mcf_slt_state *)opaque;
    switch (addr & 0xf) {
    case 0x0:
        s->tcnt = value;
        s->sr &= ~MCF_SLT_ST;
        mcf_slt_update(s);
        ptimer_stop(s->timer);
        ptimer_set_limit(s->timer, s->tcnt, 1);
        if (s->cr & MCF_SLT_TEN) {
            ptimer_run(s->timer, !(s->cr & MCF_SLT_RUN));
        }
        break;
    case 0x4:
        s->cr &= ~(MCF_SLT_RUN | MCF_SLT_IEN | MCF_SLT_TEN);
        value &= (MCF_SLT_RUN | MCF_SLT_IEN | MCF_SLT_TEN);
        s->cr |= value;
        
        ptimer_stop(s->timer);
        if (s->cr & MCF_SLT_TEN) {
            ptimer_run(s->timer, !(s->cr & MCF_SLT_RUN));   
        } else {
            ptimer_set_limit(s->timer, s->tcnt, 1);
        }
        mcf_slt_update(s);
        break;
    case 0x8:
        break;
    case 0xc:
        if (value & MCF_SLT_BE) {
            s->sr &= ~MCF_SLT_BE;
        }
        if (value & MCF_SLT_ST) {
            s->sr &= ~MCF_SLT_ST;
            mcf_slt_update(s);
        }
        break;
    default:
        hw_error("mcf_slt_write: Bad offset 0x%x\n", (int)addr);
    }
}

static uint64_t mcf_slt_read(void *opaque, hwaddr addr,
                             unsigned size)
{
    mcf_slt_state *s = (mcf_slt_state *)opaque;
    switch (addr & 0xf) {
    case 0x0:
        return s->tcnt;
    case 0x4:
        return s->cr;
    case 0x8:
        return ptimer_get_count(s->timer);
    case 0xc:
        return s->sr;
    default:
        hw_error("mcf_slt_read: Bad offset 0x%x\n", (int)addr);
        return 0;
    }
}

static const MemoryRegionOps mcf_slt_ops = {
    .read = mcf_slt_read,
    .write = mcf_slt_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void mcf_slt_trigger(void *opaque)
{
    mcf_slt_state *s = (mcf_slt_state *)opaque;
    s->sr |= MCF_SLT_ST;
    mcf_slt_update(s);
}


static void mcf_slt_mm_init(MemoryRegion *address_space, hwaddr base,
                            qemu_irq irq)
{
    mcf_slt_state *s;
    QEMUBH *bh;

    s = (mcf_slt_state *)g_malloc0(sizeof(mcf_slt_state));
    memory_region_init_io(&s->iomem, NULL, &mcf_slt_ops, s, "mcf_slt", 0x10);
    memory_region_add_subregion(address_space, base, &s->iomem);
    bh = qemu_bh_new(mcf_slt_trigger, s);
    s->timer = ptimer_init(bh);
    ptimer_set_freq(s->timer, SYS_FREQ);
    s->irq = irq;
    mcf_slt_reset(s);
}

/* Unmapped I/O */
static hwaddr unm_last = 0;
static int unm_count = 0;
static int fpga_done = 0;

static uint64_t firebee_unmapped_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    if (addr == 0xff000a27) {
        return (1 << 0) | (fpga_done << 5); 
    }
    if (unm_last != addr) {
        if (unm_count > 0) {
            fprintf(stderr, "\tx%d times\n", unm_count+1);    
        }
        fprintf(stderr, "Unmapped read @%lx(%d)\n", addr, size);
        unm_count = 0;
        unm_last = addr;
    } else {
        unm_count++;
        if (unm_count >= 500) {
            fprintf(stderr, "\tEndless loop detected, aborting.\n");
            exit(-1);   
        }
    }
    
    return 0;
}
static void firebee_unmapped_write(void *opaque, hwaddr addr,
                                   uint64_t value, unsigned size)
{
    if (addr == 0xff000a07) {
        if (value & (1 << 1)) {
            fpga_done = 1;
        }
        return;
    }
    fprintf(stderr, "Unmapped write @%lx(%d) %lx\n", addr, size, value);
    unm_last = addr;
    unm_count = 0;
}

static const MemoryRegionOps firebee_unmapped_ops = {
    .read = firebee_unmapped_read,
    .write = firebee_unmapped_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


/* Board init.  */

static void firebee_m68k_init(QEMUMachineInitArgs *args)
{
    ram_addr_t ram_size = RAM_SIZE;
    const char *cpu_model = args->cpu_model;
    const char *kernel_filename = args->kernel_filename;
    M68kCPU *cpu;
    CPUM68KState *env;
    MemoryRegion *address_space_mem =  get_system_memory();
    MemoryRegion *unmapped = g_new(MemoryRegion, 1);
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    MemoryRegion *ocram0 = g_new(MemoryRegion, 1);
    MemoryRegion *ocram1 = g_new(MemoryRegion, 1);
    MemoryRegion *sram = g_new(MemoryRegion, 1);

    /* MemoryRegion *wtf = g_new(MemoryRegion, 1); */

    int kernel_size;
    uint64_t elf_entry;
    hwaddr entry;
    DriveInfo *dinfo;
    int be;
    qemu_irq *pic;

    if (!cpu_model) {
        cpu_model = "any";
    }
    cpu = cpu_m68k_init(cpu_model);
    if (!cpu) {
        fprintf(stderr, "Unable to find m68k CPU definition\n");
        exit(1);
    }
    env = &cpu->env;

    /* Initialize CPU registers.  */
    env->vbr = 0;

    /* Unmapped */
    memory_region_init_io(unmapped, NULL, &firebee_unmapped_ops, NULL,
                          "firebee_unmapped.io", (1L << 32));
    memory_region_add_subregion(address_space_mem, 0, unmapped);

    /* SDRAM at address zero */
    memory_region_init_ram(ram, NULL, "firebee_sdram.ram", ram_size);
    vmstate_register_ram_global(ram);
    memory_region_add_subregion(address_space_mem, 0, ram);

    /* On-Chip RAM.  */
    memory_region_init_ram(ocram0, NULL, "firebee_ram0.ram", 0x1000);
    vmstate_register_ram_global(ocram0);
    memory_region_add_subregion(address_space_mem, 0xff100000, ocram0);
    
    memory_region_init_ram(ocram1, NULL, "firebee_ram1.ram", 0x1000);
    vmstate_register_ram_global(ocram1);
    memory_region_add_subregion(address_space_mem, 0xff101000, ocram1);

    /* 32Kb SRAM.  */
    memory_region_init_ram(sram, NULL, "firebee_sram.ram", 0x8000);
    vmstate_register_ram_global(sram);
    memory_region_add_subregion(address_space_mem, 0xff010000, sram);

    /* ??? */
    /* memory_region_init_ram(wtf, NULL, "wtf.ram", 0x10000);
    vmstate_register_ram_global(wtf);
    memory_region_add_subregion(address_space_mem, 0x20000000, wtf); */

    /* Internal peripherals.  */
    pic = mcf_intc_init(address_space_mem, 0xff000700, cpu);

    mcf_psc_mm_init(address_space_mem, 0xff008600, pic[35], serial_hds[0]);
    mcf_psc_mm_init(address_space_mem, 0xff008700, pic[34], serial_hds[1]);
    mcf_psc_mm_init(address_space_mem, 0xff008800, pic[33], serial_hds[2]);
    mcf_psc_mm_init(address_space_mem, 0xff008900, pic[32], serial_hds[3]);

    mcf_slt_mm_init(address_space_mem, 0xff000900, pic[54]);
    mcf_slt_mm_init(address_space_mem, 0xff000910, pic[53]);

    /* Flash */
    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (!dinfo && !qtest_enabled()) {
        fprintf(stderr, "A flash image must be given with the "
                "'pflash' parameter\n");
        exit(1);
    }

#ifdef TARGET_WORDS_BIGENDIAN
    be = 1;
#else
    be = 0;
#endif

    if (!pflash_cfi02_register(FLASH_BASE, NULL, "firebee.flash", FLASH_SIZE,
                               dinfo ? dinfo->bdrv : NULL, FLASH_SECTOR_SIZE,
                               FLASH_SIZE / FLASH_SECTOR_SIZE, 1,
                               2, 0x00BF, 0x236D, 0x0000, 0x0000,
                               0x5555, 0x2AAA, be)) {
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
