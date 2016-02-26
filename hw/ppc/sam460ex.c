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

#include "qemu/osdep.h"
#include "config.h"
#include "qemu-common.h"
#include "net/net.h"
#include "hw/hw.h"
#include "hw/i386/pc.h"
#include "hw/pci/pci.h"
#include "sysemu/blockdev.h"
#include "hw/boards.h"
#include "sysemu/kvm.h"
#include "kvm_ppc.h"
#include "hw/devices.h"
#include "sysemu/device_tree.h"
#include "sysemu/block-backend.h"
#include "hw/loader.h"
#include "elf.h"
#include "exec/address-spaces.h"
#include "exec/memory.h"
#include "hw/ppc/ppc.h"
#include "hw/ppc/ppc4xx.h"
#include "hw/ppc/ppc405.h"
#include "hw/block/flash.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "hw/i2c/i2c.h"

/*
#define DEBUG_L2SRAM
#define DEBUG_EBC
#define DEBUG_SDR
#define DEBUG_PLB
#define DEBUG_AHB
*/
#define DEBUG_I2C

#define BINARY_DEVICE_TREE_FILE "sam460ex.dtb"
#define UBOOT_FILENAME "u-boot-sam460-20100605-rebuilt.bin"
//#define UBOOT_FILENAME "u-boot-sam460-20100605.bin"
/* to extract the official U-Boot bin from the updater: */
/* dd if=updater/updater-460 bs=1 skip=$(($(stat -c '%s' updater/updater-460) - 0x80000)) of=u-boot-sam460-20100605.bin */

/* FIXME: I/O addresses are U-Boot's virtual addresses for now */

/*  */
#define RPA36(epa,pa) (((hwaddr)epa << 32) | pa)

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
    // Sam460ex IRQ MAP:
    // IRQ0  = ETH_INT
    // IRQ1  = FPGA_INT
    // IRQ2  = PCI_INT (PCIA, PCIB, PCIC, PCIB)
    // IRQ3  = FPGA_INT2
    // IRQ11 = RTC_INT
    // IRQ12 = SM502_INT

/* TODO: factor this out to a header */
#define PPC440EP_PCI_CONFIG   0xeec00000
#define PPC440EP_PCI_INTACK   0xeed00000
#define PPC440EP_PCI_SPECIAL  0xeed00000
#define PPC440EP_PCI_REGS     0xef400000
#define PPC440EP_PCI_IO       0xe8000000

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

/*****************************************************************************/
/* L2 Cache as SRAM */
#if 1
/*FIXME:fix names*/
enum {
    DCR_L2CACHE_BASE  = 0x030,
    DCR_L2CACHE_CFG   = DCR_L2CACHE_BASE,
    DCR_L2CACHE_CMD,
    DCR_L2CACHE_ADDR,
    DCR_L2CACHE_DATA,
    DCR_L2CACHE_STAT,
    DCR_L2CACHE_CVER,
    DCR_L2CACHE_SNP0,
    DCR_L2CACHE_SNP1,
    DCR_L2CACHE_END   = DCR_L2CACHE_SNP1,
};

/* XXX:base is 460ex-specific, cf. U-Boot, ppc4xx-isram.h */
enum {
    DCR_ISRAM0_BASE   = 0x020,
    DCR_ISRAM0_SB0CR  = DCR_ISRAM0_BASE,
    DCR_ISRAM0_SB1CR,
    DCR_ISRAM0_SB2CR,
    DCR_ISRAM0_SB3CR,
    DCR_ISRAM0_BEAR,
    DCR_ISRAM0_BESR0,
    DCR_ISRAM0_BESR1,
    DCR_ISRAM0_PMEG,
    DCR_ISRAM0_CID,
    DCR_ISRAM0_REVID,
    DCR_ISRAM0_DPC,
    DCR_ISRAM0_END    = DCR_ISRAM0_DPC
};

/* OCM */
enum {
    DCR_ISRAM1_BASE   = 0x0b0,
    DCR_ISRAM1_SB0CR  = DCR_ISRAM1_BASE,
    /* single bank */
    DCR_ISRAM1_BEAR   = DCR_ISRAM1_BASE + 0x04,
    DCR_ISRAM1_BESR0,
    DCR_ISRAM1_BESR1,
    DCR_ISRAM1_PMEG,
    DCR_ISRAM1_CID,
    DCR_ISRAM1_REVID,
    DCR_ISRAM1_DPC,
    DCR_ISRAM1_END    = DCR_ISRAM1_DPC
};

typedef struct ppc4xx_l2sram_t ppc4xx_l2sram_t;
struct ppc4xx_l2sram_t {
    MemoryRegion bank[4];
    uint32_t l2cache[8];
    uint32_t isram0[11];
};

#if 0
static void l2sram_update_mappings (ppc4xx_l2sram_t *l2sram,
                                 uint32_t isarc, uint32_t isacntl,
                                 uint32_t dsarc, uint32_t dsacntl)
{
#ifdef DEBUG_L2SRAM
    printf("L2SRAM update ISA %08" PRIx32 " %08" PRIx32 " (%08" PRIx32
           " %08" PRIx32 ") DSA %08" PRIx32 " %08" PRIx32
           " (%08" PRIx32 " %08" PRIx32 ")\n",
           isarc, isacntl, dsarc, dsacntl,
           l2sram->isarc, l2sram->isacntl, l2sram->dsarc, l2sram->dsacntl);
#endif
    if (l2sram->isarc != isarc ||
        (l2sram->isacntl & 0x80000000) != (isacntl & 0x80000000)) {
        if (l2sram->isacntl & 0x80000000) {
            /* Unmap previously assigned memory region */
            printf("L2SRAM unmap ISA %08" PRIx32 "\n", l2sram->isarc);
            memory_region_del_subregion(get_system_memory(), &l2sram->isarc_ram);
        }
        if (isacntl & 0x80000000) {
            /* Map new instruction memory region */
#ifdef DEBUG_L2SRAM
            printf("L2SRAM map ISA %08" PRIx32 "\n", isarc);
#endif
            memory_region_add_subregion(get_system_memory(), isarc,
                                        &l2sram->isarc_ram);
        }
    }
    if (l2sram->dsarc != dsarc ||
        (l2sram->dsacntl & 0x80000000) != (dsacntl & 0x80000000)) {
        if (l2sram->dsacntl & 0x80000000) {
            /* Beware not to unmap the region we just mapped */
            if (!(isacntl & 0x80000000) || l2sram->dsarc != isarc) {
                /* Unmap previously assigned memory region */
#ifdef DEBUG_L2SRAM
                printf("L2SRAM unmap DSA %08" PRIx32 "\n", l2sram->dsarc);
#endif
                memory_region_del_subregion(get_system_memory(),
                                            &l2sram->dsarc_ram);
            }
        }
        if (dsacntl & 0x80000000) {
            /* Beware not to remap the region we just mapped */
            if (!(isacntl & 0x80000000) || dsarc != isarc) {
                /* Map new data memory region */
#ifdef DEBUG_L2SRAM
                printf("L2SRAM map DSA %08" PRIx32 "\n", dsarc);
#endif
                memory_region_add_subregion(get_system_memory(), dsarc,
                                            &l2sram->dsarc_ram);
            }
        }
    }
}
#endif

static uint32_t dcr_read_l2sram (void *opaque, int dcrn)
{
    ppc4xx_l2sram_t *l2sram;
    uint32_t ret;

    l2sram = opaque;
    switch (dcrn) {
    case DCR_L2CACHE_CFG:
    case DCR_L2CACHE_CMD:
    case DCR_L2CACHE_ADDR:
    case DCR_L2CACHE_DATA:
    case DCR_L2CACHE_STAT:
    case DCR_L2CACHE_CVER:
    case DCR_L2CACHE_SNP0:
    case DCR_L2CACHE_SNP1:
        ret = l2sram->l2cache[dcrn - DCR_L2CACHE_BASE];
#ifdef DEBUG_L2SRAM
        printf("L2SRAM: read DCR[L2CACHE+%x]: %08" PRIx32 "\n",
               dcrn - DCR_L2CACHE_BASE, ret);
#endif
        break;

    case DCR_ISRAM0_SB0CR:
    case DCR_ISRAM0_SB1CR:
    case DCR_ISRAM0_SB2CR:
    case DCR_ISRAM0_SB3CR:
    case DCR_ISRAM0_BEAR:
    case DCR_ISRAM0_BESR0:
    case DCR_ISRAM0_BESR1:
    case DCR_ISRAM0_PMEG:
    case DCR_ISRAM0_CID:
    case DCR_ISRAM0_REVID:
    case DCR_ISRAM0_DPC:
        ret = l2sram->isram0[dcrn - DCR_ISRAM0_BASE];
#ifdef DEBUG_L2SRAM
        printf("L2SRAM: read DCR[ISRAM0+%x]: %08" PRIx32 "\n",
               dcrn - DCR_ISRAM0_BASE, ret);
#endif
        break;

    default:
        ret = 0;
        break;
    }

    return ret;
}

static void dcr_write_l2sram (void *opaque, int dcrn, uint32_t val)
{
    /*ppc4xx_l2sram_t *l2sram = opaque;*/
    /* TODO */

    switch (dcrn) {
    case DCR_L2CACHE_CFG:
    case DCR_L2CACHE_CMD:
    case DCR_L2CACHE_ADDR:
    case DCR_L2CACHE_DATA:
    case DCR_L2CACHE_STAT:
    case DCR_L2CACHE_CVER:
    case DCR_L2CACHE_SNP0:
    case DCR_L2CACHE_SNP1:
        //l2sram->l2cache[dcrn - DCR_L2CACHE_BASE] = val;
#ifdef DEBUG_L2SRAM
        printf("L2SRAM: write DCR[L2CACHE+%x]: %08" PRIx32 "\n",
               dcrn - DCR_L2CACHE_BASE, val);
#endif
        break;

    case DCR_ISRAM0_SB0CR:
    case DCR_ISRAM0_SB1CR:
    case DCR_ISRAM0_SB2CR:
    case DCR_ISRAM0_SB3CR:
    case DCR_ISRAM0_BEAR:
    case DCR_ISRAM0_BESR0:
    case DCR_ISRAM0_BESR1:
    case DCR_ISRAM0_PMEG:
    case DCR_ISRAM0_CID:
    case DCR_ISRAM0_REVID:
    case DCR_ISRAM0_DPC:
        //l2sram->isram0[dcrn - DCR_L2CACHE_BASE] = val;
#ifdef DEBUG_L2SRAM
        printf("L2SRAM: write DCR[ISRAM0+%x]: %08" PRIx32 "\n",
               dcrn - DCR_ISRAM0_BASE, val);
#endif
        break;

    case DCR_ISRAM1_SB0CR:
    case DCR_ISRAM1_BEAR:
    case DCR_ISRAM1_BESR0:
    case DCR_ISRAM1_BESR1:
    case DCR_ISRAM1_PMEG:
    case DCR_ISRAM1_CID:
    case DCR_ISRAM1_REVID:
    case DCR_ISRAM1_DPC:
        //l2sram->isram1[dcrn - DCR_L2CACHE_BASE] = val;
#ifdef DEBUG_L2SRAM
        printf("L2SRAM: write DCR[ISRAM1+%x]: %08" PRIx32 "\n",
               dcrn - DCR_ISRAM1_BASE, val);
#endif
        break;
    }
    //l2sram_update_mappings(l2sram, isarc, isacntl, dsarc, dsacntl);
}

static void l2sram_reset (void *opaque)
{
    ppc4xx_l2sram_t *l2sram;
//    uint32_t isarc, dsarc, isacntl, dsacntl;

    l2sram = opaque;
    memset(l2sram->l2cache, 0, sizeof(l2sram->l2cache));
    memset(l2sram->isram0, 0, sizeof(l2sram->isram0));
    //l2sram_update_mappings(l2sram, isarc, isacntl, dsarc, dsacntl);
}

static void ppc4xx_l2sram_init(CPUPPCState *env)
{
    ppc4xx_l2sram_t *l2sram;

    l2sram = g_malloc0(sizeof(ppc4xx_l2sram_t));
    /* XXX: Size is 4*64kB for 460ex, cf. U-Boot, ppc4xx-isram.h */
    memory_region_init_ram(&l2sram->bank[0], NULL, "ppc4xx.l2sram_bank0",
                           64 * 1024, &error_abort);
    vmstate_register_ram_global(&l2sram->bank[0]);
    memory_region_init_ram(&l2sram->bank[1], NULL, "ppc4xx.l2sram_bank1",
                           64 * 1024, &error_abort);
    vmstate_register_ram_global(&l2sram->bank[1]);
    memory_region_init_ram(&l2sram->bank[2], NULL, "ppc4xx.l2sram_bank2",
                           64 * 1024, &error_abort);
    vmstate_register_ram_global(&l2sram->bank[2]);
    memory_region_init_ram(&l2sram->bank[3], NULL, "ppc4xx.l2sram_bank3",
                           64 * 1024, &error_abort);
    vmstate_register_ram_global(&l2sram->bank[3]);
    qemu_register_reset(&l2sram_reset, l2sram);
    ppc_dcr_register(env, DCR_L2CACHE_CFG,
                     l2sram, &dcr_read_l2sram, &dcr_write_l2sram);

    ppc_dcr_register(env, DCR_ISRAM0_SB0CR,
                     l2sram, &dcr_read_l2sram, &dcr_write_l2sram);
    ppc_dcr_register(env, DCR_ISRAM0_SB1CR,
                     l2sram, &dcr_read_l2sram, &dcr_write_l2sram);
    ppc_dcr_register(env, DCR_ISRAM0_SB2CR,
                     l2sram, &dcr_read_l2sram, &dcr_write_l2sram);
    ppc_dcr_register(env, DCR_ISRAM0_SB3CR,
                     l2sram, &dcr_read_l2sram, &dcr_write_l2sram);
    ppc_dcr_register(env, DCR_ISRAM0_PMEG,
                     l2sram, &dcr_read_l2sram, &dcr_write_l2sram);
    ppc_dcr_register(env, DCR_ISRAM0_DPC,
                     l2sram, &dcr_read_l2sram, &dcr_write_l2sram);

    ppc_dcr_register(env, DCR_ISRAM1_SB0CR,
                     l2sram, &dcr_read_l2sram, &dcr_write_l2sram);
    ppc_dcr_register(env, DCR_ISRAM1_PMEG,
                     l2sram, &dcr_read_l2sram, &dcr_write_l2sram);
    ppc_dcr_register(env, DCR_ISRAM1_DPC,
                     l2sram, &dcr_read_l2sram, &dcr_write_l2sram);
}
#endif

/*****************************************************************************/
/* On Chip Memory */
#if 0
enum {
    OCM0_ISARC   = 0x018,
    OCM0_ISACNTL = 0x019,
    OCM0_DSARC   = 0x01A,
    OCM0_DSACNTL = 0x01B,
};

typedef struct ppc4xx_ocm_t ppc4xx_ocm_t;
struct ppc4xx_ocm_t {
    MemoryRegion ram;
    MemoryRegion isarc_ram;
    MemoryRegion dsarc_ram;
    uint32_t isarc;
    uint32_t isacntl;
    uint32_t dsarc;
    uint32_t dsacntl;
};

static void ocm_update_mappings (ppc4xx_ocm_t *ocm,
                                 uint32_t isarc, uint32_t isacntl,
                                 uint32_t dsarc, uint32_t dsacntl)
{
#ifdef DEBUG_OCM
    printf("OCM update ISA %08" PRIx32 " %08" PRIx32 " (%08" PRIx32
           " %08" PRIx32 ") DSA %08" PRIx32 " %08" PRIx32
           " (%08" PRIx32 " %08" PRIx32 ")\n",
           isarc, isacntl, dsarc, dsacntl,
           ocm->isarc, ocm->isacntl, ocm->dsarc, ocm->dsacntl);
#endif
    if (ocm->isarc != isarc ||
        (ocm->isacntl & 0x80000000) != (isacntl & 0x80000000)) {
        if (ocm->isacntl & 0x80000000) {
            /* Unmap previously assigned memory region */
            printf("OCM unmap ISA %08" PRIx32 "\n", ocm->isarc);
            memory_region_del_subregion(get_system_memory(), &ocm->isarc_ram);
        }
        if (isacntl & 0x80000000) {
            /* Map new instruction memory region */
#ifdef DEBUG_OCM
            printf("OCM map ISA %08" PRIx32 "\n", isarc);
#endif
            memory_region_add_subregion(get_system_memory(), isarc,
                                        &ocm->isarc_ram);
        }
    }
    if (ocm->dsarc != dsarc ||
        (ocm->dsacntl & 0x80000000) != (dsacntl & 0x80000000)) {
        if (ocm->dsacntl & 0x80000000) {
            /* Beware not to unmap the region we just mapped */
            if (!(isacntl & 0x80000000) || ocm->dsarc != isarc) {
                /* Unmap previously assigned memory region */
#ifdef DEBUG_OCM
                printf("OCM unmap DSA %08" PRIx32 "\n", ocm->dsarc);
#endif
                memory_region_del_subregion(get_system_memory(),
                                            &ocm->dsarc_ram);
            }
        }
        if (dsacntl & 0x80000000) {
            /* Beware not to remap the region we just mapped */
            if (!(isacntl & 0x80000000) || dsarc != isarc) {
                /* Map new data memory region */
#ifdef DEBUG_OCM
                printf("OCM map DSA %08" PRIx32 "\n", dsarc);
#endif
                memory_region_add_subregion(get_system_memory(), dsarc,
                                            &ocm->dsarc_ram);
            }
        }
    }
}

static uint32_t dcr_read_ocm (void *opaque, int dcrn)
{
    ppc4xx_ocm_t *ocm;
    uint32_t ret;

    ocm = opaque;
    switch (dcrn) {
    case OCM0_ISARC:
        ret = ocm->isarc;
        break;
    case OCM0_ISACNTL:
        ret = ocm->isacntl;
        break;
    case OCM0_DSARC:
        ret = ocm->dsarc;
        break;
    case OCM0_DSACNTL:
        ret = ocm->dsacntl;
        break;
    default:
        ret = 0;
        break;
    }

    return ret;
}

static void dcr_write_ocm (void *opaque, int dcrn, uint32_t val)
{
    ppc4xx_ocm_t *ocm;
    uint32_t isarc, dsarc, isacntl, dsacntl;

    ocm = opaque;
    isarc = ocm->isarc;
    dsarc = ocm->dsarc;
    isacntl = ocm->isacntl;
    dsacntl = ocm->dsacntl;
    switch (dcrn) {
    case OCM0_ISARC:
        isarc = val & 0xFC000000;
        break;
    case OCM0_ISACNTL:
        isacntl = val & 0xC0000000;
        break;
    case OCM0_DSARC:
        isarc = val & 0xFC000000;
        break;
    case OCM0_DSACNTL:
        isacntl = val & 0xC0000000;
        break;
    }
    ocm_update_mappings(ocm, isarc, isacntl, dsarc, dsacntl);
    ocm->isarc = isarc;
    ocm->dsarc = dsarc;
    ocm->isacntl = isacntl;
    ocm->dsacntl = dsacntl;
}

static void ocm_reset (void *opaque)
{
    ppc4xx_ocm_t *ocm;
    uint32_t isarc, dsarc, isacntl, dsacntl;

    ocm = opaque;
    isarc = 0x00000000;
    isacntl = 0x00000000;
    dsarc = 0x00000000;
    dsacntl = 0x00000000;
    ocm_update_mappings(ocm, isarc, isacntl, dsarc, dsacntl);
    ocm->isarc = isarc;
    ocm->dsarc = dsarc;
    ocm->isacntl = isacntl;
    ocm->dsacntl = dsacntl;
}

static void ppc4xx_ocm_init(CPUPPCState *env)
{
    ppc4xx_ocm_t *ocm;

    ocm = g_malloc0(sizeof(ppc4xx_ocm_t));
    /* XXX: Size is 4096 or 0x04000000 */
    memory_region_init_ram(&ocm->isarc_ram, "ppc4xx.ocm", 4096, &error_abort);
    vmstate_register_ram_global(&ocm->isarc_ram);
    memory_region_init_alias(&ocm->dsarc_ram, "ppc4xx.dsarc", &ocm->isarc_ram,
                             0, 4096);
    qemu_register_reset(&ocm_reset, ocm);
    ppc_dcr_register(env, OCM0_ISARC,
                     ocm, &dcr_read_ocm, &dcr_write_ocm);
    ppc_dcr_register(env, OCM0_ISACNTL,
                     ocm, &dcr_read_ocm, &dcr_write_ocm);
    ppc_dcr_register(env, OCM0_DSARC,
                     ocm, &dcr_read_ocm, &dcr_write_ocm);
    ppc_dcr_register(env, OCM0_DSACNTL,
                     ocm, &dcr_read_ocm, &dcr_write_ocm);
}
#endif

/*****************************************************************************/
/* External Bus Controller */
typedef struct ppc4xx_ebc_t ppc4xx_ebc_t;
struct ppc4xx_ebc_t {
    uint32_t addr;
    uint32_t bcr[8];
    uint32_t bap[8];
    uint32_t bear;
    uint32_t besr0;
    uint32_t besr1;
    uint32_t cfg;
};

enum {
    EBC0_CFGADDR = 0x012,
    EBC0_CFGDATA = 0x013,
};

static uint32_t dcr_read_ebc (void *opaque, int dcrn)
{
    ppc4xx_ebc_t *ebc;
    uint32_t ret;

    ebc = opaque;
    switch (dcrn) {
    case EBC0_CFGADDR:
        ret = ebc->addr;
#ifdef DEBUG_EBC
        printf("read DCR[EBCADDR]: %08" PRIx32 "\n", ret);
#endif
        break;
    case EBC0_CFGDATA:
        switch (ebc->addr) {
        case 0x00: /* B0CR */
            ret = ebc->bcr[0];
            break;
        case 0x01: /* B1CR */
            ret = ebc->bcr[1];
            break;
        case 0x02: /* B2CR */
            ret = ebc->bcr[2];
            break;
        case 0x03: /* B3CR */
            ret = ebc->bcr[3];
            break;
        case 0x04: /* B4CR */
            ret = ebc->bcr[4];
            break;
        case 0x05: /* B5CR */
            ret = ebc->bcr[5];
            break;
        case 0x06: /* B6CR */
            ret = ebc->bcr[6];
            break;
        case 0x07: /* B7CR */
            ret = ebc->bcr[7];
            break;
        case 0x10: /* B0AP */
            ret = ebc->bap[0];
            break;
        case 0x11: /* B1AP */
            ret = ebc->bap[1];
            break;
        case 0x12: /* B2AP */
            ret = ebc->bap[2];
            break;
        case 0x13: /* B3AP */
            ret = ebc->bap[3];
            break;
        case 0x14: /* B4AP */
            ret = ebc->bap[4];
            break;
        case 0x15: /* B5AP */
            ret = ebc->bap[5];
            break;
        case 0x16: /* B6AP */
            ret = ebc->bap[6];
            break;
        case 0x17: /* B7AP */
            ret = ebc->bap[7];
            break;
        case 0x20: /* BEAR */
            ret = ebc->bear;
            break;
        case 0x21: /* BESR0 */
            ret = ebc->besr0;
            break;
        case 0x22: /* BESR1 */
            ret = ebc->besr1;
            break;
        case 0x23: /* CFG */
            ret = ebc->cfg;
            break;
        default:
            ret = 0x00000000;
            break;
        }
#ifdef DEBUG_EBC
        printf("read DCR[EBCDATA]: %08" PRIx32 "\n", ret);
#endif
        break;
    default:
        ret = 0x00000000;
        break;
    }

    return ret;
}

static void dcr_write_ebc (void *opaque, int dcrn, uint32_t val)
{
    ppc4xx_ebc_t *ebc;

    ebc = opaque;
    switch (dcrn) {
    case EBC0_CFGADDR:
        ebc->addr = val;
#ifdef DEBUG_EBC
        printf("write DCR[EBCADDR]: %08" PRIx32 "\n", val);
#endif
        break;
    case EBC0_CFGDATA:
#ifdef DEBUG_EBC
        printf("write DCR[EBCDATA]: %08" PRIx32 "\n", val);
#endif
        switch (ebc->addr) {
        case 0x00: /* B0CR */
            break;
        case 0x01: /* B1CR */
            break;
        case 0x02: /* B2CR */
            break;
        case 0x03: /* B3CR */
            break;
        case 0x04: /* B4CR */
            break;
        case 0x05: /* B5CR */
            break;
        case 0x06: /* B6CR */
            break;
        case 0x07: /* B7CR */
            break;
        case 0x10: /* B0AP */
            break;
        case 0x11: /* B1AP */
            break;
        case 0x12: /* B2AP */
            break;
        case 0x13: /* B3AP */
            break;
        case 0x14: /* B4AP */
            break;
        case 0x15: /* B5AP */
            break;
        case 0x16: /* B6AP */
            break;
        case 0x17: /* B7AP */
            break;
        case 0x20: /* BEAR */
            break;
        case 0x21: /* BESR0 */
            break;
        case 0x22: /* BESR1 */
            break;
        case 0x23: /* CFG */
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void ebc_reset (void *opaque)
{
    ppc4xx_ebc_t *ebc;
    int i;

    ebc = opaque;
    ebc->addr = 0x00000000;
    ebc->bap[0] = 0x7F8FFE80;
    ebc->bcr[0] = 0xFFE28000;
    for (i = 0; i < 8; i++) {
        ebc->bap[i] = 0x00000000;
        ebc->bcr[i] = 0x00000000;
    }
    ebc->besr0 = 0x00000000;
    ebc->besr1 = 0x00000000;
    ebc->cfg = 0x80400000;
}

static void ppc4xx_ebc_init(CPUPPCState *env)
{
    ppc4xx_ebc_t *ebc;

    ebc = g_malloc0(sizeof(ppc4xx_ebc_t));
    qemu_register_reset(&ebc_reset, ebc);
    ppc_dcr_register(env, EBC0_CFGADDR,
                     ebc, &dcr_read_ebc, &dcr_write_ebc);
    ppc_dcr_register(env, EBC0_CFGDATA,
                     ebc, &dcr_read_ebc, &dcr_write_ebc);
}

/*****************************************************************************/
/* System DCRs */
typedef struct ppc4xx_sdr_t ppc4xx_sdr_t;
struct ppc4xx_sdr_t {
    uint32_t addr;
    uint32_t bcr[8];
    uint32_t bap[8];
    uint32_t bear;
    uint32_t besr0;
    uint32_t besr1;
    uint32_t cfg;
};

enum {
    SDR0_CFGADDR = 0x00e,
    SDR0_CFGDATA = 0x00f,
};

static uint32_t dcr_read_sdr (void *opaque, int dcrn)
{
    ppc4xx_sdr_t *sdr;
    uint32_t ret;

    sdr = opaque;
    switch (dcrn) {
    case SDR0_CFGADDR:
        ret = sdr->addr;
#ifdef DEBUG_SDR
        printf("read DCR[SDRADDR]: %08" PRIx32 "\n", ret);
#endif
        break;
    case SDR0_CFGDATA:
        switch (sdr->addr) {
        default:
            ret = 0x00000000;
            break;
        }
#ifdef DEBUG_SDR
        printf("read DCR[SDRDATA]: %08" PRIx32 "\n", ret);
#endif
        break;
    default:
        ret = 0x00000000;
        break;
    }

    return ret;
}

static void dcr_write_sdr (void *opaque, int dcrn, uint32_t val)
{
    ppc4xx_sdr_t *sdr;

    sdr = opaque;
    switch (dcrn) {
    case SDR0_CFGADDR:
        sdr->addr = val;
#ifdef DEBUG_SDR
        printf("write DCR[SDRADDR]: %08" PRIx32 "\n", val);
#endif
        break;
    case SDR0_CFGDATA:
#ifdef DEBUG_SDR
        printf("write DCR[SDRDATA]: %08" PRIx32 "\n", val);
#endif
        switch (sdr->addr) {
        case 0x00: /* B0CR */
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void sdr_reset (void *opaque)
{
    ppc4xx_sdr_t *sdr;
    int i;

    sdr = opaque;
    sdr->addr = 0x00000000;
    sdr->bap[0] = 0x7F8FFE80;
    sdr->bcr[0] = 0xFFE28000;
    for (i = 0; i < 8; i++) {
        sdr->bap[i] = 0x00000000;
        sdr->bcr[i] = 0x00000000;
    }
    sdr->besr0 = 0x00000000;
    sdr->besr1 = 0x00000000;
    sdr->cfg = 0x80400000;
}

static void ppc4xx_sdr_init(CPUPPCState *env)
{
    ppc4xx_sdr_t *sdr;

    sdr = g_malloc0(sizeof(ppc4xx_sdr_t));
    qemu_register_reset(&sdr_reset, sdr);
    ppc_dcr_register(env, SDR0_CFGADDR,
                     sdr, &dcr_read_sdr, &dcr_write_sdr);
    ppc_dcr_register(env, SDR0_CFGDATA,
                     sdr, &dcr_read_sdr, &dcr_write_sdr);
}

/*****************************************************************************/
/* Peripheral local bus arbitrer */
/* TODO: read & write all regs */
enum {
    PLB3A0_ACR = 0x077,
    PLB4A0_ACR = 0x081,
    PLB0_BESR  = 0x084,
    PLB0_BEAR  = 0x086,
    PLB0_ACR   = 0x087,
    PLB4A1_ACR = 0x089
};

typedef struct ppc4xx_plb_t ppc4xx_plb_t;
struct ppc4xx_plb_t {
    uint32_t acr;
    uint32_t bear;
    uint32_t besr;
};

static uint32_t dcr_read_plb (void *opaque, int dcrn)
{
    ppc4xx_plb_t *plb;
    uint32_t ret;

    plb = opaque;
    switch (dcrn) {
    case PLB0_ACR:
        ret = plb->acr;
        break;
    case PLB0_BEAR:
        ret = plb->bear;
        break;
    case PLB0_BESR:
        ret = plb->besr;
        break;
    default:
        /* Avoid gcc warning */
        ret = 0;
        break;
    }
#ifdef DEBUG_PLB
    printf("read DCR[PLB+%x]: %08" PRIx32 "\n", dcrn - 0x80, ret);
#endif

    return ret;
}

static void dcr_write_plb (void *opaque, int dcrn, uint32_t val)
{
    ppc4xx_plb_t *plb;

    plb = opaque;
#ifdef DEBUG_PLB
    printf("write DCR[PLB+%x]: %08" PRIx32 "\n", dcrn - 0x80, val);
#endif
    switch (dcrn) {
    case PLB0_ACR:
        /* We don't care about the actual parameters written as
         * we don't manage any priorities on the bus
         */
        plb->acr = val & 0xF8000000;
        break;
    case PLB0_BEAR:
        /* Read only */
        break;
    case PLB0_BESR:
        /* Write-clear */
        plb->besr &= ~val;
        break;
    }
}

static void ppc4xx_plb_reset (void *opaque)
{
    ppc4xx_plb_t *plb;

    plb = opaque;
    plb->acr = 0x00000000;
    plb->bear = 0x00000000;
    plb->besr = 0x00000000;
}

static void ppc4xx_plb_init(CPUPPCState *env)
{
    ppc4xx_plb_t *plb;

    plb = g_malloc0(sizeof(ppc4xx_plb_t));
    ppc_dcr_register(env, PLB3A0_ACR, plb, &dcr_read_plb, &dcr_write_plb);
    ppc_dcr_register(env, PLB4A0_ACR, plb, &dcr_read_plb, &dcr_write_plb);
    ppc_dcr_register(env, PLB0_ACR, plb, &dcr_read_plb, &dcr_write_plb);
    ppc_dcr_register(env, PLB0_BEAR, plb, &dcr_read_plb, &dcr_write_plb);
    ppc_dcr_register(env, PLB0_BESR, plb, &dcr_read_plb, &dcr_write_plb);
    ppc_dcr_register(env, PLB4A1_ACR, plb, &dcr_read_plb, &dcr_write_plb);
    qemu_register_reset(ppc4xx_plb_reset, plb);
}

/*****************************************************************************/
/* PLB to AHB bridge */
enum {
    AHB_TOP    = 0x0A4,
    AHB_BOT    = 0x0A5,
};

typedef struct ppc4xx_ahb_t ppc4xx_ahb_t;
struct ppc4xx_ahb_t {
    uint32_t top;
    uint32_t bot;
};

static uint32_t dcr_read_ahb (void *opaque, int dcrn)
{
    ppc4xx_ahb_t *ahb;
    uint32_t ret;

    ahb = opaque;
    switch (dcrn) {
    case AHB_TOP:
        ret = ahb->top;
        break;
    case AHB_BOT:
        ret = ahb->bot;
        break;
    default:
        /* Avoid gcc warning */
        ret = 0;
        break;
    }
#ifdef DEBUG_AHB
    printf("read DCR[AHB+%x]: %08" PRIx32 "\n", dcrn - 0xA0, ret);
#endif

    return ret;
}

static void dcr_write_ahb (void *opaque, int dcrn, uint32_t val)
{
    ppc4xx_ahb_t *ahb;

    ahb = opaque;
#ifdef DEBUG_AHB
    printf("write DCR[AHB+%x]: %08" PRIx32 "\n", dcrn - 0xA0, val);
#endif
    switch (dcrn) {
    case AHB_TOP:
        ahb->top = val;
        break;
    case AHB_BOT:
        ahb->bot = val;
        break;
    }
}

static void ppc4xx_ahb_reset (void *opaque)
{
    ppc4xx_ahb_t *ahb;

    ahb = opaque;
    /* No error */
    ahb->top = 0x00000000;
    ahb->bot = 0x00000000;
}

static void ppc4xx_ahb_init(CPUPPCState *env)
{
    ppc4xx_ahb_t *ahb;

    ahb = g_malloc0(sizeof(ppc4xx_ahb_t));
    ppc_dcr_register(env, AHB_TOP, ahb, &dcr_read_ahb, &dcr_write_ahb);
    ppc_dcr_register(env, AHB_BOT, ahb, &dcr_read_ahb, &dcr_write_ahb);
    qemu_register_reset(ppc4xx_ahb_reset, ahb);
}

/*****************************************************************************/
/* I2C controller */
/* TODO: move to hw/i2c/ppc4xx_i2c.c */
/* TODO: FIXME: regs are wrong (ppc405), cf. u-boot's include/4xx_i2c.h */

#define TYPE_PPC4xx_I2C "ppc4xx.i2c"
#define PPC4xx_I2C(obj) OBJECT_CHECK(PPC4xxI2CState, (obj), TYPE_PPC4xx_I2C)

#define PPC4xx_I2C_MEM_SIZE           0x12

typedef struct PPC4xxI2CState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;

	/* according to u-boot's include/4xx_i2c.h */
    uint8_t mdbuf;
    uint8_t res1;
    uint8_t sdbuf;
    uint8_t res2;
    uint8_t lmadr;
    uint8_t hmadr;
    uint8_t cntl;
    uint8_t mdcntl;
    uint8_t sts;
    uint8_t extsts;
    uint8_t lsadr;
    uint8_t hsadr;
    uint8_t clkdiv;
    uint8_t intrmsk;
    uint8_t xfrcnt;
    uint8_t xtcntlss;
    uint8_t directcntl;
    uint8_t intr;
/* ppc405 fields:
    uint8_t mdata;
    uint8_t lmadr;
    uint8_t hmadr;
    uint8_t cntl;
    uint8_t mdcntl;
    uint8_t sts;
    uint8_t extsts;
    uint8_t sdata;
    uint8_t lsadr;
    uint8_t hsadr;
    uint8_t clkdiv;
    uint8_t intrmsk;
    uint8_t xfrcnt;
    uint8_t xtcntlss;
    uint8_t directcntl;
*/
} PPC4xxI2CState;

static uint64_t ppc4xx_i2c_read (void *opaque, hwaddr addr, unsigned size)
{
    PPC4xxI2CState *i2c = PPC4xx_I2C(opaque);
    uint8_t ret;

#ifdef DEBUG_I2C
    printf("%s: addr " TARGET_FMT_plx " size %u\n", __func__, addr, size);
#endif
    switch (addr) {
#if 0
    case 0x00:
        //        i2c_readbyte(&i2c->mdata);
		if (i2c->lmadr == (0x50<<1)) {
			i2c->mdata = 1;
		}
        ret = i2c->mdata;
        break;
    case 0x02:
        ret = i2c->sdata;
        break;
    case 0x04:
        ret = i2c->lmadr;
        break;
    case 0x05:
        ret = i2c->hmadr;
        break;
    case 0x06:
        ret = i2c->cntl;
        break;
    case 0x07:
        ret = i2c->mdcntl;
        break;
    case 0x08:
        ret = i2c->sts;
        break;
    case 0x09:
        ret = i2c->extsts;
        break;
    case 0x0A:
        ret = i2c->lsadr;
        break;
    case 0x0B:
        ret = i2c->hsadr;
        break;
    case 0x0C:
        ret = i2c->clkdiv;
        break;
    case 0x0D:
        ret = i2c->intrmsk;
        break;
    case 0x0E:
        ret = i2c->xfrcnt;
        break;
    case 0x0F:
        ret = i2c->xtcntlss;
        break;
#endif
    case 0x10:
        ret = i2c->directcntl;
        break;
    default:
        ret = 0x00;
        break;
    }
#ifdef DEBUG_I2C
    printf("%s: addr " TARGET_FMT_plx " %02" PRIx32 "\n", __func__, addr, ret);
#endif

    return ret;
}

static void ppc4xx_i2c_write (void *opaque, hwaddr addr,
                              uint64_t value, unsigned size)
{
    PPC4xxI2CState *i2c = PPC4xx_I2C(opaque);

#ifdef DEBUG_I2C
    printf("%s: addr " TARGET_FMT_plx " val %08" PRIx64 "\n", __func__, addr,
           value);
#endif
    switch (addr) {
#if 0
    case 0x00:
        i2c->mdata = value;
        //        i2c_sendbyte(&i2c->mdata);
        break;
    case 0x02:
        i2c->sdata = value;
        break;
    case 0x04:
        i2c->lmadr = value;
        break;
    case 0x05:
        i2c->hmadr = value;
        break;
    case 0x06:
        i2c->cntl = value;
        break;
    case 0x07:
        i2c->mdcntl = value & 0xDF;
        break;
    case 0x08:
        i2c->sts &= ~(value & 0x0A);
        break;
    case 0x09:
        i2c->extsts &= ~(value & 0x8F);
        break;
    case 0x0A:
        i2c->lsadr = value;
        break;
    case 0x0B:
        i2c->hsadr = value;
        break;
    case 0x0C:
        i2c->clkdiv = value;
        break;
    case 0x0D:
        i2c->intrmsk = value;
        break;
    case 0x0E:
        i2c->xfrcnt = value & 0x77;
        break;
    case 0x0F:
        i2c->xtcntlss = value;
        break;
#endif
    case 0x10:
        i2c->directcntl = value & 0x7;
        break;
    }
}

#if 0
static uint32_t ppc4xx_i2c_readw (void *opaque, hwaddr addr)
{
    uint32_t ret;

#ifdef DEBUG_I2C
    printf("%s: addr " TARGET_FMT_plx "\n", __func__, addr);
#endif
    ret = ppc4xx_i2c_readb(opaque, addr) << 8;
    ret |= ppc4xx_i2c_readb(opaque, addr + 1);

    return ret;
}

static void ppc4xx_i2c_writew (void *opaque,
                               hwaddr addr, uint32_t value)
{
#ifdef DEBUG_I2C
    printf("%s: addr " TARGET_FMT_plx " val %08" PRIx32 "\n", __func__, addr,
           value);
#endif
    ppc4xx_i2c_writeb(opaque, addr, value >> 8);
    ppc4xx_i2c_writeb(opaque, addr + 1, value);
}

static uint32_t ppc4xx_i2c_readl (void *opaque, hwaddr addr)
{
    uint32_t ret;

#ifdef DEBUG_I2C
    printf("%s: addr " TARGET_FMT_plx "\n", __func__, addr);
#endif
    ret = ppc4xx_i2c_readb(opaque, addr) << 24;
    ret |= ppc4xx_i2c_readb(opaque, addr + 1) << 16;
    ret |= ppc4xx_i2c_readb(opaque, addr + 2) << 8;
    ret |= ppc4xx_i2c_readb(opaque, addr + 3);

    return ret;
}

static void ppc4xx_i2c_writel (void *opaque,
                               hwaddr addr, uint32_t value)
{
#ifdef DEBUG_I2C
    printf("%s: addr " TARGET_FMT_plx " val %08" PRIx32 "\n", __func__, addr,
           value);
#endif
    ppc4xx_i2c_writeb(opaque, addr, value >> 24);
    ppc4xx_i2c_writeb(opaque, addr + 1, value >> 16);
    ppc4xx_i2c_writeb(opaque, addr + 2, value >> 8);
    ppc4xx_i2c_writeb(opaque, addr + 3, value);
}
#endif

static void ppc4xx_i2c_reset (DeviceState *dev)
{
    PPC4xxI2CState *s = PPC4xx_I2C(dev);

/*
	XXX
    if (s->address != ADDR_RESET) {
        i2c_end_transfer(s->bus);
    }
*/

	s->mdbuf = 0x00;
	s->res1 = 0x00;
	s->sdbuf = 0x00;
	s->res2 = 0x00;
	s->lmadr = 0x00;
	s->hmadr = 0x00;
	s->cntl = 0x00;
	s->mdcntl = 0x00;
	s->sts = 0x00;
	s->extsts = 0x00;
	s->lsadr = 0x00;
	s->hsadr = 0x00;
	s->clkdiv = 0x00;
	s->intrmsk = 0x00;
	s->xfrcnt = 0x00;
	s->xtcntlss = 0x00;
	s->directcntl = 0x00;
	s->intr = 0x00;
}

#if 0
static void ppc4xx_i2c_init(hwaddr base, qemu_irq irq)
{
    PPC4xxI2CState *i2c;

    i2c = g_malloc0(sizeof(PPC4xxI2CState));
    i2c->irq = irq;
#ifdef DEBUG_I2C
    printf("%s: offset " TARGET_FMT_plx "\n", __func__, base);
#endif
    memory_region_init_io(&i2c->iomem, NULL, &i2c_ops, i2c, "i2c", 0x011);
    memory_region_add_subregion(get_system_memory(), base, &i2c->iomem);
    qemu_register_reset(ppc4xx_i2c_reset, i2c);
}
#endif

static const MemoryRegionOps ppc4xx_i2c_ops = {
    .read = ppc4xx_i2c_read,
    .write = ppc4xx_i2c_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 1,/*XXX: allow larger access?? */
    .endianness = DEVICE_NATIVE_ENDIAN,
/*    .old_mmio = {
        .read = { ppc4xx_i2c_readb, ppc4xx_i2c_readw, ppc4xx_i2c_readl, },
        .write = { ppc4xx_i2c_writeb, ppc4xx_i2c_writew, ppc4xx_i2c_writel, },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
*/
};

static const VMStateDescription ppc4xx_i2c_vmstate = {
    .name = TYPE_PPC4xx_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
    	VMSTATE_UINT8(mdbuf, PPC4xxI2CState),
    	VMSTATE_UINT8(res1, PPC4xxI2CState),
    	VMSTATE_UINT8(sdbuf, PPC4xxI2CState),
    	VMSTATE_UINT8(res2, PPC4xxI2CState),
    	VMSTATE_UINT8(lmadr, PPC4xxI2CState),
    	VMSTATE_UINT8(hmadr, PPC4xxI2CState),
    	VMSTATE_UINT8(cntl, PPC4xxI2CState),
    	VMSTATE_UINT8(mdcntl, PPC4xxI2CState),
    	VMSTATE_UINT8(sts, PPC4xxI2CState),
    	VMSTATE_UINT8(extsts, PPC4xxI2CState),
    	VMSTATE_UINT8(lsadr, PPC4xxI2CState),
    	VMSTATE_UINT8(hsadr, PPC4xxI2CState),
    	VMSTATE_UINT8(clkdiv, PPC4xxI2CState),
    	VMSTATE_UINT8(intrmsk, PPC4xxI2CState),
    	VMSTATE_UINT8(xfrcnt, PPC4xxI2CState),
    	VMSTATE_UINT8(xtcntlss, PPC4xxI2CState),
    	VMSTATE_UINT8(directcntl, PPC4xxI2CState),
    	VMSTATE_UINT8(intr, PPC4xxI2CState),
        VMSTATE_END_OF_LIST()
    }
};

static void ppc4xx_i2c_realize(DeviceState *dev, Error **errp)
{
    PPC4xxI2CState *s = PPC4xx_I2C(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ppc4xx_i2c_ops, s, TYPE_PPC4xx_I2C,
                          PPC4xx_I2C_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    s->bus = i2c_init_bus(DEVICE(dev), "i2c");
}

static void ppc4xx_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &ppc4xx_i2c_vmstate;
    dc->reset = ppc4xx_i2c_reset;
    dc->realize = ppc4xx_i2c_realize;
}

static const TypeInfo ppc4xx_i2c_type_info = {
    .name = TYPE_PPC4xx_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PPC4xxI2CState),
    .class_init = ppc4xx_i2c_class_init,
};

static void ppc4xx_i2c_register_types(void)
{
    type_register_static(&ppc4xx_i2c_type_info);
}

type_init(ppc4xx_i2c_register_types)

/*****************************************************************************/

static int sam460ex_load_uboot(void)
{
    DriveInfo *dinfo;
    BlockBackend *blk = NULL;
    hwaddr base = FLASH_BASE | ((hwaddr)FLASH_BASE_H << 32);
    long bios_size = FLASH_SIZE;
    int fl_sectors;

    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
        blk = blk_by_legacy_dinfo(dinfo);
        bios_size = blk_getlength(blk);
    }
    fl_sectors = (bios_size + 65535) >> 16;

    if (!pflash_cfi01_register(base, NULL, "sam460ex.flash", bios_size,
                               blk, (64 * 1024), fl_sectors,
                               1, 0x89, 0x18, 0x0000, 0x0, 1)) {
        fprintf(stderr, "qemu: Error registering flash memory.\n");
        /* XXX: return an error instead? */
        exit(1);
    }

    if (!blk) {
        /*fprintf(stderr, "No flash image given with the 'pflash' parameter,"
                " using default u-boot image\n");*/
        base = UBOOT_LOAD_BASE | ((hwaddr)FLASH_BASE_H << 32);
        rom_add_file_fixed(UBOOT_FILENAME, base, -1);
    }

    return 0;
}

static int sam460ex_load_device_tree(hwaddr addr,
                                     uint32_t ramsize,
                                     hwaddr initrd_base,
                                     hwaddr initrd_size,
                                     const char *kernel_cmdline)
{
    int ret = -1;
    uint32_t mem_reg_property[] = { 0, 0, cpu_to_be32(ramsize) };
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

    ret = qemu_fdt_setprop(fdt, "/memory", "reg", mem_reg_property,
                               sizeof(mem_reg_property));
    if (ret < 0)
        fprintf(stderr, "couldn't set /memory/reg\n");

    /* default FDT doesn't have a /chosen node... */
    qemu_fdt_add_subnode(fdt, "/chosen");

    ret = qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-start",
                                    initrd_base);
    if (ret < 0)
        fprintf(stderr, "couldn't set /chosen/linux,initrd-start\n");

    ret = qemu_fdt_setprop_cell(fdt, "/chosen", "linux,initrd-end",
                                    (initrd_base + initrd_size));
    if (ret < 0)
        fprintf(stderr, "couldn't set /chosen/linux,initrd-end\n");

    ret = qemu_fdt_setprop_string(fdt, "/chosen", "bootargs",
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

    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "clock-frequency",
                              clock_freq);
    qemu_fdt_setprop_cell(fdt, "/cpus/cpu@0", "timebase-frequency",
                              tb_freq);

    rom_add_blob_fixed(BINARY_DEVICE_TREE_FILE, fdt, fdt_size, addr);
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
                                     hwaddr pa)
{
    //struct boot_info *bi = env->load_info;
    ppcemb_tlb_t *tlb = &env->tlb.tlbe[0];
    int i = 1;

    tlb->attr = 0;
    tlb->prot = PAGE_VALID | ((PAGE_READ | PAGE_WRITE | PAGE_EXEC) << 4);
    tlb->size = 1 << 31; /* up to 0x80000000  */
    tlb->EPN = va & TARGET_PAGE_MASK;
    tlb->RPN = pa & TARGET_PAGE_MASK;
    tlb->PID = 0;

#if 0
    tlb = &env->tlb.tlbe[i++];
    tlb->attr = 0;
    tlb->prot = PAGE_VALID | ((PAGE_READ | PAGE_WRITE | PAGE_EXEC) << 4);
    tlb->size = 1 << 31; /* up to 0xffffffff  */
    tlb->EPN = 0x80000000 & TARGET_PAGE_MASK;
    tlb->RPN = 0x80000000 & TARGET_PAGE_MASK;
    tlb->PID = 0;
#endif

    /* MAP some IO space including the UART.
       Linux doesn't need it, but Haiku does, for now */
    tlb = &env->tlb.tlbe[i++];
    tlb->attr = 0;
    tlb->prot = PAGE_VALID | ((PAGE_READ | PAGE_WRITE) << 4);
    tlb->size = 1 << 24; /* 16MB  */
    tlb->EPN = 0xef000000 & TARGET_PAGE_MASK;
    tlb->RPN = (0xef000000 & TARGET_PAGE_MASK) | 0x4;
    tlb->PID = 0;

    /* HACK: just for testing */
    tlb = &env->tlb.tlbe[i++];
    tlb->attr = 0;
    tlb->prot = PAGE_VALID | ((PAGE_READ | PAGE_WRITE) << 4);
    tlb->size = 1 << 24; /* 16MB  */
    tlb->EPN = 0x80000000 & TARGET_PAGE_MASK;
    tlb->RPN = (0x80000000 & TARGET_PAGE_MASK) | 0x4;
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

static void sam460ex_init(MachineState *machine)
{
    unsigned int pci_irq_nrs[4] = { 28, 27, 26, 25 };
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *ram_memories = g_new(MemoryRegion, PPC440EP_SDRAM_NR_BANKS);
    hwaddr ram_bases[PPC440EP_SDRAM_NR_BANKS];
    hwaddr ram_sizes[PPC440EP_SDRAM_NR_BANKS];
    MemoryRegion *l2cache_ram = g_new(MemoryRegion, 1);
    MemoryRegion *ocm_ram = g_new(MemoryRegion, 1);
    qemu_irq *pic;
    qemu_irq *irqs;
    PCIBus *pci_bus;
    PowerPCCPU *cpu;
    CPUPPCState *env;
    uint64_t elf_entry;
    uint64_t elf_lowaddr;
    hwaddr entry = UBOOT_ENTRY;
    hwaddr loadaddr = 0;
    target_long initrd_size = 0;
    DeviceState *dev;
    SysBusDevice *s;
    int success;
    int i;
    struct boot_info *boot_info;

    /* Setup CPU. */
    if (machine->cpu_model == NULL) {
        machine->cpu_model = "440EP";
    }
    cpu = cpu_ppc_init(machine->cpu_model);
    if (cpu == NULL) {
        fprintf(stderr, "Unable to initialize CPU!\n");
        exit(1);
    }
    env = &cpu->env;

    qemu_register_reset(main_cpu_reset, cpu);
    boot_info = g_malloc0(sizeof(struct boot_info));
    env->load_info = boot_info;

    ppc_booke_timers_init(cpu, 400000000, 0);
    ppc_dcr_init(env, NULL, NULL);

    /* PLB arbitrer */
    ppc4xx_plb_init(env);

    /* interrupt controller */
    irqs = g_malloc0(sizeof(qemu_irq) * PPCUIC_OUTPUT_NB);
    irqs[PPCUIC_OUTPUT_INT] = ((qemu_irq *)env->irq_inputs)[PPC40x_INPUT_INT];
    irqs[PPCUIC_OUTPUT_CINT] = ((qemu_irq *)env->irq_inputs)[PPC40x_INPUT_CINT];
    pic = ppcuic_init(env, irqs, 0x0C0, 0, 1);

    /*FIXME*/
    irqs = g_malloc0(sizeof(qemu_irq) * PPCUIC_OUTPUT_NB);
    /*pic =*/ ppcuic_init(env, irqs, 0x0D0, 0, 1);
    irqs = g_malloc0(sizeof(qemu_irq) * PPCUIC_OUTPUT_NB);
    /*pic =*/ ppcuic_init(env, irqs, 0x0E0, 0, 1);
    irqs = g_malloc0(sizeof(qemu_irq) * PPCUIC_OUTPUT_NB);
    /*pic =*/ ppcuic_init(env, irqs, 0x0F0, 0, 1);

    /* SDRAM controller */
    memset(ram_bases, 0, sizeof(ram_bases));
    memset(ram_sizes, 0, sizeof(ram_sizes));
    machine->ram_size = ppc4xx_sdram_adjust(machine->ram_size,
                                   PPC440EP_SDRAM_NR_BANKS,
                                   ram_memories,
                                   ram_bases, ram_sizes,
                                   ppc440ep_sdram_bank_sizes);
printf("RAMSIZE %dMB\n", (int)(machine->ram_size / (1024 * 1024)));

    /* XXX 440EP's ECC interrupts are on UIC1, but we've only created UIC0. */
    ppc4xx_sdram_init(env, pic[14], PPC440EP_SDRAM_NR_BANKS, ram_memories,
                      ram_bases, ram_sizes, 1);

    /* External bus controller */
    ppc4xx_ebc_init(env);
    /* PLB to AHB bridge */
    ppc4xx_ahb_init(env);

    /* System DCRs */
    ppc4xx_sdr_init(env);

    /* 256K of L2 cache as memory */
    ppc4xx_l2sram_init(env);
    /* FIXME:remove this */
    memory_region_init_ram(l2cache_ram, NULL, "ppc440.l2cache_ram", 256 << 10, &error_abort);
    vmstate_register_ram_global(l2cache_ram);
    memory_region_add_subregion(address_space_mem, 0x400000000LL, l2cache_ram);

    /* 64K of on-chip memory */
    memory_region_init_ram(ocm_ram, NULL, "ppc440.ocm_ram", 64 << 10, &error_abort);
    vmstate_register_ram_global(ocm_ram);
    memory_region_add_subregion(address_space_mem, 0x400040000LL, ocm_ram);

    /* IIC controller */
    /* FIXME: irq? */
    /*
	ppc4xx_i2c_init(0x4ef600700LL, pic[2]);
    ppc4xx_i2c_init(0x4ef600800LL, pic[2]);
	*/
    for (i = 0; i < 2; i++) {
        static const struct {
            hwaddr addr;
            unsigned int irq;
        } i2c_table[2] = {
            { 0x4ef600700LL, 0 },
            { 0x4ef600800LL, 0 }
        };
		/*TODO: attach this to the machine state */
		static struct {
			PPC4xxI2CState i2c[2];
		} _s, *s = &_s;
	    Error *err = NULL;


        object_initialize(&s->i2c[i], sizeof(s->i2c[i]), TYPE_PPC4xx_I2C);
        qdev_set_parent_bus(DEVICE(&s->i2c[i]), sysbus_get_default());

        object_property_set_bool(OBJECT(&s->i2c[i]), true, "realized", &err);
        if (err) {
	        fprintf(stderr, "Unable to initialize I2C[%d]!\n", i);
            //error_propagate(errp, err);
            return;
        }
        sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2c[i]), 0, i2c_table[i].addr);
		/*
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->avic),
                                            i2c_table[i].irq));
		*/
    }

    /* PCI */
    dev = qdev_create(NULL, "ppc4xx-pcihost");
    qdev_init_nofail(dev);
    s = SYS_BUS_DEVICE(dev);
    sysbus_connect_irq(s, 0, pic[pci_irq_nrs[0]]);
    sysbus_connect_irq(s, 1, pic[pci_irq_nrs[1]]);
    sysbus_connect_irq(s, 2, pic[pci_irq_nrs[2]]);
    sysbus_connect_irq(s, 3, pic[pci_irq_nrs[3]]);
    memory_region_add_subregion(address_space_mem, PPC440EP_PCI_IO,
                                sysbus_mmio_get_region(s, 0));
/*XXX: FIXME: is this correct? */

/*
    dev = sysbus_create_varargs("ppc4xx-pcihost", PPC440EP_PCI_CONFIG,
                                pic[pci_irq_nrs[0]], pic[pci_irq_nrs[1]],
                                pic[pci_irq_nrs[2]], pic[pci_irq_nrs[3]],
                                NULL);
*/
    pci_bus = (PCIBus *)qdev_get_child_bus(dev, "pci.0");
    if (!pci_bus) {
        fprintf(stderr, "couldn't create PCI controller!\n");
        exit(1);
    }

#if 1
    sm501_init(address_space_mem, 0x10000000, SM501_VRAM_SIZE,
               /*irq[SM501]FIXME*/pic[13], serial_hds[2]);
#endif

    //sysbus_mmio_map(SYS_BUS_DEVICE(dev), 1, PPC440EP_PCI_IO);

    /* Core has 4 UARTs but the board only has one wired */
    if (serial_hds[0] != NULL) {
        serial_mm_init(address_space_mem, 0x4ef600300, 0, pic[0],
                       PPC_SERIAL_MM_BAUDBASE, serial_hds[0],
                       DEVICE_BIG_ENDIAN);
    }

    if (pci_bus) {
        /* Register network interfaces. */
        for (i = 0; i < nb_nics; i++) {
            /* There are no PCI NICs on the Bamboo board, but there are
             * PCI slots, so we can pick whatever default model we want. */
            pci_nic_init_nofail(&nd_table[i], pci_bus, "e1000", NULL);
        }
    }

    /* Load U-Boot image. */
    if (!machine->kernel_filename) {
        success = sam460ex_load_uboot();
        if (success < 0) {
            fprintf(stderr, "qemu: could not load firmware\n");
            exit(1);
        }
    }

    /* Load kernel. */
    if (machine->kernel_filename) {
        success = load_uimage(machine->kernel_filename, &entry, &loadaddr, NULL,
            NULL, NULL);
        fprintf(stderr, "load_uimage: %d\n", success);
        if (success < 0) {
            success = load_elf(machine->kernel_filename, NULL, NULL, &elf_entry,
                               &elf_lowaddr, NULL, 1, PPC_ELF_MACHINE, 0);
            entry = elf_entry;
            loadaddr = elf_lowaddr;
        }
        /* XXX try again as binary */
        if (success < 0) {
            fprintf(stderr, "qemu: could not load kernel '%s'\n",
                    machine->kernel_filename);
            exit(1);
        }
    }

    /* Load initrd. */
    if (machine->initrd_filename) {
        initrd_size = load_image_targphys(machine->initrd_filename, RAMDISK_ADDR,
                                          machine->ram_size - RAMDISK_ADDR);

        if (initrd_size < 0) {
            fprintf(stderr, "qemu: could not load ram disk '%s' at %x\n",
                    machine->initrd_filename, RAMDISK_ADDR);
            exit(1);
        }
    }

    /* If we're loading a kernel directly, we must load the device tree too. */
    if (machine->kernel_filename) {
        int dt_size;

        dt_size = sam460ex_load_device_tree(FDT_ADDR, machine->ram_size,
                                    RAMDISK_ADDR, initrd_size,
                                    machine->kernel_cmdline);
        if (dt_size < 0) {
            fprintf(stderr, "couldn't load device tree\n");
            exit(1);
        }

        boot_info->dt_base = FDT_ADDR;
        boot_info->dt_size = dt_size;
    }

    boot_info->entry = entry;
}

static void sam460ex_machine_init(MachineClass *mc)
{
    mc->desc = "aCube Sam460ex";
    mc->init = sam460ex_init;
}

DEFINE_MACHINE("sam460ex", sam460ex_machine_init)
