/* ColdFire PSC emulation.
 *
 * Heavily based on mcf_uart.c, see
 * copyright terms below.
 *
 * Copyright (c) 2014 Gregory Estrade.
 *
 * This code is licensed under the GPL 
 *
 */
/*
 * ColdFire UART emulation.
 *
 * Copyright (c) 2007 CodeSourcery.
 *
 * This code is licensed under the GPL
 */
#include "hw/hw.h"
#include "hw/m68k/mcf.h"
#include "sysemu/char.h"
#include "exec/address-spaces.h"

#define PSC_DEBUG
#ifdef PSC_DEBUG
#define DPRINTF(fmt, ...) \
do { \
    fprintf(stderr, "PSC: " fmt , ## __VA_ARGS__); \
} while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif


#define FIFO_SIZE 512

typedef struct {
    MemoryRegion iomem;
    uint8_t mr[2];      /* 0x00 R/W */
    uint16_t sr;        /* 0x04 R */
    uint8_t cr;         /* 0x08 W */
    uint8_t acr;        /* 0x10 W */
    uint16_t isr;       /* 0x14 R */
    uint16_t imr;       /* 0x14 W */
    uint8_t rx_fifo[FIFO_SIZE];
    uint8_t tb;
    int current_mr;
    int rx_fifo_len;
    int tx_enabled;
    int rx_enabled;
    qemu_irq irq;
    CharDriverState *chr;
} mcf_psc_state;

/* PSC Status Register bits.  */
#define MCF_PSC_RxRDY  (1 << 8)
#define MCF_PSC_FU     (1 << 9)
#define MCF_PSC_TxRDY  (1 << 10)
#define MCF_PSC_TxEMP  (1 << 11)

/* Interrupt flags.  */
#define MCF_PSC_TxINT  (1 << 8)
#define MCF_PSC_RxINT  (1 << 9)
#define MCF_PSC_DBINT  (1 << 10)

/* UMR1 flags.  */
#define MCF_PSC_BC0    (1 << 0)
#define MCF_PSC_BC1    (1 << 1)
#define MCF_PSC_PT     (1 << 2)
#define MCF_PSC_PM0    (1 << 3)
#define MCF_PSC_PM1    (1 << 4)
#define MCF_PSC_ERR    (1 << 5)
#define MCF_PSC_RxIRQ  (1 << 6)
#define MCF_PSC_RxRTS  (1 << 7)

static void mcf_psc_update(mcf_psc_state *s)
{
    s->isr &= ~(MCF_PSC_TxINT | MCF_PSC_RxINT);
    if (s->sr & MCF_PSC_TxRDY)
        s->isr |= MCF_PSC_TxINT;
    if ((s->sr & ((s->mr[0] & MCF_PSC_RxIRQ)
                  ? MCF_PSC_FU : MCF_PSC_RxRDY)) != 0)
        s->isr |= MCF_PSC_RxINT;

    qemu_set_irq(s->irq, (s->isr & s->imr) != 0);
}

uint64_t mcf_psc_read(void *opaque, hwaddr addr,
                       unsigned size)
{
    mcf_psc_state *s = (mcf_psc_state *)opaque;
    switch (addr & 0xff) {
    case 0x00:
        {
            uint8_t val;
            val = s->mr[s->current_mr];
            s->current_mr = 1;
            return val;
        }
    case 0x04:
        return s->sr;
    case 0x08:
        return s->cr;
    case 0x0c:
        {
            uint8_t val;
            int i;

            if (s->rx_fifo_len == 0)
                return 0;

            val = s->rx_fifo[0];
            s->rx_fifo_len--;
            for (i = 0; i < s->rx_fifo_len; i++)
                s->rx_fifo[i] = s->rx_fifo[i + 1];
            s->sr &= ~MCF_PSC_FU;
            if (s->rx_fifo_len == 0)
                s->sr &= ~MCF_PSC_RxRDY;
            mcf_psc_update(s);
            qemu_chr_accept_input(s->chr);
            return val;
        }
    case 0x10:
        /* TODO: Implement IPCR.  */
        return 0;
    case 0x14:
        return s->isr;
    default:
        DPRINTF("Unmapped read @0x%lx\n", addr);
        return 0;
    }
}

/* Update TxRDY flag and set data if present and enabled.  */
static void mcf_psc_do_tx(mcf_psc_state *s)
{
    if (s->tx_enabled && (s->sr & MCF_PSC_TxEMP) == 0) {
        if (s->chr)
            qemu_chr_fe_write(s->chr, (unsigned char *)&s->tb, 1);
        s->sr |= MCF_PSC_TxEMP;
    }
    if (s->tx_enabled) {
        s->sr |= MCF_PSC_TxRDY;
    } else {
        s->sr &= ~MCF_PSC_TxRDY;
    }
}

static void mcf_do_command(mcf_psc_state *s, uint8_t cmd)
{
    /* Misc command.  */
    switch ((cmd >> 4) & 3) {
    case 0: /* No-op.  */
        break;
    case 1: /* Reset mode register pointer.  */
        s->current_mr = 0;
        break;
    case 2: /* Reset receiver.  */
        s->rx_enabled = 0;
        s->rx_fifo_len = 0;
        s->sr &= ~(MCF_PSC_RxRDY | MCF_PSC_FU);
        break;
    case 3: /* Reset transmitter.  */
        s->tx_enabled = 0;
        s->sr |= MCF_PSC_TxEMP;
        s->sr &= ~MCF_PSC_TxRDY;
        break;
    case 4: /* Reset error status.  */
        break;
    case 5: /* Reset break-change interrupt.  */
        s->isr &= ~MCF_PSC_DBINT;
        break;
    case 6: /* Start break.  */
    case 7: /* Stop break.  */
        break;
    }

    /* Transmitter command.  */
    switch ((cmd >> 2) & 3) {
    case 0: /* No-op.  */
        break;
    case 1: /* Enable.  */
        s->tx_enabled = 1;
        mcf_psc_do_tx(s);
        break;
    case 2: /* Disable.  */
        s->tx_enabled = 0;
        mcf_psc_do_tx(s);
        break;
    case 3: /* Reserved.  */
        DPRINTF("Bad TX command\n");
        break;
    }

    /* Receiver command.  */
    switch (cmd & 3) {
    case 0: /* No-op.  */
        break;
    case 1: /* Enable.  */
        s->rx_enabled = 1;
        break;
    case 2:
        s->rx_enabled = 0;
        break;
    case 3: /* Reserved.  */
        DPRINTF("Bad RX command\n");
        break;
    }
}

void mcf_psc_write(void *opaque, hwaddr addr,
                    uint64_t val, unsigned size)
{
    mcf_psc_state *s = (mcf_psc_state *)opaque;
    switch (addr & 0xff) {
    case 0x00:
        s->mr[s->current_mr] = val;
        s->current_mr = 1;
        break;
    case 0x04:
        /* CSR is ignored.  */
        break;
    case 0x08: /* Command Register.  */
        mcf_do_command(s, val);
        break;
    case 0x0c: /* Transmit Buffer.  */
        s->sr &= ~MCF_PSC_TxEMP;
        s->tb = val;
        mcf_psc_do_tx(s);
        break;
    case 0x10:
        /* ACR is ignored.  */
        break;
    case 0x14:
        s->imr = val;
        break;
    default:
        DPRINTF("Unmapped write @0x%lx\n", addr);
        break;
    }
    mcf_psc_update(s);
}

static void mcf_psc_reset(mcf_psc_state *s)
{
    s->rx_fifo_len = 0;
    s->mr[0] = 0;
    s->mr[1] = 0;
    s->sr = MCF_PSC_TxEMP;
    s->tx_enabled = 0;
    s->rx_enabled = 0;
    s->isr = 0;
    s->imr = 0;
}

static void mcf_psc_push_byte(mcf_psc_state *s, uint8_t data)
{
    /* Break events overwrite the last byte if the rx_fifo is full.  */
    if (s->rx_fifo_len == FIFO_SIZE)
        s->rx_fifo_len--;

    s->rx_fifo[s->rx_fifo_len] = data;
    s->rx_fifo_len++;
    s->sr |= MCF_PSC_RxRDY;
    if (s->rx_fifo_len == FIFO_SIZE)
        s->sr |= MCF_PSC_FU;

    mcf_psc_update(s);
}

static void mcf_psc_event(void *opaque, int event)
{
    mcf_psc_state *s = (mcf_psc_state *)opaque;

    switch (event) {
    case CHR_EVENT_BREAK:
        s->isr |= MCF_PSC_DBINT;
        mcf_psc_push_byte(s, 0);
        break;
    default:
        break;
    }
}

static int mcf_psc_can_receive(void *opaque)
{
    mcf_psc_state *s = (mcf_psc_state *)opaque;

    return s->rx_enabled && (s->sr & MCF_PSC_FU) == 0;
}

static void mcf_psc_receive(void *opaque, const uint8_t *buf, int size)
{
    mcf_psc_state *s = (mcf_psc_state *)opaque;

    mcf_psc_push_byte(s, buf[0]);
}

void *mcf_psc_init(qemu_irq irq, CharDriverState *chr)
{
    mcf_psc_state *s;

    s = g_malloc0(sizeof(mcf_psc_state));
    s->chr = chr;
    s->irq = irq;
    if (chr) {
        qemu_chr_fe_claim_no_fail(chr);
        qemu_chr_add_handlers(chr, mcf_psc_can_receive, mcf_psc_receive,
                              mcf_psc_event, s);
    }
    mcf_psc_reset(s);
    return s;
}

static const MemoryRegionOps mcf_psc_ops = {
    .read = mcf_psc_read,
    .write = mcf_psc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void mcf_psc_mm_init(MemoryRegion *sysmem,
                      hwaddr base,
                      qemu_irq irq,
                      CharDriverState *chr)
{
    mcf_psc_state *s;

    s = mcf_psc_init(irq, chr);
    memory_region_init_io(&s->iomem, NULL, &mcf_psc_ops, s, "psc", 0x100);
    memory_region_add_subregion(sysmem, base, &s->iomem);
}
