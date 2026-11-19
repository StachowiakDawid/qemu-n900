/*
 * TI OMAP processors UART emulation.
 *
 * Copyright (C) 2006-2008 Andrzej Zaborowski  <balrog@zabor.org>
 * Copyright (C) 2007-2009 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"
#include "hw/arm/omap.h"
#include "hw/char/serial.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"

#define TYPE_OMAP_UART "omap_uart"
#define OMAP_UART(obj) OBJECT_CHECK(omap_uart_s, (obj), TYPE_OMAP_UART)

/* The OMAP UART functionality is similar to the TI16C752; it is
 * an enhanced version of the 16550A and we piggy-back on the 16550
 * model.
 *
 * Currently unmodelled functionality:
 *  + We should have a 64 byte FIFO but QEMU's SerialState emulation
 *    always uses a 16 byte FIFO
 *  + DMA
 *  + interrupts based on TCR/TLR values
 *  + XON/XOFF flow control
 *  + UASR auto-baudrate-detection
 */

typedef struct omap_uart_s {
    SysBusDevice busdev;
    MemoryRegion iomem;
    SerialState serial;
    uint32_t mmio_size;
    uint32_t baudrate;
    uint32_t revision;
    qemu_irq tx_drq;
    qemu_irq rx_drq;

    /* Register access mode, which affects what registers you see */
    enum {
        regs_operational,
        regs_config_a,
        regs_config_b
    } access_mode;

    uint8_t eblr;
    uint8_t syscontrol;
    uint8_t wkup;
    uint8_t cfps;
    uint8_t mdr[2];
    uint8_t scr;
    uint8_t clksel;
    uint8_t blr;
    uint8_t acreg;

    uint8_t mcr_cache;
    uint8_t efr;
    uint8_t tcr;
    uint8_t tlr;
    uint8_t xon[2], xoff[2];
} omap_uart_s;

static uint64_t omap_uart_serial_read(struct omap_uart_s *s, hwaddr addr)
{
    return serial_io_ops.read(&s->serial, addr >> 2, 1);
}

static void omap_uart_serial_write(struct omap_uart_s *s, hwaddr addr,
                                   uint64_t value)
{
    serial_io_ops.write(&s->serial, addr >> 2, value & 0xff, 1);
}

static int tcr_tlr_mode(struct omap_uart_s *s)
{
    /* Return true if registers 0x18 and 0x1c are TCR/TLR
     * (as opposed to SPR/MSR/XOFF)
     */
    return (s->efr & 0x10) && (s->mcr_cache & 0x40);
}

static void omap_uart_reset(DeviceState *qdev)
{
    struct omap_uart_s *s = OMAP_UART(qdev);

    s->eblr = 0x00;
    s->syscontrol = 0;
    s->wkup = 0x3f;
    s->cfps = 0x69;
    s->clksel = 0;
    s->blr = 0x40;
    s->acreg = 0;
    s->access_mode = regs_operational;

    s->mcr_cache = 0;
    s->tcr = 0x0f;
    s->tlr = 0;
    s->efr = 0;
    s->xon[0] = s->xon[1] = 0;
    s->xoff[0] = s->xoff[1] = 0;
}

static uint64_t omap_uart_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    struct omap_uart_s *s = (struct omap_uart_s *) opaque;

    switch (addr) {
    case 0x00:
    case 0x04:
    case 0x0c:
        return omap_uart_serial_read(s, addr);
    case 0x08:
        if (s->access_mode == regs_config_b) {
            return s->efr;
        }
        return omap_uart_serial_read(s, addr);
    case 0x10:
    case 0x14:
        if (s->access_mode == regs_config_b) {
            return s->xon[(addr & 7) >> 2];
        } else if (addr == 0x10) {
            /* MCR. Bits 5 and 6 are handled by us, the rest by
             * the underlying serial implementation.
             */
            return omap_uart_serial_read(s, addr) | s->mcr_cache;
        }
        return omap_uart_serial_read(s, addr);
    case 0x18:
    case 0x1c:
        if (tcr_tlr_mode(s)) {
            return (addr == 0x18) ? s->tcr : s->tlr;
        }
        if (s->access_mode == regs_config_b) {
            return s->xoff[(addr & 7) >> 2];
        }
        return omap_uart_serial_read(s, addr);
    case 0x20:	/* MDR1 */
        return s->mdr[0];
    case 0x24:	/* MDR2 */
        return s->mdr[1];
    case 0x28: /* SFLSR */
        return 0;
    case 0x2c: /* RESUME */
        return 0;
    case 0x30: /* SFREGL */
        return 0;
    case 0x34: /* SFREGH */
        return 0;
    case 0x38: /* UASR/BLR */
        if (s->access_mode != regs_operational) {
            return 0; /* FIXME: return correct autodetect value */
        }
        return s->blr;
    case 0x3c: /* ACREG */
        return (s->access_mode != regs_operational) ? 0 : s->acreg;
    case 0x40:	/* SCR */
        return s->scr;
    case 0x44:	/* SSR */
        return 0x0;
    case 0x48:	/* EBLR (OMAP2) */
        return s->eblr;
    case 0x4C:	/* OSC_12M_SEL (OMAP1) */
        return s->clksel;
    case 0x50:	/* MVR */
        return s->revision;
    case 0x54:	/* SYSC (OMAP2) */
        return s->syscontrol;
    case 0x58:	/* SYSS (OMAP2) */
        return 1;
    case 0x5c:	/* WER (OMAP2) */
        return s->wkup;
    case 0x60:	/* CFPS (OMAP2) */
        return s->cfps;
    case 0x64:  /* RXFIFO_LVL_REG (OMAP36xx) */
        if (s->revision >= 0x52) {
            return fifo8_num_used(&s->serial.recv_fifo) & 0xff;
        }
        break;
    case 0x68:  /* TXFIFO_LVL_REG (OMAP36xx) */
        if (s->revision >= 0x52) {
            return fifo8_num_used(&s->serial.xmit_fifo) & 0xff;
        }
        break;
    }

    OMAP_BAD_REG(addr);
    return 0;
}

static void omap_uart_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    struct omap_uart_s *s = (struct omap_uart_s *) opaque;

    switch (addr) {
    case 0x00:
    case 0x04:
        omap_uart_serial_write(s, addr, value);
        break;
    case 0x08:
        if (s->access_mode == regs_config_b) {
            s->efr = value;
        } else {
            omap_uart_serial_write(s, addr, value);
        }
        break;
    case 0x0c:
        if ((value & 0xff) == 0xbf) {
            s->access_mode = regs_config_b;
        } else if (value & 0x80) {
            s->access_mode = regs_config_a;
        } else {
            s->access_mode = regs_operational;
        }
        omap_uart_serial_write(s, addr, value);
        break;
    case 0x10:
    case 0x14:
        if (s->access_mode == regs_config_b) {
            s->xon[(addr & 7) >> 2] = value;
        } else {
            if (addr == 0x10) {
                /* Bits 5 and 6 are handled at this level; they can
                 * only be written if EFR_REG:ENHANCED_EN is set.
                 */
                if (s->efr & 0x10) {
                    s->mcr_cache = value & 0x60;
                }
            }
            omap_uart_serial_write(s, addr, value);
        }
        break;
    case 0x18:
    case 0x1c:
        if (tcr_tlr_mode(s)) {
            if (addr == 0x18) {
                s->tcr = value & 0xff;
            } else {
                s->tlr = value & 0xff;
            }
        } else if (s->access_mode == regs_config_b) {
            s->xoff[(addr & 7) >> 2] = value;
        } else {
            omap_uart_serial_write(s, addr, value);
        }
        break;
    case 0x20:	/* MDR1 */
        s->mdr[0] = value & 0x7f;
        break;
    case 0x24:	/* MDR2 */
        s->mdr[1] = value & 0xff;
        break;
    case 0x28: /* TXFLL */
    case 0x2c: /* TXFLH */
    case 0x30: /* RXFLL */
    case 0x34: /* RXFLH */
        /* ignored */
        break;
    case 0x38: /* BLR */
        if (s->access_mode == regs_operational) {
            s->blr = value & 0xc0;
        }
        break;
    case 0x3c: /* ACREG */
        if (s->access_mode == regs_operational) {
            s->acreg = value & 0xff;
        }
        break;
    case 0x40:	/* SCR */
        s->scr = value & 0xff;
        break;
    case 0x44:	/* SSR */
        OMAP_RO_REG(addr);
        break;
    case 0x48:	/* EBLR (OMAP2) */
        s->eblr = value & 0xff;
        break;
    case 0x4C:	/* OSC_12M_SEL (OMAP1) */
        s->clksel = value & 1;
        break;
    case 0x50:	/* MVR */
        OMAP_RO_REG(addr);
        break;
    case 0x54:	/* SYSC (OMAP2) */
        s->syscontrol = value & 0x1d;
        if (value & 2) {
            /* TODO: reset s->serial also. */
            omap_uart_reset(DEVICE(s));
        }
        break;
    case 0x58:	/* SYSS (OMAP2) */
        OMAP_RO_REG(addr);
        break;
    case 0x5c:	/* WER (OMAP2) */
        s->wkup = value & 0x7f;
        break;
    case 0x60:	/* CFPS (OMAP2) */
        s->cfps = value & 0xff;
        break;
    default:
        OMAP_BAD_REG(addr);
    }
}

static const MemoryRegionOps omap_uart_ops = {
    .read = omap_uart_read,
    .write = omap_uart_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void omap_uart_instance_init(Object *obj)
{
    struct omap_uart_s *s = OMAP_UART(obj);

    object_initialize_child(obj, "serial", &s->serial, TYPE_SERIAL);
    /* the 16550 core owns the character device (one mux frontend only) */
    object_property_add_alias(obj, "chardev", OBJECT(&s->serial), "chardev");
}

static void omap_uart_realize(DeviceState *dev, Error **errp)
{
    struct omap_uart_s *s = OMAP_UART(dev);
    SysBusDevice *busdev = SYS_BUS_DEVICE(dev);

    /* TODO: DMA (TXRDY/RXRDY) is not modelled; the DMA request lines
     * exist for completeness. */
    qdev_prop_set_uint32(DEVICE(&s->serial), "baudbase", s->baudrate);
    if (!qdev_realize(DEVICE(&s->serial), NULL, errp)) {
        return;
    }
    sysbus_init_irq(busdev, &s->serial.irq);
    sysbus_init_irq(busdev, &s->tx_drq);
    sysbus_init_irq(busdev, &s->rx_drq);
    memory_region_init_io(&s->iomem, OBJECT(s), &omap_uart_ops, s,
                          "omap_uart", s->mmio_size);
    /* The DMA request this device raises from inside a register
     * access makes the sDMA access its data register synchronously (as in
     * qemu 1.x); allow that nested access */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(busdev, &s->iomem);
}

static Property omap_uart_properties[] = {
    DEFINE_PROP_UINT32("revision", struct omap_uart_s, revision, 0x30),
    DEFINE_PROP_UINT32("mmio_size", struct omap_uart_s, mmio_size, 0x400),
    DEFINE_PROP_UINT32("baudrate", struct omap_uart_s, baudrate, 0),
    DEFINE_PROP_END_OF_LIST()
};

static void omap_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = omap_uart_realize;
    device_class_set_props(dc, omap_uart_properties);
    dc->reset = omap_uart_reset;
}

static const TypeInfo omap_uart_info = {
    .name = "omap_uart",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct omap_uart_s),
    .instance_init = omap_uart_instance_init,
    .class_init = omap_uart_class_init,
};

static void omap_uart_register_types(void)
{
    type_register_static(&omap_uart_info);
}

type_init(omap_uart_register_types)
