/*
 * Nokia N900 (RX-51) machine, OMAP3430.
 *
 * Ported to QEMU 9.1 from qemu-linaro 1.7.0-2014.01 (hw/arm/nseries.c),
 * Copyright (C) 2008-2010 Nokia Corporation and the qemu-linaro authors.
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
 */
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/arm/omap.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "hw/ssi/omap_spi_bus.h"
#include "hw/block/flash.h"
#include "hw/input/tsc2xxx.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "sysemu/blockdev.h"
#include "sysemu/block-backend.h"
#include "sysemu/reset.h"
#include "ui/console.h"
#include "exec/address-spaces.h"
#include "cpu.h"

#define N900_SDRAM_SIZE (256 * MiB)

#define N900_ONENAND_GPIO       65
#define N900_CAMFOCUS_GPIO      68
#define N900_CAMLAUNCH_GPIO     69
#define N900_SLIDE_GPIO         71
#define N900_PROXIMITY_GPIO     89
#define N900_HEADPHONE_EN_GPIO  98
#define N900_TSC2005_IRQ_GPIO   100
#define N900_TSC2005_RESET_GPIO 104
#define N900_CAMCOVER_GPIO      110
#define N900_KBLOCK_GPIO        113
#define N900_SDCOVER_GPIO       160 /* back cover switch of the microSD slot (MMC1) */
#define N900_HEADPHONE_GPIO     177
#define N900_LIS302DL_INT2_GPIO 180
#define N900_LIS302DL_INT1_GPIO 181

#define TRACE_BQ2415X(...)
#define TRACE_TPA6130(...)
#define TRACE_LIS302DL(...)
#define TRACE_MIPID(...)

/* ------------------------------------------------------------------------ */
/* SSI (modem link) stub */

static uint64_t ssi_read(void *opaque, hwaddr addr, unsigned size)
{
    switch (addr) {
    case 0x00: /* REVISION */
        return 0x10;
    case 0x14: /* SYSSTATUS */
        return 1; /* RESETDONE */
    default:
        break;
    }
    return 0;
}

static void ssi_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
}

static const MemoryRegionOps ssi_ops = {
    .read = ssi_read,
    .write = ssi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/* ------------------------------------------------------------------------ */
/* LCD MIPI DBI-C controller (acx565akm on the N900) */

#define TYPE_LCD_MIPID "lcd_mipid"
OBJECT_DECLARE_SIMPLE_TYPE(MipidState, LCD_MIPID)

struct MipidState {
    SPIDevice spi;
    int resp[4];
    int param[4];
    int p;
    int pm;
    int cmd;

    int sleep;
    int booster;
    int te;
    int selfcheck;
    int partial;
    int normal;
    int vscr;
    int invert;
    int onoff;
    int gamma;
    uint32_t id;

    uint8_t n900;
    int cabc;
    int brightness;
    int ctrl;
};

static void mipid_reset(DeviceState *qdev)
{
    MipidState *s = LCD_MIPID(qdev);

    s->pm = 0;
    s->cmd = 0;

    s->sleep = 1;
    s->booster = 0;
    s->selfcheck =
            (1 << 7) |  /* Register loading OK.  */
            (1 << 5) |  /* The chip is attached.  */
            (1 << 4);   /* Display glass still in one piece.  */
    s->te = 0;
    s->partial = 0;
    s->normal = 1;
    s->vscr = 0;
    s->invert = 0;
    s->onoff = 1;
    s->gamma = 0;
}

static uint32_t mipid_txrx(SPIDevice *spidev, uint32_t cmd, int len)
{
    MipidState *s = LCD_MIPID(spidev);
    uint8_t ret;

    if (s->n900 && len == 10) {
        cmd >>= 1;
        len--;
    }

    if (len > 9) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bad SPI word width %i\n",
                      __func__, len);
        return 0;
    }

    if (s->p >= ARRAY_SIZE(s->resp)) {
        ret = 0;
    } else {
        ret = s->resp[s->p++];
    }
    if (s->pm-- > 0) {
        s->param[s->pm] = cmd;
    } else {
        s->cmd = cmd;
    }

    switch (s->cmd) {
    case 0x00:  /* NOP */
        break;

    case 0x01:  /* SWRESET */
        mipid_reset(DEVICE(s));
        break;

    case 0x02:  /* BSTROFF */
        s->booster = 0;
        break;
    case 0x03:  /* BSTRON */
        s->booster = 1;
        break;

    case 0x04:  /* RDDID */
        s->p = 0;
        s->resp[0] = (s->id >> 16) & 0xff;
        s->resp[1] = (s->id >>  8) & 0xff;
        s->resp[2] = (s->id >>  0) & 0xff;
        break;

    case 0x06:  /* RD_RED */
    case 0x07:  /* RD_GREEN */
    case 0x08:  /* RD_BLUE */
        s->p = 0;
        s->resp[0] = 0x01;
        break;

    case 0x09:  /* RDDST */
        s->p = 0;
        s->resp[0] = s->booster << 7;
        s->resp[1] = (5 << 4) | (s->partial << 2) |
                (s->sleep << 1) | s->normal;
        s->resp[2] = (s->vscr << 7) | (s->invert << 5) |
                (s->onoff << 2) | (s->te << 1) | (s->gamma >> 2);
        s->resp[3] = s->gamma << 6;
        break;

    case 0x0a:  /* RDDPM */
        s->p = 0;
        s->resp[0] = (s->onoff << 2) | (s->normal << 3) | (s->sleep << 4) |
                (s->partial << 5) | (s->sleep << 6) | (s->booster << 7);
        break;
    case 0x0b:  /* RDDMADCTR */
        s->p = 0;
        s->resp[0] = 0;
        break;
    case 0x0c:  /* RDDCOLMOD */
        s->p = 0;
        s->resp[0] = 5; /* 65K colours */
        break;
    case 0x0d:  /* RDDIM */
        s->p = 0;
        s->resp[0] = (s->invert << 5) | (s->vscr << 7) | s->gamma;
        break;
    case 0x0e:  /* RDDSM */
        s->p = 0;
        s->resp[0] = s->te << 7;
        break;
    case 0x0f:  /* RDDSDR */
        s->p = 0;
        s->resp[0] = s->selfcheck;
        break;

    case 0x10:  /* SLPIN */
        s->sleep = 1;
        break;
    case 0x11:  /* SLPOUT */
        s->sleep = 0;
        s->selfcheck ^= 1 << 6; /* POFF self-diagnosis Ok */
        break;

    case 0x12:  /* PTLON */
        s->partial = 1;
        s->normal = 0;
        s->vscr = 0;
        break;
    case 0x13:  /* NORON */
        s->partial = 0;
        s->normal = 1;
        s->vscr = 0;
        break;

    case 0x20:  /* INVOFF */
        s->invert = 0;
        break;
    case 0x21:  /* INVON */
        s->invert = 1;
        break;

    case 0x22:  /* APOFF */
    case 0x23:  /* APON */
        goto bad_cmd;

    case 0x25:  /* WRCNTR */
        if (s->pm < 0) {
            s->pm = 1;
        }
        goto bad_cmd;

    case 0x26:  /* GAMSET */
        if (!s->pm) {
            s->gamma = ctz32(s->param[0] & 0xf);
        } else if (s->pm < 0) {
            s->pm = 1;
        }
        break;

    case 0x28:  /* DISPOFF */
        s->onoff = 0;
        break;
    case 0x29:  /* DISPON */
        s->onoff = 1;
        break;

    case 0x2a:  /* CASET */
    case 0x2b:  /* RASET */
    case 0x2c:  /* RAMWR */
    case 0x2d:  /* RGBSET */
    case 0x2e:  /* RAMRD */
    case 0x30:  /* PTLAR */
    case 0x33:  /* SCRLAR */
        goto bad_cmd;

    case 0x34:  /* TEOFF */
        s->te = 0;
        break;
    case 0x35:  /* TEON */
        if (!s->pm) {
            s->te = 1;
        } else if (s->pm < 0) {
            s->pm = 1;
        }
        break;

    case 0x36:  /* MADCTR */
        goto bad_cmd;

    case 0x37:  /* VSCSAD */
        s->partial = 0;
        s->normal = 0;
        s->vscr = 1;
        break;

    case 0x38:  /* IDMOFF */
    case 0x39:  /* IDMON */
        goto bad_cmd;
    case 0x3a:  /* COLMOD */
        if (s->pm < 0) {
            s->pm = 1;
        }
        break;

    case 0x51: /* WRITE_BRIGHTNESS */
        if (!s->n900) {
            goto bad_cmd;
        }
        if (!s->pm) {
            s->brightness = s->param[0] & 0xff;
        } else if (s->pm < 0) {
            s->pm = 1;
        }
        break;
    case 0x52: /* READ_BRIGHTNESS */
        if (!s->n900) {
            goto bad_cmd;
        }
        s->p = 0;
        s->resp[0] = s->brightness;
        break;
    case 0x53: /* WRITE_CTRL */
        if (!s->n900) {
            goto bad_cmd;
        }
        if (!s->pm) {
            s->ctrl = s->param[0] & 0xff;
        } else if (s->pm < 0) {
            s->pm = 1;
        }
        break;
    case 0x54: /* READ_CTRL */
        if (!s->n900) {
            goto bad_cmd;
        }
        s->p = 0;
        s->resp[0] = s->ctrl;
        break;
    case 0x55: /* WRITE_CABC */
        if (!s->n900) {
            goto bad_cmd;
        }
        if (!s->pm) {
            s->cabc = s->param[0] & 0xff;
        } else if (s->pm < 0) {
            s->pm = 1;
        }
        break;
    case 0x56: /* READ_CABC */
        if (!s->n900) {
            goto bad_cmd;
        }
        s->p = 0;
        s->resp[0] = s->cabc;
        break;

    case 0xb0:  /* CLKINT / DISCTL */
    case 0xb1:  /* CLKEXT */
        if (s->pm < 0) {
            s->pm = 2;
        }
        break;

    case 0xb4:  /* FRMSEL */
        break;

    case 0xb5:  /* FRM8SEL */
    case 0xb6:  /* TMPRNG / INIESC */
    case 0xb7:  /* TMPHIS / NOP2 */
    case 0xb8:  /* TMPREAD / MADCTL */
    case 0xba:  /* DISTCTR */
    case 0xbb:  /* EPVOL */
        goto bad_cmd;

    case 0xbd:  /* Unknown */
        s->p = 0;
        s->resp[0] = 0;
        s->resp[1] = 1;
        break;

    case 0xc2:  /* IFMOD */
        if (s->pm < 0) {
            s->pm = (s->n900) ? 3 : 2;
        }
        break;

    case 0xc6:  /* PWRCTL */
    case 0xc7:  /* PPWRCTL */
    case 0xd0:  /* EPWROUT */
    case 0xd1:  /* EPWRIN */
    case 0xd4:  /* RDEV */
    case 0xd5:  /* RDRR */
        goto bad_cmd;

    case 0xda:  /* RDID1 */
        s->p = 0;
        s->resp[0] = (s->id >> 16) & 0xff;
        break;
    case 0xdb:  /* RDID2 */
        s->p = 0;
        s->resp[0] = (s->id >>  8) & 0xff;
        break;
    case 0xdc:  /* RDID3 */
        s->p = 0;
        s->resp[0] = (s->id >>  0) & 0xff;
        break;

    default:
    bad_cmd:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown command %02x\n", __func__, s->cmd);
        break;
    }

    return ret;
}

static Property mipid_properties[] = {
    DEFINE_PROP_UINT32("id", MipidState, id, 0),
    DEFINE_PROP_UINT8("n900", MipidState, n900, 0),
    DEFINE_PROP_END_OF_LIST()
};

static void mipid_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SPIDeviceClass *k = SPI_DEVICE_CLASS(klass);

    k->txrx = mipid_txrx;
    dc->reset = mipid_reset;
    device_class_set_props(dc, mipid_properties);
}

static const TypeInfo mipid_info = {
    .name = TYPE_LCD_MIPID,
    .parent = TYPE_SPI_DEVICE,
    .instance_size = sizeof(MipidState),
    .class_init = mipid_class_init,
};

/* ------------------------------------------------------------------------ */
/* LIS302DL accelerometer */

#define TYPE_LIS302DL "lis302dl"
OBJECT_DECLARE_SIMPLE_TYPE(LIS302DLState, LIS302DL)

struct LIS302DLState {
    I2CSlave i2c;
    int firstbyte;
    uint8_t reg;

    qemu_irq irq[2];
    int8_t axis_max, axis_step;
    int noise, dr_test_ack;

    uint8_t ctrl1, ctrl2, ctrl3;
    uint8_t status;
    struct {
        uint8_t cfg, src, ths, dur;
    } ff_wu[2];
    struct {
        uint8_t cfg, src, thsy_x, thsz;
        uint8_t timelimit, latency, window;
    } click;

    int32_t x, y, z;
};

static void lis302dl_interrupt_update(LIS302DLState *s)
{
    int active = (s->ctrl3 & 0x80) ? 0 : 1;
    int cond, latch;
    int i;
    for (i = 0; i < 2; i++) {
        switch ((s->ctrl3 >> (i * 3)) & 0x07) {
        case 1:
            cond = s->ff_wu[0].src & 0x40;
            latch = s->ff_wu[0].cfg & 0x40;
            break;
        case 2:
            cond = s->ff_wu[1].src & 0x40;
            latch = s->ff_wu[1].cfg & 0x40;
            break;
        case 3:
            cond = ((s->ff_wu[0].src | s->ff_wu[1].src) & 0x40);
            latch = ((s->ff_wu[0].cfg | s->ff_wu[1].cfg) & 0x40);
            break;
        case 4:
            cond = (((s->ff_wu[0].src | s->ff_wu[1].src) & 0x3f) &
                    (((s->ctrl1 & 0x01) ? 0x03 : 0x00) |
                     ((s->ctrl1 & 0x02) ? 0x0c : 0x00) |
                     ((s->ctrl1 & 0x04) ? 0x30 : 0x00)));
            latch = 0;
            break;
        case 7:
            cond = s->click.src & 0x40;
            latch = s->click.cfg & 0x40;
            break;
        default:
            cond = 0;
            latch = 0;
            break;
        }
        qemu_set_irq(s->irq[i], cond ? active : !active);
        if (cond && !latch) {
            qemu_set_irq(s->irq[i], !active);
        }
    }
}

static void lis302dl_trigger(LIS302DLState *s, int axis, int value)
{
    uint8_t bit;
    int i;

    if (value > s->axis_max) {
        value = s->axis_max;
    }
    if (value < -s->axis_max) {
        value = -s->axis_max;
    }
    switch (axis) {
    case 0: s->x = value; break;
    case 1: s->y = value; break;
    case 2: s->z = value; break;
    default: break;
    }
    if (s->status & (0x01 << axis)) {
        s->status |= 0x10 << axis;
    } else {
        s->status |= 0x01 << axis;
    }
    if ((s->status & 0x07) == 0x07) {
        s->status |= 0x08;
    }
    if ((s->status & 0x70) == 0x70) {
        s->status |= 0x80;
    }
    bit = 0x02 << (axis << 1); /* over threshold */
    s->ff_wu[0].src |= bit;
    s->ff_wu[1].src |= bit;

    for (i = 0; i < 2; i++) {
        if (s->ff_wu[i].src & 0x3f) {
            if (s->ff_wu[i].cfg & 0x80) {
                if ((s->ff_wu[i].cfg & 0x3f) == (s->ff_wu[i].src & 0x3f)) {
                    s->ff_wu[i].src |= 0x40;
                }
            } else {
                if (s->ff_wu[i].src & s->ff_wu[i].cfg & 0x3f) {
                    s->ff_wu[i].src |= 0x40;
                }
            }
        }
    }

    lis302dl_interrupt_update(s);
}

static void lis302dl_step(DeviceState *dev, int axis, int high, int activate)
{
    LIS302DLState *s = LIS302DL(dev);
    if (activate) {
        int v = 0;
        switch (axis) {
        case 0: v = s->x + (high ? s->axis_step : -s->axis_step); break;
        case 1: v = s->y + (high ? s->axis_step : -s->axis_step); break;
        case 2: v = s->z + (high ? s->axis_step : -s->axis_step); break;
        default: break;
        }
        if (v > s->axis_max) {
            v = -(s->axis_max - s->axis_step);
        }
        if (v < -s->axis_max) {
            v = s->axis_max - s->axis_step;
        }
        lis302dl_trigger(s, axis, v);
    }
}

static void lis302dl_reset(DeviceState *ds)
{
    LIS302DLState *s = LIS302DL(ds);

    s->firstbyte = 0;
    s->reg = 0;

    s->noise = 4;
    s->dr_test_ack = 0;

    s->ctrl1 = 0x03;
    s->ctrl2 = 0x00;
    s->ctrl3 = 0x00;
    s->status = 0x00;

    memset(s->ff_wu, 0x00, sizeof(s->ff_wu));
    memset(&s->click, 0x00, sizeof(s->click));

    s->x = 0;
    s->y = -s->axis_max;
    s->z = 0;

    lis302dl_interrupt_update(s);
}

static int lis302dl_event(I2CSlave *i2c, enum i2c_event event)
{
    LIS302DLState *s = LIS302DL(i2c);
    if (event == I2C_START_SEND) {
        s->firstbyte = 1;
    }
    return 0;
}

static uint8_t lis302dl_readcoord(LIS302DLState *s, int coord)
{
    int v;

    switch (coord) {
    case 0:
        v = s->x;
        break;
    case 1:
        v = s->y;
        break;
    default:
        v = s->z;
        break;
    }
    s->status &= ~(0x88 | (0x11 << coord));
    if (s->ctrl1 & 0x10) {
        int dr1, dr2;
        switch (coord) {
        case 0:
            v -= s->noise;
            break;
        default:
            v += s->noise;
            break;
        }
        if (++s->noise == 32) {
            s->noise = 4;
        }
        dr1 = ((s->ctrl3 & 0x07) == 4);
        dr2 = (((s->ctrl3 >> 3) & 0x07) == 4);
        if (!s->dr_test_ack++) {
            if (dr1) {
                qemu_irq_pulse(s->irq[0]);
            }
            if (dr2) {
                qemu_irq_pulse(s->irq[1]);
            }
        } else if (s->dr_test_ack == 1 + (dr1 + dr2) * 3) {
            s->dr_test_ack = 0;
        }
    }
    return (uint8_t)v;
}

static uint8_t lis302dl_rx(I2CSlave *i2c)
{
    LIS302DLState *s = LIS302DL(i2c);
    int value = 0;
    int n = 0;
    switch (s->reg & 0x7f) {
    case 0x0f:
        value = 0x3b; /* WHO_AM_I */
        break;
    case 0x20:
        value = s->ctrl1;
        break;
    case 0x21:
        value = s->ctrl2;
        break;
    case 0x22:
        value = s->ctrl3;
        break;
    case 0x27:
        value = s->status;
        break;
    case 0x29:
        value = lis302dl_readcoord(s, 0);
        break;
    case 0x2b:
        value = lis302dl_readcoord(s, 1);
        break;
    case 0x2d:
        value = lis302dl_readcoord(s, 2);
        break;
    case 0x34:
        n++;
        /* fall through */
    case 0x30:
        value = s->ff_wu[n].cfg;
        break;
    case 0x35:
        n++;
        /* fall through */
    case 0x31:
        value = s->ff_wu[n].src;
        s->ff_wu[n].src = 0;
        lis302dl_interrupt_update(s);
        break;
    case 0x36:
        n++;
        /* fall through */
    case 0x32:
        value = s->ff_wu[n].ths;
        break;
    case 0x37:
        n++;
        /* fall through */
    case 0x33:
        value = s->ff_wu[n].dur;
        break;
    case 0x38:
        value = s->click.cfg;
        break;
    case 0x39:
        value = s->click.src;
        s->click.src &= ~0x40;
        lis302dl_interrupt_update(s);
        break;
    case 0x3b:
        value = s->click.thsy_x;
        break;
    case 0x3c:
        value = s->click.thsz;
        break;
    case 0x3d:
        value = s->click.timelimit;
        break;
    case 0x3e:
        value = s->click.latency;
        break;
    case 0x3f:
        value = s->click.window;
        break;
    default:
        /* unused/reserved registers read as zero */
        value = 0;
        break;
    }
    if (s->reg & 0x80) { /* auto-increment? */
        s->reg = (s->reg + 1) | 0x80;
    }
    return value;
}

static int lis302dl_tx(I2CSlave *i2c, uint8_t data)
{
    LIS302DLState *s = LIS302DL(i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        int n = 0;
        switch (s->reg & 0x7f) {
        case 0x20:
            s->ctrl1 = data;
            break;
        case 0x21:
            s->ctrl2 = data;
            break;
        case 0x22:
            s->ctrl3 = data;
            lis302dl_interrupt_update(s);
            break;
        case 0x34:
            n++;
            /* fall through */
        case 0x30:
            s->ff_wu[n].cfg = data;
            break;
        case 0x36:
            n++;
            /* fall through */
        case 0x32:
            s->ff_wu[n].ths = data;
            break;
        case 0x37:
            n++;
            /* fall through */
        case 0x33:
            s->ff_wu[n].dur = data;
            break;
        case 0x38:
            s->click.cfg = data;
            break;
        case 0x39:
            s->click.src = data;
            break;
        case 0x3b:
            s->click.thsy_x = data;
            break;
        case 0x3c:
            s->click.thsz = data;
            break;
        case 0x3d:
            s->click.timelimit = data;
            break;
        case 0x3e:
            s->click.latency = data;
            break;
        case 0x3f:
            s->click.window = data;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: unknown register 0x%02x (value 0x%02x)\n",
                          __func__, s->reg & 0x7f, data);
            break;
        }
        if (s->reg & 0x80) { /* auto-increment? */
            s->reg = (s->reg + 1) | 0x80;
        }
    }
    return 0;
}

static void lis302dl_realize(DeviceState *dev, Error **errp)
{
    LIS302DLState *s = LIS302DL(dev);
    s->axis_max = 58;
    s->axis_step = s->axis_max;
    qdev_init_gpio_out(dev, s->irq, 2);
}

static void lis302dl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    dc->realize = lis302dl_realize;
    k->event = lis302dl_event;
    k->recv = lis302dl_rx;
    k->send = lis302dl_tx;
    dc->reset = lis302dl_reset;
}

static const TypeInfo lis302dl_info = {
    .name = TYPE_LIS302DL,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LIS302DLState),
    .class_init = lis302dl_class_init,
};

/* ------------------------------------------------------------------------ */
/* BQ24150 charger */

#define TYPE_BQ2415X "bq2415x"
OBJECT_DECLARE_SIMPLE_TYPE(BQ2415XState, BQ2415X)

struct BQ2415XState {
    I2CSlave i2c;
    int firstbyte;
    uint8_t reg;

    uint8_t id;
    uint8_t st_ctrl;
    uint8_t ctrl;
    uint8_t bat_v;
    uint8_t tcc;
};

static void bq2415x_reset(DeviceState *ds)
{
    BQ2415XState *s = BQ2415X(ds);

    s->firstbyte = 0;
    s->reg = 0;

    s->st_ctrl = 0x50 | 0x80;
    s->ctrl = 0x30;
    s->bat_v = 0x0a;
    s->tcc = 0xa1;
}

static int bq2415x_event(I2CSlave *i2c, enum i2c_event event)
{
    BQ2415XState *s = BQ2415X(i2c);
    if (event == I2C_START_SEND) {
        s->firstbyte = 1;
    }
    return 0;
}

static uint8_t bq2415x_rx(I2CSlave *i2c)
{
    BQ2415XState *s = BQ2415X(i2c);
    int value;
    switch (s->reg) {
    case 0x00:
        value = s->st_ctrl;
        break;
    case 0x01:
        value = s->ctrl;
        break;
    case 0x02:
        value = s->bat_v;
        break;
    case 0x03:
    case 0x3b:
        value = s->id;
        break;
    case 0x04:
        value = s->tcc;
        break;
    default:
        value = 0;
        break;
    }
    s->reg++;
    return value;
}

static int bq2415x_tx(I2CSlave *i2c, uint8_t data)
{
    BQ2415XState *s = BQ2415X(i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        switch (s->reg) {
        case 0x00:
            s->st_ctrl = (s->st_ctrl & 0x3f) | (data & 0x40) | 0x80;
            break;
        case 0x01:
            s->ctrl = data;
            break;
        case 0x02:
            s->bat_v = data;
            break;
        case 0x04:
            s->tcc = data | 0x80;
            break;
        default:
            break;
        }
        s->reg++;
    }
    return 0;
}

static Property bq2415x_properties[] = {
    DEFINE_PROP_UINT8("id", BQ2415XState, id, 0x49),
    DEFINE_PROP_END_OF_LIST(),
};

static void bq2415x_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    k->event = bq2415x_event;
    k->recv = bq2415x_rx;
    k->send = bq2415x_tx;
    device_class_set_props(dc, bq2415x_properties);
    dc->reset = bq2415x_reset;
}

static const TypeInfo bq2415x_info = {
    .name = TYPE_BQ2415X,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(BQ2415XState),
    .class_init = bq2415x_class_init,
};

/* ------------------------------------------------------------------------ */
/* TPA6130A2 headphone amplifier */

#define TYPE_TPA6130 "tpa6130"
OBJECT_DECLARE_SIMPLE_TYPE(TPA6130State, TPA6130)

struct TPA6130State {
    I2CSlave i2c;
    int firstbyte;
    int reg;
    uint8_t data[3];
};

static void tpa6130_reset(DeviceState *ds)
{
    TPA6130State *s = TPA6130(ds);
    s->firstbyte = 0;
    s->reg = 0;
    memset(s->data, 0, sizeof(s->data));
}

static int tpa6130_event(I2CSlave *i2c, enum i2c_event event)
{
    TPA6130State *s = TPA6130(i2c);
    if (event == I2C_START_SEND) {
        s->firstbyte = 1;
    }
    return 0;
}

static uint8_t tpa6130_rx(I2CSlave *i2c)
{
    TPA6130State *s = TPA6130(i2c);
    int value = 0;
    switch (s->reg) {
    case 1 ... 3:
        value = s->data[s->reg - 1];
        break;
    case 4: /* VERSION */
        value = 0x01;
        break;
    default:
        break;
    }
    s->reg++;
    return value;
}

static int tpa6130_tx(I2CSlave *i2c, uint8_t data)
{
    TPA6130State *s = TPA6130(i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        switch (s->reg) {
        case 1 ... 3:
            s->data[s->reg - 1] = data;
            break;
        default:
            break;
        }
        s->reg++;
    }
    return 0;
}

static void tpa6130_irq(void *opaque, int n, int level)
{
    /* headphone enable: nothing to do */
}

static void tpa6130_realize(DeviceState *dev, Error **errp)
{
    qdev_init_gpio_in(dev, tpa6130_irq, 1);
}

static void tpa6130_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    dc->realize = tpa6130_realize;
    k->event = tpa6130_event;
    k->recv = tpa6130_rx;
    k->send = tpa6130_tx;
    dc->reset = tpa6130_reset;
}

static const TypeInfo tpa6130_info = {
    .name = TYPE_TPA6130,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TPA6130State),
    .class_init = tpa6130_class_init,
};

/* ------------------------------------------------------------------------ */
/* TLV320AIC3x (AIC34) audio codec: control registers only, no audio path.
 * Without it every codec access NACKs and Linux spends ~1 min in I2C
 * error handling while booting. */

#define TYPE_TLV320AIC3X "tlv320aic3x"
OBJECT_DECLARE_SIMPLE_TYPE(AIC3xState, TLV320AIC3X)

struct AIC3xState {
    I2CSlave i2c;
    int firstbyte;
    uint8_t reg;
    uint8_t regs[2][128];   /* page 0: control, page 1: filter coefficients */
};

static void aic3x_reset_regs(AIC3xState *s)
{
    memset(s->regs, 0, sizeof(s->regs));
    s->regs[0][102] = 0x02; /* clock generation control: CLKDIV_IN = MCLK */
}

static void aic3x_reset(DeviceState *ds)
{
    AIC3xState *s = TLV320AIC3X(ds);
    s->firstbyte = 0;
    s->reg = 0;
    aic3x_reset_regs(s);
}

static uint8_t aic3x_read_reg(AIC3xState *s, uint8_t reg)
{
    uint8_t *r = s->regs[0];
    int page = r[0] & 1;

    if (reg == 0) {
        return r[0];
    }
    if (page == 0 && reg == 94) {
        /* module power status: DACs, HP and line outputs follow their
         * power-enable bits (the driver polls these after power changes) */
        return ((r[37] & 0x80) ? 0x80 : 0) |   /* left DAC */
               ((r[37] & 0x40) ? 0x40 : 0) |   /* right DAC */
               ((r[79] & 1) ? 0x20 : 0) |      /* MONO_LOP/M */
               ((r[86] & 1) ? 0x10 : 0) |      /* LEFT_LOP/M */
               ((r[93] & 1) ? 0x08 : 0) |      /* RIGHT_LOP/M */
               ((r[51] & 1) ? 0x04 : 0) |      /* HPLOUT */
               ((r[65] & 1) ? 0x02 : 0);       /* HPROUT */
    }
    if (page == 0 && reg == 95) {
        return ((r[58] & 1) ? 0x08 : 0) |      /* HPLCOM */
               ((r[72] & 1) ? 0x04 : 0);       /* HPRCOM */
    }
    return reg < 128 ? s->regs[page][reg] : 0;
}

static void aic3x_write_reg(AIC3xState *s, uint8_t reg, uint8_t value)
{
    int page = s->regs[0][0] & 1;

    if (reg == 0) {
        s->regs[0][0] = value & 1;
    } else if (page == 0 && reg == 1 && (value & 0x80)) {
        aic3x_reset_regs(s);                   /* software reset */
    } else if (reg < 128 && !(page == 0 && (reg == 94 || reg == 95))) {
        s->regs[page][reg] = value;
    }
}

static int aic3x_event(I2CSlave *i2c, enum i2c_event event)
{
    AIC3xState *s = TLV320AIC3X(i2c);
    if (event == I2C_START_SEND) {
        s->firstbyte = 1;
    }
    return 0;
}

static uint8_t aic3x_rx(I2CSlave *i2c)
{
    AIC3xState *s = TLV320AIC3X(i2c);
    return aic3x_read_reg(s, s->reg++);
}

static int aic3x_tx(I2CSlave *i2c, uint8_t data)
{
    AIC3xState *s = TLV320AIC3X(i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        aic3x_write_reg(s, s->reg++, data);
    }
    return 0;
}

static void aic3x_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    k->event = aic3x_event;
    k->recv = aic3x_rx;
    k->send = aic3x_tx;
    dc->reset = aic3x_reset;
}

static const TypeInfo aic3x_info = {
    .name = TYPE_TLV320AIC3X,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(AIC3xState),
    .class_init = aic3x_class_init,
};

/* ------------------------------------------------------------------------ */
/* The machine */

struct n900_s {
    struct omap_mpu_state_s *cpu;
    void *twl4030;
    DeviceState *nand;
    DeviceState *mipid;
    DeviceState *tsc2005;
    DeviceState *bq2415x;
    DeviceState *tpa6130;
    DeviceState *lis302dl;
    DeviceState *smc;
    int extended_key;
    int slide_open;
    int camera_cover_open;
    int headphone_connected;
    int sd_cover_open;
};

static void n900_gpio(struct n900_s *s, int line, int level)
{
    qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio, line), level);
}

/* this takes care of the keys which are not located on the
 * n900 keypad (note that volume up/down keys are handled by
 * the keypad eventhough the keys are not located on the keypad)
 * as well as triggering some other hardware button/switch-like
 * events that are mapped to the host keyboard:
 *
 * escape ... power
 * f1 ....... keypad slider open/close
 * f2 ....... keypad lock
 * f3 ....... camera lens cover open/close
 * f4 ....... camera focus
 * f5 ....... camera take picture
 * f6 ....... stereo headphone connect/disconnect
 * f9 ....... back cover (microSD slot) open/close
 * kp1 ...... decrease accelerometer x axis value
 * kp2 ...... increase accelerometer x axis value
 * kp4 ...... decrease accelerometer y axis value
 * kp5 ...... increase accelerometer y axis value
 * kp7 ...... decrease accelerometer z axis value
 * kp8 ...... increase accelerometer z axis value
 *
 * Everything else goes to the TWL4030 keypad matrix. (qemu-linaro only
 * delivered keys to the most recently registered handler, so its keypad
 * handler never saw any; here the machine forwards them.)
 */
static void n900_key_handler(void *opaque, int keycode)
{
    struct n900_s *s = opaque;
    int raw = keycode;

    if (!s->extended_key && keycode == 0xe0) {
        s->extended_key = 0x80;
        twl4030_key_event(s->twl4030, raw);
        return;
    } else {
        int release = keycode & 0x80;
        keycode = (keycode & 0x7f) | s->extended_key;
        s->extended_key = 0;
        switch (keycode) {
        case 0x01: /* escape */
            twl4030_set_powerbutton_state(s->twl4030, !release);
            break;
        case 0x3b: /* f1 */
            if (release) {
                s->slide_open = !s->slide_open;
                n900_gpio(s, N900_SLIDE_GPIO, !s->slide_open);
            }
            break;
        case 0x3c: /* f2 */
            n900_gpio(s, N900_KBLOCK_GPIO, !!release);
            break;
        case 0x3d: /* f3 */
            if (release) {
                s->camera_cover_open = !s->camera_cover_open;
                n900_gpio(s, N900_CAMCOVER_GPIO, s->camera_cover_open);
            }
            break;
        case 0x3e: /* f4 */
            n900_gpio(s, N900_CAMFOCUS_GPIO, !!release);
            break;
        case 0x3f: /* f5 */
            n900_gpio(s, N900_CAMLAUNCH_GPIO, !!release);
            break;
        case 0x40: /* f6 */
            if (release) {
                s->headphone_connected = !s->headphone_connected;
                n900_gpio(s, N900_HEADPHONE_GPIO, !s->headphone_connected);
            }
            break;
        case 0x43: /* f9 */
            if (release) {
                s->sd_cover_open = !s->sd_cover_open;
                fprintf(stderr, "n900: back cover %s\n",
                        s->sd_cover_open ? "opened (microSD removed)"
                                         : "closed (microSD inserted)");
                n900_gpio(s, N900_SDCOVER_GPIO, s->sd_cover_open);
            }
            break;
        case 0x4f ... 0x50: /* kp1,2 */
            lis302dl_step(s->lis302dl, 0, keycode - 0x4f, !release);
            break;
        case 0x4b ... 0x4c: /* kp4,5 */
            lis302dl_step(s->lis302dl, 1, keycode - 0x4b, !release);
            break;
        case 0x47 ... 0x48: /* kp7,8 */
            lis302dl_step(s->lis302dl, 2, keycode - 0x47, !release);
            break;
        default:
            if (keycode & 0x80) {
                twl4030_key_event(s->twl4030, 0xe0);
            }
            twl4030_key_event(s->twl4030, raw);
            break;
        }
    }
}

static void n900_reset(void *opaque)
{
    struct n900_s *s = opaque;
    s->slide_open = 1;
    s->camera_cover_open = 0;
    s->headphone_connected = 0;
    s->sd_cover_open = 0;
    n900_gpio(s, N900_KBLOCK_GPIO, 1);
    n900_gpio(s, N900_HEADPHONE_GPIO, !s->headphone_connected);
    n900_gpio(s, N900_CAMLAUNCH_GPIO, 1);
    n900_gpio(s, N900_CAMFOCUS_GPIO, 1);
    n900_gpio(s, N900_CAMCOVER_GPIO, s->camera_cover_open);
    n900_gpio(s, N900_SLIDE_GPIO, !s->slide_open);
    n900_gpio(s, N900_SDCOVER_GPIO, s->sd_cover_open);
    omap3_boot_rom_emu(s->cpu);
}

static uint16_t n900_twl4030_madc_callback(twl4030_adc_type type, int ch)
{
    return 0x3ff;
}

static const TWL4030KeyMap n900_twl4030_keymap[] = {
    {0x10, 0, 0}, /* Q */
    {0x11, 0, 1}, /* W */
    {0x12, 0, 2}, /* E */
    {0x13, 0, 3}, /* R */
    {0x14, 0, 4}, /* T */
    {0x15, 0, 5}, /* Y */
    {0x16, 0, 6}, /* U */
    {0x17, 0, 7}, /* I */
    {0x18, 1, 0}, /* O */
    {0x20, 1, 1}, /* D */
    {0x34, 1, 2}, /* . */
    {0x2f, 1, 3}, /* V */
    {0xd0, 1, 4}, /* DOWN */
    {0x41, 1, 7}, /* F7 -- volume/zoom down */
    {0x19, 2, 0}, /* P */
    {0x21, 2, 1}, /* F */
    {0xc8, 2, 2}, /* UP */
    {0x30, 2, 3}, /* B */
    {0xcd, 2, 4}, /* RIGHT */
    {0x42, 2, 7}, /* F8 -- volume/zoom up */
    {0x33, 3, 0}, /* , */
    {0x22, 3, 1}, /* G */
    {0x1c, 3, 2}, /* ENTER */
    {0x31, 3, 3}, /* N */
    {0x0e, 4, 0}, /* BACKSPACE */
    {0x23, 4, 1}, /* H */
    {0x32, 4, 3}, /* M */
    {0x1d, 4, 4}, /* LEFTCTRL */
    {0x9d, 4, 4}, /* RIGHTCTRL */
    {0x24, 5, 1}, /* J */
    {0x2c, 5, 2}, /* Z */
    {0x39, 5, 3}, /* SPACE */
    {0x38, 5, 4}, /* LEFTALT -- "fn" */
    {0xb8, 5, 4}, /* RIGHTALT -- "fn" */
    {0x1e, 6, 0}, /* A */
    {0x25, 6, 1}, /* K */
    {0x2d, 6, 2}, /* X */
    {0x39, 6, 3}, /* SPACE */
    {0x2a, 6, 4}, /* LEFTSHIFT */
    {0x36, 6, 4}, /* RIGHTSHIFT */
    {0x1f, 7, 0}, /* S */
    {0x26, 7, 1}, /* L */
    {0x2e, 7, 2}, /* C */
    {0xcb, 7, 3}, /* LEFT */
    {-1, -1, -1}
};

static MouseTransformInfo n900_pointercal = {
    .x = 800,
    .y = 480,
    .a = {14114,  18, -2825064,  34,  -8765, 32972906, 65536},
};

/*
 * Linux guests occasionally froze for many minutes: the main loop was
 * asleep in ppoll() with a timeout computed before the guest
 * re-armed its clockevent (GPTIMER1, a few ms ahead) and never woke for it;
 * any main-loop activity (e.g. a monitor command) resumed the guest at once.
 * The root cause was not found; a 10 ms heartbeat bounds such a sleep, so a
 * missed wakeup costs at most one heartbeat period.
 */
#define N900_HEARTBEAT_MS 10

static void n900_heartbeat(void *opaque)
{
    QEMUTimer *t = opaque;
    timer_mod(t, qemu_clock_get_ms(QEMU_CLOCK_REALTIME) + N900_HEARTBEAT_MS);
}

static void n900_init(MachineState *machine)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ssi_iomem = g_new(MemoryRegion, 1);
    struct n900_s *s = g_malloc0(sizeof(*s));
    DriveInfo *dmtd = drive_get(IF_MTD, 0, 0);
    DriveInfo *dsd  = drive_get(IF_SD, 0, 0);
    I2CBus *i2c2, *i2c3;
    NICInfo *nd;
    uint64_t sdram_size;

    if (!dmtd && !dsd) {
        error_report("n900: an eMMC (-sd) or OneNAND (-mtdblock) image "
                     "is required");
        exit(1);
    }

    /* The N900 has 256 MiB. "-m" can raise this (up to the 1 GiB SDRC
     * window at 0x80000000) for guests that need more memory, e.g. a modern
     * distribution started with "mem=512M"; the default (and anything
     * smaller) stays 256 MiB. */
    sdram_size = N900_SDRAM_SIZE;
    if (machine->ram_size > sdram_size) {
        sdram_size = MIN(machine->ram_size, 1 * GiB);
    }

    s->cpu = omap3_mpu_init(sysmem, omap3430, sdram_size, machine->cpu_type,
                            serial_hd(1), serial_hd(2),
                            serial_hd(0), NULL);
    omap_lcd_panel_attach(s->cpu->dss);

    s->tsc2005 = spi_create_device(omap_mcspi_bus(s->cpu->mcspi, 0),
                                   "tsc2005", 0);
    qdev_connect_gpio_out(s->tsc2005, 0,
                          qdev_get_gpio_in(s->cpu->gpio,
                                           N900_TSC2005_IRQ_GPIO));
    tsc2005_set_transform(s->tsc2005, &n900_pointercal, 600, 1500);

    s->mipid = spi_create_device_noinit(omap_mcspi_bus(s->cpu->mcspi, 0),
                                        TYPE_LCD_MIPID, 2);
    qdev_prop_set_uint32(s->mipid, "id", 0x101234);
    qdev_prop_set_uint8(s->mipid, "n900", 1);
    spi_realize_device(s->mipid);

    s->nand = qdev_new("onenand");
    qdev_prop_set_uint16(s->nand, "manufacturer_id", NAND_MFR_SAMSUNG);
    qdev_prop_set_uint16(s->nand, "device_id", 0x40);
    qdev_prop_set_uint16(s->nand, "version_id", 0x121);
    qdev_prop_set_int32(s->nand, "shift", 1);
    if (dmtd) {
        qdev_prop_set_drive_err(s->nand, "drive", blk_by_legacy_dinfo(dmtd),
                                &error_fatal);
    }
    sysbus_realize_and_unref(SYS_BUS_DEVICE(s->nand), &error_fatal);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->nand), 0,
                       qdev_get_gpio_in(s->cpu->gpio, N900_ONENAND_GPIO));
    omap_gpmc_attach(s->cpu->gpmc, 0,
                     sysbus_mmio_get_region(SYS_BUS_DEVICE(s->nand), 0));

    /* -sd / "-drive if=sd,index=0" is the internal eMMC on MMC2,
     * "-drive if=sd,index=1" the external microSD slot on MMC1 */
    if (dsd) {
        omap3_mmc_attach(s->cpu->omap3_mmc[1], blk_by_legacy_dinfo(dsd), 0, 1);
    }
    dsd = drive_get(IF_SD, 0, 1);
    if (dsd) {
        omap3_mmc_attach(s->cpu->omap3_mmc[0], blk_by_legacy_dinfo(dsd), 0, 0);
    }

    memory_region_init_io(ssi_iomem, NULL, &ssi_ops, NULL, "n900_ssi", 0x3c00);
    memory_region_add_subregion(sysmem, 0x48058000, ssi_iomem);

    s->twl4030 = twl4030_init(omap_i2c_bus(s->cpu->i2c[0]),
                              qdev_get_gpio_in(s->cpu->ih[0],
                                               OMAP_INT_3XXX_SYS_NIRQ),
                              NULL, n900_twl4030_keymap);
    twl4030_madc_attach(s->twl4030, n900_twl4030_madc_callback);
    i2c2 = omap_i2c_bus(s->cpu->i2c[1]);
    s->bq2415x = DEVICE(i2c_slave_create_simple(i2c2, TYPE_BQ2415X, 0x6b));
    s->tpa6130 = DEVICE(i2c_slave_create_simple(i2c2, TYPE_TPA6130, 0x60));
    /* TLV320AIC34: two codec instances */
    i2c_slave_create_simple(i2c2, TYPE_TLV320AIC3X, 0x18);
    i2c_slave_create_simple(i2c2, TYPE_TLV320AIC3X, 0x19);
    qdev_connect_gpio_out(s->cpu->gpio, N900_HEADPHONE_EN_GPIO,
                          qdev_get_gpio_in(s->tpa6130, 0));
    i2c3 = omap_i2c_bus(s->cpu->i2c[2]);
    s->lis302dl = DEVICE(i2c_slave_create_simple(i2c3, TYPE_LIS302DL, 0x1d));
    qdev_connect_gpio_out(s->lis302dl, 0,
                          qdev_get_gpio_in(s->cpu->gpio,
                                           N900_LIS302DL_INT1_GPIO));
    qdev_connect_gpio_out(s->lis302dl, 1,
                          qdev_get_gpio_in(s->cpu->gpio,
                                           N900_LIS302DL_INT2_GPIO));

    /* qemu-linaro insisted on a NIC here (an smc91c111 on GPMC CS1 that the
     * real phone does not have); it is only added when one is configured. */
    nd = qemu_find_nic_info("smc91c111", false, NULL);
    if (nd) {
        s->smc = qdev_new("smc91c111");
        qdev_set_nic_properties(s->smc, nd);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(s->smc), &error_fatal);
        sysbus_connect_irq(SYS_BUS_DEVICE(s->smc), 0,
                           qdev_get_gpio_in(s->cpu->gpio, 54));
        omap_gpmc_attach(s->cpu->gpmc, 1,
                         sysbus_mmio_get_region(SYS_BUS_DEVICE(s->smc), 0));
    }

    qemu_add_kbd_event_handler(n900_key_handler, s);

    qemu_register_reset(n900_reset, s);

    {
        QEMUTimer *hb = g_new0(QEMUTimer, 1);
        timer_init_ms(hb, QEMU_CLOCK_REALTIME, n900_heartbeat, hb);
        n900_heartbeat(hb);
    }
}

static void n900_machine_init(MachineClass *mc)
{
    mc->desc = "Nokia N900 (OMAP3)";
    mc->init = n900_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-a8");
    mc->default_ram_size = N900_SDRAM_SIZE;
    mc->default_boot_order = "";
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("n900", n900_machine_init)

static void n900_register_types(void)
{
    type_register_static(&bq2415x_info);
    type_register_static(&tpa6130_info);
    type_register_static(&lis302dl_info);
    type_register_static(&mipid_info);
    type_register_static(&aic3x_info);
}

type_init(n900_register_types)
