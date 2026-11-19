/*
 * TI OMAP: accesses with a width a register block does not support.
 * Moved out of hw/arm/omap1.c (not built for the N900) unchanged.
 *
 * Copyright (C) 2006-2008 Andrzej Zaborowski  <balrog@zabor.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "exec/cpu-common.h"
#include "hw/arm/omap.h"


static inline void omap_log_badwidth(const char *funcname, hwaddr addr, int sz)
{
    qemu_log_mask(LOG_GUEST_ERROR, "%s: %d-bit register %#08" HWADDR_PRIx "\n",
                  funcname, 8 * sz, addr);
}

/* Should signal the TCMI/GPMC */
uint32_t omap_badwidth_read8(void *opaque, hwaddr addr)
{
    uint8_t ret;

    omap_log_badwidth(__func__, addr, 1);
    cpu_physical_memory_read(addr, &ret, 1);
    return ret;
}

void omap_badwidth_write8(void *opaque, hwaddr addr,
                uint32_t value)
{
    uint8_t val8 = value;

    omap_log_badwidth(__func__, addr, 1);
    cpu_physical_memory_write(addr, &val8, 1);
}

uint32_t omap_badwidth_read16(void *opaque, hwaddr addr)
{
    uint16_t ret;

    omap_log_badwidth(__func__, addr, 2);
    cpu_physical_memory_read(addr, &ret, 2);
    return ret;
}

void omap_badwidth_write16(void *opaque, hwaddr addr,
                uint32_t value)
{
    uint16_t val16 = value;

    omap_log_badwidth(__func__, addr, 2);
    cpu_physical_memory_write(addr, &val16, 2);
}

uint32_t omap_badwidth_read32(void *opaque, hwaddr addr)
{
    uint32_t ret;

    omap_log_badwidth(__func__, addr, 4);
    cpu_physical_memory_read(addr, &ret, 4);
    return ret;
}

void omap_badwidth_write32(void *opaque, hwaddr addr,
                uint32_t value)
{
    omap_log_badwidth(__func__, addr, 4);
    cpu_physical_memory_write(addr, &value, 4);
}
