/*
 * SPI interface (qemu-linaro OMAP McSPI bus: word transfers with a bit
 * length and a channel / chip-select number).
 *
 * Copyright (C) 2007-2010 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) any later version of the License.
 */
#ifndef HW_OMAP_SPI_BUS_H
#define HW_OMAP_SPI_BUS_H

#include "hw/qdev-core.h"
#include "qom/object.h"

typedef struct SPIDevice SPIDevice;
typedef struct SPIBus SPIBus;

#define TYPE_SPI_DEVICE "omap-spi-device"
OBJECT_DECLARE_TYPE(SPIDevice, SPIDeviceClass, SPI_DEVICE)

typedef uint32_t (*spi_txrx_cb)(SPIDevice *dev, uint32_t, int);

struct SPIDeviceClass {
    DeviceClass parent_class;

    DeviceRealize realize;
    spi_txrx_cb txrx;
};

struct SPIDevice {
    DeviceState qdev;

    /* internal fields used by SPI code */
    uint8_t channel;
};

SPIBus *spi_init_bus(DeviceState *parent, const char *name, int num_channels);
uint32_t spi_txrx(SPIBus *bus, int channel, uint32_t data, int len);

DeviceState *spi_create_device(SPIBus *bus, const char *name, int ch);
DeviceState *spi_create_device_noinit(SPIBus *bus, const char *name, int ch);
void spi_realize_device(DeviceState *dev);

#endif
