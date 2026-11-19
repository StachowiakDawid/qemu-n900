/*
 * SPI interface.
 *
 * Copyright (C) 2007-2010 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) any later version of the License.
 */
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/omap_spi_bus.h"

struct SPIBus {
    BusState qbus;
    int channels;
    SPIDevice **device;
    void *pending_bus; /* bus a not-yet-realized device belongs to */
};

#define TYPE_SPI_BUS "omap-spi-bus"
OBJECT_DECLARE_SIMPLE_TYPE(SPIBus, SPI_BUS)

static Property spi_props[] = {
    DEFINE_PROP_UINT8("channel", SPIDevice, channel, 0),
    DEFINE_PROP_END_OF_LIST()
};

static const TypeInfo spi_bus_info = {
    .name = TYPE_SPI_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(SPIBus),
};

SPIBus *spi_init_bus(DeviceState *parent, const char *name, int num_channels)
{
    SPIBus *bus = SPI_BUS(qbus_new(TYPE_SPI_BUS, parent, name));
    bus->channels = num_channels;
    bus->device = g_new0(SPIDevice *, bus->channels);
    return bus;
}

uint32_t spi_txrx(SPIBus *bus, int channel, uint32_t data, int len)
{
    SPIDevice *dev;
    SPIDeviceClass *sc;

    if (channel < bus->channels) {
        dev = bus->device[channel];
        if (dev) {
            sc = SPI_DEVICE_GET_CLASS(dev);
            if (sc->txrx) {
                return sc->txrx(dev, data, len);
            }
        }
    } else {
        hw_error("%s: invalid channel %d\n", __func__, channel);
    }
    return 0;
}

DeviceState *spi_create_device_noinit(SPIBus *bus, const char *name, int ch)
{
    DeviceState *qdev;

    if (ch >= bus->channels) {
        hw_error("%s: invalid channel %d\n", __func__, ch);
    }
    if (bus->device[ch]) {
        hw_error("%s: channel %d already has a device attached\n",
                 __func__, ch);
    }
    qdev = qdev_new(name);
    qdev_prop_set_uint8(qdev, "channel", ch);
    bus->device[ch] = SPI_DEVICE(qdev);
    /* remember the bus until the device is realized */
    object_property_add_const_link(OBJECT(qdev), "omap-spi-bus",
                                   OBJECT(bus));
    return qdev;
}

void spi_realize_device(DeviceState *dev)
{
    BusState *bus = BUS(object_property_get_link(OBJECT(dev), "omap-spi-bus",
                                                 &error_abort));
    qdev_realize_and_unref(dev, bus, &error_fatal);
}

DeviceState *spi_create_device(SPIBus *bus, const char *name, int ch)
{
    DeviceState *dev = spi_create_device_noinit(bus, name, ch);
    spi_realize_device(dev);
    return dev;
}

static void spi_device_realize(DeviceState *dev, Error **errp)
{
    SPIDeviceClass *sc = SPI_DEVICE_GET_CLASS(dev);
    if (sc->realize) {
        sc->realize(dev, errp);
    }
}

static void spi_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);
    k->realize = spi_device_realize;
    k->bus_type = TYPE_SPI_BUS;
    device_class_set_props(k, spi_props);
}

static const TypeInfo spi_device_type_info = {
    .name = TYPE_SPI_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(SPIDevice),
    .abstract = true,
    .class_size = sizeof(SPIDeviceClass),
    .class_init = spi_device_class_init,
};

static void spi_device_register_types(void)
{
    type_register_static(&spi_bus_info);
    type_register_static(&spi_device_type_info);
}

type_init(spi_device_register_types)
