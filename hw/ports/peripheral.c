#include "qemu/osdep.h"
#include "hw/ports/peripheral.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

static int avr_peripheral_can_receive(void *opaque)
{
    printf("PERIPHERAL CAN RECEIVE :(\n");
	return 1;
}

static void avr_peripheral_receive(void *opaque, const uint8_t *buffer, int size)
{
	printf("Calling avr_peripheral_receive\n");
}

static void avr_peripheral_reset(DeviceState *dev)
{
	AVRPeripheralState *port = AVR_PERIPHERAL(dev);

    qemu_set_irq(port->rxc_irq, 0);
    qemu_set_irq(port->txc_irq, 0);
    qemu_set_irq(port->dre_irq, 0);
}

static uint64_t avr_peripheral_read(void *opaque, hwaddr addr, unsigned int size)
{
	printf("Call peri read\n");
	
	return 0;
}

static void avr_peripheral_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
	printf("peripheral_write\n");
}

/*static const MemoryRegionOps avr_peripheral_ops = {
    .read = avr_peripheral_read,
    .write = avr_peripheral_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.min_access_size = 1, .max_access_size = 1}
};*/

static void avr_peripheral_pr(void *opaque, int irq, int level)
{
    printf("Peripheral PR\n");
    //AVRPeripheralState *s = AVR_PERIPHERAL(opaque);
}

static void avr_peripheral_init(Object *obj)
{
    AVRPeripheralState *s = AVR_PERIPHERAL(obj);
    /*sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->rxc_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dre_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->txc_irq);*/
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in(DEVICE(s), avr_peripheral_pr, 1);
    //s->enabled = true;
    printf("AVR Peripheral object init\n");
}

/*static void avr_peripheral_realize(DeviceState *dev, Error **errp)
{
    printf("Peripheral realize!\n");
    AVRPeripheralState *s = AVR_PERIPHERAL(dev);
    qemu_chr_fe_set_handlers(&s->chr, avr_peripheral_can_receive,
                             avr_peripheral_receive, NULL, NULL,
                             s, NULL, true);
    avr_peripheral_reset(dev);
}*/

static void avr_peripheral_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AVRPeripheralClass * pc = AVR_PERIPHERAL_CLASS(klass);

    dc->reset = avr_peripheral_reset;
    //dc->realize = avr_peripheral_realize;

    pc->can_receive = avr_peripheral_can_receive;
    pc->read = avr_peripheral_read;
    pc->write = avr_peripheral_write;
    pc->receive = avr_peripheral_receive;

    //printf("Peripheral class initiated\n");
}

static const TypeInfo avr_peripheral_info = {
    .name          = TYPE_AVR_PERIPHERAL,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AVRPeripheralState),
    .instance_init = avr_peripheral_init,
    .class_init    = avr_peripheral_class_init,
    .class_size    = sizeof(AVRPeripheralClass),
    .abstract      = true
};

static void avr_peripheral_register_types(void)
{
	//printf("Init Peripheral Types!\n");
    type_register_static(&avr_peripheral_info);
}

type_init(avr_peripheral_register_types)
