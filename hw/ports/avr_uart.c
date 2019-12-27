#include "qemu/osdep.h"
#include "hw/ports/avr_uart.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

static int avr_uart_can_receive(void *opaque)
{
    return 0;
}

static int avr_uart_is_active(void *opaque, uint32_t pinno)
{
    return 0;
}

static void avr_uart_receive(void *opaque, const uint8_t *buffer, int msgid, int pinno)
{

}

static uint64_t avr_uart_read(void *opaque, hwaddr addr, unsigned int size)
{	
	return 0;
}

static void avr_uart_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{

}

static void avr_uart_class_init(ObjectClass *klass, void *data)
{
    //DeviceClass *dc = DEVICE_CLASS(klass);
    AVRPeripheralClass *pc = AVR_PERIPHERAL_CLASS(klass);
    AVRUARTClass * uart = AVR_UART_CLASS(klass);

    /*dc->reset = avr_peripheral_reset;
    dc->props = avr_peripheral_properties;
    dc->realize = avr_peripheral_realize;*/
  
    uart->parent_can_receive = pc->can_receive;
    uart->parent_receive = pc->receive;
    uart->parent_read = pc->read;
    uart->parent_write = pc->write;
    uart->parent_is_active = pc->is_active;

    pc->can_receive = avr_uart_can_receive;
    pc->read = avr_uart_read;
    pc->write = avr_uart_write;
    pc->receive = avr_uart_receive;
    pc->is_active = avr_uart_is_active;
}

static const MemoryRegionOps avr_uart_ops = {
    .read = avr_uart_read,
    .write = avr_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.min_access_size = 1, .max_access_size = 1}
};

static void avr_uart_init(Object *obj)
{
    AVRPeripheralState *s = AVR_PERIPHERAL(obj);
    memory_region_init_io(&s->mmio, obj, &avr_uart_ops, s, TYPE_AVR_UART, 8);

    /*sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->rxc_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dre_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->txc_irq);
    memory_region_init_io(&s->mmio, obj, &avr_peripheral_ops, s, TYPE_AVR_PERIPHERAL, 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in(DEVICE(s), avr_peripheral_pr, 1);*/
    //s->enabled = true;
    printf("AVR UART object init\n");
}

static const TypeInfo avr_uart_info = {
    .name          = TYPE_AVR_UART,
    .parent        = TYPE_AVR_PERIPHERAL,
    .class_init    = avr_uart_class_init,
    .class_size    = sizeof(AVRUARTClass),
    .instance_size = sizeof(AVRPeripheralState),
    .instance_init = avr_uart_init
};

static void avr_uart_register_types(void)
{
    printf("UART rausgeballert\n");
    type_register_static(&avr_uart_info);
}

type_init(avr_uart_register_types)