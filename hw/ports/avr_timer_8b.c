#include "qemu/osdep.h"
#include "hw/ports/avr_timer_8b.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

static int avr_timer_8b_can_receive(void *opaque)
{
    return 0;
}

static int avr_timer_8b_is_active(void *opaque, uint32_t pinno)
{
    return 0;
}

static void avr_timer_8b_receive(void *opaque, const uint8_t *buffer, int msgid, int pinno)
{

}

static uint64_t avr_timer_8b_read(void *opaque, hwaddr addr, unsigned int size)
{
    return 0;
}

static void avr_timer_8b_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    
}

static uint32_t avr_timer_8b_serialize(void * opaque, uint32_t pinno, uint8_t * pData)
{
    return 0;
}

static void avr_timer_8b_class_init(ObjectClass *klass, void *data)
{
    //DeviceClass *dc = DEVICE_CLASS(klass);
    AVRPeripheralClass *pc = AVR_PERIPHERAL_CLASS(klass);
    AVRTimer8bClass * adc = AVR_TIMER_8b_CLASS(klass);
  
    adc->parent_can_receive = pc->can_receive;
    adc->parent_receive = pc->receive;
    adc->parent_read = pc->read;
    adc->parent_write = pc->write;
    adc->parent_is_active = pc->is_active;
    adc->parent_serialize = pc->serialize;

    pc->can_receive = avr_timer_8b_can_receive;
    pc->read = avr_timer_8b_read;
    pc->write = avr_timer_8b_write;
    pc->receive = avr_timer_8b_receive;
    pc->is_active = avr_timer_8b_is_active;
    pc->serialize = avr_timer_8b_serialize;

    //printf("ADC class initiated\n");
}

static const MemoryRegionOps avr_timer_8b_ops = {
    .read = avr_timer_8b_read,
    .write = avr_timer_8b_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.min_access_size = 1, .max_access_size = 1}
};

static void avr_timer_8b_init(Object *obj)
{
    AVRPeripheralState *s = AVR_PERIPHERAL(obj);
    memory_region_init_io(&s->mmio, obj, &avr_timer_8b_ops, s, TYPE_AVR_TIMER_8b, 8);

    /*sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->rxc_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dre_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->txc_irq);
    memory_region_init_io(&s->mmio, obj, &avr_peripheral_ops, s, TYPE_AVR_PERIPHERAL, 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in(DEVICE(s), avr_peripheral_pr, 1);*/
    //s->enabled = true;
    printf("AVR Timer8b object init\n");
}

static const TypeInfo avr_timer_8b_info = {
    .name          = TYPE_AVR_TIMER_8b,
    .parent        = TYPE_AVR_PERIPHERAL,
    .class_init    = avr_timer_8b_class_init,
    .class_size    = sizeof(AVRTimer8bClass),
    .instance_size = sizeof(AVRPeripheralState),
    .instance_init = avr_timer_8b_init
};

static void avr_timer_8b_register_types(void)
{
    type_register_static(&avr_timer_8b_info);
}

type_init(avr_timer_8b_register_types)