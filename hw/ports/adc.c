#include "qemu/osdep.h"
#include "hw/ports/adc.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"


static int avr_adc_can_receive(void *opaque)
{
    printf("ADC CAN RECEIVE JAAAAAAAAA\n");
	return 1;
}

static void avr_adc_receive(void *opaque, const uint8_t *buffer, int size)
{
	printf("Calling avr_adc_receive\n");
}

/*static void avr_adc_reset(DeviceState *dev)
{
    printf("AVR reset\n");
	//AVRPeripheralState *port = AVR_PERIPHERAL(dev);
}*/

static uint64_t avr_adc_read(void *opaque, hwaddr addr, unsigned int size)
{
	printf("Call adc read\n");
	
	return 0;
}

static void avr_adc_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
	printf("adc_write\n");
}

/*static const MemoryRegionOps avr_adc_ops = {
    .read = avr_adc_read,
    .write = avr_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.min_access_size = 1, .max_access_size = 1}
};*/

//static Property avr_adc_properties[] = {
    //DEFINE_PROP_CHR("chardev", AVRPeripheralState, chr),
    //DEFINE_PROP_END_OF_LIST(),
//};

//static void avr_adc_pr(void *opaque, int irq, int level)
//{
    //printf("adc PR\n");
    //AVRPeripheralState *s = AVR_PERIPHERAL(opaque);
//}

//static void avr_adc_init(Object *obj)
//{
    /*AVRPeripheralState *s = AVR_PERIPHERAL(obj);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->rxc_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dre_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->txc_irq);
    memory_region_init_io(&s->mmio, obj, &avr_peripheral_ops, s, TYPE_AVR_PERIPHERAL, 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in(DEVICE(s), avr_peripheral_pr, 1);
    //s->enabled = true;*/
//}

//static void avr_peripheral_realize(DeviceState *dev, Error **errp)
//{
    /*AVRPeripheralState *s = AVR_PERIPHERAL(dev);
    qemu_chr_fe_set_handlers(&s->chr, avr_peripheral_can_receive,
                             avr_peripheral_receive, NULL, NULL,
                             s, NULL, true);
    avr_peripheral_reset(dev);*/
//}

static void avr_adc_class_init(ObjectClass *klass, void *data)
{
    //DeviceClass *dc = DEVICE_CLASS(klass);
    AVRPeripheralClass *pc = AVR_PERIPHERAL_CLASS(klass);
    AVRADCClass * adc = AVR_ADC_CLASS(klass);

    /*dc->reset = avr_peripheral_reset;
    dc->props = avr_peripheral_properties;
    dc->realize = avr_peripheral_realize;*/
  
    adc->parent_can_receive = pc->can_receive;
    adc->parent_receive = pc->receive;
    adc->parent_read = pc->read;
    adc->parent_write = pc->write;

    pc->can_receive = avr_adc_can_receive;
    pc->read = avr_adc_read;
    pc->write = avr_adc_write;
    pc->receive = avr_adc_receive;

    printf("ADC class initiated\n");
}

static const TypeInfo avr_adc_info = {
    .name          = TYPE_AVR_ADC,
    .parent        = TYPE_AVR_PERIPHERAL,
    .class_init    = avr_adc_class_init,
    .class_size    = sizeof(AVRADCClass)
};

static void avr_adc_register_types(void)
{
	printf("Init ADC Types!\n");

    printf("%lu <= %lu\n", sizeof(AVRPeripheralClass), sizeof(AVRADCClass));
    type_register_static(&avr_adc_info);
}

type_init(avr_adc_register_types)