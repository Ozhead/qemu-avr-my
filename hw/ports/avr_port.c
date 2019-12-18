#include "qemu/osdep.h"
#include "hw/ports/avr_port.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

static int avr_port_can_receive(void *opaque)
{
    AVRPortState *gpio = opaque; 
	
	/* if DDR is set to 0xFF, all pins are set as outputs... */
	
	if(gpio->ddr == 0xFF)
		return 0;
	return 1;
}

static void avr_port_receive(void *opaque, const uint8_t *buffer, int size)
{
	printf("Calling avr_port_receive\n");
	
    AVRPortState *gpio = opaque;
    //assert(size == 1);

	//uint8_t update_val = buffer[0] & ~gpio->ddr;
	// only set those bits that are set as input by DDR; for this, DDR is used as mask (inverted!)
	//gpio->input_values = (gpio->input_values & gpio->ddr) | (update_val & ~gpio->ddr);
	gpio->input_values = buffer[0];
	//printf("%i\n", gpio->input_values);
	
	if(buffer[0] & gpio->ddr)
	{
		printf("Caution: You are trying to write data in output ports!\n");
	}

    /*if (usart->csrb & USART_CSRB_RXCIE) {
        qemu_set_irq(usart->rxc_irq, 1);
    }*/
}

static void avr_port_reset(DeviceState *dev)
{
	AVRPortState *port = AVR_PORT(dev);
	port->port = 0;
	port->ddr = 0;
	port->input_values = 0;
	port->output_values = 0;

    qemu_set_irq(port->rxc_irq, 0);
    qemu_set_irq(port->txc_irq, 0);
    qemu_set_irq(port->dre_irq, 0);
}

static uint64_t avr_port_read(void *opaque, hwaddr addr, unsigned int size)
{
	printf("Call gpio read\n");
    //AVRGpioState *gpio = opaque;
	
	return 0;
}

static void avr_port_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
	printf("gpio_write\n");
	
	printf("addr = %i\n", (int)addr);
}

static const MemoryRegionOps avr_port_ops = {
    .read = avr_port_read,
    .write = avr_port_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.min_access_size = 1, .max_access_size = 1}
};

static Property avr_port_properties[] = {
    DEFINE_PROP_CHR("chardev", AVRPortState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void avr_port_pr(void *opaque, int irq, int level)
{
    AVRPortState *s = AVR_PORT(opaque);

    s->enabled = !level;

    if (!s->enabled) {
        avr_port_reset(DEVICE(s));
    }
}

static void avr_port_init(Object *obj)
{
    AVRPortState *s = AVR_PORT(obj);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->rxc_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dre_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->txc_irq);
    memory_region_init_io(&s->mmio, obj, &avr_port_ops, s, TYPE_AVR_PORT, 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in(DEVICE(s), avr_port_pr, 1);
    s->enabled = true;
}

static void avr_port_realize(DeviceState *dev, Error **errp)
{
    AVRPortState *s = AVR_PORT(dev);
    qemu_chr_fe_set_handlers(&s->chr, avr_port_can_receive,
                             avr_port_receive, NULL, NULL,
                             s, NULL, true);
    avr_port_reset(dev);
}

static void avr_port_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = avr_port_reset;
    dc->props = avr_port_properties;
    dc->realize = avr_port_realize;
}

static const TypeInfo avr_port_info = {
    .name          = TYPE_AVR_PORT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AVRPortState),
    .instance_init = avr_port_init,
    .class_init    = avr_port_class_init,
};

static void avr_port_register_types(void)
{
	printf("Init GPIOTypes!\n");
    type_register_static(&avr_port_info);
}

type_init(avr_port_register_types)
