/*
 * AVR USART
 *
 * Copyright (c) 2018 University of Kent
 * Author: Sarah Harris
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see
 * <http://www.gnu.org/licenses/lgpl-2.1.html>
 */

#include "qemu/osdep.h"
#include "hw/ports/avr_gpio.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"


static int avr_gpio_can_receive(void *opaque)
{
    AVRGpioState *gpio = opaque; 

    /*if (gpio->data_valid || !(gpio->csrb & USART_CSRB_RXEN)) {
        return 0;
    }
    return 1;*/
	
	/* if DDR is set to 0xFF, all pins are set as outputs... */
	
	if(gpio->ddr == 0xFF)
		return 0;
	return 1;
}

static void avr_gpio_receive(void *opaque, const uint8_t *buffer, int size)
{
	printf("Calling avr_gpio_receive\n");
	
    AVRGpioState *gpio = opaque;
    assert(size == 1);

	//uint8_t update_val = buffer[0] & ~gpio->ddr;
	// only set those bits that are set as input by DDR; for this, DDR is used as mask (inverted!)
	//gpio->input_values = (gpio->input_values & gpio->ddr) | (update_val & ~gpio->ddr);
	gpio->input_values = buffer[0];
	printf("%i\n", gpio->input_values);
	
	if(buffer[0] & gpio->ddr)
	{
		printf("Caution: You are trying to write data in output ports!\n");
	}

	
    /*if (usart->csrb & USART_CSRB_RXCIE) {
        qemu_set_irq(usart->rxc_irq, 1);
    }*/
}

static void avr_gpio_reset(DeviceState *dev)
{
	AVRGpioState *gpio = AVR_GPIO(dev);
	gpio->port = 0;
	gpio->ddr = 0;
	gpio->input_values = 0;
	gpio->output_values = 0;

    qemu_set_irq(gpio->rxc_irq, 0);
    qemu_set_irq(gpio->txc_irq, 0);
    qemu_set_irq(gpio->dre_irq, 0);
}

static uint64_t avr_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
	printf("Call gpio read\n");
    AVRGpioState *gpio = opaque;
	
	switch(addr)
	{
		case PIN:
		{
			//return gpio->pin;
			// read only those bits that are set as input ports!
			return gpio->input_values & (~gpio->ddr);
		}
		break;
		case PORT:
		{
			return gpio->port;
		}
		break;
		case DDR:
		{
			return gpio->ddr;
		}
		break;
		default:
		{
			printf("QEMU ERROR: Reading wrong HWADDR???\n");
		}
		break;
	}
	
	return 0;
}

static void avr_gpio_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
	printf("gpio_write\n");
	
	printf("addr = %i\n", (int)addr);
	
    AVRGpioState *gpio = opaque;
	uint8_t data;

	switch(addr)
	{
		case PIN:
		{
			printf("Writing to PIN... But nothing happened!\n");
			//TODO: Actually this will toggle PORT independant of DDR
		}
		break;
		case PORT:
		{
			/* only write those that are set by DDR to 1! */
			uint8_t update_val = value & 0xFF;
			//gpio->port = (gpio->port & ~gpio->ddr) | (update_val & gpio->ddr);
			gpio->port = update_val;
			
			
			data = gpio->port & gpio->ddr;
			// only send update if the output values changed!
			if(data != gpio->output_values)
			{
				gpio->output_values = data;
				qemu_chr_fe_write_all(&gpio->chr, &data, 1);
			}
		}
		break;
		case DDR:
		{
			gpio->ddr = value;
		}
		break;
	}

}

static const MemoryRegionOps avr_gpio_ops = {
    .read = avr_gpio_read,
    .write = avr_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.min_access_size = 1, .max_access_size = 1}
};

static Property avr_gpio_properties[] = {
    DEFINE_PROP_CHR("chardev", AVRGpioState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void avr_gpio_pr(void *opaque, int irq, int level)
{
    AVRGpioState *s = AVR_GPIO(opaque);

    s->enabled = !level;

    if (!s->enabled) {
        avr_gpio_reset(DEVICE(s));
    }
}

static void avr_gpio_init(Object *obj)
{
    AVRGpioState *s = AVR_GPIO(obj);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->rxc_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dre_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->txc_irq);
    memory_region_init_io(&s->mmio, obj, &avr_gpio_ops, s, TYPE_AVR_GPIO, 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in(DEVICE(s), avr_gpio_pr, 1);
    s->enabled = true;
}

static void avr_gpio_realize(DeviceState *dev, Error **errp)
{
    AVRGpioState *s = AVR_GPIO(dev);
    qemu_chr_fe_set_handlers(&s->chr, avr_gpio_can_receive,
                             avr_gpio_receive, NULL, NULL,
                             s, NULL, true);
    avr_gpio_reset(dev);
}

static void avr_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = avr_gpio_reset;
    dc->props = avr_gpio_properties;
    dc->realize = avr_gpio_realize;
}

static const TypeInfo avr_gpio_info = {
    .name          = TYPE_AVR_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AVRGpioState),
    .instance_init = avr_gpio_init,
    .class_init    = avr_gpio_class_init,
};

static void avr_gpio_register_types(void)
{
	printf("Init GPIOTypes!\n");
    type_register_static(&avr_gpio_info);
}

type_init(avr_gpio_register_types)
