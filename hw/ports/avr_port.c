#include "qemu/osdep.h"
#include "hw/ports/avr_port.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

//TODO: Correct return size!
static int avr_port_can_receive(void *opaque)
{
    AVRPortState *port = opaque; 
	
	/* if DDR is set to 0xFF, all pins are set as outputs... */
	/*for(uint32_t i = 0; i < port->peripheral_counter; i++)
    {
        if(port->periphs[i] != NULL && port->periphs[i]->can_receive(port->periphs[i]))
            return 4096;
    }*/

    // TODO later! check all "unmapped" pins if they can receive something!
	if(port->ddr == 0xFF)
		return 0;
	return 4096;
}

static void avr_port_receive(void *opaque, const uint8_t *buffer, int size)
{
    size_t ptr = 0;
    printf("Calling avr_port_receive\n");
    AVRPortState *port = opaque; 
    while(ptr != size)
    {
        const uint8_t header = buffer[ptr];

        //Protocol Draft 1:
        const uint8_t pin_id = (header & 0b11100000) >> 5;
        const uint8_t len = (header & 0b00011110) >> 1;

        printf("Header ID = %i Len = %i Size = %i\n", pin_id, len, size);

        ptr++;

        if(port->periphs_in_pin[pin_id] != NULL)
        {
            AVRPeripheralState * pState = NULL;
            for(int i = 0; i < port->peripheral_counter; i++)
            {
                if(port->periphs[i] == port->periphs_in_pin[pin_id])
                {
                    pState = port->states[i];
                    break;
                }
            }

            if(pState == NULL)
            {
                printf("FATAL ERROR: Can not find a state to given peripheral class\n");
                assert(false);
            }

            port->periphs_in_pin[pin_id]->receive(pState, buffer + ptr, len);
        }
        else
        {
            printf("DigIO TODO\n");
        }
        
        ptr += len; //advance the current pointer
    }
	//uint8_t update_val = buffer[0] & ~gpio->ddr;
	// only set those bits that are set as input by DDR; for this, DDR is used as mask (inverted!)
	//gpio->input_values = (gpio->input_values & gpio->ddr) | (update_val & ~gpio->ddr);
}

static void avr_port_reset(DeviceState *dev)
{
	AVRPortState *port = AVR_PORT(dev);
	port->port = 0;
	port->ddr = 0;
	port->input_values = 0;
	port->output_values = 0;
}

static uint64_t avr_port_read(void *opaque, hwaddr addr, unsigned int size)
{
	printf("Call port base read\n");
    //AVRGpioState *gpio = opaque;
	
	return 0;
}

static void avr_port_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
	printf("Port base write\n");
}

static const MemoryRegionOps avr_port_ops = {
    .read = avr_port_read,
    .write = avr_port_write,
    .endianness = DEVICE_NATIVE_ENDIAN
    //.impl = {.min_access_size = 9, .max_access_size = 16}
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
    memory_region_init_io(&s->mmio, obj, &avr_port_ops, s, TYPE_AVR_PORT, 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in(DEVICE(s), avr_port_pr, 1);

    for(uint32_t i = 0; i < NUM_PINS; i++)
    {
        s->periphs[i] = NULL;
        s->states[i] = NULL;
        s->periphs_in_pin[i] = NULL;
    }

    s->peripheral_counter = 0;
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
    type_register_static(&avr_port_info);
}

type_init(avr_port_register_types)
