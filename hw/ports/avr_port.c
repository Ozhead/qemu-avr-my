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
    printf("Calling avr_port_receive %c(%d)\n", buffer[0], size);
    AVRPortState *port = opaque; 
    while(ptr != size)
    {
        const uint8_t header = buffer[ptr];

        //Protocol Draft 1:
        const uint8_t pin_id = (header & 0b11100000) >> 5;
        const uint8_t msg_id = (header & 0b00011110) >> 1;

        printf("Header ID = %i MsgID = %i Size = %i\n", pin_id, msg_id, size);

        ptr++;

        //if(port->periphs_in_pin[pin_id]->is_active(port->states_in_pin[pin_id], pin_id))
        if(msg_id)  //not zero equals not digio
        {
            port->periphs_in_pin[pin_id]->receive(port->states_in_pin[pin_id], buffer + ptr, peripheral_msg_lengths[msg_id], pin_id);
        }
        else
        {
            printf("DIGIO %d\n", msg_id);
            if(header & 1)      // set to 1
                port->pin |= (1 << pin_id);
            else
                port->pin &= ~(1 << pin_id);

            printf("PIN = %d\n", port->pin);
        }
        
        ptr += peripheral_msg_lengths[msg_id]; //advance the current pointer
    }

    printf("------------------------------\n");
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
    AVRPortState *port = opaque;

    switch(addr)
    {
        case PIN:
            return port->pin;
        case PORT:
            return port->port;
        case DDR:
            return port->ddr;
    }
	
	return 0;
}

static void avr_port_send_data(void *opaque)
{
    size_t data_ptr = 0;
    uint8_t data[4096];
    memset(data, 0, 4096);
    AVRPortState *port = opaque;

    printf("Try to send Data of Port %c\n", port->name);
    for(uint32_t i = 0; i < NUM_PINS; i++)
    {
        uint16_t pin_mask = 1 << i;

        // is it defined?
        if(port->periphs_in_pin[i] != NULL)
        {
            printf("Checking pin %d\n", i);
            if(!port->periphs_in_pin[i]->is_active(port->states_in_pin[i], i))      //not active => this is a GPIO pin!
            {
            // this pin is set to input pin => ignore it; may be users fault if he sets DDR wrong!
                if((port->ddr & pin_mask) == 0)
                {
                    printf("PIN %d is set as input pin and thus ignored...\n", i);
                    continue;
                }

                printf("Serializing DigIO\n");
                uint8_t val = (i << 5) & 0b11100000;    // write Pin ID, set the rest to zero!
                if((port->port & pin_mask))
                    val |= 1;                           // set the last bit to 1 due to logic one

                data[data_ptr] = val;
                data_ptr++;
            }
            else
            {
                printf("Sending peripheral data stuff...\n");
                //assert(false);
                // serialize the data, put it into the array (happens inside the func) and increment data_ptr
                data_ptr += port->periphs_in_pin[i]->serialize(port->states_in_pin[i], i, data + data_ptr);
                //port->periphs_in_pin[i]->serialize(port->states_in_pin[i], i, data + data_ptr);
            }
        }
        else    // there is no pin here! so this must be DigIO anyway!
        {   // TODO: Move this to a function...
            if((port->ddr & pin_mask) == 0)
            {
                //printf("PIN %d is set as input pin and thus ignored...\n", i);
                continue;
            }

            printf("Serializing DigIO t2\n");
            uint8_t val = (i << 5) & 0b11100000;    // write Pin ID, set the rest to zero!
            if((port->port & pin_mask))
                val |= 1;                           // set the last bit to 1 due to logic one

            data[data_ptr] = val;
            data_ptr++;
        }
        
    }

    if(data_ptr > 0)
        qemu_chr_fe_write_all(&port->chr, data, data_ptr);  //send
}

static void avr_port_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
	printf("Port base write\n");
    AVRPortState *port = opaque;

    switch(addr)
	{
		case PIN:
		{
			printf("Writing to PIN... But nothing happened! TODO: Toggle\n");
			//TODO: Actually this will toggle PORT independant of DDR
		}
		break;
		case PORT:
		{
            printf("Update PORT\n");
			/* only write those that are set by DDR to 1! */
			uint8_t update_val = value & 0xFF;
			//gpio->port = (gpio->port & ~gpio->ddr) | (update_val & gpio->ddr);
			port->port = update_val;
			
			//uint8_t data = port->port & port->ddr;
			
            avr_port_send_data(port);   // trigger a sending of all data!
		}
		break;
		case DDR:
		{
            printf("Update DDR\n");
			port->ddr = value;
		}
		break;
	}
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

    s->send_data = avr_port_send_data;
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
