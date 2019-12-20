#ifndef HW_AVR_PORT_H
#define HW_AVR_PORT_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"
#include "include/hw/ports/peripheral.h"

/* Offsets of registers. */
#define PIN 0x00
#define DDR 0x01
#define PORT 0x02

#define NUM_PERIPHS 8
typedef void (*PeripheralFunc)(Object* obj, AVRPeripheralClass * P);

#define TYPE_AVR_PORT "avr-port"
#define AVR_PORT(obj) \
    OBJECT_CHECK(AVRPortState, (obj), TYPE_AVR_PORT)

typedef struct 
{
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    CharBackend chr;

    bool enabled;

    uint8_t data;
    /* Control and Status Registers */
    uint8_t port;
    uint8_t ddr;
    
	//uint8_t pin;
	uint8_t output_values;
	uint8_t input_values;

    /* Receive Complete */
    qemu_irq rxc_irq;
    /* Transmit Complete */
    qemu_irq txc_irq;
    /* Data Register Empty */
    qemu_irq dre_irq;


    AVRPeripheralClass * periphs[NUM_PERIPHS];
    uint8_t peripheral_counter;
} AVRPortState;

static inline void add_peripheral_to_port(AVRPortState * port, AVRPeripheralClass * periph)
{
    if(port->peripheral_counter == NUM_PERIPHS)
    {
        printf("ERROR: Cannot add anymore peripherals to port!\n");
        return;
    }

    port->periphs[port->peripheral_counter] = periph;
    port->peripheral_counter++;
    printf("Added peripheral to port...\n");
}

#endif
