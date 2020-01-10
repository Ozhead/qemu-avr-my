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

typedef void (*PeripheralFunc)(Object* obj, AVRPeripheralClass * P);

#define TYPE_AVR_PORT "avr-port"
#define AVR_PORT(obj) \
    OBJECT_CHECK(AVRPortState, (obj), TYPE_AVR_PORT)

typedef void (*SendData)(void *opaque);

typedef struct 
{
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    CharBackend chr;

    SendData send_data;
    SendData finalize;
    
    char name;

    bool enabled;

    uint8_t data;
    /* Control and Status Registers */
    uint8_t port;
    uint8_t ddr;
    uint8_t pin;
    
	//uint8_t pin;
	uint8_t output_values;
	uint8_t input_values;

    /* Receive Complete */
    //qemu_irq rxc_irq;
    /* Transmit Complete */
    //qemu_irq txc_irq;
    /* Data Register Empty */
    //qemu_irq dre_irq;


    AVRPeripheralState * states[NUM_PINS];
    AVRPeripheralState * states_in_pin[NUM_PINS];

    AVRPeripheralClass * periphs[NUM_PINS]; // general list of peripherals...
    AVRPeripheralClass * periphs_in_pin[NUM_PINS];

    uint8_t peripheral_counter;
} AVRPortState;

static inline void add_peripheral_to_port(AVRPortState * port, AVRPeripheralClass * periph, AVRPeripheralState * state)
{
    if(port->peripheral_counter == NUM_PINS)
    {
        printf("ERROR: Cannot add anymore peripherals to port!\n");
        return;
    }

    if(port->periphs[port->peripheral_counter] != NULL)
    {
        printf("ERROR: Peripheral in pin %d is already initialized!\n", port->peripheral_counter);
        return;
    }

    port->periphs[port->peripheral_counter] = periph;
    port->states[port->peripheral_counter] = state;
    port->peripheral_counter++;
    state->father_port = (AVRPortState_t*)port;     //let the peripheral know who's boss it is
    printf("Added peripheral to port...\n");
}

static inline void map_peripheral_to_pin(AVRPortState * port, AVRPeripheralClass * periph, AVRPeripheralState * state, uint32_t portno)
{
    if(port->periphs_in_pin[portno] != NULL)
    {
        printf("ERROR: PIN %d already initiated!\n", portno);
        return;
    }

    port->periphs_in_pin[portno] = periph;
    port->states_in_pin[portno] = state;
    printf("Added peripheral to pin %d\n", portno);
}

#endif
