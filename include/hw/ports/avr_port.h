#ifndef HW_AVR_PORT_H
#define HW_AVR_PORT_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"

/* Offsets of registers. */
#define PIN 0x00
#define DDR 0x01
#define PORT 0x02

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
} AVRPortState;

#endif
