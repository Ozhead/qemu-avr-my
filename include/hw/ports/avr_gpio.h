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

#ifndef HW_AVR_GPIO_H
#define HW_AVR_GPIO_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"

/* Offsets of registers. */
#define PIN 0x00
#define DDR 0x01
#define PORT 0x02

/* Relevant bits in regiters. */
#define PIN0		(1 << 0)
#define PIN1		(1 << 1)
#define PIN2		(1 << 2)
#define PIN3		(1 << 3)
#define PIN4		(1 << 4)
#define PIN5		(1 << 5)
#define PIN6		(1 << 6)
#define PIN7		(1 << 7)

#define DDR0 	(1 << 0)
#define DDR1 	(1 << 1)
#define DDR2 	(1 << 2)
#define DDR3 	(1 << 3)
#define DDR4 	(1 << 4)
#define DDR5 	(1 << 5)
#define DDR6 	(1 << 6)
#define DDR7 	(1 << 7)

#define PORT0	(1 << 0)
#define PORT1	(1 << 1)
#define PORT2	(1 << 2)
#define PORT3	(1 << 3)
#define PORT4	(1 << 4)
#define PORT5	(1 << 5)
#define PORT6	(1 << 6)
#define PORT7 	(1 << 7)

#define TYPE_AVR_GPIO "avr-gpio"
#define AVR_GPIO(obj) \
    OBJECT_CHECK(AVRGpioState, (obj), TYPE_AVR_GPIO)

typedef struct {
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    CharBackend chr;

    bool enabled;

    uint8_t data;
    //bool data_valid;
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
} AVRGpioState;

#endif /* HW_AVR_USART_H */
