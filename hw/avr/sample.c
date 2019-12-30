/*
 * QEMU AVR CPU
 *
 * Copyright (c) 2019 Michael Rolnik
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

/*
 *  NOTE:
 *      This is not a real AVR board, this is an example!
 *      The CPU is an approximation of an ATmega2560, but is missing various
 *      built-in peripherals.
 *
 *      This example board loads provided binary file into flash memory and
 *      executes it from 0x00000000 address in the code memory space.
 *
 *      Currently used for AVR CPU validation
 *
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "sysemu/qtest.h"
#include "ui/console.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "qemu/error-report.h"
#include "exec/address-spaces.h"
#include "include/hw/sysbus.h"
#include "include/hw/char/avr_usart.h"
#include "include/hw/timer/avr_timer16.h"
#include "include/hw/misc/avr_mask.h"
#include "include/hw/ports/avr_gpio.h"
#include "include/hw/ports/avr_port.h"
#include "include/hw/ports/avr_uart.h"
#include "include/hw/ports/avr_timer_8b.h"
#include "elf.h"
#include "hw/misc/unimp.h"
#include "include/hw/ports/adc.h"

//#define SIZE_FLASH 0x00040000
//#define SIZE_SRAM 0x00002000

// WURDE HIER KORREKT ANGEPASST; SEITDEM FUNKTIONIERT ES AUCH AAAAA
#define SIZE_FLASH 0x00020000
#define SIZE_SRAM 0x00004000
/*
 * Size of additional "external" memory, as if the AVR were configured to use
 * an external RAM chip.
 * Note that the configuration registers that normally enable this feature are
 * unimplemented.
 */
#define SIZE_EXMEM 0x00000000

/* Offsets of peripherals in emulated memory space (i.e. not host addresses)  */
#define PRR0_BASE 0x64
#define PRR1_BASE 0x65
#define USART_BASE 0xc0
#define TIMER1_BASE 0x80
#define TIMER1_IMSK_BASE 0x6f
#define TIMER1_IFR_BASE 0x36
#define TIMER0_BASE 0x44
#define TIMER0_IMSK_BASE 0x6E
#define TIMER0_IFR_BASE 0x35

#define PORTA_BASE 0x20
#define PORTB_BASE 0x23
#define PORTC_BASE 0x26
#define PORTD_BASE 0x29

/* Interrupt numbers used by peripherals */
#define USART_RXC_IRQ 24
#define USART_DRE_IRQ 25
#define USART_TXC_IRQ 26

/* ATMEGA2560 */
#define TIMER1_CAPT_IRQ 15
#define TIMER1_COMPA_IRQ 16
#define TIMER1_COMPB_IRQ 17
#define TIMER1_COMPC_IRQ 18
#define TIMER1_OVF_IRQ 19

/* ATMEGA1284P */
#define TIMER0_COMPA_IRQ 15
#define TIMER0_COMPB_IRQ 16
#define TIMER0_OVF_IRQ 17

/*  Power reduction     */
#define PRR1_BIT_PRTIM5     0x05    /*  Timer/Counter5  */
#define PRR1_BIT_PRTIM4     0x04    /*  Timer/Counter4  */
#define PRR1_BIT_PRTIM3     0x03    /*  Timer/Counter3  */
#define PRR1_BIT_PRUSART3   0x02    /*  USART3  */
#define PRR1_BIT_PRUSART2   0x01    /*  USART2  */
#define PRR1_BIT_PRUSART1   0x00    /*  USART1  */

#define PRR0_BIT_PRTWI      0x06    /*  TWI */
#define PRR0_BIT_PRTIM2     0x05    /*  Timer/Counter2  */
#define PRR0_BIT_PRTIM0     0x04    /*  Timer/Counter0  */
#define PRR0_BIT_PRTIM1     0x03    /*  Timer/Counter1  */
#define PRR0_BIT_PRSPI      0x02    /*  Serial Peripheral Interface */
#define PRR0_BIT_PRUSART0   0x01    /*  USART0  */
#define PRR0_BIT_PRADC      0x00    /*  ADC */

typedef struct {
    MachineClass parent;
} SampleMachineClass;

typedef struct {
    MachineState parent;
    MemoryRegion *ram;
    MemoryRegion *flash;
    AVRUsartState *usart0;
    AVRTimer16State *timer1;
    AVRMaskState *prr[2];
	
    /* PORT A */
    AVRPortState * porta;
    AVRPeripheralState *adc;

    /* PORT B */
    AVRPortState * portb;
    AVRPeripheralState * timer0;


    /* PORT D */
    AVRPortState * portd;
    AVRPeripheralState * uart0;
} SampleMachineState;

#define TYPE_SAMPLE_MACHINE MACHINE_TYPE_NAME("sample")

#define SAMPLE_MACHINE(obj) \
    OBJECT_CHECK(SampleMachineState, obj, TYPE_SAMPLE_MACHINE)
#define SAMPLE_MACHINE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(SampleMachineClass, obj, TYPE_SAMPLE_MACHINE)
#define SAMPLE_MACHINE_CLASS(klass) \
    OBJECT_CLASS_CHECK(SampleMachineClass, klass, TYPE_SAMPLE_MACHINE)

static void sample_init(MachineState *machine)
{
    SampleMachineState *sms = SAMPLE_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    AVRCPU *cpu;
    const char *firmware = NULL;
    const char *filename = NULL;
    const char *cpu_type = NULL;
    uint32_t flags;
    int bytes_loaded;
    SysBusDevice *busdev;
	//SysBusDevice *busdev1;
    DeviceState *cpudev;

    system_memory = get_system_memory();
    sms->ram = g_new(MemoryRegion, 1);
    sms->flash = g_new(MemoryRegion, 1);

    /* if ELF file is provided, determine CPU type reading ELF flags */
    cpu_type = machine->cpu_type;
    firmware = machine->firmware;
    if (firmware != NULL) {
        filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, firmware);
        if (filename == NULL) {
            error_report("Unable to find %s", firmware);
            exit(1);
        }

        bytes_loaded = load_elf_ram_sym(filename, NULL, NULL, NULL, NULL, NULL,
                NULL, 0, EM_AVR, &flags, 0, 0, NULL, 0, 0);
        if (bytes_loaded > 0) {
            cpu_type = avr_flags_to_cpu_type(flags, cpu_type);
        }
    }

    cpu = AVR_CPU(cpu_create(cpu_type));
    cpudev = DEVICE(cpu);

    memory_region_init_rom(sms->flash, NULL, "avr.flash", SIZE_FLASH,
            &error_fatal);
    memory_region_add_subregion(system_memory, OFFSET_CODE, sms->flash);


	//Changing to Atmega1284p. Highest register is 0xFF!
    create_unimplemented_device("usart 1", OFFSET_DATA + 0x00c8, 0x0007);
    create_unimplemented_device("usart 0", OFFSET_DATA + 0x00c0, 0x0007);
    create_unimplemented_device("twi", OFFSET_DATA + 0x00b8, 0x0006);					// 2-wire-interface
	
    create_unimplemented_device("timer-counter-async-8bit 2",
            OFFSET_DATA + 0x00b0, 0x0007);
			
    create_unimplemented_device("timer-counter-16bit 3",
            OFFSET_DATA + 0x0090, 0x000e);
    create_unimplemented_device("timer-counter-16bit 1",
            OFFSET_DATA + 0x0080, 0x000e);
			
    create_unimplemented_device("ac / adc",
            OFFSET_DATA + 0x0078, 0x0008);
			
    create_unimplemented_device("int-controller",
            OFFSET_DATA + 0x0068, 0x000c);
			
    create_unimplemented_device("sys",
            OFFSET_DATA + 0x0060, 0x0007);
			
    create_unimplemented_device("spi",
            OFFSET_DATA + 0x004c, 0x0003);
			
    create_unimplemented_device("ext-mem-iface",		//GPIO
            OFFSET_DATA + 0x004a, 0x0002);
			
    create_unimplemented_device("timer-counter-pwm-8bit 0",
            OFFSET_DATA + 0x0043, 0x0006);
			
    create_unimplemented_device("ext-mem-iface",		//GPIO
            OFFSET_DATA + 0x003e, 0x0005);
			
    create_unimplemented_device("int-controller",
            OFFSET_DATA + 0x0035, 0x0009);
			
    create_unimplemented_device("gpio D", OFFSET_DATA + 0x0029, 0x0003);
    create_unimplemented_device("gpio C", OFFSET_DATA + 0x0026, 0x0003);
    create_unimplemented_device("gpio B", OFFSET_DATA + 0x0023, 0x0003);
    create_unimplemented_device("gpio A", OFFSET_DATA + 0x0020, 0x0003);

    memory_region_allocate_system_memory(
        sms->ram, NULL, "avr.ram", SIZE_SRAM + SIZE_EXMEM);
    memory_region_add_subregion(system_memory, OFFSET_DATA + 0x200, sms->ram);

    /* Power Reduction built-in peripheral */
    sms->prr[0] = AVR_MASK(sysbus_create_simple(TYPE_AVR_MASK,
                    OFFSET_DATA + PRR0_BASE, NULL));
    sms->prr[1] = AVR_MASK(sysbus_create_simple(TYPE_AVR_MASK,
                    OFFSET_DATA + PRR1_BASE, NULL));

    /* USART 0 built-in peripheral */
    /*sms->usart0 = AVR_USART(object_new(TYPE_AVR_USART));
    busdev = SYS_BUS_DEVICE(sms->usart0);
    qdev_prop_set_chr(DEVICE(sms->usart0), "chardev", serial_hd(0));
    object_property_set_bool(OBJECT(sms->usart0), true, "realized",
            &error_fatal);
    sysbus_mmio_map(busdev, 0, OFFSET_DATA + USART_BASE);*/
	
    /*
     * These IRQ numbers don't match the datasheet because we're counting from
     * zero and not including reset.
     */
    /*sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(cpudev, USART_RXC_IRQ));
    sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(cpudev, USART_DRE_IRQ));
    sysbus_connect_irq(busdev, 2, qdev_get_gpio_in(cpudev, USART_TXC_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(sms->prr[1]), PRR1_BIT_PRUSART1,
            qdev_get_gpio_in(DEVICE(sms->usart0), 0));*/
	
	/* PORT A */
    sms->porta = AVR_PORT(object_new(TYPE_AVR_PORT));
    busdev = SYS_BUS_DEVICE(sms->porta);
    sysbus_mmio_map(busdev, 0, OFFSET_DATA + PORTA_BASE);
    qdev_prop_set_chr(DEVICE(sms->porta), "chardev", serial_hd(0));
	object_property_set_bool(OBJECT(sms->porta), true, "realized",
			&error_fatal);

    // PORT A ADC!
    sms->adc = AVR_ADC(object_new(TYPE_AVR_ADC));
    //AVRADCClass *dc = AVR_ADC_GET_CLASS(sms->adc);
    AVRPeripheralClass *pc = AVR_PERIPHERAL_GET_CLASS(sms->adc);
    add_peripheral_to_port(sms->porta, pc, sms->adc);
    for(uint32_t i = 0; i < NUM_PINS; i++)
        map_peripheral_to_pin(sms->porta, pc, sms->adc, i);

    busdev = SYS_BUS_DEVICE(sms->adc);
    sysbus_mmio_map(busdev, 0, OFFSET_DATA + 0x78);
    object_property_set_bool(OBJECT(sms->adc), true, "realized",
        &error_fatal);
    sms->porta->name = 'A';
    printf("Port A initiated\n");

    /* PORT B */
    sms->portb = AVR_PORT(object_new(TYPE_AVR_PORT));
    busdev = SYS_BUS_DEVICE(sms->portb);
    sysbus_mmio_map(busdev, 0, OFFSET_DATA + PORTB_BASE);
    qdev_prop_set_chr(DEVICE(sms->portb), "chardev", serial_hd(1));
	object_property_set_bool(OBJECT(sms->portb), true, "realized",
			&error_fatal);
    sms->portb->name = 'B';

    /* PORT B Timer 0 */
    sms->timer0 = AVR_TIMER_8b(object_new(TYPE_AVR_TIMER_8b));
    AVRPeripheralClass *pc2 = AVR_PERIPHERAL_GET_CLASS(sms->timer0);
    add_peripheral_to_port(sms->portb, pc2, sms->timer0);
    map_peripheral_to_pin(sms->portb, pc2, sms->timer0, 3);
    map_peripheral_to_pin(sms->portb, pc2, sms->timer0, 4);

    busdev = SYS_BUS_DEVICE(sms->timer0);
    sysbus_mmio_map(busdev, 0, OFFSET_DATA + TIMER0_BASE);
    sysbus_mmio_map(busdev, 1, OFFSET_DATA + TIMER0_IMSK_BASE);
    sysbus_mmio_map(busdev, 2, OFFSET_DATA + TIMER0_IFR_BASE);
    sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(cpudev, TIMER0_COMPA_IRQ));
    sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(cpudev, TIMER0_COMPB_IRQ));
    sysbus_connect_irq(busdev, 2, qdev_get_gpio_in(cpudev, TIMER0_OVF_IRQ));
    object_property_set_bool(OBJECT(sms->timer0), true, "realized",
        &error_fatal);

    /* PORT D */
    sms->portd = AVR_PORT(object_new(TYPE_AVR_PORT));
    busdev = SYS_BUS_DEVICE(sms->portd);
    sysbus_mmio_map(busdev, 0, OFFSET_DATA + PORTD_BASE);
    qdev_prop_set_chr(DEVICE(sms->portd), "chardev", serial_hd(2));
	object_property_set_bool(OBJECT(sms->portd), true, "realized",
			&error_fatal);
    sms->portd->name = 'D';

    // PORT D UART!
    sms->uart0 = AVR_UART(object_new(TYPE_AVR_UART));
    AVRPeripheralClass *pc1 = AVR_PERIPHERAL_GET_CLASS(sms->uart0);
    add_peripheral_to_port(sms->portd, pc1, sms->uart0);
    map_peripheral_to_pin(sms->portd, pc1, sms->uart0, 0);
    map_peripheral_to_pin(sms->portd, pc1, sms->uart0, 1);

    busdev = SYS_BUS_DEVICE(sms->uart0);
    sysbus_mmio_map(busdev, 0, OFFSET_DATA + USART_BASE);
    object_property_set_bool(OBJECT(sms->uart0), true, "realized",
        &error_fatal);

    printf("Port D initiated\n---------------------------------\n");

    /* Timer 1 built-in periphal */
    
    /*sms->timer1 = AVR_TIMER16(object_new(TYPE_AVR_TIMER16));
    object_property_set_bool(OBJECT(sms->timer1), true, "realized",
            &error_fatal);
    busdev = SYS_BUS_DEVICE(sms->timer1);
    sysbus_mmio_map(busdev, 0, OFFSET_DATA + TIMER1_BASE);
    sysbus_mmio_map(busdev, 1, OFFSET_DATA + TIMER1_IMSK_BASE);
    sysbus_mmio_map(busdev, 2, OFFSET_DATA + TIMER1_IFR_BASE);*/
    /*sysbus_connect_irq(busdev, 0, qdev_get_gpio_in(cpudev, TIMER1_CAPT_IRQ));
    sysbus_connect_irq(busdev, 1, qdev_get_gpio_in(cpudev, TIMER1_COMPA_IRQ));
    sysbus_connect_irq(busdev, 2, qdev_get_gpio_in(cpudev, TIMER1_COMPB_IRQ));
    sysbus_connect_irq(busdev, 3, qdev_get_gpio_in(cpudev, TIMER1_COMPC_IRQ));
    sysbus_connect_irq(busdev, 4, qdev_get_gpio_in(cpudev, TIMER1_OVF_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(sms->prr[0]), PRR0_BIT_PRTIM1,
            qdev_get_gpio_in(DEVICE(sms->timer1), 0));*/
    

    /* Load firmware (contents of flash) trying to auto-detect format */
    if (filename != NULL) {
        bytes_loaded = load_elf(
            filename, NULL, NULL, NULL, NULL, NULL, NULL, 0, EM_NONE, 0, 0);
        if (bytes_loaded < 0) {
            bytes_loaded = load_image_targphys(
                filename, OFFSET_CODE, SIZE_FLASH);
        }
        if (bytes_loaded < 0) {
            error_report(
                "Unable to load firmware image %s as ELF or raw binary",
                firmware);
            exit(1);
        }
    }
}

static void sample_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "ATMega1284p";
    mc->init = sample_init;
    mc->default_cpus = 1;
    mc->min_cpus = mc->default_cpus;
    mc->max_cpus = mc->default_cpus;
    mc->default_cpu_type = "avr51-avr-cpu"; /* ATmega1284p. */
    mc->is_default = 1;
}

static const TypeInfo sample_info = {
    .name = TYPE_SAMPLE_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(SampleMachineState),
    .class_size = sizeof(SampleMachineClass),
    .class_init = sample_class_init,
};

static void sample_machine_init(void)
{
    type_register_static(&sample_info);
}

type_init(sample_machine_init);
