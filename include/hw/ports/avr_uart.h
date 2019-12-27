#ifndef HW_AVR_UART_H
#define HW_AVR_UART_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"
#include "hw/ports/peripheral.h"

#define TYPE_AVR_UART "avr-uart"

#define AVR_UART(obj) \
    OBJECT_CHECK(AVRPeripheralState, (obj), TYPE_AVR_UART)
#define AVR_UART_GET_CLASS(obj) \
    OBJECT_GET_CLASS(AVRUARTClass, obj, TYPE_AVR_UART)
#define AVR_UART_CLASS(klass) \
    OBJECT_CLASS_CHECK(AVRUARTClass, klass, TYPE_AVR_UART)

 typedef struct AVRUARTClass
 {
    AVRPeripheralClass parent_class;

    CanReceive parent_can_receive;
    Receive parent_receive;
    Read parent_read;
    Write parent_write;
    IsActive parent_is_active;
 } AVRUARTClass;

#endif