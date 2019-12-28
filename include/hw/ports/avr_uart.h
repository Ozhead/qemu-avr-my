#ifndef HW_AVR_UART_H
#define HW_AVR_UART_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"
#include "hw/ports/peripheral.h"

/* Offsets of registers. */
#define USART_DR   0x06
#define USART_CSRA  0x00
#define USART_CSRB  0x01
#define USART_CSRC  0x02
#define USART_BRRH 0x05
#define USART_BRRL 0x04

/* Relevant bits in regiters. */
#define USART_CSRA_RXC    (1 << 7)
#define USART_CSRA_TXC    (1 << 6)
#define USART_CSRA_DRE    (1 << 5)
#define USART_CSRA_MPCM   (1 << 0)

#define USART_CSRB_RXCIE  (1 << 7)
#define USART_CSRB_TXCIE  (1 << 6)
#define USART_CSRB_DREIE  (1 << 5)
#define USART_CSRB_RXEN   (1 << 4)
#define USART_CSRB_TXEN   (1 << 3)
#define USART_CSRB_CSZ2   (1 << 2)
#define USART_CSRB_RXB8   (1 << 1)
#define USART_CSRB_TXB8   (1 << 0)

#define USART_CSRC_MSEL1  (1 << 7)
#define USART_CSRC_MSEL0  (1 << 6)
#define USART_CSRC_PM1    (1 << 5)
#define USART_CSRC_PM0    (1 << 4)
#define USART_CSRC_CSZ1   (1 << 2)
#define USART_CSRC_CSZ0   (1 << 1)

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
    Serialize parent_serialize;
 } AVRUARTClass;

#endif