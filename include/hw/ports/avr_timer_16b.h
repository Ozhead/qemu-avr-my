#ifndef HW_AVR_TIMER_16b_H
#define HW_AVR_TIMER_16b_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"
#include "hw/ports/peripheral.h"

#define TYPE_AVR_TIMER_16b "avr-timer-16b"

#define AVR_TIMER_16b(obj) \
    OBJECT_CHECK(AVRPeripheralState, (obj), TYPE_AVR_TIMER_16b)
#define AVR_TIMER_16b_GET_CLASS(obj) \
    OBJECT_GET_CLASS(AVRTimer16bClass, obj, TYPE_AVR_TIMER_16b)
#define AVR_TIMER_16b_CLASS(klass) \
    OBJECT_CLASS_CHECK(AVRTimer16bClass, klass, TYPE_AVR_TIMER_16b)


 typedef struct AVRTimer16bClass
 {
    AVRPeripheralClass parent_class;

    CanReceive parent_can_receive;
    Receive parent_receive;
    Read parent_read;
    Write parent_write;
    IsActive parent_is_active;
    Serialize parent_serialize;

    /* additional timer functions */
    Read parent_read_ifr;
    Read parent_read_imsk;
    Write parent_write_ifr;
    Write parent_write_imsk;
 } AVRTimer16bClass;

#endif