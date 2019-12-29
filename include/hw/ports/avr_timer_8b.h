#ifndef HW_AVR_TIMER_8b_H
#define HW_AVR_TIMER_8b_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"
#include "hw/ports/peripheral.h"

#define TYPE_AVR_TIMER_8b "avr-timer-8b"

#define AVR_TIMER_8b(obj) \
    OBJECT_CHECK(AVRPeripheralState, (obj), TYPE_AVR_TIMER_8b)
#define AVR_TIMER_8b_GET_CLASS(obj) \
    OBJECT_GET_CLASS(AVRADCClass, obj, TYPE_AVR_TIMER_8b)
#define AVR_TIMER_8b_CLASS(klass) \
    OBJECT_CLASS_CHECK(AVRTimer8bClass, klass, TYPE_AVR_TIMER_8b)


 typedef struct AVRTimer8bClass
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
 } AVRTimer8bClass;



#endif