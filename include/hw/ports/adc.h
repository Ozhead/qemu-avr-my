#ifndef HW_AVR_ADC_H
#define HW_AVR_ADC_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"
#include "hw/ports/peripheral.h"

#define TYPE_AVR_ADC "avr-adc"

#define AVR_ADC(obj) \
    OBJECT_CHECK(AVRPeripheralState, (obj), TYPE_AVR_ADC)
#define AVR_ADC_GET_CLASS(obj) \
    OBJECT_GET_CLASS(AVRADCClass, obj, TYPE_AVR_ADC)
#define AVR_ADC_CLASS(klass) \
    OBJECT_CLASS_CHECK(AVRADCClass, klass, TYPE_AVR_ADC)

 typedef struct AVRADCClass
 {
    AVRPeripheralClass parent_class;

    CanReceive parent_can_receive;
    Receive parent_receive;
    Read parent_read;
    Write parent_write;
    IsActive parent_is_active;
 } AVRADCClass;



#endif