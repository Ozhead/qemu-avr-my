#ifndef HW_AVR_PERIPHERAL_H
#define HW_AVR_PERIPHERAL_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "hw/hw.h"

#define ID_DIGIO 0
#define ID_USART 1
#define ID_PWM 2
#define ID_ADC 8

extern const uint8_t peripheral_msg_lengths[];

#define NUM_PINS 8
#define VCC 5.0

#define TYPE_AVR_PERIPHERAL "avr-peripheral"
#define AVR_PERIPHERAL(obj) \
    OBJECT_CHECK(AVRPeripheralState, (obj), TYPE_AVR_PERIPHERAL)

// for teh evil circular dependancy >:(
typedef struct AVRPortState AVRPortState_t;

#define INTERRUPT_OVERFLOW 0
#define INTERRUPT_COMPA 1
#define INTERRUPT_COMPB 2


typedef struct 
{
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    MemoryRegion mmio_imsk;
    MemoryRegion mmio_ifr;

    CharBackend chr;

    AVRPortState_t * father_port;
    
    /* UART START */
    bool enabled;
    uint8_t data;
    bool data_valid;
    uint8_t char_mask;
    /* Control and Status Registers */
    uint8_t csra;
    uint8_t csrb;
    uint8_t csrc;
    /* Baud Rate Registers (low/high byte) */
    uint8_t brrh;
    uint8_t brrl;

    /* Receive Complete */
    qemu_irq rxc_irq;
    /* Transmit Complete */
    qemu_irq txc_irq;
    /* Data Register Empty */
    qemu_irq dre_irq;
    /* UART END */

    /* ADC START */
    uint8_t adcsr;
    uint8_t admux;
    uint16_t adc;   // 10 bit => 2 byte

    double adc_voltages[NUM_PINS];
    /* ADC END */

    /* Timer 0 Start */
    QEMUTimer *timer;
    qemu_irq compa_irq;
    qemu_irq compb_irq;
    qemu_irq ovf_irq;

    /* registers */
    uint8_t cra;        // TCCR0A
    uint8_t crb;        // TCCR0B
    uint8_t cnt;        // TCNT0
    uint8_t ocra;       // OCR0A
    uint8_t ocrb;       // OCR0B

    uint8_t rtmp;
    uint8_t imsk;
    uint8_t ifr;
    
    uint8_t last_pwm;
    uint8_t last_ocra;
    uint8_t last_ocrb;

    uint64_t cpu_freq_hz;
    uint64_t freq_hz;
    uint64_t period_ns;
    uint64_t reset_time_ns;
    
    uint8_t next_interrupt;
    /* Timer 0 End */

} AVRPeripheralState;

/* Virtual Functions:
    can_receive
    receive
    read
    write
*/
typedef int (*CanReceive)(void *opaque);
typedef void (*Receive)(void *opaque, const uint8_t *buffer, int size, int pinno);
typedef uint64_t (*Read)(void *opaque, hwaddr addr, unsigned int size);
typedef void (*Write)(void *opaque, hwaddr addr, uint64_t value, unsigned int size);
typedef int (*IsActive)(void * opaque, uint32_t pinno);
typedef uint32_t (*Serialize)(void * opaque, uint32_t pinno, uint8_t * pData);

#define AVR_PERIPHERAL_GET_CLASS(obj) \
    OBJECT_GET_CLASS(AVRPeripheralClass, obj, TYPE_AVR_PERIPHERAL)
#define AVR_PERIPHERAL_CLASS(klass) \
    OBJECT_CLASS_CHECK(AVRPeripheralClass, klass, TYPE_AVR_PERIPHERAL)

typedef struct AVRPeripheralClass 
{
    SysBusDevice parent_class;

    CanReceive can_receive;
    Receive receive;
    Read read;
    Write write;
    IsActive is_active;
    Serialize serialize;

    /* additional read & write functions! */
    Read read_imsk;
    Write write_imsk;
    Read read_ifr;
    Write write_ifr;
} AVRPeripheralClass;

#endif
