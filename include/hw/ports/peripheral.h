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

// for internal code...
#define INTERRUPT_OVERFLOW 0
#define INTERRUPT_COMPA 1
#define INTERRUPT_COMPB 2
#define INTERRUPT_COMPC 3
#define INTERRUPT_CAPT  4


typedef struct 
{
    AVRPortState_t * pPort;
    uint8_t PinNum;
} PinID;

typedef struct 
{
    /* <private> */
    SysBusDevice parent_obj;

    /* <public> */
    MemoryRegion mmio;

    MemoryRegion mmio_imsk;
    MemoryRegion mmio_ifr;

    CharBackend chr;

    AVRPortState_t * father_port[128];
    
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

    //int16_t pinno_rx;
    //int16_t pinno_tx;
    PinID Pin_RX;
    PinID Pin_TX;

    /* Receive Complete */
    qemu_irq rxc_irq;
    /* Transmit Complete */
    qemu_irq txc_irq;
    /* Data Register Empty */
    qemu_irq dre_irq;
    /* UART END */

    /* ADC START */
    uint8_t adcsra;
    uint8_t adcsrb;
    uint8_t admux;
    uint16_t adc;   // 10 bit => 2 byte

    qemu_irq adc_conv_irq;

    double adc_voltages[NUM_PINS];
    /* ADC END */

    /* Timer 0 Start */
    QEMUTimer *timer;
    qemu_irq capt_irq; // only 16b timer
    qemu_irq compa_irq;
    qemu_irq compb_irq;
    qemu_irq compc_irq;
    qemu_irq ovf_irq;

    /* registers 8bit */
    uint8_t cra;        // TCCR0A
    uint8_t crb;        // TCCR0B
    uint8_t cnt;        // TCNT0
    uint8_t ocra;       // OCR0A
    uint8_t ocrb;       // OCR0B

    PinID Output_A;
    PinID Output_B;

    /* 16 bit: */
    uint8_t crc;
    uint8_t cntl;
    uint8_t cnth;
    uint8_t ocral;
    uint8_t ocrah;
    uint8_t ocrbl;
    uint8_t ocrbh;
    uint8_t ocrcl;
    uint8_t ocrch;
    uint8_t icrl;
    uint8_t icrh;

    uint8_t rtmp;
    uint8_t imsk;
    uint8_t ifr;
    
    uint8_t last_mode;
    uint8_t last_pwm;
    uint16_t last_ocra;
    uint16_t last_ocrb;
    uint8_t prev_clk_src;

    uint64_t cpu_freq_hz;
    uint64_t freq_hz;
    uint64_t period_ns;
    uint64_t reset_time_ns;
    
    uint8_t next_interrupt;
    PinID Output_C;
    /* Timer End */

} AVRPeripheralState;

/* Virtual Functions:
    can_receive
    receive
    read
    write
*/
typedef int (*CanReceive)(void *opaque);
typedef void (*Receive)(void *opaque, const uint8_t *buffer, int size, PinID pin);
typedef uint64_t (*Read)(void *opaque, hwaddr addr, unsigned int size);
typedef void (*Write)(void *opaque, hwaddr addr, uint64_t value, unsigned int size);
typedef int (*IsActive)(void * opaque, PinID pin);
typedef uint32_t (*Serialize)(void * opaque, PinID pin, uint8_t * pData);


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


/*static inline PinID * gen_pin_id(struct AVRPortState * pClass, uint8_t pinno)
{
    PinID * pPinID = (PinID*) malloc(sizeof(PinID));
    pPinID->pPort = pClass;
    pPinID->PinNum = pinno;

    return pPinID;
}*/

#endif

