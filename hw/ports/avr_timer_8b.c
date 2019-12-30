#include "qemu/osdep.h"
#include "hw/ports/avr_timer_8b.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ports/avr_port.h"

#define WGM00   0
#define WGM01   1
#define WGM02   11

#define MODE_NORMAL 0
#define MODE_CTC 2
#define MODE_FAST_PWM 3
#define MODE_PHASE_PWM 1


/* Clock source values */
#define T16_CLKSRC_STOPPED     0
#define T16_CLKSRC_DIV1        1
#define T16_CLKSRC_DIV8        2
#define T16_CLKSRC_DIV64       3
#define T16_CLKSRC_DIV256      4
#define T16_CLKSRC_DIV1024     5
#define T16_CLKSRC_EXT_FALLING 6
#define T16_CLKSRC_EXT_RISING  7

#define T16_INT_TOV    0x1 /* Timer overflow */
#define T16_INT_OCA    0x2 /* Output compare A */
#define T16_INT_OCB    0x4 /* Output compare B */


                    // fourth bit, set to bit 3 + bits 0 and 1 of cra! == WGM02:0
#define MODE(t16)   ((t16->cra & 3))
#define CLKSRC(t16) (t16->crb & 7)
#define CNT(t16)    (t16->cnt)
#define OCRA(t16)   (t16->ocra)
#define OCRB(t16)   (t16->ocrb)

static int avr_timer_8b_can_receive(void *opaque)
{
    return 0;
}

// if PWM is active or pin somehow other used...
static int avr_timer_8b_is_active(void *opaque, uint32_t pinno)
{
    printf("Timer is active on Pin %d\n", pinno);
    AVRPeripheralState *t16 = opaque;

    if (CLKSRC(t16) == T16_CLKSRC_EXT_FALLING ||
        CLKSRC(t16) == T16_CLKSRC_EXT_RISING ||
        CLKSRC(t16) == T16_CLKSRC_STOPPED) {
        /* Timer is disabled or set to external clock source (unsupported) */
        printf("No supported clock set...\n");
        return 0;
    }

    if(MODE(t16) == MODE_NORMAL || MODE(t16) == MODE_CTC)
    {
        return 0;
    }

    // the pin must be set to output port!
    AVRPortState * pPort = (AVRPortState*)t16->father_port;
    uint8_t pin_mask = (1 << pinno);
    if(pPort->ddr & pin_mask)
        return 1;

    return 0;
}

static void avr_timer_8b_receive(void *opaque, const uint8_t *buffer, int msgid, int pinno)
{
    assert(false);
}

static uint64_t avr_timer_8b_read_ifr(void *opaque, hwaddr addr, unsigned int size)
{
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    if (addr != 0) 
    {
        printf("Reading IFR that is not implemented!\n");
        return 0;
    }
    return t16->ifr;
}

static uint64_t avr_timer_8b_read_imsk(void *opaque, hwaddr addr, unsigned int size)
{
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    if (addr != 0) 
    {
        printf("Reading IMSK that is not implemented!\n");
        return 0;
    }
    return t16->imsk;
}

static void avr_timer_8b_clksrc_update(AVRPeripheralState *t16)
{
    uint16_t divider = 0;
    switch (CLKSRC(t16)) {
    case T16_CLKSRC_EXT_FALLING:
    case T16_CLKSRC_EXT_RISING:
        printf("external clock source unsupported\n");
        goto end;
    case T16_CLKSRC_STOPPED:
        goto end;
    case T16_CLKSRC_DIV1:
        divider = 1;
        break;
    case T16_CLKSRC_DIV8:
        divider = 8;
        break;
    case T16_CLKSRC_DIV64:
        divider = 64;
        break;
    case T16_CLKSRC_DIV256:
        divider = 256;
        break;
    case T16_CLKSRC_DIV1024:
        divider = 1024;
        break;
    default:
        goto end;
    }
    t16->freq_hz = t16->cpu_freq_hz / divider;
    t16->period_ns = NANOSECONDS_PER_SECOND / t16->freq_hz;
    printf("Timer frequency %" PRIu64 " hz, period %" PRIu64 " ns (%f s)\n",
             t16->freq_hz, t16->period_ns, 1 / (double)t16->freq_hz);
end:
    return;
}

//
static void avr_timer_8b_set_alarm(AVRPeripheralState *t16)
{
    if (CLKSRC(t16) == T16_CLKSRC_EXT_FALLING ||
        CLKSRC(t16) == T16_CLKSRC_EXT_RISING ||
        CLKSRC(t16) == T16_CLKSRC_STOPPED) {
        /* Timer is disabled or set to external clock source (unsupported) */
        printf("No clock set...\n");
        goto end;
    }

    uint64_t alarm_offset = 0xff;
    uint8_t next_interrupt = INTERRUPT_OVERFLOW;

    switch (MODE(t16)) 
    {
        case MODE_NORMAL:
            /* Normal mode */
            if (OCRA(t16) < alarm_offset && OCRA(t16) > CNT(t16) &&
                (t16->imsk & T16_INT_OCA)) 
            {
                alarm_offset = OCRA(t16);
                next_interrupt = INTERRUPT_COMPA;
            }
            break;
        case MODE_CTC:
            /* CTC mode, top = ocra */
            if (OCRA(t16) < alarm_offset && OCRA(t16) > CNT(t16)) {
                alarm_offset = OCRA(t16);
                next_interrupt = INTERRUPT_COMPA;
            }
        break;
        case MODE_FAST_PWM:
        {
            // nothing to do...
            ;
        }
        break;
        default:
            printf("pwm modes are unsupported\n");
            goto end;
    }

    if (OCRB(t16) < alarm_offset && OCRB(t16) > CNT(t16) &&
        (t16->imsk & T16_INT_OCB)) 
    {
        alarm_offset = OCRB(t16);
        next_interrupt = INTERRUPT_COMPB;
    }

    alarm_offset -= CNT(t16);

    t16->next_interrupt = next_interrupt;
    uint64_t alarm_ns = t16->reset_time_ns + ((CNT(t16) + alarm_offset) * t16->period_ns);
    timer_mod(t16->timer, alarm_ns);

    //printf("next alarm %" PRIu64 " ns from now\n", alarm_offset * t16->period_ns);

end:
    return;
}

static inline void avr_timer_8b_recalc_reset_time(AVRPeripheralState *t16)
{
    t16->reset_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                         CNT(t16) * t16->period_ns;
}

static uint64_t avr_timer_8b_read(void *opaque, hwaddr addr, unsigned int size)
{
    return 0;
}

// only called when pwm is really enabled!
static void avr_timer_8b_toggle_pwm(AVRPeripheralState * t16)
{
    uint8_t curr_pwm = t16->cra & 0b11110011;

    // PWM mode changed OR one of the compare registers changed OR the clock has been stopped!
    if(curr_pwm != t16->last_pwm || t16->ocra != t16->last_ocra || t16->ocrb != t16->last_ocrb || CLKSRC(t16) == T16_CLKSRC_STOPPED)
    {
        printf("Change in PWM detected!\n");
        t16->last_pwm = curr_pwm;
        t16->last_ocra = t16->ocra;
        t16->last_ocrb = t16->ocrb;

        AVRPortState * pPort = (AVRPortState*)t16->father_port;
        pPort->send_data(pPort);
    }
}

static void avr_timer_8b_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    //printf("AVR Timer Write\n");
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    uint8_t val8 = (uint8_t)value;
    uint8_t prev_clk_src = CLKSRC(t16);

    //printf("write %d to addr %d\n", val8, (uint8_t)addr);

    switch (addr) 
    {
    case 0:     // CRA
        t16->cra = val8;
        if (t16->cra & 0b11110000) 
        {
            printf("output compare pins unsupported\n");
        }
        avr_timer_8b_toggle_pwm(t16);
        break;
    case 1:     // CRB
        t16->crb = val8;

        if (CLKSRC(t16) != prev_clk_src) 
        {
            avr_timer_8b_clksrc_update(t16);
            if (prev_clk_src == T16_CLKSRC_STOPPED) {
                t16->reset_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            }

            if(CLKSRC(t16) == T16_CLKSRC_STOPPED && prev_clk_src != T16_CLKSRC_STOPPED)
            {
                avr_timer_8b_toggle_pwm(t16);
            }
        }
        break;
    case 2:     // CNT
        /*
         * CNT is the 16-bit counter value, it must be read/written via
         * a temporary register (rtmp) to make the read/write atomic.
         */
        /* ICR also has this behaviour, and shares rtmp */
        /*
         * Writing CNT blocks compare matches for one clock cycle.
         * Writing CNT to TOP or to an OCR value (if in use) will
         * skip the relevant interrupt
         */
        t16->cnt = val8;
        //t16->cnth = t16->rtmp;
        avr_timer_8b_recalc_reset_time(t16);
        break;
    case 3:     // OCRA
        /*
         * OCRn cause the relevant output compare flag to be raised, and
         * trigger an interrupt, when CNT is equal to the value here
         */
        t16->ocra = val8;
        avr_timer_8b_toggle_pwm(t16);   // OCRA can also lead to a change! because it changes the frequency!
        break;
    case 4:     // OCRB
        t16->ocrb = val8;
        avr_timer_8b_toggle_pwm(t16);   
        break;
    default:
        printf("Writing to AVR Timer 8b that is not defined??\n");
        break;
    }
    //printf("Set alarm...\n");
    avr_timer_8b_set_alarm(t16);
}

static void avr_timer_8b_write_imsk(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    //printf("Write imsk\n");
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    if (addr != 0) 
    {
        printf("Writing IMSK that is not implemented!\n");
        return;
    }
    t16->imsk = (uint8_t)value;
    //printf("Write imsk done %d\n", t16->imsk);
}

static void avr_timer_8b_write_ifr(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    //printf("Write ifr\n");
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    if (addr != 0) 
    {
        printf("Writing IFR that is not implemented!\n");
        return;
    }
    t16->ifr = (uint8_t)value;
}

static uint32_t avr_timer_8b_serialize(void * opaque, uint32_t pinno, uint8_t * pData)
{
    AVRPeripheralState *t16 = opaque;
    uint8_t hdr, mode;
    uint8_t top = 0xFF;

    if(t16->crb & 8)
    {
        top = t16->ocra;
        printf("Overwriting TOP to %d\n", top);
    }

    if(pinno == 3)
    {
        hdr = 0b01100100;
        mode = (t16->cra & 0b11000000) >> 6;
    }
    else if(pinno == 4)
    {
        hdr = 0b10000100;
        mode = (t16->cra & 0b00110000) >> 4;
    }
    else
    {
        printf("Wrong pinno given to avr timer serialize\n");
        return 0;
    }
    
    pData[0] = hdr;
    double val = 0;

    if(mode == 0)
    {
        printf("MODE 0 LOL => disabled\n");
    }
    else if(mode == 1)
    {
        printf("Mode 1 for CMPA not enabled\n");
    }
    else if(mode == 3)
        val = (double)(top + 1) / 256.0;
    else if(mode == 2)
        val = (double)(255 - top) / 256.0;

    printf("Val = %5.2f with top = %d\n", val, top);
    memcpy(pData+1, &val, sizeof(double));
    return 9;
}

//
static void avr_timer_8b_clock_reset(AVRPeripheralState *t16)
{
    t16->cnt = 0;
    t16->reset_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

//
static inline int64_t avr_timer_8b_ns_to_ticks(AVRPeripheralState *t16, int64_t t)
{
    if (t16->period_ns == 0) {
        return 0;
    }
    return t / t16->period_ns;
}

//
static void avr_timer_8b_update_cnt(AVRPeripheralState *t16)
{
    uint16_t cnt;
    cnt = avr_timer_8b_ns_to_ticks(t16, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                                       t16->reset_time_ns);
    t16->cnt = (uint8_t)(cnt & 0xff);
}

//
static void avr_timer_8b_interrupt(void *opaque)
{
    //printf("Der interrupt ballert\n");
    AVRPeripheralState *t16 = opaque;
    uint8_t mode = MODE(t16);

    avr_timer_8b_update_cnt(t16);

    if (CLKSRC(t16) == T16_CLKSRC_EXT_FALLING ||
        CLKSRC(t16) == T16_CLKSRC_EXT_RISING ||
        CLKSRC(t16) == T16_CLKSRC_STOPPED) {
        // Timer is disabled or set to external clock source (unsupported) 
        return;
    }

    //printf("interrupt, cnt = %d\n", CNT(t16));

    // Counter overflow 
    if (t16->next_interrupt == INTERRUPT_OVERFLOW) {
        //printf("0xff overflow\n");
        avr_timer_8b_clock_reset(t16);
        //printf("Before interrupt: %d\n", t16->imsk);
        if (t16->imsk & T16_INT_TOV) 
        {
            //printf("Starting overflow...\n");
            t16->ifr |= T16_INT_TOV;
            qemu_set_irq(t16->ovf_irq, 1);
        }
    }
    // Check for ocra overflow in CTC mode 
    if (mode == MODE_CTC && t16->next_interrupt == INTERRUPT_COMPA) 
    {
        printf("CTC OCRA overflow\n");
        avr_timer_8b_clock_reset(t16);
    }
    // Check for output compare interrupts 
    if (t16->imsk & T16_INT_OCA && t16->next_interrupt == INTERRUPT_COMPA) 
    {
        printf("Set Compa irq\n");
        t16->ifr |= T16_INT_OCA;
        qemu_set_irq(t16->compa_irq, 1);
    }
    if (t16->imsk & T16_INT_OCB && t16->next_interrupt == INTERRUPT_COMPB) 
    {
        printf("Set compb irq\n");
        t16->ifr |= T16_INT_OCB;
        qemu_set_irq(t16->compb_irq, 1);
    }

    avr_timer_8b_set_alarm(t16);
}

//
static void avr_timer_8b_reset(DeviceState *dev)
{
    AVRPeripheralState *t16 = AVR_TIMER_8b(dev);

    avr_timer_8b_clock_reset(t16);
    avr_timer_8b_clksrc_update(t16);
    avr_timer_8b_set_alarm(t16);

    qemu_set_irq(t16->compa_irq, 0);
    qemu_set_irq(t16->compb_irq, 0);
    qemu_set_irq(t16->ovf_irq, 0);
}

static void avr_timer_8b_pr(void *opaque, int irq, int level)
{
    AVRPeripheralState *s = AVR_TIMER_8b(opaque);

    s->enabled = !level;

    if (!s->enabled) {
        avr_timer_8b_reset(DEVICE(s));
    }
}

/* Class functions below... */
static Property avr_timer_8b_properties[] = {
    DEFINE_PROP_UINT64("cpu-frequency-hz", AVRPeripheralState,
                       cpu_freq_hz, 20000000),
    DEFINE_PROP_END_OF_LIST(),
};

static void avr_timer_8b_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AVRPeripheralClass *pc = AVR_PERIPHERAL_CLASS(klass);
    AVRTimer8bClass * adc = AVR_TIMER_8b_CLASS(klass);

    dc->reset = avr_timer_8b_reset;
    dc->props = avr_timer_8b_properties;
  
    adc->parent_can_receive = pc->can_receive;
    adc->parent_receive = pc->receive;
    adc->parent_read = pc->read;
    adc->parent_write = pc->write;
    adc->parent_is_active = pc->is_active;
    adc->parent_serialize = pc->serialize;

    adc->parent_read_ifr = pc->read_ifr;
    adc->parent_read_imsk = pc->read_imsk;
    adc->parent_write_ifr = pc->write_ifr;
    adc->parent_write_imsk = pc->write_imsk;

    pc->can_receive = avr_timer_8b_can_receive;
    pc->read = avr_timer_8b_read;
    pc->write = avr_timer_8b_write;
    pc->receive = avr_timer_8b_receive;
    pc->is_active = avr_timer_8b_is_active;
    pc->serialize = avr_timer_8b_serialize;

    pc->write_ifr = avr_timer_8b_write_ifr;
    pc->write_imsk = avr_timer_8b_write_imsk;
    pc->read_ifr = avr_timer_8b_read_ifr;
    pc->read_imsk = avr_timer_8b_read_imsk;
    printf("ADC class initiated\n");
}


static const MemoryRegionOps avr_timer_8b_ops = {
    .read = avr_timer_8b_read,
    .write = avr_timer_8b_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.min_access_size = 1, .max_access_size = 1}
};

static const MemoryRegionOps avr_timer_8b_imsk_ops = {
    .read = avr_timer_8b_read_imsk,
    .write = avr_timer_8b_write_imsk,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.max_access_size = 1}
};

static const MemoryRegionOps avr_timer_8b_ifr_ops = {
    .read = avr_timer_8b_read_ifr,
    .write = avr_timer_8b_write_ifr,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.max_access_size = 1}
};

static void avr_timer_8b_init(Object *obj)
{
    AVRPeripheralState *s = AVR_PERIPHERAL(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->compa_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->compb_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->ovf_irq);


    memory_region_init_io(&s->mmio, obj, &avr_timer_8b_ops, s, TYPE_AVR_TIMER_8b, 8);

    memory_region_init_io(&s->mmio_imsk, obj, &avr_timer_8b_imsk_ops,
                          s, TYPE_AVR_TIMER_8b, 0x1);
    memory_region_init_io(&s->mmio_ifr, obj, &avr_timer_8b_ifr_ops,
                          s, TYPE_AVR_TIMER_8b, 0x1);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio_imsk);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio_ifr);

    qdev_init_gpio_in(DEVICE(s), avr_timer_8b_pr, 1);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, avr_timer_8b_interrupt, s);
    s->enabled = true;
    printf("AVR Timer8b object init\n");
}

static const TypeInfo avr_timer_8b_info = {
    .name          = TYPE_AVR_TIMER_8b,
    .parent        = TYPE_AVR_PERIPHERAL,
    .class_init    = avr_timer_8b_class_init,
    .class_size    = sizeof(AVRTimer8bClass),
    .instance_size = sizeof(AVRPeripheralState),
    .instance_init = avr_timer_8b_init
};

static void avr_timer_8b_register_types(void)
{
    type_register_static(&avr_timer_8b_info);
}

type_init(avr_timer_8b_register_types)