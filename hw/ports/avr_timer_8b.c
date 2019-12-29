#include "qemu/osdep.h"
#include "hw/ports/avr_timer_8b.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

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

#define T16_INT_TOV    0x0 /* Timer overflow */
#define T16_INT_OCA    0x1 /* Output compare A */
#define T16_INT_OCB    0x2 /* Output compare B */


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

static int avr_timer_8b_is_active(void *opaque, uint32_t pinno)
{
    return 0;
}

static void avr_timer_8b_receive(void *opaque, const uint8_t *buffer, int msgid, int pinno)
{

}

static uint64_t avr_timer_8b_read_ifr(void *opaque, hwaddr addr, unsigned int size)
{
    return 0;
}

static uint64_t avr_timer_8b_read_imsk(void *opaque, hwaddr addr, unsigned int size)
{
    return 0;
}

static uint64_t avr_timer_8b_read(void *opaque, hwaddr addr, unsigned int size)
{
    return 0;
}

static void avr_timer_8b_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    
}

static void avr_timer_8b_write_imsk(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    
}

static void avr_timer_8b_write_ifr(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    
}

static uint32_t avr_timer_8b_serialize(void * opaque, uint32_t pinno, uint8_t * pData)
{
    return 0;
}

static void avr_timer_8b_reset(DeviceState *dev)
{

}

static void avr_timer_8b_clock_reset(AVRPeripheralState *t16)
{
    t16->cnt = 0;
    t16->reset_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static inline int64_t avr_timer_8b_ns_to_ticks(AVRPeripheralState *t16, int64_t t)
{
    if (t16->period_ns == 0) {
        return 0;
    }
    return t / t16->period_ns;
}

static void avr_timer_8b_update_cnt(AVRPeripheralState *t16)
{
    uint16_t cnt;
    cnt = avr_timer_8b_ns_to_ticks(t16, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                                       t16->reset_time_ns);
    t16->cnt = (uint8_t)(cnt & 0xff);
}

static void avr_timer_8b_set_alarm(AVRPeripheralState *t16)
{
    if (CLKSRC(t16) == T16_CLKSRC_EXT_FALLING ||
        CLKSRC(t16) == T16_CLKSRC_EXT_RISING ||
        CLKSRC(t16) == T16_CLKSRC_STOPPED) {
        /* Timer is disabled or set to external clock source (unsupported) */
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

    printf("next alarm %" PRIu64 " ns from now", alarm_offset * t16->period_ns);

end:
    return;
}

static void avr_timer_8b_interrupt(void *opaque)
{
    printf("Der interrupt ballert\n");
    AVRPeripheralState *t16 = opaque;
    uint8_t mode = MODE(t16);

    avr_timer_8b_update_cnt(t16);

    if (CLKSRC(t16) == T16_CLKSRC_EXT_FALLING ||
        CLKSRC(t16) == T16_CLKSRC_EXT_RISING ||
        CLKSRC(t16) == T16_CLKSRC_STOPPED) {
        // Timer is disabled or set to external clock source (unsupported) 
        return;
    }

    printf("interrupt, cnt = %d\n", CNT(t16));

    // Counter overflow 
    if (t16->next_interrupt == INTERRUPT_OVERFLOW) {
        printf("0xff overflow\n");
        avr_timer_8b_clock_reset(t16);
        if (t16->imsk & T16_INT_TOV) 
        {
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
        t16->ifr |= T16_INT_OCA;
        qemu_set_irq(t16->compa_irq, 1);
    }
    if (t16->imsk & T16_INT_OCB && t16->next_interrupt == INTERRUPT_COMPB) 
    {
        t16->ifr |= T16_INT_OCB;
        qemu_set_irq(t16->compb_irq, 1);
    }

    avr_timer_8b_set_alarm(t16);
}

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
    .read = avr_timer_8b_read,
    .write = avr_timer_8b_write,
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