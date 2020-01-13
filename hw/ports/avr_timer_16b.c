#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ports/avr_timer_16b.h"
#include "hw/ports/avr_port.h"

/* Register offsets */
#define T16_CRA     0x0
#define T16_CRB     0x1
#define T16_CRC     0x2
#define T16_CNTL    0x4
#define T16_CNTH    0x5
#define T16_ICRL    0x6
#define T16_ICRH    0x7
#define T16_OCRAL   0x8
#define T16_OCRAH   0x9
#define T16_OCRBL   0xa
#define T16_OCRBH   0xb
#define T16_OCRCL   0xc
#define T16_OCRCH   0xd

/* Field masks */
#define T16_CRA_WGM01   0x3
#define T16_CRA_COMC    0xc
#define T16_CRA_COMB    0x30
#define T16_CRA_COMA    0xc0
#define T16_CRA_OC_CONF \
    (T16_CRA_COMA | T16_CRA_COMB | T16_CRA_COMC)

#define T16_CRB_CS      0x7
#define T16_CRB_WGM23   0x18
#define T16_CRB_ICES    0x40
#define T16_CRB_ICNC    0x80

#define T16_CRC_FOCC    0x20
#define T16_CRC_FOCB    0x40
#define T16_CRC_FOCA    0x80

/* Fields masks both TIMSK and TIFR (interrupt mask/flag registers) */
#define T16_INT_TOV    0x1 /* Timer overflow */
#define T16_INT_OCA    0x2 /* Output compare A */
#define T16_INT_OCB    0x4 /* Output compare B */
#define T16_INT_OCC    0x8 /* Output compare C */
#define T16_INT_IC     0x20 /* Input capture */

/* Clock source values */
#define T16_CLKSRC_STOPPED     0
#define T16_CLKSRC_DIV1        1
#define T16_CLKSRC_DIV8        2
#define T16_CLKSRC_DIV64       3
#define T16_CLKSRC_DIV256      4
#define T16_CLKSRC_DIV1024     5
#define T16_CLKSRC_EXT_FALLING 6
#define T16_CLKSRC_EXT_RISING  7

/* Timer mode values (not including PWM modes) */
#define T16_MODE_NORMAL     0
#define T16_MODE_CTC_OCRA   4
#define T16_MODE_CTC_ICR    12

#define T16_MODE_PHASE_PWM_8    1
#define T16_MODE_PHASE_PWM_9    2
#define T16_MODE_PHASE_PWM_10   3
#define T16_MODE_FAST_PWM_8     5
#define T16_MODE_FAST_PWM_9     6
#define T16_MODE_FAST_PWM_10    7
#define T16_MODE_PHASE_FREQUENCY_PWM_ICR    8
#define T16_MODE_PHASE_FREQUENCY_PWM_OCRA   9
#define T16_MODE_PHASE_PWM_ICRA 10
#define T16_MODE_PHASE_PWM_OCRA 11
#define T16_MODE_FAST_PWM_ICRA  14
#define T16_MODE_FAST_PWM_OCRA  15

/* Accessors */
#define CLKSRC(t16) (t16->crb & T16_CRB_CS)
#define MODE(t16)   (((t16->crb & T16_CRB_WGM23) >> 1) | \
                     (t16->cra & T16_CRA_WGM01))
#define CNT(t16)    VAL16(t16->cntl, t16->cnth)
#define OCRA(t16)   VAL16(t16->ocral, t16->ocrah)
#define OCRB(t16)   VAL16(t16->ocrbl, t16->ocrbh)
#define OCRC(t16)   VAL16(t16->ocrcl, t16->ocrch)
#define ICR(t16)    VAL16(t16->icrl, t16->icrh)

/* Helper macros */
#define VAL16(l, h) ((h << 8) | l)
#define ERROR(fmt, args...) \
    qemu_log_mask(LOG_GUEST_ERROR, "%s: " fmt "\n", __func__, ## args)
#define DB_PRINT(fmt, args...) /* Nothing */
/*#define DB_PRINT(fmt, args...) printf("%s: " fmt "\n", __func__, ## args)*/


/* IO Functions */
static int avr_timer_16b_can_receive(void *opaque)
{
    return 0;
}


static int avr_timer_16b_is_active(void *opaque, uint32_t pinno)
{
    printf("Call Timer16 is active on Pin %d\n", pinno);
    AVRPeripheralState *t16 = opaque;

    if (CLKSRC(t16) == T16_CLKSRC_EXT_FALLING ||
        CLKSRC(t16) == T16_CLKSRC_EXT_RISING ||
        CLKSRC(t16) == T16_CLKSRC_STOPPED) {
        /* Timer is disabled or set to external clock source (unsupported) */
        printf("No supported clock set...\n");
        return 0;
    }

    if(MODE(t16) == T16_MODE_NORMAL || MODE(t16) == T16_MODE_CTC_ICR || MODE(t16) == T16_MODE_CTC_OCRA)
    {
        // CAUTION: If the last mode was PWM and now not while the timer is still running, there must be sent a 0 to the target
        // because DigIO is not active right now!
        if(t16->last_mode == T16_MODE_NORMAL || t16->last_mode == T16_MODE_CTC_ICR || t16->last_mode == T16_MODE_CTC_OCRA)
            return 0;
    }

        // the pin must be set to output port!
    AVRPortState * pPort = (AVRPortState*)t16->father_port;
    uint8_t pin_mask = (1 << pinno);
    if(pPort->ddr & pin_mask)
        return 1;

    return 0;
}

static void avr_timer_16b_receive(void *opaque, const uint8_t *buffer, int msgid, int pinno)
{
    assert(false);
}

static uint32_t avr_timer_16b_serialize(void * opaque, uint32_t pinno, uint8_t * pData)
{
    AVRPeripheralState *t16 = opaque;
    uint8_t mode = MODE(t16);
    uint8_t com_mode, hdr;
    uint16_t top;
    uint16_t compmatch;

    // set correct top value
    switch(mode)
    {
        case T16_MODE_FAST_PWM_8:
        case T16_MODE_PHASE_PWM_8:
            top = 0xFF + 1;
            //compmatch = OCRA(t16);
            break;
        case T16_MODE_FAST_PWM_9:
        case T16_MODE_PHASE_PWM_9:
            top = 0x1FF + 1;
            //compmatch = OCRA(t16);
            break;
        case T16_MODE_FAST_PWM_10:
        case T16_MODE_PHASE_PWM_10:
            top = 0x3FF + 1;
            //top = OCRA(t16);
            break;
        case T16_MODE_FAST_PWM_ICRA:
        case T16_MODE_PHASE_PWM_ICRA:
        case T16_MODE_PHASE_FREQUENCY_PWM_ICR:
            top = ICR(t16); //TODO
            if(top < 3)
            {
                printf("Warning: Minimum value for TOP is 3. Given top value is %d. Setting TOP = 3\n", top);
                top = 3;
            }
            break;
        case T16_MODE_FAST_PWM_OCRA:
        case T16_MODE_PHASE_PWM_OCRA:
        case T16_MODE_PHASE_FREQUENCY_PWM_OCRA:
            top = OCRA(t16);    //TODO
            if(top < 3)
            {
                printf("Warning: Minimum value for TOP is 3. Given top value is %d. Setting TOP = 3\n", top);
                top = 3;
            }
            break;
        default:
            top = 0xFFFF;
    }

    if(pinno == 4)
    {
        hdr = 0b10000100;
        com_mode = (t16->cra & 0b11000000) >> 6;
        compmatch = OCRA(t16);
        printf("Pin3 Mode set to %d\n", mode);
    }
    else if(pinno == 5)
    {
        hdr = 0b10100100;
        com_mode = (t16->cra & 0b00110000) >> 4;
        compmatch = OCRB(t16);
        printf("Pin4 Mode = %d\n", mode);
    }
    else
    {
        printf("Wrong pinno given to avr timer serialize\n");
        return 0;
    }
    
    pData[0] = hdr;
    double val = 0;

    if(mode == T16_MODE_NORMAL)
    {
        ;   // we send the 0
    }
    else if(mode == T16_MODE_CTC_ICR || mode == T16_MODE_CTC_OCRA)
    {
        printf("TODO: T16_MODE_CTC & OCRA\n");
        return 0;
    }
   /* else if(mode == T16_MODE_FAST_PWM_8 || mode == T16_MODE_PHASE_PWM_8)
    {
        if(com_mode == 2)
            val = (double)(255 - top) / 256.0;
        else if(com_mode == 3)
            val = (double)(top + 1) / 256.0;    
        else
            printf("Error: COM is set to 0 or 1 which disables the alternate port function for this port!\n");
    }
    else if(mode == T16_MODE_FAST_PWM_9 || mode == T16_MODE_PHASE_PWM_9)
    {
        if(com_mode == 2)
            val = (double)(511 - top) / 512.0;
        else if(com_mode == 3)
            val = (double)(top + 1) / 512.0;    
        else
            printf("Error: COM is set to 0 or 1 which disables the alternate port function for this port!\n");
    }
    else if(mode == T16_MODE_FAST_PWM_10 || mode == T16_MODE_PHASE_PWM_10)
    {
        if(com_mode == 2)
            val = (double)(1023 - top) / 1024.0;
        else if(com_mode == 3)
            val = (double)(top + 1) / 1024.0;    
        else
            printf("Error: COM is set to 0 or 1 which disables the alternate port function for this port!\n");
    }*/
    else
    {
        if(compmatch >= top)
        {
            printf("Warning: OCRnx >= defined TOP value for the timer. No PWM possible.\n");
        }
        else
        {
            if(com_mode == 2)
            {
                val = (double)(top-1 - compmatch) / (double)(top);
                printf("Val = %5.6f with compmatch = %d and top = %d (non-inverted)\n", val, compmatch, top);
            }
            else if(com_mode == 3)
            {
                //TODO
                val = (double)(compmatch + 1) / (double)(top);
                printf("Val = %5.6f with compmatch = %d and top = %d (inverted)\n", val, compmatch, top);
            }
            else if(com_mode == 1 && pinno == 4)    // TODO
                val = 0.5;
            else
            {
                printf("This port is disabled for this COMnx configuration\n");
                val = 0.0;
            }
        }
    }

    memcpy(pData+1, &val, sizeof(double));
    return 9;
}

static void avr_timer_16b_toggle_pwm(AVRPeripheralState * t16)
{
    uint8_t curr_mode = MODE(t16);
    uint8_t was_pwm = 1, is_pwm = 1;
    //uint8_t curr_pwm = t16->cra & 0b11110011;

    if(curr_mode != t16->last_mode)
    {
        if(curr_mode == T16_MODE_NORMAL || curr_mode == T16_MODE_CTC_ICR || curr_mode == T16_MODE_CTC_OCRA)
            is_pwm = 0;
        if(t16->last_mode == T16_MODE_NORMAL || t16->last_mode == T16_MODE_CTC_ICR || t16->last_mode == T16_MODE_CTC_OCRA)
            was_pwm = 0;

        // a change in DDR is not captured anyway; a change in DDR will toggle a resend in the port itself
        if(is_pwm != was_pwm || OCRA(t16) != t16->last_ocra || OCRB(t16) != t16->last_ocrb || CLKSRC(t16) != t16->prev_clk_src)   //or clock changed
        {
            t16->last_ocra = OCRA(t16);
            t16->last_ocrb = OCRB(t16);

            AVRPortState * pPort = (AVRPortState*)t16->father_port;
            pPort->send_data(pPort);
        }

        t16->last_mode = curr_mode;
    }

    // PWM mode changed OR one of the compare registers changed OR the clock has been stopped!
    /*if(curr_pwm != t16->last_pwm || t16->ocra != t16->last_ocra || t16->ocrb != t16->last_ocrb || CLKSRC(t16) == T16_CLKSRC_STOPPED)
    {
        printf("Change in PWM detected!\n");
        t16->last_pwm = curr_pwm;
        t16->last_ocra = t16->ocra;
        t16->last_ocrb = t16->ocrb;

        AVRPortState * pPort = (AVRPortState*)t16->father_port;
        pPort->send_data(pPort);
    }*/
}

/* Timer functions */
static inline int64_t avr_timer_16b_ns_to_ticks(AVRPeripheralState *t16, int64_t t)
{
    if (t16->period_ns == 0) {
        return 0;
    }
    return t / t16->period_ns;
}

static void avr_timer_16b_update_cnt(AVRPeripheralState *t16)
{
    uint16_t cnt;
    cnt = avr_timer_16b_ns_to_ticks(t16, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                                       t16->reset_time_ns);
    t16->cntl = (uint8_t)(cnt & 0xff);
    t16->cnth = (uint8_t)((cnt & 0xff00) >> 8);
}

static inline void avr_timer_16b_recalc_reset_time(AVRPeripheralState *t16)
{
    t16->reset_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                         CNT(t16) * t16->period_ns;
}

static void avr_timer_16b_clock_reset(AVRPeripheralState *t16)
{
    t16->cntl = 0;
    t16->cnth = 0;
    t16->reset_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static void avr_timer_16b_clksrc_update(AVRPeripheralState *t16)
{
    uint16_t divider = 0;
    switch (CLKSRC(t16)) {
    case T16_CLKSRC_EXT_FALLING:
    case T16_CLKSRC_EXT_RISING:
        ERROR("external clock source unsupported");
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
    DB_PRINT("Timer frequency %" PRIu64 " hz, period %" PRIu64 " ns (%f s)",
             t16->freq_hz, t16->period_ns, 1 / (double)t16->freq_hz);
end:
    return;
}

static void avr_timer_16b_set_alarm(AVRPeripheralState *t16)
{
    if (CLKSRC(t16) == T16_CLKSRC_EXT_FALLING ||
        CLKSRC(t16) == T16_CLKSRC_EXT_RISING ||
        CLKSRC(t16) == T16_CLKSRC_STOPPED) {
        /* Timer is disabled or set to external clock source (unsupported) */
        goto end;
    }

    uint64_t alarm_offset = 0xffff;
    uint8_t next_interrupt = INTERRUPT_OVERFLOW;

    switch (MODE(t16)) {
    case T16_MODE_NORMAL:
        /* Normal mode */
        if (OCRA(t16) < alarm_offset && OCRA(t16) > CNT(t16) &&
            (t16->imsk & T16_INT_OCA)) {
            alarm_offset = OCRA(t16);
            next_interrupt = INTERRUPT_COMPA;
        }
        break;
    case T16_MODE_CTC_OCRA:
        /* CTC mode, top = ocra */
        if (OCRA(t16) < alarm_offset && OCRA(t16) > CNT(t16)) {
            alarm_offset = OCRA(t16);
            next_interrupt = INTERRUPT_COMPA;
        }
       break;
    case T16_MODE_CTC_ICR:
        /* CTC mode, top = icr */
        if (ICR(t16) < alarm_offset && ICR(t16) > CNT(t16)) {
            alarm_offset = ICR(t16);
            next_interrupt = INTERRUPT_CAPT;
        }
        if (OCRA(t16) < alarm_offset && OCRA(t16) > CNT(t16) &&
            (t16->imsk & T16_INT_OCA)) {
            alarm_offset = OCRA(t16);
            next_interrupt = INTERRUPT_COMPA;
        }
        break;
    case T16_MODE_FAST_PWM_8:
    case T16_MODE_FAST_PWM_9:
    case T16_MODE_FAST_PWM_10:
    case T16_MODE_FAST_PWM_ICRA:
    case T16_MODE_FAST_PWM_OCRA:
        // execute all interrupts!
        if (ICR(t16) < alarm_offset && ICR(t16) > CNT(t16)) {
            alarm_offset = ICR(t16);
            next_interrupt = INTERRUPT_CAPT;
        }
        if (OCRA(t16) < alarm_offset && OCRA(t16) > CNT(t16) &&
            (t16->imsk & T16_INT_OCA)) {
            alarm_offset = OCRA(t16);
            next_interrupt = INTERRUPT_COMPA;
        }
        // TODO: Maybe set the Pins? or is it unnecessary?
        break;
    default:
        printf("pwm modes are unsupported\n");
        goto end;
    }
    if (OCRB(t16) < alarm_offset && OCRB(t16) > CNT(t16) &&
        (t16->imsk & T16_INT_OCB)) {
        alarm_offset = OCRB(t16);
        next_interrupt = INTERRUPT_COMPB;
    }
    if (OCRC(t16) < alarm_offset && OCRC(t16) > CNT(t16) &&
        (t16->imsk & T16_INT_OCC)) {
        alarm_offset = OCRC(t16);
        next_interrupt = INTERRUPT_COMPC;
    }
    alarm_offset -= CNT(t16);

    t16->next_interrupt = next_interrupt;
    uint64_t alarm_ns =
        t16->reset_time_ns + ((CNT(t16) + alarm_offset) * t16->period_ns);
    timer_mod(t16->timer, alarm_ns);

    //printf("16b next alarm %" PRIu64 " ns from now\n",
    //    alarm_offset * t16->period_ns);

end:
    return;
}

static void avr_timer_16b_interrupt(void *opaque)
{
    AVRPeripheralState *t16 = opaque;
    uint8_t mode = MODE(t16);

    avr_timer_16b_update_cnt(t16);

    if (CLKSRC(t16) == T16_CLKSRC_EXT_FALLING ||
        CLKSRC(t16) == T16_CLKSRC_EXT_RISING ||
        CLKSRC(t16) == T16_CLKSRC_STOPPED) {
        /* Timer is disabled or set to external clock source (unsupported) */
        return;
    }

    DB_PRINT("interrupt, cnt = %d", CNT(t16));

    /* Counter overflow */
    if (t16->next_interrupt == INTERRUPT_OVERFLOW) {
        DB_PRINT("0xffff overflow");
        avr_timer_16b_clock_reset(t16);
        if (t16->imsk & T16_INT_TOV) {
            t16->ifr |= T16_INT_TOV;
            qemu_set_irq(t16->ovf_irq, 1);
        }
    }
    /* Check for ocra overflow in CTC mode */
    if (mode == T16_MODE_CTC_OCRA && t16->next_interrupt == INTERRUPT_COMPA) {
        DB_PRINT("CTC OCRA overflow");
        avr_timer_16b_clock_reset(t16);
    }
    /* Check for icr overflow in CTC mode */
    if (mode == T16_MODE_CTC_ICR && t16->next_interrupt == INTERRUPT_CAPT) {
        DB_PRINT("CTC ICR overflow");
        avr_timer_16b_clock_reset(t16);
        if (t16->imsk & T16_INT_IC) {
            t16->ifr |= T16_INT_IC;
            qemu_set_irq(t16->capt_irq, 1);
        }
    }
    /* Check for output compare interrupts */
    if (t16->imsk & T16_INT_OCA && t16->next_interrupt == INTERRUPT_COMPA) {
        t16->ifr |= T16_INT_OCA;
        qemu_set_irq(t16->compa_irq, 1);
    }
    if (t16->imsk & T16_INT_OCB && t16->next_interrupt == INTERRUPT_COMPB) {
        t16->ifr |= T16_INT_OCB;
        qemu_set_irq(t16->compb_irq, 1);
    }
    if (t16->imsk & T16_INT_OCC && t16->next_interrupt == INTERRUPT_COMPC) {
        t16->ifr |= T16_INT_OCC;
        qemu_set_irq(t16->compc_irq, 1);
    }
    avr_timer_16b_set_alarm(t16);
}

static void avr_timer_16b_reset(DeviceState *dev)
{
    AVRPeripheralState *t16 =  AVR_TIMER_16b(dev);

    avr_timer_16b_clock_reset(t16);
    avr_timer_16b_clksrc_update(t16);
    avr_timer_16b_set_alarm(t16);

    qemu_set_irq(t16->capt_irq, 0);
    qemu_set_irq(t16->compa_irq, 0);
    qemu_set_irq(t16->compb_irq, 0);
    qemu_set_irq(t16->compc_irq, 0);
    qemu_set_irq(t16->ovf_irq, 0);
}

static uint64_t avr_timer_16b_read(void *opaque, hwaddr offset, unsigned size)
{
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    uint8_t retval = 0;

    switch (offset) {
    case T16_CRA:
        retval = t16->cra;
        break;
    case T16_CRB:
        retval = t16->crb;
        break;
    case T16_CRC:
        retval = t16->crc;
        break;
    case T16_CNTL:
        avr_timer_16b_update_cnt(t16);
        t16->rtmp = t16->cnth;
        retval = t16->cntl;
        break;
    case T16_CNTH:
        retval = t16->rtmp;
        break;
    case T16_ICRL:
        /*
         * The timer copies cnt to icr when the input capture pin changes
         * state or when the analog comparator has a match. We don't
         * emulate this behaviour. We do support it's use for defining a
         * TOP value in T16_MODE_CTC_ICR
         */
        t16->rtmp = t16->icrh;
        retval = t16->icrl;
        break;
    case T16_ICRH:
        retval = t16->rtmp;
        break;
    case T16_OCRAL:
        retval = t16->ocral;
        break;
    case T16_OCRAH:
        retval = t16->ocrah;
        break;
    case T16_OCRBL:
        retval = t16->ocrbl;
        break;
    case T16_OCRBH:
        retval = t16->ocrbh;
        break;
    case T16_OCRCL:
        retval = t16->ocrcl;
        break;
    case T16_OCRCH:
        retval = t16->ocrch;
        break;
    default:
        break;
    }
    return (uint64_t)retval;
}


static uint64_t avr_timer_16b_read_ifr(void *opaque, hwaddr addr, unsigned int size)
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

static uint64_t avr_timer_16b_read_imsk(void *opaque, hwaddr addr, unsigned int size)
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

static void avr_timer_16b_write_imsk(void *opaque, hwaddr addr, uint64_t value,
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

static void avr_timer_16b_write_ifr(void *opaque, hwaddr addr, uint64_t value,
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

static void avr_timer_16b_write(void *opaque, hwaddr offset,
                              uint64_t val64, unsigned size)
{
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    uint8_t val8 = (uint8_t)val64;
    t16->prev_clk_src = CLKSRC(t16);

    DB_PRINT("write %d to offset %d", val8, (uint8_t)offset);

    switch (offset) {
    case T16_CRA:
        t16->cra = val8;
        if (t16->cra & T16_CRA_OC_CONF) {
            ERROR("output compare pins unsupported");
        }
        avr_timer_16b_toggle_pwm(t16);
        break;
    case T16_CRB:
        t16->crb = val8;
        if (t16->crb & T16_CRB_ICNC) {
            ERROR("input capture noise canceller unsupported");
        }
        if (t16->crb & T16_CRB_ICES) {
            ERROR("input capture unsupported");
        }
        if (CLKSRC(t16) != t16->prev_clk_src) {
            avr_timer_16b_clksrc_update(t16);
            if (t16->prev_clk_src == T16_CLKSRC_STOPPED) {
                t16->reset_time_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            }
        }
        avr_timer_16b_toggle_pwm(t16);
        break;
    case T16_CRC:
        t16->crc = val8;
        avr_timer_16b_toggle_pwm(t16);
        ERROR("output compare pins unsupported");
        break;
    case T16_CNTL:
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
        t16->cntl = val8;
        t16->cnth = t16->rtmp;
        avr_timer_16b_recalc_reset_time(t16);
        break;
    case T16_CNTH:
        t16->rtmp = val8;
        break;
    case T16_ICRL:
        /* ICR can only be written in mode T16_MODE_CTC_ICR */
        if (MODE(t16) == T16_MODE_CTC_ICR) {
            t16->icrl = val8;
            t16->icrh = t16->rtmp;
            avr_timer_16b_toggle_pwm(t16);
        }
        break;
    case T16_ICRH:
        if (MODE(t16) == T16_MODE_CTC_ICR) {
            t16->rtmp = val8;
            avr_timer_16b_toggle_pwm(t16);
        }
        break;
    case T16_OCRAL:
        /*
         * OCRn cause the relevant output compare flag to be raised, and
         * trigger an interrupt, when CNT is equal to the value here
         */
        t16->ocral = val8;
        avr_timer_16b_toggle_pwm(t16);
        break;
    case T16_OCRAH:
        t16->ocrah = val8;
        avr_timer_16b_toggle_pwm(t16);
        break;
    case T16_OCRBL:
        t16->ocrbl = val8;
        avr_timer_16b_toggle_pwm(t16);
        break;
    case T16_OCRBH:
        t16->ocrbh = val8;
        avr_timer_16b_toggle_pwm(t16);
        break;
    case T16_OCRCL:
        t16->ocrcl = val8;
        avr_timer_16b_toggle_pwm(t16);
        break;
    case T16_OCRCH:
        t16->ocrch = val8;
        avr_timer_16b_toggle_pwm(t16);
        break;
    default:
        break;
    }
    avr_timer_16b_set_alarm(t16);
}

static uint64_t avr_timer_16b_imsk_read(void *opaque,
                                      hwaddr offset,
                                      unsigned size)
{
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    if (offset != 0) {
        return 0;
    }
    return t16->imsk;
}

static void avr_timer_16b_imsk_write(void *opaque, hwaddr offset,
                                   uint64_t val64, unsigned size)
{
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    if (offset != 0) {
        return;
    }
    t16->imsk = (uint8_t)val64;
}

static uint64_t avr_timer_16b_ifr_read(void *opaque,
                                     hwaddr offset,
                                     unsigned size)
{
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    if (offset != 0) {
        return 0;
    }
    return t16->ifr;
}

static void avr_timer_16b_ifr_write(void *opaque, hwaddr offset,
                                  uint64_t val64, unsigned size)
{
    assert(size == 1);
    AVRPeripheralState *t16 = opaque;
    if (offset != 0) {
        return;
    }
    t16->ifr = (uint8_t)val64;
}

static const MemoryRegionOps avr_timer_16b_ops = {
    .read = avr_timer_16b_read,
    .write = avr_timer_16b_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.max_access_size = 1}
};

static const MemoryRegionOps avr_timer_16b_imsk_ops = {
    .read = avr_timer_16b_imsk_read,
    .write = avr_timer_16b_imsk_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.max_access_size = 1}
};

static const MemoryRegionOps avr_timer_16b_ifr_ops = {
    .read = avr_timer_16b_ifr_read,
    .write = avr_timer_16b_ifr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.max_access_size = 1}
};

static Property avr_timer_16b_properties[] = {
    DEFINE_PROP_UINT64("cpu-frequency-hz", AVRPeripheralState,
                       cpu_freq_hz, 20000000),
    DEFINE_PROP_END_OF_LIST(),
};

static void avr_timer_16b_pr(void *opaque, int irq, int level)
{
    AVRPeripheralState *s = AVR_TIMER_16b(opaque);

    s->enabled = !level;

    if (!s->enabled) {
        avr_timer_16b_reset(DEVICE(s));
    }
}

static void avr_timer_16b_init(Object *obj)
{
    AVRPeripheralState *s = AVR_PERIPHERAL(obj);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->capt_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->compa_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->compb_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->compc_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->ovf_irq);


    memory_region_init_io(&s->mmio, obj, &avr_timer_16b_ops, s, TYPE_AVR_TIMER_16b, 0xE);   //TODO: This is overlapping! But QEMU seems to handle it correct!

    memory_region_init_io(&s->mmio_imsk, obj, &avr_timer_16b_imsk_ops,
                          s, TYPE_AVR_TIMER_16b, 0x1);
    memory_region_init_io(&s->mmio_ifr, obj, &avr_timer_16b_ifr_ops,
                          s, TYPE_AVR_TIMER_16b, 0x1);

    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio_imsk);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio_ifr);

    qdev_init_gpio_in(DEVICE(s), avr_timer_16b_pr, 1);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, avr_timer_16b_interrupt, s);
    s->enabled = true;
    printf("AVR Timer16b object init\n");
}

static void avr_timer_16b_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AVRPeripheralClass *pc = AVR_PERIPHERAL_CLASS(klass);
    AVRTimer16bClass * adc = AVR_TIMER_16b_CLASS(klass);

    dc->reset = avr_timer_16b_reset;
    dc->props = avr_timer_16b_properties;
  
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

    pc->can_receive = avr_timer_16b_can_receive;
    pc->read = avr_timer_16b_read;
    pc->write = avr_timer_16b_write;
    pc->receive = avr_timer_16b_receive;
    pc->is_active = avr_timer_16b_is_active;
    pc->serialize = avr_timer_16b_serialize;

    pc->write_ifr = avr_timer_16b_write_ifr;
    pc->write_imsk = avr_timer_16b_write_imsk;
    pc->read_ifr = avr_timer_16b_read_ifr;
    pc->read_imsk = avr_timer_16b_read_imsk;
    printf("Timer 16b class initiated\n");
}

static const TypeInfo avr_timer_16b_info = {
    .name          = TYPE_AVR_TIMER_16b,
    .parent        = TYPE_AVR_PERIPHERAL,
    .class_init    = avr_timer_16b_class_init,
    .class_size    = sizeof(AVRTimer16bClass),
    .instance_size = sizeof(AVRPeripheralState),
    .instance_init = avr_timer_16b_init
};

static void avr_timer_16b_register_types(void)
{
    type_register_static(&avr_timer_16b_info);
}

type_init(avr_timer_16b_register_types)