#include "qemu/osdep.h"
#include "hw/ports/adc.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

#define ADCEN (1 << 7)
#define ADLAR (1 << 5)
#define ADIE  (1 << 3)
#define ADIF  (1 << 4)
#define ADSC  (1 << 6)

//#define dprintf(fmt, args...)    fprintf(stderr, fmt, ## args)
#define dprintf(fmt, args...) 

static void adc_convert(void * opaque)
{       
    AVRPeripheralState *p = opaque;
    double val, vref;
    // get the correct input value...
    uint8_t curr_pin = ((p->adcsrb & 0b00001000) << 2) | (p->admux & 0b00011111);
    if(curr_pin < 8)
    {
        val = p->adc_voltages[curr_pin];
    }
    else if(curr_pin >= 32 && curr_pin <= 39)
    {
        val = p->adc_voltages[curr_pin-24];
    }
    else    // todo different conversions!
    {
        printf("Curr_pin = %d\n", curr_pin);
        assert(false);
    }

    uint8_t vref_selected = p->admux >> 6;
    switch(vref_selected)
    {
        case 0:
            printf("AREF is not enabled!");
            assert(false);
        case 1:
            // AVCC which must be ~ VCC
            vref = VCC * 1000;
            break;
        case 2:
            vref = 1100;
            break;
        case 3:
            vref = 2560;
    }

    double x = val * 1000 * 1024 / vref;
    x = x + 0.5;        // Rounding!
    dprintf("ADC calculated %lf\n", x);
    uint16_t final;

    if(x < 0)   // negative voltage
    {
        x = -x;
        final = (uint16_t)x;
        final &= 0b0000001111111111;
        final = (~final + 1) & 0b0000001111111111;
    }
    else
    {
        final = (uint16_t)x;
        final &= 0b0000001111111111;
    }

    if(p->admux & ADLAR)        // left adjust
        final = final >> 6;
    
    p->adc = final;     //only take lower 10 bits!
    p->adcsra |= ADIF;  // set ADIF flag to 1
    dprintf("ADC val = %i on pin %d\n", (int)p->adc, curr_pin);

    p->adcsra &= ~ADSC; // set ADSC flag to 0 to signalize a finished conversion!
    if(p->adcsra & ADIE)
    {
        qemu_set_irq(p->adc_conv_irq, 1);
        p->adcsra &= ~ADIF;  // set it to 0 afterwards
    }
}

static int avr_adc_can_receive(void *opaque)
{
    AVRPeripheralState *p = opaque;

    // if ADC is enabled, it generally CAN receive something...
    if(p->adcsra & ADCEN)
	    return 9;       // it is supposed to be a float! 8+1

    return 0;
}

static int avr_adc_is_active(void *opaque, PinID pin)
{
    AVRPeripheralState *p = opaque;

    uint8_t pinno = pin.PinNum;

    //printf("%d == %d", pinno, (p->admux & 0b00011111));

    if(p->adcsra & ADCEN)
    {
        // TODO: Add further possibilites from datasheet!
        if((p->admux & 0b00011111) == pinno)
        {
            dprintf("ADC Active = 1\n");
            return 1;
        }
    }
    dprintf("ADC Active = 0\n");
    return 0;
}

static void avr_adc_receive(void *opaque, const uint8_t *buffer, int msgid, PinID pin)
{
    AVRPeripheralState *p = opaque;
	//printf("Calling avr_adc_receive %d\n", msgid);
    assert(msgid == 8);  //TODO
    // float convert to int in ADC!
    double val;
    memcpy(&val, buffer, sizeof(double));

    dprintf("Recv V = %f V on pin %d\n", val, pin.PinNum);


    uint8_t pinno = pin.PinNum;
    if(pin.pPort == p->ADC_Port2)
        pinno += NUM_PINS;

    if(pin.pPort == p->ADC_Port1 || pin.pPort == p->ADC_Port2)
        p->adc_voltages[pinno] = val;

    /*if(avr_adc_is_active(opaque, pinno))
    {
        printf("We should be setting it now correctly!");
        adc_convert(opaque);
    }*/
}

static void avr_adc_reset(DeviceState *dev)
{
	//AVRPeripheralState *port = AVR_PERIPHERAL(dev);
    AVRPeripheralState * adc = AVR_ADC(dev);
    qemu_set_irq(adc->adc_conv_irq, 0);
}

static uint64_t avr_adc_read(void *opaque, hwaddr addr, unsigned int size)
{
	AVRPeripheralState *p = opaque;

    switch(addr)
    {
        case 2:
            return p->adcsra;
        case 3:
            return p->adcsrb;
        case 4:
            return p->admux;
        case 0: //ADCL
            dprintf("Reading ADCL: %i -> %i\n", p->adc, p->adc & 0x00FF);
            return p->adc & 0x00FF;     // 8 bits
        case 1: //ADCH
            dprintf("Reading ADCL: %i -> %i\n", p->adc, ((p->adc & 0x0300) >> 8));
            return ((p->adc & 0x0300) >> 8);    //bits 8 & 9
        default:
            assert(false);
    }
	
	return 0;
}

static void avr_adc_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    AVRPeripheralState *p = opaque;
	//printf("addr = %i\n", (int)addr);

    switch(addr)
    {
        case 2:
            p->adcsra = value;
            if(p->adcsra & ADSC)
                adc_convert(p);
            break;
        case 3:
            p->adcsrb = value;
            break;
        case 4:
            p->admux = value;
            break;
        default:
            printf("NOT IMPLEMENTED ADC %i\n", (int)addr);
            assert(false);
    }
}


//static Property avr_adc_properties[] = {
    //DEFINE_PROP_CHR("chardev", AVRPeripheralState, chr),
    //DEFINE_PROP_END_OF_LIST(),
//};

static void avr_adc_pr(void *opaque, int irq, int level)
{
    AVRPeripheralState *s = AVR_ADC(opaque);

    s->enabled = !level;

    if (!s->enabled) {
        avr_adc_reset(DEVICE(s));
    }
}

//static void avr_adc_init(Object *obj)
//{
    /*AVRPeripheralState *s = AVR_PERIPHERAL(obj);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->rxc_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dre_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->txc_irq);
    memory_region_init_io(&s->mmio, obj, &avr_peripheral_ops, s, TYPE_AVR_PERIPHERAL, 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in(DEVICE(s), avr_peripheral_pr, 1);
    //s->enabled = true;*/
//}

//static void avr_peripheral_realize(DeviceState *dev, Error **errp)
//{
    /*AVRPeripheralState *s = AVR_PERIPHERAL(dev);
    qemu_chr_fe_set_handlers(&s->chr, avr_peripheral_can_receive,
                             avr_peripheral_receive, NULL, NULL,
                             s, NULL, true);
    avr_peripheral_reset(dev);*/
//}

/* ADC is pure input so it can't send anything */
static uint32_t avr_adc_serialize(void * opaque, PinID pin, uint8_t * pData)
{
    return 0;
}

static void avr_adc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AVRPeripheralClass *pc = AVR_PERIPHERAL_CLASS(klass);
    AVRADCClass * adc = AVR_ADC_CLASS(klass);

    dc->reset = avr_adc_reset; 
    /*dc->props = avr_peripheral_properties;
    dc->realize = avr_peripheral_realize;*/
  
    adc->parent_can_receive = pc->can_receive;
    adc->parent_receive = pc->receive;
    adc->parent_read = pc->read;
    adc->parent_write = pc->write;
    adc->parent_is_active = pc->is_active;
    adc->parent_serialize = pc->serialize;

    pc->can_receive = avr_adc_can_receive;
    pc->read = avr_adc_read;
    pc->write = avr_adc_write;
    pc->receive = avr_adc_receive;
    pc->is_active = avr_adc_is_active;
    pc->serialize = avr_adc_serialize;

    //printf("ADC class initiated\n");
}

static const MemoryRegionOps avr_adc_ops = {
    .read = avr_adc_read,
    .write = avr_adc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.min_access_size = 1, .max_access_size = 1}
};

static void avr_adc_init(Object *obj)
{
    AVRPeripheralState *s = AVR_PERIPHERAL(obj);
    memory_region_init_io(&s->mmio, obj, &avr_adc_ops, s, TYPE_AVR_ADC, 8);

    /*sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->rxc_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dre_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->txc_irq);
    memory_region_init_io(&s->mmio, obj, &avr_peripheral_ops, s, TYPE_AVR_PERIPHERAL, 8);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    qdev_init_gpio_in(DEVICE(s), avr_peripheral_pr, 1);*/
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->adc_conv_irq);
    qdev_init_gpio_in(DEVICE(s), avr_adc_pr, 1);
    s->enabled = true;
}

static const TypeInfo avr_adc_info = {
    .name          = TYPE_AVR_ADC,
    .parent        = TYPE_AVR_PERIPHERAL,
    .class_init    = avr_adc_class_init,
    .class_size    = sizeof(AVRADCClass),
    .instance_size = sizeof(AVRPeripheralState),
    .instance_init = avr_adc_init
};

static void avr_adc_register_types(void)
{
    type_register_static(&avr_adc_info);
}

type_init(avr_adc_register_types)