#include "qemu/osdep.h"
#include "hw/ports/avr_uart.h"
#include "qemu/log.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ports/avr_port.h"

static int avr_uart_can_receive(void *opaque)
{
    AVRPeripheralState *usart = opaque;

    if (usart->data_valid || !(usart->csrb & USART_CSRB_RXEN)) 
    {
        return 0;
    }
    return 1;
}

static int avr_uart_is_active(void *opaque, PinID pin)
{
    //printf("UART is active\n");
    AVRPeripheralState *usart = opaque;
    if(pin.PinNum == usart->Pin_RX.PinNum && pin.pPort == usart->Pin_RX.pPort && usart->csrb & USART_CSRB_RXEN)
    {
        return 1;
    }
    else if(pin.PinNum == usart->Pin_TX.PinNum && pin.pPort == usart->Pin_TX.pPort && usart->csrb & USART_CSRB_TXEN)
    {
        return 1;
    }

    printf("UART is disabled\n");
    return 0;
}

static void avr_uart_receive(void *opaque, const uint8_t *buffer, int msgid, PinID pin)
{
    AVRPeripheralState *usart = opaque;
    //assert(size == 1);
    //assert(!usart->data_valid);   // this may lead to the fact that it crashes when new data is received but not retrieved!

    printf("UART Receive\n");
    if(pin.PinNum != usart->Pin_RX.PinNum || pin.pPort != usart->Pin_RX.pPort)
    {
        printf("Error: Trying to receive on a UART pin that is not set as RX\n");
        return;
    }

    usart->data = buffer[0];
    printf("Received UART data: %d\n", buffer[0]);
    usart->data_valid = true;
    usart->csra |= USART_CSRA_RXC;
    if (usart->csrb & USART_CSRB_RXCIE) 
    {
        printf("Interrupt?\n");
        qemu_set_irq(usart->rxc_irq, 1);
    }
    else 
        printf("Interrupt not set!\n");
}

static void update_char_mask(AVRPeripheralState *usart)
{
    uint8_t mode = ((usart->csrc & USART_CSRC_CSZ0) ? 1 : 0) |
        ((usart->csrc & USART_CSRC_CSZ1) ? 2 : 0) |
        ((usart->csrb & USART_CSRB_CSZ2) ? 4 : 0);
    switch (mode) {
    case 0:
        usart->char_mask = 0b11111;
        break;
    case 1:
        usart->char_mask = 0b111111;
        break;
    case 2:
        usart->char_mask = 0b1111111;
        break;
    case 3:
        usart->char_mask = 0b11111111;
        break;
    case 4:
        /* Fallthrough. */
    case 5:
        /* Fallthrough. */
    case 6:
        qemu_log_mask(
            LOG_GUEST_ERROR,
            "%s: Reserved character size 0x%x\n",
            __func__,
            mode);
        break;
    case 7:
        qemu_log_mask(
            LOG_GUEST_ERROR,
            "%s: Nine bit character size not supported (forcing eight)\n",
            __func__);
        usart->char_mask = 0b11111111;
        break;
    default:
        assert(0);
    }
}

static void avr_uart_reset(DeviceState *dev)
{
    AVRPeripheralState *usart = AVR_UART(dev);
    usart->data_valid = false;
    usart->csra = 0b00100000;
    usart->csrb = 0b00000000;
    usart->csrc = 0b00000110;
    usart->brrl = 0;
    usart->brrh = 0;
    update_char_mask(usart);
    qemu_set_irq(usart->rxc_irq, 0);
    qemu_set_irq(usart->txc_irq, 0);
    qemu_set_irq(usart->dre_irq, 0);
}

static uint64_t avr_uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    AVRPeripheralState *usart = opaque;
    uint8_t data;
    assert(size == 1);

    if (!usart->enabled) 
    {
        return 0;
    }

    switch (addr) {
    case USART_DR:
        if (!(usart->csrb & USART_CSRB_RXEN)) {
            /* Receiver disabled, ignore. */
            printf("Receiver disabled... Returning 0 @ read access\n");
            return 0;
        }
        if (usart->data_valid) {
            data = usart->data & usart->char_mask;
            usart->data_valid = false;
        } else {
            data = 0;
        }
        usart->csra &= 0xff ^ USART_CSRA_RXC;
        qemu_set_irq(usart->rxc_irq, 0);
        qemu_chr_fe_accept_input(&usart->chr);
        return data;
    case USART_CSRA:
        return usart->csra;
    case USART_CSRB:
        return usart->csrb;
    case USART_CSRC:
        return usart->csrc;
    case USART_BRRL:
        return usart->brrl;
    case USART_BRRH:
        return usart->brrh;
    default:
        qemu_log_mask(
            LOG_GUEST_ERROR,
            "%s: Bad offset 0x%"HWADDR_PRIx"\n",
            __func__,
            addr);
    }
    return 0;
}

static void avr_uart_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    AVRPeripheralState *usart = opaque;
    uint8_t mask;
    //uint8_t data;
    assert((value & 0xff) == value);
    assert(size == 1);

    if (!usart->enabled) {
        return;
    }

    switch (addr) {
    case USART_DR:
        if (!(usart->csrb & USART_CSRB_TXEN)) {
            /* Transmitter disabled, ignore. */
            return;
        }
        usart->csra |= USART_CSRA_TXC;
        usart->csra |= USART_CSRA_DRE;
        if (usart->csrb & USART_CSRB_TXCIE) {
            qemu_set_irq(usart->txc_irq, 1);
            usart->csra &= 0xff ^ USART_CSRA_TXC;
        }
        if (usart->csrb & USART_CSRB_DREIE) {
            qemu_set_irq(usart->dre_irq, 1);
        }
        //data = value;
        //qemu_chr_fe_write_all(&usart->chr, &data, 1);
        usart->data = value;
        AVRPortState * pPort = (AVRPortState*)usart->father_port[0];
        pPort->send_data(pPort);
        break;
    case USART_CSRA:
        mask = 0b01000011;
        /* Mask read-only bits. */
        value = (value & mask) | (usart->csra & (0xff ^ mask));
        usart->csra = value;
        if (value & USART_CSRA_TXC) {
            usart->csra ^= USART_CSRA_TXC;
            qemu_set_irq(usart->txc_irq, 0);
        }
        if (value & USART_CSRA_MPCM) {
            qemu_log_mask(
                LOG_GUEST_ERROR,
                "%s: MPCM not supported by USART\n",
                __func__);
        }
        break;
    case USART_CSRB:
        mask = 0b11111101;
        /* Mask read-only bits. */
        value = (value & mask) | (usart->csrb & (0xff ^ mask));
        usart->csrb = value;
        if (!(value & USART_CSRB_RXEN)) {
            /* Receiver disabled, flush input buffer. */
            usart->data_valid = false;
        }
        qemu_set_irq(usart->rxc_irq,
            ((value & USART_CSRB_RXCIE) &&
            (usart->csra & USART_CSRA_RXC)) ? 1 : 0);
        qemu_set_irq(usart->txc_irq,
            ((value & USART_CSRB_TXCIE) &&
            (usart->csra & USART_CSRA_TXC)) ? 1 : 0);
        qemu_set_irq(usart->dre_irq,
            ((value & USART_CSRB_DREIE) &&
            (usart->csra & USART_CSRA_DRE)) ? 1 : 0);
        update_char_mask(usart);
        break;
    case USART_CSRC:
        usart->csrc = value;
        if ((value & USART_CSRC_MSEL1) && (value & USART_CSRC_MSEL0)) {
            qemu_log_mask(
                LOG_GUEST_ERROR,
                "%s: SPI mode not supported by USART\n",
                __func__);
        }
        if ((value & USART_CSRC_MSEL1) && !(value & USART_CSRC_MSEL0)) {
            qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad USART mode\n", __func__);
        }
        if (!(value & USART_CSRC_PM1) && (value & USART_CSRC_PM0)) {
            qemu_log_mask(
                LOG_GUEST_ERROR,
                "%s: Bad USART parity mode\n",
                __func__);
        }
        update_char_mask(usart);
        break;
    case USART_BRRL:
        usart->brrl = value;
        break;
    case USART_BRRH:
        usart->brrh = value & 0b00001111;
        break;
    default:
        qemu_log_mask(
            LOG_GUEST_ERROR,
            "%s: Bad offset 0x%"HWADDR_PRIx"\n",
            __func__,
            addr);
    }
}

static uint32_t avr_uart_serialize(void * opaque, PinID pin, uint8_t * pData)
{
    //uint8_t hdr, data;
    AVRPeripheralState *usart = opaque;

    uint8_t pinno = pin.PinNum;

    if(pinno != usart->Pin_TX.PinNum || usart->Pin_TX.pPort != pin.pPort)
    {
        printf("Error: Trying to send over a UART Pin that is not set as TX\n");
        return 0;
    }

    printf("AVR UART SERIALIZE\n");
    uint8_t hdr = pinno << 5;
    hdr |= 2;   //encoding UART...
    pData[0] = hdr;
    pData[1] = usart->data;
    return 2;
}

static void avr_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    AVRPeripheralClass *pc = AVR_PERIPHERAL_CLASS(klass);
    AVRUARTClass * uart = AVR_UART_CLASS(klass);

    dc->reset = avr_uart_reset;
    //dc->props = avr_uart_properties;
    /*dc->realize = avr_peripheral_realize;*/
  
    uart->parent_can_receive = pc->can_receive;
    uart->parent_receive = pc->receive;
    uart->parent_read = pc->read;
    uart->parent_write = pc->write;
    uart->parent_is_active = pc->is_active;
    uart->parent_serialize = pc->serialize;

    pc->can_receive = avr_uart_can_receive;
    pc->read = avr_uart_read;
    pc->write = avr_uart_write;
    pc->receive = avr_uart_receive;
    pc->is_active = avr_uart_is_active;
    pc->serialize = avr_uart_serialize;
}

static void avr_uart_pr(void *opaque, int irq, int level)
{
    AVRPeripheralState *s = AVR_UART(opaque);

    s->enabled = !level;

    if (!s->enabled) {
        avr_uart_reset(DEVICE(s));
    }
}

static const MemoryRegionOps avr_uart_ops = {
    .read = avr_uart_read,
    .write = avr_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {.min_access_size = 1, .max_access_size = 1}
};

static void avr_uart_init(Object *obj)
{
    AVRPeripheralState *s = AVR_PERIPHERAL(obj);
    memory_region_init_io(&s->mmio, obj, &avr_uart_ops, s, TYPE_AVR_UART, 8);

    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->rxc_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->dre_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->txc_irq);
    
    qdev_init_gpio_in(DEVICE(s), avr_uart_pr, 1);
    s->enabled = true;


    printf("AVR UART object init\n");
}

static const TypeInfo avr_uart_info = {
    .name          = TYPE_AVR_UART,
    .parent        = TYPE_AVR_PERIPHERAL,
    .class_init    = avr_uart_class_init,
    .class_size    = sizeof(AVRUARTClass),
    .instance_size = sizeof(AVRPeripheralState),
    .instance_init = avr_uart_init
};

static void avr_uart_register_types(void)
{
    printf("UART rausgeballert\n");
    type_register_static(&avr_uart_info);
}

type_init(avr_uart_register_types)