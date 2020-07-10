#include "qemu/counter.h"

QEMUCounter_t * counter_list;

void counter_increment(void)
{
    QEMUCounter_t * node = counter_list;
    if(node)
    {
        node->cb(node->opaque);
        node = node->next;
    }
}