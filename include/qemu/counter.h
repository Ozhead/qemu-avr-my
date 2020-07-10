#ifndef QEMU_COUNTER_H
#define QEMU_COUNTER_H

#include <glib.h>


typedef void QEMUCounterCB(void *opaque);

typedef struct QEMUCounter {
    QEMUCounterCB *cb;
    void *opaque;
    struct QEMUCounter *next;
} QEMUCounter_t;

extern QEMUCounter_t * counter_list;

static inline QEMUCounter_t *counter_new(QEMUCounterCB *cb, void * opaque)
{
    QEMUCounter_t *ts = g_malloc0(sizeof(QEMUCounter_t));
    ts->cb = cb;
    ts->opaque = opaque;

    if(!counter_list)
        counter_list = ts;
    else
    {
        QEMUCounter_t * node = counter_list;
        while(node->next)
            node = node->next;

        node->next = ts;
    }

    //printf("Created new counter...\n");
    return ts;
}

void counter_increment(void);

#endif