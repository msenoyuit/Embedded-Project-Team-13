#ifndef _MASTER_CONTROL_PUBLIC_H
#define _MASTER_CONTROL_PUBLIC_H

#include "FreeRTOS.h"
#include "queue.h"
#include "queue_utils.h"

typedef enum {
    MASTER_CONTROL_MSG_WIFLY,
    MASTER_CONTROL_MSG_IR_READING,
} MasterControlMessageType;


/* Message type for this tasks queue */
typedef struct {
    MasterControlMessageType type;
    int data1;
    int data2;
} MasterControlQueueMessage;


BaseType_t masterControlSendMsgToQFromISR(StandardQueueMessage * message,
                                        BaseType_t * higherPriorityTaskWoken);

#endif // _MASTER_CONTROL_PUBLIC_H
