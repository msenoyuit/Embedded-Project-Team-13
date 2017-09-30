#ifndef _MASTER_CONTROL_PUBLIC_H
#define _MASTER_CONTROL_PUBLIC_H

#include "FreeRTOS.h"
#include "queue.h"
#include "queue_utils.h"

BaseType_t masterControlSendMsgToQFromISR(StandardQueueMessage * message,
                                        BaseType_t * higherPriorityTaskWoken);

BaseType_t masterControlSendMsgToQ(StandardQueueMessage * message,
                                        TickType_t time);

#endif // _MASTER_CONTROL_PUBLIC_H
