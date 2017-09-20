#ifndef _MASTER_CONTROL_PUBLIC_H
#define _MASTER_CONTROL_PUBLIC_H

#include "FreeRTOS.h"
#include "queue.h"

#include "master_control.h"

BaseType_t usartOutputSendMsgToQFromISR(QueueMessage * message,
                                        BaseType_t * higherPriorityTaskWoken);

#endif // _MASTER_CONTROL_PUBLIC_H
