#ifndef _MOTOR_CONTROL_PUBLIC_H
#define _MOTOR_CONTROL_PUBLIC_H

#include "FreeRTOS.h"
#include "queue.h"
#include "queue_utils.h"

BaseType_t motorControlSendMsgToQFromISR(StandardQueueMessage * message,
                                        BaseType_t * higherPriorityTaskWoken);

BaseType_t motorControlSendMsgToQ(StandardQueueMessage * message,
                                        TickType_t time);
#endif // _MOTOR_CONTROL_PUBLIC_H