#ifndef _DRIVE_CONTROL_PUBLIC_H
#define _DRIVE_CONTROL_PUBLIC_H

#include "FreeRTOS.h"
#include "queue.h"
#include "motor_data_types.h"
#include "queue_utils.h"

struct StandardQueueMessage;

BaseType_t driveControlSendMsgToQFromISR(StandardQueueMessage * message,
                                         BaseType_t * higherPriorityTaskWoken);

BaseType_t driveControlSendMsgToQ(StandardQueueMessage * message, TickType_t time);


#endif

