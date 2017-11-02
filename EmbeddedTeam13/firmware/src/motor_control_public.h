#ifndef _MOTOR_CONTROL_PUBLIC_H
#define _MOTOR_CONTROL_PUBLIC_H

#include "FreeRTOS.h"
#include "queue.h"
#include "queue_utils.h"
#include "motor_data_types.h"

#define MOTOR_MAX_SPEED 520      /* In counts/s; approximate measured value */

struct StandardQueueMessage;

struct StandardQueueMessage makeMotorSpeeds(MotorSpeeds speeds);
struct StandardQueueMessage makeMotorSpeedsReport(MotorSpeeds speeds);
MotorSpeeds getSpeeds(struct StandardQueueMessage * msg);

BaseType_t motorControlSendMsgToQFromISR(struct StandardQueueMessage * message,
                                         BaseType_t * higherPriorityTaskWoken);
BaseType_t motorControlSendMsgToQ(struct StandardQueueMessage * message,
                                  TickType_t time);

#endif // _MOTOR_CONTROL_PUBLIC_H
