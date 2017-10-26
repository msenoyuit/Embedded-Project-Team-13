#ifndef _MOTOR_CONTROL_PUBLIC_H
#define _MOTOR_CONTROL_PUBLIC_H

#include "FreeRTOS.h"
#include "queue.h"
#include "queue_utils.h"

struct StandardQueueMessage;

typedef enum {
    LEFT,
    RIGHT,
} RobotDriveSide;

/* Motor setting for standard queue message */
typedef struct {    
    float leftSpeed, rightSpeed;
} MotorSpeeds;

struct StandardQueueMessage makeMotorSpeeds(float left, float right);
struct StandardQueueMessage makeMotorSpeedsReport(float left, float right);
float getLeftSpeed(struct StandardQueueMessage * msg);
float getRightSpeed(struct StandardQueueMessage * msg);

BaseType_t motorControlSendMsgToQFromISR(struct StandardQueueMessage * message,
                                         BaseType_t * higherPriorityTaskWoken);
BaseType_t motorControlSendMsgToQ(struct StandardQueueMessage * message,
                                  TickType_t time);

#endif // _MOTOR_CONTROL_PUBLIC_H
