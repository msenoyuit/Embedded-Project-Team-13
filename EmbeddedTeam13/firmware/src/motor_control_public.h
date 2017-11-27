#ifndef _MOTOR_CONTROL_PUBLIC_H
#define _MOTOR_CONTROL_PUBLIC_H

#include "FreeRTOS.h"
#include "queue.h"
#include "queue_utils.h"

struct StandardQueueMessage;

/* Encoder value reporting for StandardQueueMessage */
typedef enum {
    L_ENCODER,
    R_ENCODER,
} EncoderId;

typedef struct {
    EncoderId encoder;
    int counts;
} EncoderReading;

struct StandardQueueMessage makeEncoderReading(EncoderId encoder, int counts);
EncoderId getEncoderId(const struct StandardQueueMessage * msg);
int getEncoderCount(const struct StandardQueueMessage * msg);

/* Motor setting for standard queue message */
typedef struct {
    int leftSpeed, rightSpeed;
} MotorSpeeds;

struct StandardQueueMessage makeMotorSpeeds(int left, int right);
int getLeftSpeed(struct StandardQueueMessage * msg);
int getRightSpeed(struct StandardQueueMessage * msg);


BaseType_t motorControlSendMsgToQFromISR(struct StandardQueueMessage * message,
                                         BaseType_t * higherPriorityTaskWoken);
BaseType_t motorControlSendMsgToQ(struct StandardQueueMessage * message,
                                  TickType_t time);

#endif // _MOTOR_CONTROL_PUBLIC_H
