/* 
 * File:   queue-utils.h
 * Author: nyle
 *
 * Created on September 27, 2017, 11:19 AM
 */

#ifndef QUEUE_UTILS_H
#define	QUEUE_UTILS_H
#include "FreeRTOS.h"
#include "queue.h"
#include "wifly_public.h"
#include "motor_data_types.h"

typedef enum {
    MESSAGE_COLOR_READING, 
    MESSAGE_DISTANCE_READING,
    MESSAGE_LINE_READING,
    MESSAGE_WIFLY_MESSAGE,
    // Both of the following correspond to a MotorSpeeds type. The first is for
    // setting the desired speeds. The second is sending the current motor
    // speed values
    MESSAGE_MOTOR_SPEEDS, 
    MESSAGE_MOTOR_SPEEDS_REPORT,
    MESSAGE_DRIVE_COMMAND,
} MessageType;

typedef enum moveCommand{
    MOVE_FOWARD = 0,
    MOVE_BACKWARD = 1,
    TURN_LEFT = 2,
    TURN_RIGHT = 3,
    ALL_STOP = 4,
} moveCommandType;
    
typedef struct {moveCommandType command; int messageId;} DriveCommand;
typedef struct { int red; int green; int blue; int clear; } ColorReading; 
typedef struct { int distance; } DistanceReading;
typedef struct { int line; } LineReading;
typedef struct StandardQueueMessage {
    MessageType type;
    union {
        ColorReading colorReading;
        DistanceReading distanceReading;
        LineReading lineReading;
        WiflyMsg wiflyMessage;
        MotorSpeeds motorSpeeds;
        DriveCommand driveCommand;
    };
} StandardQueueMessage;

// Wrappers for FreeRTOS queue functions
BaseType_t sendStandardQueueMessageToBack(QueueHandle_t xQueue,
                                          StandardQueueMessage * msg,
                                          TickType_t xTicksToWait);
BaseType_t
sendStandardQueueMessageToBackFromISR(QueueHandle_t xQueue,
                                      StandardQueueMessage * msg,
                                      BaseType_t *pxHigherPriorityTaskWoken);
BaseType_t standardQueueMessageReceive(QueueHandle_t xQueue,
                                       StandardQueueMessage * msg,
                                       TickType_t xTicksToWait);

/* Check given message before pulling out information */
void checkMessageType(const StandardQueueMessage * msg, MessageType type);
void checkMessageType2(const StandardQueueMessage * msg, MessageType type1,
                       MessageType type2);

StandardQueueMessage makeColorReading(int reg, int green, int blue, int clear);
int getRed(const StandardQueueMessage * msg);
int getGreen(const StandardQueueMessage * msg);
int getBlue(const StandardQueueMessage * msg);
int getClear(const StandardQueueMessage * msg);

StandardQueueMessage makeDistanceReading(int distance);
int getDistance(const StandardQueueMessage * msg);

StandardQueueMessage makeLineReading(int line);
int getLine(const StandardQueueMessage * msg);

StandardQueueMessage makeDriveCommand(moveCommandType command, int messageId);
moveCommandType getCommand(const StandardQueueMessage * msg);
int getMessageId(const StandardQueueMessage * msg);

StandardQueueMessage makeWiflyMessage(const char * text);
StandardQueueMessage printfWiflyMessage(const char * fmt, ...);
const char * getWiflyText(const StandardQueueMessage * msg);

#endif	/* QUEUE_UTILS_H */

