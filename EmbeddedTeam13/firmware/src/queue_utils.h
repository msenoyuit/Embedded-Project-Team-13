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

typedef enum {
    MESSAGE_COLOR_READING, 
    MESSAGE_DISTANCE_READING,
    MESSAGE_LINE_READING,
    MESSAGE_WIFLY_MESSAGE, 
} MessageType;
    
typedef struct { int red; int green; int blue; int clear; } ColorReading; 
typedef struct { int distance; } DistanceReading;
typedef struct { int line; } LineReading;
typedef struct StandardQueueMessage {
    MessageType type;
    union {
        ColorReading colorReading; /* Color reading */
        DistanceReading distanceReading; /* Distance reading */
        LineReading lineReading;
        WiflyMsg wiflyMessage;
    };
} StandardQueueMessage;

// Wrappers for FreeRTOS queue functions
BaseType_t sendStandardQueueMessageToBack(QueueHandle_t xQueue, StandardQueueMessage * msg, TickType_t xTicksToWait);
BaseType_t sendStandardQueueMessageToBackFromISR(QueueHandle_t xQueue, StandardQueueMessage * msg, BaseType_t *pxHigherPriorityTaskWoken);
BaseType_t standardQueueMessageReceive(QueueHandle_t xQueue, StandardQueueMessage * msg, TickType_t xTicksToWait);

StandardQueueMessage makeColorReading(int reg, int green, int blue, int clear);
int getRed(const StandardQueueMessage * msg);
int getGreen(const StandardQueueMessage * msg);
int getBlue(const StandardQueueMessage * msg);
int getClear(const StandardQueueMessage * msg);

StandardQueueMessage makeDistanceReading(int distance);
int getDistance(const StandardQueueMessage * msg);

StandardQueueMessage makeLineReading(int line);
int getLine(const StandardQueueMessage * msg);

StandardQueueMessage makeWiflyMessage(const char * text);
StandardQueueMessage printfWiflyMessage(const char * fmt, ...);
const char * getWiflyText(const StandardQueueMessage * msg);

#endif	/* QUEUE_UTILS_H */

