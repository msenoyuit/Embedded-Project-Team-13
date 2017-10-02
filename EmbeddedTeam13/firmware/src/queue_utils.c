#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "queue_utils.h"
#include "debug.h"

/*  
 * Wrapper function for xQueueSendToBack to enforce correct type
 */
BaseType_t sendStandardQueueMessageToBack(  QueueHandle_t xQueue, 
                                            StandardQueueMessage * msg, 
                                            TickType_t xTicksToWait) {
    if (!xQueue || !msg) {
        dbgFatalError(DBG_ERROR_NULL_POINTER);
    }
    
    dbgOutputLoc(DBG_TASK_BEFORE_QUEUE_SEND);
    BaseType_t toReturn = xQueueSendToBack(xQueue, msg, xTicksToWait);
    dbgOutputLoc(DBG_TASK_AFTER_QUEUE_SEND);
    
    return toReturn;
}

/*  
 * Wrapper function for xQueueSendToBackFromISR to enforce correct type
 */
BaseType_t sendStandardQueueMessageToBackFromISR(   QueueHandle_t xQueue, 
                                                    StandardQueueMessage * msg, 
                                                    BaseType_t *pxHigherPriorityTaskWoken) {
    if (!xQueue || !msg) {
        dbgFatalError(DBG_ERROR_NULL_POINTER);
    }
    
    dbgOutputLoc(DBG_ISR_BEFORE_QUEUE_SEND);
    BaseType_t toReturn = xQueueSendToBackFromISR(xQueue, msg, pxHigherPriorityTaskWoken);
    dbgOutputLoc(DBG_ISR_AFTER_QUEUE_SEND);
    
    return toReturn;
}

/*
 * Wrapper function for xQueueReceive to enforce correct type
 */
BaseType_t standardQueueMessageReceive( QueueHandle_t xQueue, 
                                        StandardQueueMessage * msg, 
                                        TickType_t xTicksToWait) {
     if (!xQueue || !msg) {
        dbgFatalError(DBG_ERROR_NULL_POINTER);
    }
    
    dbgOutputLoc(DBG_TASK_BEFORE_QUEUE_SEND);
    BaseType_t toReturn = xQueueReceive(xQueue, msg, xTicksToWait);
    dbgOutputLoc(DBG_TASK_BEFORE_QUEUE_SEND);
    
    return toReturn;
}

static void checkMessageType(const StandardQueueMessage * msg,
                             MessageType type) {
    if (!msg) {
        dbgFatalError(DBG_ERROR_NULL_POINTER);
    }
    if (msg->type != type) {
        dbgFatalError(DBG_ERROR_QUEUE_TYPE_WRONG);
    }
}

StandardQueueMessage makeColorReading(int red, int green, int blue, int clear) {
    StandardQueueMessage msg = {
        .type = MESSAGE_COLOR_READING,
        .colorReading.red = red,
        .colorReading.green = green,
        .colorReading.blue = blue,
        .colorReading.clear = clear,
    };
    return msg;
}

int getRed(const StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_COLOR_READING);
    return msg->colorReading.red;
}

int getGreen(const StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_COLOR_READING);
    return msg->colorReading.green;
}

int getBlue(const StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_COLOR_READING);
    return msg->colorReading.blue;
}

int getClear(const StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_COLOR_READING);
    return msg->colorReading.clear;
}

StandardQueueMessage makeDistanceReading(int distance) {
    StandardQueueMessage msg = {
        .type = MESSAGE_DISTANCE_READING,
        .distanceReading.distance = distance,
    };
    return msg;
}

int getDistance(const StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_DISTANCE_READING);
    return msg->distanceReading.distance;
}

StandardQueueMessage makeLineReading(int line) {
    StandardQueueMessage msg = {
        .type = MESSAGE_LINE_READING,
        .lineReading.line = line,
    };
    return msg;
    
}

int getLine(const StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_LINE_READING);
    return msg->lineReading.line;
}

StandardQueueMessage makeWiflyMessage(const char * text) {
    StandardQueueMessage msg = {
        .type = MESSAGE_WIFLY_MESSAGE,
    };
    strncpy(msg.wiflyMessage.text, text, WIFLY_MAX_MSG_LEN);
    
    return msg;
}

StandardQueueMessage printfWiflyMessage(const char * fmt, ...) {
    va_list args;
    va_start(args, fmt);

    StandardQueueMessage msg = {
        .type = MESSAGE_WIFLY_MESSAGE,
    };

    int chars_printed = vsnprintf(msg.wiflyMessage.text, WIFLY_MAX_MSG_LEN, fmt,
                                args);
    // TODO: Handle errors better
    if (chars_printed >= WIFLY_MAX_MSG_LEN) {
        strncpy(msg.wiflyMessage.text, "Message too long!\n\r",
               WIFLY_MAX_MSG_LEN);
    } else if (chars_printed < 0) {
        strncpy(msg.wiflyMessage.text, "vsnprintf error!!\n\r",
                WIFLY_MAX_MSG_LEN);
    }
    
    va_end(args);

    return msg;
}

const char * getWiflyText(const StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_WIFLY_MESSAGE);
    return msg->wiflyMessage.text;
}
