#include "queue-utils.h"
#include "debug.h"

static checkMessageType(StandardQueueMessage msg, MessageType type) {
    if (msg.type != type) {
        dbgFatalError(DBG_ERROR_QUEUE_TYPE_WRONG);
    }
}

StandardQueueMessage makeColorReading(int red, int green, int blue) {
    StandardQueueMessage msg = {
        .type = MESSAGE_COLOR_READING,
        .red = red,
        .green = green,
        .blue = blue,
    };
    return msg;
}

int getRed(StandardQueueMessage msg) {
    checkMessageType(msg, MESSAGE_COLOR_READING);
    return msg.red;
}

int getGreen(StandardQueueMessage msg) {
    checkMessageType(msg, MESSAGE_COLOR_READING);
    return msg.green;
}

int getBlue(StandardQueueMessage msg) {
    checkMessageType(msg, MESSAGE_COLOR_READING);
    return msg.blue;
}

int getClear(StandardQueueMessage msg) {
    checkMessageType(msg, MESSAGE_COLOR_READING);
    return msg.clear;
}

StandardQueueMessage makeDistanceReading(int distance) {
    StandardQueueMessage msg = {
        .type = MESSAGE_DISTANCE_READING,
        .distance = distance,
    };
    return msg;
}

int getDistance(StandardQueueMessage msg) {
    checkMessageType(msg, MESSAGE_DISTANCE_READING);
    return msg.distance;
}
