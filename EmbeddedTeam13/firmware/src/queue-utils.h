/* 
 * File:   queue-utils.h
 * Author: nyle
 *
 * Created on September 27, 2017, 11:19 AM
 */

#ifndef QUEUE_UTILS_H
#define	QUEUE_UTILS_H

typedef enum {
    MESSAGE_COLOR_READING, MESSAGE_DISTANCE_READING,
} MessageType;
    
    
typedef struct {
    MessageType type;
    union {
        struct { int red; int green; int blue; int clear; }; /* Color reading */
        struct { int distance; }; /* Distance reading */
    };
} StandardQueueMessage;


StandardQueueMessage makeColorReading(int reg, int green, int blue);
int getRed(StandardQueueMessage msg);
int getGreen(StandardQueueMessage msg);
int getBlue(StandardQueueMessage msg);
int getClear(StandardQueueMessage msg);

StandardQueueMessage makeDistanceReading(int distance);
int getDistance(StandardQueueMessage msg);

#endif	/* QUEUE_UTILS_H */

