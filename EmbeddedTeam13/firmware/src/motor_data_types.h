#ifndef MOTOR_DATA_TYPES_H
#define MOTOR_DATA_TYPES_H

// Speeds/Counts are stored in {left, right} 2 element arrays
typedef enum {
    LEFT_SIDE = 0,
    RIGHT_SIDE = 1,
} RobotDriveSide;

typedef struct {
    int counts[2];
} EncoderCounts;

typedef struct {
    int signals[2];
} MotorSignals;

typedef struct {
    float speeds[2];
} MotorSpeeds;

#endif // MOTOR_DATA_TYPES_H
