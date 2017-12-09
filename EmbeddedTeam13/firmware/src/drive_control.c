
// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "drive_control.h"
#include "motor_control_public.h"
#include "encoders.c"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define DRIVE_CONTROL_QUEUE_LEN 10
#define DRIVE_CONTROL_INTERNAL_QUEUE_LEN 16
#ifdef IS_SCOUT
#define DRIVE_SPEED (MOTOR_MAX_SPEED * 0.95)
#else
#define DRIVE_SPEED (MOTOR_MAX_SPEED * 0.8)
#endif
#define TURN_SLOW_MULTIPLIER 0.5
#define TURN_FAST_MULTIPLIER 1.0
#define FOLLOW_CORRECT_SLOW_MULTIPLIER 0.7
#define FOLLOW_CORRECT_FAST_MULTIPLIER 1.0

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

DRIVE_CONTROL_DATA driveControlData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

static void setMotorSpeeds(float left, float right) {
    MotorSpeeds speeds = {.speeds={left, right}};
    StandardQueueMessage msg = makeMotorSpeeds(speeds);
    if (motorControlSendMsgToQ(&msg, 0) == errQUEUE_FULL) {
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_RUN);
    }
}

typedef enum {
    FORWARD,
    REVERSE,
    SPIN,
} DriveDir;

typedef enum {
    STRAIGHT,
    CORRECT_TO_LEFT,
    CORRECT_TO_RIGHT,
    TURN_TO_LEFT,
    TURN_TO_RIGHT,
} TurnSpec;

static void drive(DriveDir direction, TurnSpec turn) {
    float dirMult = (direction != REVERSE) ? 1.0 : -1.0;
    float leftMult, rightMult;
    switch (turn) {
    case STRAIGHT:
        leftMult = rightMult = 1.0;
        break;
    case CORRECT_TO_LEFT:
        leftMult = FOLLOW_CORRECT_SLOW_MULTIPLIER;
        rightMult = FOLLOW_CORRECT_FAST_MULTIPLIER;
        break;    
    case CORRECT_TO_RIGHT:
        leftMult = FOLLOW_CORRECT_FAST_MULTIPLIER;
        rightMult = FOLLOW_CORRECT_SLOW_MULTIPLIER;
        break;
    case TURN_TO_LEFT:
        leftMult = ((direction == SPIN) ? -1 * TURN_FAST_MULTIPLIER:
                    TURN_SLOW_MULTIPLIER);
        rightMult = TURN_FAST_MULTIPLIER;
        break;
    case TURN_TO_RIGHT:
        leftMult = TURN_FAST_MULTIPLIER;
        rightMult = ((direction == SPIN) ? -1 * TURN_FAST_MULTIPLIER:
                     TURN_SLOW_MULTIPLIER);
        break;
    default:
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_RUN);
    }
    setMotorSpeeds(dirMult * leftMult * DRIVE_SPEED,
                   dirMult * rightMult * DRIVE_SPEED);
}

static void stopMotors() {
    setMotorSpeeds(0, 0);
}

static void startNextDriveCommand(void);

static void notifyCommandCompleted() {
    if (masterControlSendMsgToQ(&driveControlData.currentCommandMsg, 0)
        == errQUEUE_FULL) {
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_RUN);
    }
}

static void updateDriveOutput(void) {
    LinePosition pos = driveControlData.linePos;
    moveCommandType cmd = driveControlData.currentCommand;
    
    if (cmd == MOVE_FORWARD || cmd == MOVE_BACKWARD) {
        // Go forward till we hit an intersection, then set a distance alert
        if (pos == INTERSECTION && !driveControlData.startCleared) {
            registerDistanceAlert(720);
            driveControlData.startCleared = true;
        }

        // Do the actual line following
        DriveDir dir = cmd == MOVE_FORWARD ? FORWARD : REVERSE;
        if (pos == LINE_TO_RIGHT) {
            drive(dir, CORRECT_TO_RIGHT);
        } else if (pos == LINE_TO_LEFT) {
            drive(dir, CORRECT_TO_LEFT);
        } else {
            drive(dir, STRAIGHT);
        }
    } else if (cmd == TURN_LEFT || cmd == TURN_RIGHT) {
        // Turn till the sensor is over all white
        if (pos == UNKNOWN) {
            driveControlData.startCleared = true;
        }
        // Then keep turning till we're centered on the next line
        if (driveControlData.startCleared && (pos == LINE_CENTERED)) {
            notifyCommandCompleted();
            startNextDriveCommand();
            return;
        }

        // Just spin in the direction we're turning
        drive(SPIN, (cmd == TURN_LEFT) ? TURN_TO_LEFT : TURN_TO_RIGHT);
    } else {
        stopMotors();
    }
}

static void executeAllStop(StandardQueueMessage allStopCommand) {
    StandardQueueMessage toDiscard;
    // empty the queue
    while (xQueueReceive(driveControlData.internalQueue, &toDiscard, 0)
           != errQUEUE_EMPTY) {}
    driveControlData.currentCommand = ALL_STOP;
    updateDriveOutput();
    // Tell master control we've done it
    if (masterControlSendMsgToQ(&allStopCommand, 0) == errQUEUE_FULL) {
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_RUN);
    }
}

static void startNextDriveCommand(void) {
    if (uxQueueMessagesWaiting(driveControlData.internalQueue) == 0) {
        driveControlData.currentCommand = ALL_STOP;
        updateDriveOutput();
        return;
    }
 
    xQueueReceive(driveControlData.internalQueue,
                  &driveControlData.currentCommandMsg, 0);

    driveControlData.startCleared = false;

    // If we're the truck, we'll only get told to drive into things if
    // we're trying to pick them up
#ifdef IS_SCOUT
    // If there's something in the way abort
    if (driveControlData.forwardClearCheck < 0 && 
        getCommand(&driveControlData.currentCommandMsg) == MOVE_FORWARD) {
        // Change the command to an all stop
        driveControlData.currentCommandMsg.driveCommand.command = ALL_STOP;
        executeAllStop(driveControlData.currentCommandMsg);
        return;
    }
#endif
    
    driveControlData.currentCommand =
        getCommand(&driveControlData.currentCommandMsg);
    updateDriveOutput();
}

static void addDriveCommand(StandardQueueMessage command) {
    // Execute ALL_STOP immediately
    if (getCommand(&command) == ALL_STOP) {
        executeAllStop(command);
        return;
    }

    if (sendStandardQueueMessageToBack(driveControlData.internalQueue,
                                       &command, 0) == errQUEUE_FULL) {
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_INTERNAL_QUEUE_FULL);
    }
    // If we're not doing anything at the moment, start executing this
    if (driveControlData.currentCommand == ALL_STOP) {
        startNextDriveCommand();
    }
}
    
static LinePosition interpretLineSensorReading(char reading) {
    bool farLeft = (reading & 0x80) != 0;
    bool left = (reading & 0x70) != 0;
    bool right = (reading & 0x0E) != 0;
    bool farRight = (reading & 0x01) != 0;
    
    if (!farLeft && !left && !right && !farRight) {
        return UNKNOWN;
    } else if (!farLeft && left && right && !farRight) {
        return LINE_CENTERED;
    } else if (!farLeft && !left) {
        return LINE_TO_RIGHT;
    } else if (!right && !farRight) {
        return LINE_TO_LEFT;
    } else {
        return INTERSECTION;
    }
}

static void handleLineSensorReading(char reading) {
    driveControlData.linePos = interpretLineSensorReading(reading);
    updateDriveOutput();
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void DRIVE_CONTROL_Initialize ( void )

  Remarks:
    See prototype in drive_control.h.
 */

void DRIVE_CONTROL_Initialize ( void ) {
    driveControlData.queue = xQueueCreate(DRIVE_CONTROL_QUEUE_LEN,
                                          sizeof(StandardQueueMessage));
    if(driveControlData.queue == NULL) {
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_INIT);
    }

    driveControlData.internalQueue =
        xQueueCreate(DRIVE_CONTROL_INTERNAL_QUEUE_LEN,
                     sizeof(StandardQueueMessage));
    if(driveControlData.internalQueue == NULL) {
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_INIT);
    }

    driveControlData.currentCommand = ALL_STOP;
    driveControlData.linePos = UNKNOWN;
    driveControlData.startCleared = false;
    driveControlData.forwardClearCheck = 0;
}

BaseType_t driveControlSendMsgToQFromISR(StandardQueueMessage * message,
                                        BaseType_t * higherPriorityTaskWoken) {
    return sendStandardQueueMessageToBackFromISR(driveControlData.queue,
                                                 message,
                                   higherPriorityTaskWoken);
}

BaseType_t driveControlSendMsgToQ(StandardQueueMessage * message,
                                  TickType_t time) {
    return sendStandardQueueMessageToBack(driveControlData.queue, message,
                                          time);
}

/******************************************************************************
  Function:
    void DRIVE_CONTROL_Tasks ( void )

  Remarks:
    See prototype in drive_control.h.
 */


void DRIVE_CONTROL_Tasks ( void ) {
    StandardQueueMessage receivedMessage;
    StandardQueueMessage toSend;

    /* To hard code a set of directions for testing
    static bool initialized = false;
    if (!initialized) {
        StandardQueueMessage msg = makeDriveCommand(MOVE_FORWARD, 0);
        driveControlSendMsgToQ(&msg, 0);
        msg = makeDriveCommand(MOVE_FORWARD, 1);
        driveControlSendMsgToQ(&msg, 0);
        msg = makeDriveCommand(TURN_LEFT, 2);
        driveControlSendMsgToQ(&msg, 0);
        msg = makeDriveCommand(MOVE_FORWARD, 3);
        driveControlSendMsgToQ(&msg, 0);
        msg = makeDriveCommand(TURN_LEFT, 4);
        driveControlSendMsgToQ(&msg, 0);
        msg = makeDriveCommand(TURN_LEFT, 5);
        driveControlSendMsgToQ(&msg, 0);
        msg = makeDriveCommand(MOVE_FORWARD, 6);
        driveControlSendMsgToQ(&msg, 0);
        initialized = true;
    }
    */
    xQueueReceive(driveControlData.queue, &receivedMessage, portMAX_DELAY);
    switch (receivedMessage.type) {
    case MESSAGE_DRIVE_COMMAND:
        addDriveCommand(receivedMessage);
        break;
    case MESSAGE_LINE_READING:
        handleLineSensorReading(getLine(&receivedMessage));
        break;
    case MESSAGE_MOTOR_SPEEDS_REPORT:
        // This is sent when a distance alert we set has been triggered
        // It indicates that we should stop the current drive forward/backward
        // command
        notifyCommandCompleted();
        startNextDriveCommand();
        break;
    case MESSAGE_DISTANCE_READING:
        ;
        int distance = getDistance(&receivedMessage);
        // These seem to correspond to the range of readings we get if there's
        // something in front
        if (3 < distance && distance <= 15 &&
            driveControlData.forwardClearCheck > -2) {
            driveControlData.forwardClearCheck--;                
        } else if (driveControlData.forwardClearCheck < 2) {
            driveControlData.forwardClearCheck++;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
