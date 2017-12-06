
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
#define DRIVE_SPEED (MOTOR_MAX_SPEED * 0.75)
#define TURN_SLOW_MULTIPLIER 0.5
#define TURN_FAST_MULTIPLIER 1.0
#define FOLLOW_CORRECT_SLOW_MULTIPLIER 0.5
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

static void updateDriveOutput(void) {
    LinePosition pos = driveControlData.linePos;
    moveCommandType cmd = driveControlData.currentCommand;
    
    bool cmdComplete = false;
    if (cmd == MOVE_FORWARD || cmd == MOVE_BACKWARD) {
        DriveDir dir = cmd == MOVE_FORWARD ? FORWARD : REVERSE;
        driveControlData.startCleared = (driveControlData.startCleared ||
                                         pos != INTERSECTION);
        if (driveControlData.startCleared && pos == INTERSECTION) {
            cmdComplete = true;
        } else if (pos == LINE_TO_RIGHT) {
            drive(dir, CORRECT_TO_RIGHT);
        } else if (pos == LINE_TO_LEFT) {
            drive(dir, CORRECT_TO_LEFT);
        } else {
            drive(dir, STRAIGHT);
        }
    } else if (cmd == TURN_LEFT || cmd == TURN_RIGHT) {
        if (!driveControlData.distAlertSet) {
            // Clear the intersection, then set an alert
            if (pos != INTERSECTION) {
                registerDistanceAlert(600);
                driveControlData.distAlertSet = true;
            }
            
            drive(FORWARD, STRAIGHT);
        } else if (!driveControlData.distAlertReceived) {
            // Follow the line till we get alerted
            if (pos == LINE_TO_RIGHT) {
                drive(FORWARD, CORRECT_TO_RIGHT);
            } else if (pos == LINE_TO_LEFT) {
                drive(FORWARD, CORRECT_TO_LEFT);
            } else {
                drive(FORWARD, STRAIGHT);
            }
        } else {
            // Turn on our point until we center on the next line
            driveControlData.startCleared = (driveControlData.startCleared
                                             || pos == UNKNOWN);
            if (driveControlData.startCleared && (pos == LINE_CENTERED)) {
                cmdComplete = true;
            } else {
                drive(SPIN,
                      (cmd == TURN_LEFT) ? TURN_TO_LEFT : TURN_TO_RIGHT);
            }
        }
    } else {
        stopMotors();
    }
    if (cmdComplete) {
        if (masterControlSendMsgToQ(&driveControlData.currentCommandMsg, 0)
              == errQUEUE_FULL) {
            dbgFatalError(DBG_ERROR_DRIVE_CONTROL_RUN);
        }
        startNextDriveCommand();
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
    if (driveControlData.currentCommand == TURN_LEFT ||
        driveControlData.currentCommand == TURN_RIGHT) {
        // If we just turned we're already positioned at the right distance
        driveControlData.distAlertSet = true;
        driveControlData.distAlertReceived = true;
    } else {
        driveControlData.distAlertSet = false;
        driveControlData.distAlertReceived = false;
    }
        driveControlData.startCleared = false;
    driveControlData.currentCommand =
        getCommand(&driveControlData.currentCommandMsg);
    updateDriveOutput();
}

static void addDriveCommand(StandardQueueMessage command) {
    if (getCommand(&command) == ALL_STOP) {
        StandardQueueMessage toDiscard;
        // empty the queue
        while (xQueueReceive(driveControlData.internalQueue, &toDiscard, 0)
               != errQUEUE_EMPTY) {}
        driveControlData.currentCommand = ALL_STOP;
        updateDriveOutput();
        // Tell master control we've done it
        if (masterControlSendMsgToQ(&command, 0) == errQUEUE_FULL) {
            dbgFatalError(DBG_ERROR_DRIVE_CONTROL_RUN);
        }
        return;
    }

    if (sendStandardQueueMessageToBack(driveControlData.internalQueue,
                                       &command, 0) == errQUEUE_FULL) {
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_INTERNAL_QUEUE_FULL);
    }
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
        msg = makeDriveCommand(TURN_LEFT, 3);
        driveControlSendMsgToQ(&msg, 0);
        msg = makeDriveCommand(TURN_LEFT, 4);
        driveControlSendMsgToQ(&msg, 0);
        msg = makeDriveCommand(MOVE_FORWARD, 5);
        driveControlSendMsgToQ(&msg, 0);        
        initialized = true;
    }
    */
    xQueueReceive(driveControlData.queue, &receivedMessage, portMAX_DELAY);
    piSpecifierType command;
    int commandId;
    switch (receivedMessage.type) {
    case MESSAGE_DRIVE_COMMAND:
        addDriveCommand(receivedMessage);
        break;
    case MESSAGE_LINE_READING:
        handleLineSensorReading(getLine(&receivedMessage));
        break;
    case MESSAGE_MOTOR_SPEEDS_REPORT:
        driveControlData.distAlertReceived = true;
        updateDriveOutput();
        break;
    }
}

 

/*******************************************************************************
 End of File
 */
