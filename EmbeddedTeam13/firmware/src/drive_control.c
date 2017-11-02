/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    drive_control.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "drive_control.h"
#include "motor_control_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
#define DRIVE_CONTROL_QUEUE_LEN 10
#define DRIVE_CONTROL_INTERNAL_QUEUE_LEN 16
#define FORWARD_DRIVE_SPEED (MOTOR_MAX_SPEED * 0.75)
#define REVERSE_DRIVE_SPEED (-1 * MOTOR_MAX_SPEED * 0.75)
#define TURN_SLOW_MULTIPLIER 0.5
#define TURN_FAST_MULTIPLIER 1.0

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


typedef enum {
    LINE_CENTERED,
    LINE_TO_RIGHT,
    LINE_TO_LEFT,
    INTERSECTION,
    LOST,
} LinePosition;
    
static LinePosition interpretLineSensorReading(char reading) {
    bool farLeft = (reading & 0x80) != 0;
    bool left = (reading & 0x70) != 0;
    bool right = (reading & 0x0E);
    bool farRight = (reading & 0x01) != 0;
    
    if (!farLeft && !left && !right && !farRight) {
        return LOST;
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

static void setMotorSpeeds(float left, float right) {
    MotorSpeeds speeds = {.speeds={left, right}}
    StardardQueueMessage msg = makeMotorSpeeds(speeds);
    if (motorControlSendMsgToQ(&msg, 0) == errQUEUE_FULL) {
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_RUN);
    }
}

static void startNextDriveCommand(void) {
    StandardQueueMessage command;
    if (xQueuePeek(driveControlData.internal_queue, &command, 0)
        == errQUEUE_EMPTY) {
        return;
    }
    switch (getCommand(&command)) {
    case MOVE_FORWARD:
        setMotorSpeeds(FOREWARD_DRIVE_SPEED, FOREWARD_DRIVE_SPEED);
        break;
    case MOVE_BACKWARD:
        setMotorSpeeds(REVERSE_DRIVE_SPEED, REVERSE_DRIVE_SPEED);
        break;
    case TURN_LEFT:
        setMotorSpeeds(FOREWARD_DRIVE_SPEED * TURN_SLOW_MULTIPLIER,
                       FOREWARD_DRIVE_SPEED * TURN_FAST_MULTIPLIER);
        break;
    case TURN_RIGHT:
        break;
        setMotorSpeeds(FOREWARD_DRIVE_SPEED * TURN_FAST_MULTIPLIER,
                       FOREWARD_DRIVE_SPEED * TURN_SLOW_MULTIPLIER);
    default:
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_RUN);
    }
}

static void addDriveCommand(StandardQueueMessage command) {
    if (getCommand(&command) == ALL_STOP) {
        StandardQueueMessage toDiscard;
        // empty the queue
        while (xQueueReceive(driveControlData.internal_queue, &toDiscard, 0)
               != errQUEUE_EMPTY) {}
        // stop the motors
        setMotorSpeeds(0, 0);
        // Tell master control we've done it
        if (masterControlSendMsgToQ(&command, 0) == errQUE_FULL) {
            dbgFatalError(DBG_ERROR_DRIVE_CONTROL_RUN);
        }
        return;
    }

    if (sendStandardQueueMessageToBack(driveControlData.internal_queue,
                                       &command, 0) == errQUEUE_FULL) {
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_INTERNAL_QUEUE_FULL);
    }
    if (uxQueueMessagesWaiting(driveControlData.internal_queue) == 1) {
        startNextDriveCommand();
    }
}

static void updateDrive(LinePosition position) {
    
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

    driveControlData.internal_queue =
        xQueueCreate(DRIVE_CONTROL_INTERNAL_QUEUE_LEN,
                     sizeof(StandardQueueMessage));
    if(driveControlData.internal_queue == NULL) {
        dbgFatalError(DBG_ERROR_DRIVE_CONTROL_INIT);
    }
    
}

BaseType_t driveControlSendMsgToQFromISR(StandardQueueMessage * message,
                                        BaseType_t * higherPriorityTaskWoken) {
    return sendStandardQueueMessageToBackFromISR(driveControlData.queue, message,
                                   higherPriorityTaskWoken);
}

BaseType_t driveControlSendMsgToQ(StandardQueueMessage * message,
                                  TickType_t time) {
    return sendStandardQueueMessageToBack(driveControlData.queue, message, time);
}

/******************************************************************************
  Function:
    void DRIVE_CONTROL_Tasks ( void )

  Remarks:
    See prototype in drive_control.h.
 */

void DRIVE_CONTROL_Tasks ( void ) {
    StandardQueueMessage receivedMessage;

    // TESTING
    StandardQueueMessage msg =
        makeMotorSpeeds((MotorSpeeds){.speeds={300, 300}});
    motorControlSendMsgToQ(&msg, portMAX_DELAY);

    xQueueReceive(driveControlData.queue, &receivedMessage, portMAX_DELAY);

    switch (receivedMessage.type) {
    case MESSAGE_DRIVE_COMMAND:
        addDriveCommand(receivedMessage);
    case MESSAGE_LINE_READING:
        updateDrive(interpretLineSensorReading(getline(&receivedmessage));
        break;
    }
}

 

/*******************************************************************************
 End of File
 */
