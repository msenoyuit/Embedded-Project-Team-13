/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    master_control.c

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

#include "master_control.h"
#include "master_control_public.h"
#include "wifly_public.h"
//#include "ir_sensor.h"
#include "queue_utils.h"
#include "color_sensor.h"
#include "line_sensor.h"
#include <stdio.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// Queue related constants
#define MASTER_CONTROL_QUEUE_LEN 10

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

MASTER_CONTROL_DATA masterControlData;
SEND_DATA_TAG masterDataTag;
int otherLineOn;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t masterControlSendMsgToQFromISR(StandardQueueMessage * message,
                                          BaseType_t * higherPriorityTaskWoken) {
    return sendStandardQueueMessageToBackFromISR(masterControlData.queue, message,
                                   higherPriorityTaskWoken);

}

BaseType_t masterControlSendMsgToQ(StandardQueueMessage * message,
                                   TickType_t time) {
    return sendStandardQueueMessageToBack(masterControlData.queue, message, time);

}

static StandardQueueMessage sendLineReading(char reading) {
    StandardQueueMessage msg = makeLineReading(reading);
    driveControlSendMsgToQ(&msg, portMAX_DELAY);
    msg = printfWiflyMessage("Line follower reading of %02x sent", reading);
    return msg;
}

static char * moveCommandTypeToStr(moveCommandType cmd) {
    switch (cmd) {
    case MOVE_FORWARD:
        return "MOVE_FORWARD";
    case MOVE_BACKWARD:
        return "MOVE_BACKWARD";
    case TURN_LEFT:
        return "TURN_LEFT";
    case TURN_RIGHT:
        return "TURN_RIGHT";
    case ALL_STOP:
        return "ALL_STOP";
    default:
        return "invalid move command!";
    }
}

static StandardQueueMessage sendDriveCommand(moveCommandType cmd) {
    static int id = 0;
    StandardQueueMessage msg = makeDriveCommand(cmd, id);
    driveControlSendMsgToQ(&msg, portMAX_DELAY);
    msg = printfWiflyMessage("Drive command %s with id %d sent",
                             moveCommandTypeToStr(cmd), id);
    id++;
    return msg;
}

static StandardQueueMessage handleMessage(const char * message) {
    if (strcmp(message, "reading centered") == 0) {
        return sendLineReading(0x70 | 0x0E);
    } else if (strcmp(message, "reading right") == 0) {
        return sendLineReading(0x00 | 0x0E);
    } else if (strcmp(message, "reading left") == 0) {
        return sendLineReading(0x70 | 0x00);
    } else if (strcmp(message, "reading intersection") == 0) {
        return sendLineReading(0xF0 | 0x0F);
    } else if (strcmp(message, "reading unknown") == 0) {
        return sendLineReading(0x00 | 0x00);
    } else if (strcmp(message, "go forward") == 0) {
        return sendDriveCommand(MOVE_FORWARD);
    } else if (strcmp(message, "go backward") == 0) {
        return sendDriveCommand(MOVE_BACKWARD);
    } else if (strcmp(message, "go left") == 0) {
        return sendDriveCommand(TURN_LEFT);
    } else if (strcmp(message, "go right") == 0) {
        return sendDriveCommand(TURN_RIGHT);
    } else if (strcmp(message, "stop") == 0) {
        return sendDriveCommand(ALL_STOP);
    } else {
        return printfWiflyMessage("'%s' is not a recognized command", message);
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MASTER_CONTROL_Initialize ( void )

  Remarks:
    See prototype in master_control.h.
 */
void MASTER_CONTROL_Initialize ( void ) {
    /* Initialize debugging utilities */
    dbgInit();
    dbgOutputLoc(DBG_TASK_ENTRY);


    /* Initialize Sensors */
    irSensorInit();
    colorSensorInit();
    lineSensorInit();
    masterDataTag.color = 0;
    masterDataTag.lineOn = 0;
    masterDataTag.distance = 1;
    masterControlData.motorQueueCount = 0;
    masterControlData.direction = 1;
    /* Configure Queue */
    masterControlData.queue = xQueueCreate(MASTER_CONTROL_QUEUE_LEN,
                                         sizeof(StandardQueueMessage));
    if(masterControlData.queue == NULL) {
        dbgFatalError(DBG_ERROR_MAIN_TASK_INIT);
    }

    dbgOutputLoc(DBG_TASK_BEFORE_LOOP);
}


/******************************************************************************
  Function:
    void MASTER_CONTROL_Tasks ( void )

  Remarks:
    See prototype in master_control.h.
 */

StandardQueueMessage handleWiflyCommand(const StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_WIFLY_MESSAGE);
    StandardQueueMessage outMsg;
    StandardQueueMessage toSend;
    int turns = 0;
    const char * text = getWiflyText(msg);

    piCommandType piCommand = text[0] - INT_CHAR_DISTANCE;
    piSpecifierType piSpecifier = text[2] - INT_CHAR_DISTANCE;
    piFlagsType piFlag = COMMAND_RECEIVED;
    outMsg = printfWiflyMessage("%d %d %d", piCommand, piSpecifier, piFlag);

    switch(piCommand)
    {
        case MOVE_COMMAND:
            while(piSpecifier != masterControlData.direction)
            {
                if(piSpecifier == 4)
                {
                    toSend = makeDriveCommand(ALL_STOP, masterControlData.motorQueueCount++);
                    driveControlSendMsgToQ(&toSend, portMAX_DELAY);
                    break;
                }
                    
                turns = masterControlData.direction - piSpecifier;
                if((turns < 0 && turns > -3) || turns == 3)
                {
                    toSend = makeDriveCommand(TURN_RIGHT, masterControlData.motorQueueCount++);
                    driveControlSendMsgToQ(&toSend, portMAX_DELAY);
                    masterControlData.direction++;
                }
                else
                {
                    toSend = makeDriveCommand(TURN_LEFT, masterControlData.motorQueueCount++);
                    driveControlSendMsgToQ(&toSend, portMAX_DELAY);
                    masterControlData.direction--;
                }
                if(masterControlData.direction < 0)
                {
                    masterControlData.direction = 3;
                }
                else
                {
                    masterControlData.direction %= 4;
                }
            }
            toSend = makeDriveCommand(piSpecifier, masterControlData.motorQueueCount++);
            driveControlSendMsgToQ(&toSend, portMAX_DELAY);
            break;
        case READ_COMMAND:
            //currently not used
            break;
        case PICKUP_COMMAND:
            //start pickup sequence
            break;
        case STREAM_START:
            masterDataTag.color |=  (piSpecifier == COLOR_SENSOR);
            masterDataTag.lineOn |=  (piSpecifier == LINE_SENSOR);
            masterDataTag.distance |= (piSpecifier == DISTANCE_SENSOR);
            break;
        case STREAM_STOP:
            masterDataTag.color &=  (piSpecifier != COLOR_SENSOR);
            masterDataTag.lineOn &=  (piSpecifier != LINE_SENSOR);
            masterDataTag.distance &= (piSpecifier != DISTANCE_SENSOR);

            break;
        default:
            outMsg = printfWiflyMessage(text);
    }

    return outMsg;
}


void MASTER_CONTROL_Tasks ( void ){
    StandardQueueMessage receivedMessage;
    StandardQueueMessage toSend;

    standardQueueMessageReceive(masterControlData.queue, &receivedMessage,
                                portMAX_DELAY);

    // Handle the message
    switch (receivedMessage.type) {
    case MESSAGE_WIFLY_MESSAGE:
        toSend = handleWiflyCommand(&receivedMessage);
        wiflySendMsg(&toSend, portMAX_DELAY);
        break;
    case MESSAGE_DISTANCE_READING:
        if(masterDataTag.distance)
        {
            toSend = printfWiflyMessage("Distance (cm): %d",
                                    getDistance(&receivedMessage));
            wiflySendMsg(&toSend, portMAX_DELAY);
        }
        break;
    case MESSAGE_LINE_READING:
        if(masterDataTag.lineOn)
        {
            toSend = printfWiflyMessage("Line Reading: 0x%x",
                                        getLine(&receivedMessage));
            wiflySendMsg(&toSend, portMAX_DELAY);
        }
        break;
    case MESSAGE_COLOR_READING:
        if(masterDataTag.color)
        {
            toSend = printfWiflyMessage("Color Reading: R %d G %d B %d C %d",
                                        getRed(&receivedMessage),
                                        getGreen(&receivedMessage),
                                        getBlue(&receivedMessage),
                                        getClear(&receivedMessage));
            wiflySendMsg(&toSend, portMAX_DELAY);
        }
        break;
    }
}

/*******************************************************************************
 End of File
 */
