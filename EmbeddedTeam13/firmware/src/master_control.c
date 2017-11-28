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
    //lineSensorInit();
 
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

void MASTER_CONTROL_Tasks ( void ){
    StandardQueueMessage receivedMessage;
    StandardQueueMessage toSend;

    standardQueueMessageReceive(masterControlData.queue, &receivedMessage, portMAX_DELAY);
    
    // Handle the message
    switch (receivedMessage.type) {
    case MESSAGE_WIFLY_MESSAGE:
        toSend = printfWiflyMessage("Wifly Echo: %s\n\r", getWiflyText(&receivedMessage));
        wiflySendMsg(&toSend, portMAX_DELAY);
        break;
    case MESSAGE_DISTANCE_READING:
        toSend = printfWiflyMessage("Distance (cm): %d\n\r",
                                    getDistance(&receivedMessage));
        wiflySendMsg(&toSend, portMAX_DELAY);
        break;
    case MESSAGE_LINE_READING:
        toSend = printfWiflyMessage("Line Reading: 0x%x\n\r",
                                    getLine(&receivedMessage));
        wiflySendMsg(&toSend, portMAX_DELAY);
        break;
    case MESSAGE_COLOR_READING:
        toSend = printfWiflyMessage("Color Reading: R %d, G %d, B %d, C %d\n\r",
                                    getRed(&receivedMessage),
                                    getGreen(&receivedMessage),
                                    getBlue(&receivedMessage),
                                    getClear(&receivedMessage));
        wiflySendMsg(&toSend, portMAX_DELAY);
        break;
    case MESSAGE_ENCODER_READING:
        toSend = printfWiflyMessage("%s encoder: %d counts\n\r",
                                    (getEncoderId(&receivedMessage) ==
                                     L_ENCODER ? "Left" : "Right"),
                                    getEncoderCount(&receivedMessage));
        wiflySendMsg(&toSend, portMAX_DELAY);
        break;
    }
}

/*******************************************************************************
 End of File
 */
