/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motor_control.c

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


#include "motor_control.h"
#include "motor_control_public.h"
#include "encoders.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define ENCODER_READ_FREQUENCY_MS 100


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

MOTOR_CONTROL_DATA motorControlData;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

static void encoderReadCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    unsigned int reading = 0;
    if (getLeftEncoderCountISR(&reading, &higherPriorityTaskWoken) != pdTRUE) {
        dbgFatalError(DBG_ERROR_MOTOR_CONTROL_RUN);
    }
    StandardQueueMessage msg = makeEncoderReading(L_ENCODER, reading);
    if(driveControlSendMsgToQFromISR(&msg, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_MOTOR_CONTROL_RUN);
    }

    if (getRightEncoderCountISR(&reading, &higherPriorityTaskWoken) != pdTRUE) {
        dbgFatalError(DBG_ERROR_MOTOR_CONTROL_RUN);
    }
    msg = makeEncoderReading(R_ENCODER, reading);
    if(driveControlSendMsgToQFromISR(&msg, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_MOTOR_CONTROL_RUN);
    }
    
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


BaseType_t motorControlSendMsgToQFromISR(StandardQueueMessage * message,
                                        BaseType_t * higherPriorityTaskWoken) {
    return sendStandardQueueMessageToBackFromISR(motorControlData.queue,
                                                 message,
                                                 higherPriorityTaskWoken);
}
    
BaseType_t motorControlSendMsgToQR(StandardQueueMessage * message,
                                        TickType_t time) {
    return sendStandardQueueMessageToBack(motorControlData.queue, message,
                                          time);
}

struct StandardQueueMessage makeEncoderReading(EncoderId encoder, int counts) {
    StandardQueueMessage msg = {
        .type = MESSAGE_ENCODER_READING,
        .encoderReading.encoder = encoder,
        .encoderReading.counts = counts,
    };
    return msg;
}

EncoderId getEncoderId(const struct StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_ENCODER_READING);
    return msg->encoderReading.encoder;
}

int getEncoderCount(const struct StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_ENCODER_READING);
    return msg->encoderReading.counts;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MOTOR_CONTROL_Initialize ( void )

  Remarks:
    See prototype in motor_control.h.
 */

void MOTOR_CONTROL_Initialize ( void ) {
    motorControlData.queue = xQueueCreate(MOTOR_CONTROL_QUEUE_LEN,
                                         sizeof(StandardQueueMessage)); 
    if(motorControlData.queue == NULL) {
        dbgFatalError(DBG_ERROR_MOTOR_CONTROL_INIT);
    }

    motorControlData.encoderReadTimer =
        xTimerCreate("Encoder read timer",
                     pdMS_TO_TICKS(ENCODER_READ_FREQUENCY_MS), pdTRUE,
                     ( void * ) 0, encoderReadCallback);
    if(motorControlData.encoderReadTimer == NULL) {
        dbgFatalError(DBG_ERROR_MOTOR_CONTROL_INIT);
    }
    // Start the timer
    if(xTimerStart(motorControlData.encoderReadTimer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_MOTOR_CONTROL_INIT);
    }

    encodersInit();
}

/******************************************************************************
  Function:
    void MOTOR_CONTROL_Tasks ( void )

  Remarks:
    See prototype in motor_control.h.
 */

void MOTOR_CONTROL_Tasks ( void ) {
    StandardQueueMessage receivedMessage;
    dbgOutputLoc(DBG_MOTOR_CONTROL_TASK_BEFORE_QUEUE_RECEIVE);
    standardQueueMessageReceive(motorControlData.queue, &receivedMessage,
                                portMAX_DELAY);
    dbgOutputLoc(DBG_MOTOR_CONTROL_TASK_AFTER_QUEUE_RECEIVE);
    
    StandardQueueMessage msg;
    msg.type = MESSAGE_WIFLY_MESSAGE;
    switch(receivedMessage.type) {
    case MESSAGE_LINE_READING:
        
        break;
    }
}

 

/*******************************************************************************
 End of File
 */
