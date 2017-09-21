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
#include <stdio.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// Queue related constants
#define MASTER_CONTROL_QUEUE_LEN 10
#define TIMER_FREQUENCY_MS 1000

// Message to be output over UART and IO lines
#define MESSAGE_LENGTH 7
static unsigned char message[MESSAGE_LENGTH] = "Team 13";
static unsigned int messageIndex = 0;

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

MASTER_CONTROL_DATA usartOutputData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void timerCallbackFn(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    
    //Indicate life
    SYS_PORTS_PinToggle(0, PORT_CHANNEL_A, 3);
    /*  TODO: Remove this block
    // Pretend to be the IR sensor
    MasterControlQueueMessage message = {
        MASTER_CONTROL_MSG_IR_READING,
        1234,
        0
    };
    dbgOutputLoc(DBG_ISR_BEFORE_QUEUE_SEND);
    if(usartOutputSendMsgToQFromISR(&message, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_MAIN_TASK_RUN);
    }
    dbgOutputLoc(DBG_ISR_AFTER_QUEUE_SEND);*/
    
    // Start getting the next ADC reading
    PLIB_ADC_SampleAutoStartEnable(ADC_ID_1);
    
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

// TODO: Rename this function to something more descriptive/better
/*******************************************************************************
  Function:
    void APP_ADC_Average ( void )

  Remarks:
    See prototype in usart_output.h.
 */

void APP_ADC_Average (void) {
    int i;
    for (i = 0; i < 16; i++) {
        usartOutputData.ADC_avg += PLIB_ADC_ResultGetByIndex(ADC_ID_1, i);
    }
    usartOutputData.ADC_avg = usartOutputData.ADC_avg / 16;
    usartOutputData.ADC_avg = 625881/(usartOutputData.ADC_avg*200 - 3413);
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    MasterControlQueueMessage message = {
        MASTER_CONTROL_MSG_IR_READING,
        usartOutputData.ADC_avg,
        0
    };
    dbgOutputLoc(DBG_ISR_BEFORE_QUEUE_SEND);
    if(usartOutputSendMsgToQFromISR(&message, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_MAIN_TASK_RUN); // TODO: Change to something better
    }
    dbgOutputLoc(DBG_ISR_AFTER_QUEUE_SEND);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t usartOutputSendMsgToQFromISR(MasterControlQueueMessage * message,
                                        BaseType_t * higherPriorityTaskWoken) {
    return xQueueSendToBackFromISR(usartOutputData.queue, message,
                                   higherPriorityTaskWoken);
    
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

    /* Configure Queue */
    usartOutputData.queue = xQueueCreate(MASTER_CONTROL_QUEUE_LEN,
                                         sizeof(MasterControlQueueMessage));
    if(usartOutputData.queue == NULL) {
        dbgFatalError(DBG_ERROR_MAIN_TASK_INIT);
    }
    
    /* Configure Timer */
    usartOutputData.timer = xTimerCreate("50 ms Timer",
                                  pdMS_TO_TICKS(TIMER_FREQUENCY_MS), pdTRUE,
                                  ( void * ) 0, timerCallbackFn);
    if(usartOutputData.timer == NULL) {
        dbgFatalError(DBG_ERROR_MAIN_TASK_INIT);
    }
    // Start the timer
    if(xTimerStart(usartOutputData.timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_MAIN_TASK_INIT);
    }
    
    // Enable the ADC
    DRV_ADC_Open();
    
    dbgOutputLoc(DBG_TASK_BEFORE_LOOP);
}


/******************************************************************************
  Function:
    void MASTER_CONTROL_Tasks ( void )

  Remarks:
    See prototype in master_control.h.
 */

void MASTER_CONTROL_Tasks ( void ){
    MasterControlQueueMessage receivedMessage;

    dbgOutputLoc(DBG_TASK_BEFORE_QUEUE_RECEIVE);
    xQueueReceive(usartOutputData.queue, &receivedMessage, portMAX_DELAY);
    dbgOutputLoc(DBG_TASK_AFTER_QUEUE_RECEIVE);
    // Handle the message
    WiflyMsg msg;
    switch (receivedMessage.type) {
    case MASTER_CONTROL_MSG_WIFLY:
        // We've received a wifly message, do something about this
        SYS_PORTS_PinToggle(0, PORT_CHANNEL_C, 1);
        ;   
        msg.text[0] = receivedMessage.data1;
        msg.text[1] = '\n';
        msg.text[2] = '\r';
        msg.text[3] = 0;
        wiflySendMsg(&msg, 0);
        break;
    case MASTER_CONTROL_MSG_IR_READING:
        ;
        sprintf(msg.text, "%d\n\r", receivedMessage.data1);
        wiflySendMsg(&msg, 0);
        break;
    }
}

/*******************************************************************************
 End of File
 */
