/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    usart_output.c

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

#include "usart_output.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// Queue related constants
#define QUEUE_LENGTH 10
#define QUEUE_ITEM_SIZE sizeof(QueueMessage)
#define TIMER_FREQUENCY_MS 50

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

USART_OUTPUT_DATA usartOutputData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void timerCallbackFn(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    QueueMessage message;
    message.id = 0;
    if(xQueueSendToBackFromISR(usartOutputData.queue, &message,
                               &higherPriorityTaskWoken)
       != pdTRUE) {
        // Queue is full, preventing data from being added
        dbgOutputVal('e');
    }
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

BaseType_t usartOutputSendMsgToQFromISR(QueueMessage * message,
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
    void USART_OUTPUT_Initialize ( void )

  Remarks:
    See prototype in usart_output.h.
 */
void USART_OUTPUT_Initialize ( void ) {
    /* Initialize debugging utilities */
    dbgInit();
    unsigned char testChar = 'a';
    dbgOutputVal(testChar);
    dbgOutputLoc(DBG_TASK_BEFORE_QUEUE_RECEIVE);

    /* Configure Queue */
    usartOutputData.queue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if(usartOutputData.queue == NULL) {
        // Queue could not be created
        dbgOutputVal('e');
    }
    
    /* Configure Timer */
    usartOutputData.timer = xTimerCreate("50 ms Timer",
                                  pdMS_TO_TICKS(TIMER_FREQUENCY_MS), pdTRUE,
                                  ( void * ) 0, timerCallbackFn);
    if(usartOutputData.timer == NULL) {
        // Timer could not be created
        dbgOutputVal('e');
    }
    // Start the timer
    if(xTimerStart(usartOutputData.timer, 0) != pdPASS) {
        // Timer could not be set into active state
        dbgOutputVal('e');
    }
}


/******************************************************************************
  Function:
    void USART_OUTPUT_Tasks ( void )

  Remarks:
    See prototype in usart_output.h.
 */

void USART_OUTPUT_Tasks ( void ){
    QueueMessage receivedMessage;
    // Block and wait for a message
    xQueueReceive(usartOutputData.queue, &receivedMessage, portMAX_DELAY);
    // Handle the message
    SYS_PORTS_PinToggle(0, PORT_CHANNEL_A, 3);
    dbgUARTVal('A');
}

 

/*******************************************************************************
 End of File
 */
