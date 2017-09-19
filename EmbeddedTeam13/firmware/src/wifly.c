/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    wifly.c

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
 ******************************************************************************/

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
 ******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "wifly.h"
#include "debug.h"
#include <string.h>

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

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

WIFLY_DATA wiflyData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

void wiflyUsartReceiveEventHandler(const SYS_MODULE_INDEX index) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    
    char readByte = DRV_USART_ReadByte(wiflyData.usartHandle);
    if (readByte == START_CHAR) wiflyData.rxMsgLen = 0;
    } else if (readByte == STOP_CHAR) {
        wiflyData.rxBuff[wiflyData.rxMsgLen] = 0; // null terminate        
        qQueueSendToBackFromISR(wiflyData.rxMsgQ, msg,
                                &higherPriorityTaskWoken);
    } else wiflyData.rxBuff[wiflyData.rxMsgLen++] = readByte;
        
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/*
 * Send the next byte from wiflyData.txBuff, until we reach the null character
 */
void wiflyUsartSendEventHandler(const SYS_MODULE_INDEX index) {
    if (wiflyData.txBuff[txSentChars] == 0) {
        // End of message
        wiflyData.isSending = false;
    } else {
        // Fill the hardware transmit buffer
        while(!DRV_USART_TransmitBufferIsFull(wiflyData.usartHandle)) {
            DRV_USART_WriteByte(wiflyData.usartHandle,
                                wiflyData.txBuff[txSentChars++]);
        }
    }
}
    

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* 
 * Initiate sending a message over UART
 * 
 * Precondition:
 * There should not be a message currently being sent (!wiflyData.isSending)
 */
void startSendingMessage(char * msg) {
    if (strlen(msg) >= WIFLY_SEND_BUFFER_SIZE) {
        dbgFatalError(DBG_ERROR_WIFLY_SEND_MSG_TOO_LONG);
    }
    strcpy(wiflyData.txBuff, msg);
    wiflyData.txSentChars = 0;
    wiflyData.isSending = true;
    // Start transmitting
    DRV_USART_WriteByte(wiflyData.usartHandle, wiflyData.txBuff[txSentChars++]);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void WIFLY_Initialize ( void )

  Remarks:
    See prototype in wifly.h.
 */

void WIFLY_Initialize ( void ) {
    // Setup USART
    wiflyData.usartHandle = DRV_USART_Open(WIFLY_USART_INDEX,
                                           DRV_IO_INTENT_READWRITE|
                                           DRV_IO_INTENT_NONBLOCKING);
    if (wiflyData.usartHandle == DRV_HANDLE_INVALD) {
        dbgFatalError(DBG_ERROR_WIFLY_INIT);
    }
    // Add callbacks to USART
    DRV_USART_ByteReceiveCallbackSet(WIFLY_USART_INDEX,
                                     wiflyUsartReceiveEventHandler);
    DRV_USART_ByteSendCallbackSet(WIFLY_USART_INDEX,
                                  wiflyUsartSendEventHandler);

    wiflyData.msgsToSend = xQueueCreate(
}


/******************************************************************************
  Function:
    void WIFLY_Tasks ( void )

  Remarks:
    See prototype in wifly.h.
 */

void WIFLY_Tasks ( void ) {
    
 
}

 

/*******************************************************************************
 End of File
 */
