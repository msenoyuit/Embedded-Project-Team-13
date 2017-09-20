// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "wifly.h"
#include "wifly_public.h"
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

/*
 * Callback function to handle characters received over UART
 * 
 * When a full message has been received (starting at a start character, 
 */
void wiflyUsartReceiveEventHandler(const SYS_MODULE_INDEX index) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    
    char readByte = DRV_USART_ReadByte(wiflyData.usartHandle);
    if (readByte == STOP_CHAR) {
        wiflyData.rxBuff[wiflyData.rxMsgLen] = 0; // null terminate
        // TODO: pull out the message into an appropriate type and send it off
        //       to whatever queue it should go to
        /* if (xQueueSendToBackFromISR(wiflyData.rxMsgQ, msg, */
        /*                             &higherPriorityTaskWoken) != pdTRUE) { */
        /*     dbgFatalError(DBG_ERROR_WIFLY_RUN); */
        /* } */
        wiflyData.rxMsgLen = 0;
    } else if (readByte == START_CHAR) {
        wiflyData.rxMsgLen = 0;
    } else {
        if (wiflyData.rxMsgLen >= WIFLY_MAX_MSG_LEN) {
            // We've overflowed; start writing over the begining
            // TODO: Make this fail non-silently, without halting the system
            //       either
            wiflyData.rxMsgLen = 0;
        }
        wiflyData.rxBuff[wiflyData.rxMsgLen++] = readByte;
    }
        
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/*
 * Send the next byte from wiflyData.txBuff, until we reach the null character
 */
void wiflyUsartTransmitEventHandler(const SYS_MODULE_INDEX index) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    
    if (wiflyData.txBuff[wiflyData.txSentChars] == 0) {
        // End of message; give back the tx buffer
        xSemaphoreGiveFromISR(wiflyData.txBufferSemaphoreHandle,
                              &higherPriorityTaskWoken);
    } else {
        // Fill the hardware transmit buffer
        while(!DRV_USART_TransmitBufferIsFull(wiflyData.usartHandle)) {
            DRV_USART_WriteByte(wiflyData.usartHandle,
                                wiflyData.txBuff[wiflyData.txSentChars++]);
        }
    }

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
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
 * There should not be a message currently being sent
 */
void startMsgSend(WiflyMsg msg) {
    // Obtain tx uart
    xSemaphoreTake(pdTRUE, portMAX_DELAY);
    dbgOutputLoc(DBG_WIFLY_AFTER_MSG_SEND_SEMAPHORE_TAKE);
    // Copy the string into the buffer
    strcpy(wiflyData.txBuff, msg.text);
    wiflyData.txSentChars = 0;
    // Start transmitting
    DRV_USART_WriteByte(wiflyData.usartHandle,
                        wiflyData.txBuff[wiflyData.txSentChars++]);
    dbgOutputLoc(DBG_WIFLY_AFTER_USART_WRITE);
    // Once this byte gets succesfully sent, the wiflyUsartTransmitEventHandler
    // will take over sending the rest
}

BaseType_t wiflySendMsg(WiflyMsg * message, TickType_t ticksToWait) {
    return xQueueSendToBack(wiflyData.toSendQ, message, ticksToWait);
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
    if (wiflyData.usartHandle == DRV_HANDLE_INVALID) {
        dbgFatalError(DBG_ERROR_WIFLY_INIT);
    }
    // Add callbacks to USART
    DRV_USART_ByteReceiveCallbackSet(WIFLY_USART_INDEX,
                                     wiflyUsartReceiveEventHandler);
    DRV_USART_ByteTransmitCallbackSet(WIFLY_USART_INDEX,
                                      wiflyUsartTransmitEventHandler);

    // Queue of things 
    wiflyData.toSendQ = xQueueCreate(WIFLY_QUEUE_LENGTH, sizeof(WiflyMsg));

    // Initialize the semaphore for the tx buffer
    wiflyData.txBufferSemaphoreHandle = xSemaphoreCreateBinary();
    if (wiflyData.txBufferSemaphoreHandle == NULL) {
        dbgFatalError(DBG_ERROR_WIFLY_INIT);
    }
    // Initial 'giving' to make it available
    xSemaphoreGive(wiflyData.txBufferSemaphoreHandle);
}


/******************************************************************************
  Function:
    void WIFLY_Tasks ( void )

  Remarks:
    See prototype in wifly.h.
 */

void WIFLY_Tasks ( void ) {
    WiflyMsg toSend;
    dbgOutputLoc(DBG_WIFLY_BEFORE_QUEUE_RECEIVE);
    xQueueReceive(wiflyData.toSendQ, &toSend, portMAX_DELAY);
    dbgOutputLoc(DBG_WIFLY_AFTER_QUEUE_RECEIVE);
    startMsgSend(toSend);
    dbgOutputLoc(DBG_WIFLY_AFTER_MSG_SEND);
}

 

/*******************************************************************************
 End of File
 */
