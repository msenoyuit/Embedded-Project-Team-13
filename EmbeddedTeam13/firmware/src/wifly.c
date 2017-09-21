// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "wifly.h"
#include "wifly_public.h"
#include "master_control_public.h"
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

    dbgOutputLoc(DBG_WIFLY_RECEIVE_CALLBACK_START);
    
    if (DRV_USART_ReceiverBufferIsEmpty(wiflyData.usartHandle)) {
        dbgOutputLoc(DBG_WIFLY_RECEIVE_CALLBACK_EMPTY_BUFFER);
        return;                 /* Nothing to read */
    }
    
    //char readByte = DRV_USART_ReadByte(wiflyData.usartHandle);
    char readByte = PLIB_USART_ReceiverByteReceive(USART_ID_1);
    dbgOutputVal(readByte);
    dbgOutputLoc(DBG_WIFLY_RECEIVE_CALLBACK_MIDDLE);

     if (readByte == readByte) { 
         wiflyData.rxBuff[wiflyData.rxMsgLen] = 0; // null terminate 
         MasterControlQueueMessage message = { 
             MASTER_CONTROL_MSG_WIFLY, readByte, readByte 
         }; 
         if (usartOutputSendMsgToQFromISR(&message, &higherPriorityTaskWoken) 
             != pdTRUE) { 
             dbgFatalError(DBG_ERROR_WIFLY_RUN); 
         } 
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

    
    dbgOutputLoc(DBG_WIFLY_RECEIVE_CALLBACK_END);
        
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/*
 * Send the next byte from wiflyData.txBuff, until we reach the null character
 */
void wiflyUsartTransmitEventHandler(const SYS_MODULE_INDEX index) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    dbgOutputLoc(DBG_WIFLY_TRANSMIT_CALLBACK_START);
    xSemaphoreGiveFromISR(wiflyData.txBufferSemaphoreHandle,
                          &higherPriorityTaskWoken);
    dbgOutputLoc(DBG_WIFLY_TRANSMIT_CALLBACK_END);

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
    unsigned int sentChars = 0;
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, START_CHAR);
    while (msg.text[sentChars] != 0) {
        // Wait till the tx buffer is free and transmit
        xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
        DRV_USART_WriteByte(wiflyData.usartHandle,
                            msg.text[sentChars++]);
    }
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, START_CHAR);
    dbgOutputLoc(DBG_WIFLY_AFTER_USART_WRITE);
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
