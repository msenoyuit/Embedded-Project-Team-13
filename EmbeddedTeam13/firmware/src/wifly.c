// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "wifly.h"
#include "wifly_public.h"
#include "master_control_public.h"
#include "debug.h"
#include "queue_utils.h"
#include <string.h>

#define USE_START_STOP 1

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
    dbgOutputLoc(DBG_WIFLY_RECEIVE_CALLBACK_MIDDLE);

#if USE_START_STOP
    /* TODO: Make all of this not terrible */
     if (readByte == STOP_CHAR) { 
         wiflyData.rxBuff[wiflyData.rxMsgLen] = 0; // null terminate 
         wiflyData.rxState = OUT_OF_STATE;
         StandardQueueMessage message = makeWiflyMessage(wiflyData.rxBuff);
         /* 
             MASTER_CONTROL_MSG_WIFLY, readByte, readByte 
         }; */
         if (masterControlSendMsgToQFromISR(&message, &higherPriorityTaskWoken) 
             != pdTRUE) { 
             dbgFatalError(DBG_ERROR_WIFLY_RUN); 
         } 
         wiflyData.rxMsgLen = 0;  
     } else if (readByte == START_CHAR) { 
         wiflyData.rxMsgLen = 0; 
         wiflyData.rxState = ROVER_ID;
     } else { 
         //44 == ','
         if (readByte == 44) { 
             if(!wiflyData.stateFinished || wiflyData.rxState == CHECKSUM)
             {
                 dbgFatalError(DBG_ERROR_WIFLY_STATE_CHANGE_INVALID); 
             }
             wiflyData.rxState++;
             wiflyData.stateFinished = false;
         } 
         else
         {
         switch(wiflyData.rxState)
         {
             case ROVER_ID:
                if(readByte != 48 + THIS_ROVER_ID)
                {
                    dbgFatalError(DBG_ERROR_WIFLY_WRONG_ROVER_ID_RECIVED);
                }
                wiflyData.stateFinished = true;
                break;
             case SEQUENCE_COUNT:
                 if(wiflyData.rxBuffLen < 2)
                 {
                     wiflyData.rxStateBuff[wiflyData.rxBuffLen++] = readByte;
                 }
                 else
                 {
                     wiflyData.rxBuffLen = 0;
                     int givenCount = (wiflyData.rxStateBuff[0] - 48)*100;
                     givenCount += (wiflyData.rxStateBuff[1] - 48)*10;
                     givenCount += (readByte - 48);
                     if(givenCount != wiflyData.rxSequenceCount++)
                     {
                         dbgFatalError(DBG_ERROR_WIFLY_WRONG_SEQ_COUNT_RECIVED);
                     }
                     wiflyData.stateFinished = true;
                 }
                 break;
             case MESSAGE_LENGTH:
                 if(wiflyData.rxBuffLen < 1)
                 {
                     wiflyData.rxStateBuff[wiflyData.rxBuffLen++] = readByte;
                 }
                 else
                 {
                     wiflyData.rxBuffLen = 0;
                     wiflyData.givenMessageLength = (wiflyData.rxStateBuff[0] - 48)*10;
                     wiflyData.givenMessageLength += (readByte - 48);
                     if(wiflyData.givenMessageLength > WIFLY_MAX_MSG_LEN)
                     {
                         dbgFatalError(DBG_ERROR_WIFLY_MESSAGE_TOO_LONG);
                     }
                     wiflyData.stateFinished = true;
                 }
                 break;
             case MESSAGE_BODY:
                 if(wiflyData.rxMsgLen >= wiflyData.givenMessageLength)
                 {
                     dbgFatalError(DBG_ERROR_WIFLY_MESSAGE_LONGER_THAN_EXPECTED);
                 }
                 wiflyData.rxBuff[wiflyData.rxMsgLen++] = readByte;
                 if(wiflyData.rxMsgLen == wiflyData.givenMessageLength)
                 {
                     wiflyData.stateFinished = true;
                 }
                 break;
             case CHECKSUM:
                 if(wiflyData.rxBuffLen < 2)
                 {
                     wiflyData.rxStateBuff[wiflyData.rxBuffLen++] = readByte;
                 }
                 else
                 {
                     wiflyData.rxBuffLen = 0;
                     int givenCheckSum = (wiflyData.rxStateBuff[0] - 48)*100;
                     givenCheckSum += (wiflyData.rxStateBuff[1] - 48)*10;
                     givenCheckSum += (readByte - 48);
                     int i = 0;
                     int messCheckSum = 0;
                     while(i < wiflyData.rxMsgLen)
                     {
                         messCheckSum += wiflyData.rxBuff[i++];
                     }
                     messCheckSum = messCheckSum % 256;
                     if(messCheckSum != givenCheckSum)
                     {
                         dbgFatalError(DBG_ERROR_WIFLY_CHECKSUM_MISSMATCH);
                     }
                     wiflyData.stateFinished = true;
                 }
                 break;
             default:
                 dbgFatalError(DBG_ERROR_WIFLY_INVALID_RX_STATE);
         }
                 
                 
                 
         }
     } 
#else
    char tempBuffer[2] = {'a', '\0'};;
    if(readByte == START_CHAR)
    {
        tempBuffer[0] = 'b';
    }
     if(readByte == STOP_CHAR)
    {
        tempBuffer[0] = 'c';
    }
    
    StandardQueueMessage message = makeWiflyMessage(tempBuffer);
    if (masterControlSendMsgToQFromISR(&message, &higherPriorityTaskWoken) 
        != pdTRUE) { 
        dbgFatalError(DBG_ERROR_WIFLY_RUN); 
    } 
#endif   
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
 * Send message over UART
 * 
 * Precondition:
 * There should not be a message currently being sent
 */
void sendMsg(StandardQueueMessage msg) {
    unsigned int sentChars = 0;
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, START_CHAR);
    const char * text = getWiflyText(&msg);
    while (text[sentChars] != 0) {
        // Wait till the tx buffer is free and transmit
        xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
        DRV_USART_WriteByte(wiflyData.usartHandle,
                            text[sentChars++]);
    }
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, STOP_CHAR);
    dbgOutputLoc(DBG_WIFLY_AFTER_USART_WRITE);
}

BaseType_t wiflySendMsg(StandardQueueMessage * message,
                        TickType_t ticksToWait) {
    return sendStandardQueueMessageToBack(wiflyData.toSendQ, message, ticksToWait);
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
    wiflyData.toSendQ = xQueueCreate(WIFLY_QUEUE_LENGTH,
                                     sizeof(StandardQueueMessage));

    // Initialize the semaphore for the tx buffer
    wiflyData.txBufferSemaphoreHandle = xSemaphoreCreateBinary();
    if (wiflyData.txBufferSemaphoreHandle == NULL) {
        dbgFatalError(DBG_ERROR_WIFLY_INIT);
    }
    wiflyData.rxState = OUT_OF_STATE;
    wiflyData.rxSequenceCount = 0;
    wiflyData.givenMessageLength = 0;
    wiflyData.rxBuffLen = 0;
    wiflyData.stateFinished = false;
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
    StandardQueueMessage toSend;
    dbgOutputLoc(DBG_WIFLY_BEFORE_QUEUE_RECEIVE);
    standardQueueMessageReceive(wiflyData.toSendQ, &toSend, portMAX_DELAY);
    dbgOutputLoc(DBG_WIFLY_AFTER_QUEUE_RECEIVE);
    sendMsg(toSend);
    dbgOutputLoc(DBG_WIFLY_AFTER_MSG_SEND);
}

 

/*******************************************************************************
 End of File
 */
