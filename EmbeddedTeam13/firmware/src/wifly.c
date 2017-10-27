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
    StandardQueueMessage toSend;
    /* TODO: Make all of this not terrible */
    
     if (readByte == START_CHAR) {
         wiflyData.rxMsgLen = 0; 
         wiflyData.rxState = ROVER_ID;
         wiflyData.checkSum = 0;
         wiflyData.stateFinished = true;
     } else if (wiflyData.rxState == OUT_OF_STATE){
             wiflyData.rxMsgLen = 0; 
             wiflyData.stateFinished = true;
     } else  if (readByte == STOP_CHAR && wiflyData.rxMsgLen > 0) { 
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
     } else { 
         //44 == ','
         if (readByte == 44) { 
             if(!wiflyData.stateFinished)
             {
                 toSend = makeWiflyMessage("\r\r ERROR state error \r\r");
                 wiflySendMsg(&toSend, portMAX_DELAY);
                 wiflyData.stateFinished = true;
                 wiflyData.rxState = OUT_OF_STATE;
             }
             else{
             wiflyData.rxState++;
             wiflyData.stateFinished = false;
             }
         } 
         else
         {
         switch(wiflyData.rxState)
         {
             case ROVER_ID:
                if(readByte != INT_CHAR_DISTANCE + THIS_ROVER_ID)
                {
                        toSend = makeWiflyMessage("\r\r ERROR wrong rover id \r\r");
                        wiflySendMsg(&toSend, portMAX_DELAY);
                        wiflyData.stateFinished = true;
                        wiflyData.rxState = OUT_OF_STATE;
                        break;
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
                     int givenCount = (wiflyData.rxStateBuff[0] - INT_CHAR_DISTANCE)*100;
                     givenCount += (wiflyData.rxStateBuff[1] - INT_CHAR_DISTANCE)*10;
                     givenCount += (readByte - INT_CHAR_DISTANCE);
                     if(givenCount != wiflyData.rxSequenceCount++)
                     {
                        toSend = printfWiflyMessage("\r\r ERROR Message count mismatch %d\r\r", wiflyData.rxSequenceCount);
                        wiflySendMsg(&toSend, portMAX_DELAY);
                        wiflyData.stateFinished = true;
                        wiflyData.rxState = OUT_OF_STATE;
                        wiflyData.rxSequenceCount = givenCount + 1;
                        break;
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
                     wiflyData.givenMessageLength = (wiflyData.rxStateBuff[0] - INT_CHAR_DISTANCE)*10;
                     wiflyData.givenMessageLength += (readByte - INT_CHAR_DISTANCE);
                     if(wiflyData.givenMessageLength > WIFLY_MAX_MSG_LEN)
                     {
                         toSend = makeWiflyMessage("\r\r ERROR Message Too Long \r\r");
                        wiflySendMsg(&toSend, portMAX_DELAY);
                        wiflyData.stateFinished = true;
                        wiflyData.rxState = OUT_OF_STATE;
                        break;
                     }
                     wiflyData.stateFinished = true;
                 }
                 break;
             case MESSAGE_BODY:
                 if(wiflyData.rxMsgLen >= wiflyData.givenMessageLength)
                 {
                     toSend = makeWiflyMessage("\r\r ERROR Invalid message. given length doesn't match \r\r");
                    wiflySendMsg(&toSend, portMAX_DELAY);
                    wiflyData.stateFinished = true;
                    wiflyData.rxState = OUT_OF_STATE;
                    break;
                 }
                 wiflyData.rxBuff[wiflyData.rxMsgLen++] = readByte;
                 wiflyData.checkSum += readByte;
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
                     int givenCheckSum = (wiflyData.rxStateBuff[0] - INT_CHAR_DISTANCE)*100;
                     givenCheckSum += (wiflyData.rxStateBuff[1] - INT_CHAR_DISTANCE)*10;
                     givenCheckSum += (readByte - INT_CHAR_DISTANCE);
                     wiflyData.checkSum = wiflyData.checkSum % 256;
                     if(wiflyData.checkSum != givenCheckSum)
                     {
                         toSend = makeWiflyMessage("\r\r ERROR Invalid CheckSum \r\r");
                         wiflySendMsg(&toSend, portMAX_DELAY);
                         wiflyData.stateFinished = true;
                         wiflyData.rxState = OUT_OF_STATE;
                         break;
                     }
                     wiflyData.stateFinished = true;
                 }
                 break;
             default:
                 toSend = makeWiflyMessage("\r\r ERROR Invalid input state entered \r\r");
                 wiflySendMsg(&toSend, portMAX_DELAY);
                 wiflyData.stateFinished = true;
                 wiflyData.rxState = OUT_OF_STATE;
                 break;
                 
         }
                 
                 
                 
         }
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
 * Send message over UART
 * 
 * Precondition:
 * There should not be a message currently being sent
 */
void sendMsg(StandardQueueMessage msg, char *checkSum, char *messageLength, char *mCount) {
    unsigned int sentChars = 0;
    //Start Char
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, START_CHAR);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, ',');
    
    //Rover ID
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, THIS_ROVER_ID + INT_CHAR_DISTANCE);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, ',');
    
    //count
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, mCount[0]);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, mCount[1]);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, mCount[2]);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, ',');
    
    
    //message length
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, messageLength[0]);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, messageLength[1]);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, messageLength[2]);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, ',');
    
    //message
    const char * text = getWiflyText(&msg);
    while (text[sentChars] != 0) {
        // Wait till the tx buffer is free and transmit
        xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
        DRV_USART_WriteByte(wiflyData.usartHandle,
                            text[sentChars++]);
    }
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, ',');

    //checksum
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, messageLength[0]);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, messageLength[1]);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, messageLength[2]);
    xSemaphoreTake(wiflyData.txBufferSemaphoreHandle, portMAX_DELAY);
    DRV_USART_WriteByte(wiflyData.usartHandle, ',');
    
    //stop char
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


/* 
 *Generate check sum of message and message length
 * 
 * Precondition:
 * message should be wifly message
 */
void messAnalysis(StandardQueueMessage msg, char *mesCheckSum, char *mesCount, char *mCount)
{
    const char * text = getWiflyText(&msg);
    unsigned int charPlace = 0;
    unsigned int checkSum = 0;
    while (text[charPlace] != 0) {
        checkSum += text[charPlace++];
    }
    checkSum = checkSum % 256;
    mesCheckSum[0] = (int)(checkSum / 100) + INT_CHAR_DISTANCE;
    mesCheckSum[1] = (int)(checkSum%100 / 10) + INT_CHAR_DISTANCE;
    mesCheckSum[2] = (int)(checkSum%10) + INT_CHAR_DISTANCE;
    
    mesCount[0] = (int)(charPlace / 100) + INT_CHAR_DISTANCE;
    mesCount[1] = (int)(charPlace%100 / 10) + INT_CHAR_DISTANCE;
    mesCount[2] = (int)(charPlace%10) + INT_CHAR_DISTANCE;
    
    mCount[0] = (int)(wiflyData.txSequenceCount / 100) + INT_CHAR_DISTANCE;
    mCount[1] = (int)(wiflyData.txSequenceCount%100 / 10) + INT_CHAR_DISTANCE;
    mCount[2] = (int)(wiflyData.txSequenceCount++%10) + INT_CHAR_DISTANCE;
    
    //itoa(mesCount, charPlace, 10);
    //itoa(mCount, wiflyData.rxSequenceCount++, 10);
}


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
    wiflyData.checkSum = 0;
    wiflyData.txSequenceCount = 0;
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
    char mlength[3] = "000";
    char cSum[3] = "000";
    char mCount[3] = "000";
    messAnalysis(toSend, cSum, mlength, mCount);
    sendMsg(toSend, cSum, mlength, mCount);
    dbgOutputLoc(DBG_WIFLY_AFTER_MSG_SEND);
}

 

/*******************************************************************************
 End of File
 */
