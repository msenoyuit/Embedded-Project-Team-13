/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

#include "debug.h"
#include "../framework/driver/usart/drv_usart.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Constants                                                         */
/* ************************************************************************** */
/* ************************************************************************** */

// Message to be output over UART and IO lines
#define ERROR_STRING_LENGTH 8
static unsigned char ERROR_STRING[ERROR_STRING_LENGTH] = "-Error- ";

static DRV_HANDLE dbgUsartHandle = 0; // Handle for the USART driver
                                      // assigned in dbgInit()

/*
 * Perform any initialization necessary to use functions below.
 * Should be called once during program initialization
 * 
 * Configures ports/pins for debug output over i/o lines
 * Configures USART module 1 for debug output
 * 
 */
void dbgInit(){
    dbgClrErrorLed();

    /*
     * Port/Pin configuration:
     */
    
    //clear bits for dbgOutputLoc
    int mask = 0x00FF;
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_E, mask);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_10);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_5);
        
    /*
     * USART configuration:
     */
    
    dbgUsartHandle = DRV_USART_Open(DBG_USART_INDEX, DRV_IO_INTENT_WRITE);

}

void dbgOutputLoc(unsigned char outVal){
    PORTS_DATA_MASK mask = (PORTS_DATA_MASK)0x00FF;
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_E, mask);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_E, outVal, mask);
}

/*
 * Sends a single character out of the UART
 */
void dbgUARTVal(unsigned char outVal){
    taskENTER_CRITICAL();
    // Write a byte to the UART
    DRV_USART_WriteByte(dbgUsartHandle, outVal);
    taskEXIT_CRITICAL();
}

void dbgSetErrorLed() {
    SYS_PORTS_PinSet(0, PORT_CHANNEL_C, 1);
}

void dbgClrErrorLed() {
    SYS_PORTS_PinClear(0, PORT_CHANNEL_C, 1);
}

void dbgFatalError(dbgErrorType errorType) {
    int messageIndex = 0;
    taskENTER_CRITICAL();
    dbgSetErrorLed();
    SYS_PORTS_PinToggle(0, PORT_CHANNEL_A, 3);
    while(1) {
        dbgUARTVal(ERROR_STRING[messageIndex]);
        messageIndex = (messageIndex + 1);
        if(messageIndex >= ERROR_STRING_LENGTH)
        {
            messageIndex = 0;
        }
    }
    taskEXIT_CRITICAL();
}


/* *****************************************************************************
 End of File
 */
