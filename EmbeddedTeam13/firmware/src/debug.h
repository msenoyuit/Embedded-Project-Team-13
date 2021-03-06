/* ************************************************************************** */
/** Debug

  @File Name
    debug.h

  @Description
    Contains declarations for standard debugging utility functions.
 */
/* ************************************************************************** */

#ifndef _DEBUG_H    /* Guard against multiple inclusion */
#define _DEBUG_H

#include "system_definitions.h"

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Constants                                                         */
/* ************************************************************************** */
/* ************************************************************************** */

#define DBG_USART_INDEX DRV_USART_INDEX_1

/*
 * dbgLocationType:
 *      These constants should be used in calls to dbgOutputLoc to specify 
 *      where in the code those calls are made.
 */
typedef enum dbgLocation {
    // Within individual tasks:
    DBG_TASK_ENTRY = 0,             // Immediately upon entering a task
    DBG_TASK_BEFORE_LOOP = 1,           // Before task's while(1)
    DBG_TASK_BEFORE_QUEUE_SEND = 2,     // Before sending to a queue
    DBG_TASK_BEFORE_QUEUE_RECEIVE = 3,  // Before receiving from a queue
    DBG_TASK_AFTER_QUEUE_SEND = 4,      // After sending to a queue
    DBG_TASK_AFTER_QUEUE_RECEIVE = 5,   // After receiving from a queue
    // Within ISRs:
    DBG_ISR_ENTRY = 6,                  // Immediately upon entering ISR
    DBG_ISR_EXIT = 7,                   // Immediately before leaving ISR
    DBG_ISR_BEFORE_QUEUE_SEND = 8,      // Before sending to a queue
    DBG_ISR_BEFORE_QUEUE_RECEIVE = 9,   // Before receiving from a queue
    DBG_ISR_AFTER_QUEUE_SEND = 10,       // After sending to a queue
    DBG_ISR_AFTER_QUEUE_RECEIVE = 11,     // After receiving from a queue

    DBG_WIFLY_BEFORE_QUEUE_RECEIVE = 12,
    DBG_WIFLY_AFTER_QUEUE_RECEIVE = 13,
    DBG_WIFLY_AFTER_MSG_SEND_SEMAPHORE_TAKE = 14,
    DBG_WIFLY_AFTER_USART_WRITE = 15,
    DBG_WIFLY_AFTER_MSG_SEND = 16,

    DBG_TASK_BEFORE_MSG_SEND = 17,
    DBG_TASK_AFTER_MSG_SEND = 18,

    DBG_WIFLY_TRANSMIT_CALLBACK_START = 19,
    DBG_WIFLY_TRANSMIT_CALLBACK_END = 20,

    DBG_WIFLY_RECEIVE_CALLBACK_START = 21,
    DBG_WIFLY_RECEIVE_CALLBACK_END = 22,
    DBG_WIFLY_RECEIVE_CALLBACK_MIDDLE = 22,
    DBG_WIFLY_RECEIVE_CALLBACK_EMPTY_BUFFER = 23,
    
    DBG_MOTOR_CONTROL_TASK_BEFORE_QUEUE_RECEIVE = 24,
    DBG_MOTOR_CONTROL_TASK_AFTER_QUEUE_RECEIVE = 25,
     
    DBG_MOTOR_CONTROL_TASK_BEFORE_QUEUE_SEND = 26,
    DBG_MOTOR_CONTROL_TASK_AFTER_QUEUE_SEND = 27,
    

} dbgLocationType;


//add more errors as they become relevant
//DON'T USE ZERO AS AN ERROR CODE
typedef enum dbgError {
    DBG_ERROR_WIFLY_INIT = 1,
    DBG_ERROR_WIFLY_RUN = 2,
    DBG_ERROR_WIFLY_SEND_MSG_TOO_LONG = 3,
    DBG_ERROR_MAIN_TASK_INIT = 4,
    DBG_ERROR_MAIN_TASK_RUN = 5,
    DBG_ERROR_QUEUE_TYPE_WRONG = 6,
    DBG_ERROR_IR_INIT = 7,
    DBG_ERROR_IR_RUN = 8,
    DBG_ERROR_LINE_INIT = 9,
    DBG_ERROR_LINE_RUN = 10,
    DBG_ERROR_COLOR_INIT = 11,
    DBG_ERROR_COLOR_RUN = 12,
    DBG_ERROR_NULL_POINTER = 13,
    DBG_ERROR_DRIVE_CONTROL_INIT = 14,
    DBG_ERROR_TEST = 15,
    DBG_ERROR_MOTOR_CONTROL_INIT = 16,
    DBG_ERROR_ENCODER_ISR = 17,
    DBG_ERROR_MOTOR_CONTROL_RUN = 18,
    DBG_ERROR_WIFLY_WRONG_ROVER_ID_RECIVED = 19,
    DBG_ERROR_WIFLY_WRONG_SEQ_COUNT_RECIVED = 20,
    DBG_ERROR_WIFLY_MESSAGE_TOO_LONG = 21,
    DBG_ERROR_WIFLY_MESSAGE_LONGER_THAN_EXPECTED = 22,
    DBG_ERROR_WIFLY_CHECKSUM_MISSMATCH = 23,
    DBG_ERROR_WIFLY_INVALID_RX_STATE = 24,
    DBG_ERROR_WIFLY_STATE_CHANGE_INVALID = 25,
    DBG_ERROR_ENCODER_INIT = 26,
    DBG_ERROR_DRIVE_CONTROL_INTERNAL_QUEUE_FULL = 27,
    DBG_ERROR_DRIVE_CONTROL_RUN = 28,
} dbgErrorType;
// *****************************************************************************
// *****************************************************************************
// Section: Function Declarations
// *****************************************************************************
// *****************************************************************************

// TODO: More specific function headers should go in debug.c

/*
 * All functions have void returns for now; not sure if it's worth checking
 * outputs of debug functions. If we can't successfully output debug info
 * then there's probably not much we can do, but we can revisit this later.
 */

/*
 * Perform any initialization necessary to use functions below.
 * Should be called once during program initialization
 */
void dbgInit();

/*
 * Output an 8-bit value over 8 PIC i/o lines
 */
void dbgOutputVal(unsigned char outVal);

/*
 * Sends a single character out of the UART
 */
void dbgUARTVal(unsigned char outVal);

/*
 * Sends a value to a different set of PIC i/o lines than those used by 
 * dbgOutputVal. The value should be a constant defined in dbgLocationType
 */
void dbgOutputLoc(unsigned char outVal);

void dbgFatalError(dbgErrorType errorType);

void dbgClrErrorLed();

void dbgSetErrorLed();

#endif /* _DEBUG_H */

/* *****************************************************************************
 End of File
 */
