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
            DBG_ISR_AFTER_QUEUE_RECEIVE = 11     // After receiving from a queue
} dbgLocationType;


//add more errors as they become relevant
//DON'T USE ZERO AS AN ERROR CODE
typedef enum dbgError {
    TEST_ERROR = 15
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

#endif /* _DEBUG_H */

/* *****************************************************************************
 End of File
 */
