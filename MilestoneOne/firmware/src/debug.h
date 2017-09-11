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
            DBG_TASK_BEFORE_LOOP,           // Before task's while(1)
            DBG_TASK_BEFORE_QUEUE_SEND,     // Before sending to a queue
            DBG_TASK_BEFORE_QUEUE_RECEIVE,  // Before receiving from a queue
            DBG_TASK_AFTER_QUEUE_SEND,      // After sending to a queue
            DBG_TASK_AFTER_QUEUE_RECEIVE,   // After receiving from a queue
    // Within ISRs:
            DBG_ISR_ENTRY,                  // Immediately upon entering ISR
            DBG_ISR_EXIT,                   // Immediately before leaving ISR
            DBG_ISR_BEFORE_QUEUE_SEND,      // Before sending to a queue
            DBG_ISR_BEFORE_QUEUE_RECEIVE,   // Before receiving from a queue
            DBG_ISR_AFTER_QUEUE_SEND,       // After sending to a queue
            DBG_ISR_AFTER_QUEUE_RECEIVE     // After receiving from a queue
} dbgLocationType;

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

#endif /* _DEBUG_H */

/* *****************************************************************************
 End of File
 */
