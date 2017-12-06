#ifndef _DRIVE_CONTROL_H
#define _DRIVE_CONTROL_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "debug.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "drive_control_public.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

typedef enum {
    LINE_CENTERED,
    LINE_TO_RIGHT,
    LINE_TO_LEFT,
    INTERSECTION,
    UNKNOWN,
} LinePosition;
    

    
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct {
    bool startCleared;
    bool distAlertSet;
    bool distAlertReceived;
    moveCommandType currentCommand;
    StandardQueueMessage currentCommandMsg;
    LinePosition linePos;
    /* Queue handle */
    QueueHandle_t queue;
    // Internal queue for received move commands
    QueueHandle_t internalQueue;
} DRIVE_CONTROL_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void DRIVE_CONTROL_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    DRIVE_CONTROL_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void DRIVE_CONTROL_Initialize ( void );


/*******************************************************************************
  Function:
    void DRIVE_CONTROL_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    DRIVE_CONTROL_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void DRIVE_CONTROL_Tasks( void );


#endif /* _DRIVE_CONTROL_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

