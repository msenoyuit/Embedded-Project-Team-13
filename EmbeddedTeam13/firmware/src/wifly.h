#ifndef _WIFLY_H
#define _WIFLY_H

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
#include "../framework/driver/usart/drv_usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "wifly_public.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

#define WIFLY_USART_INDEX DRV_USART_INDEX_0
#define WIFLY_QUEUE_LENGTH 10
//rover 0 is the scout, rover 1 is the truck
#define THIS_ROVER_ID 0
#define INT_CHAR_DISTANCE 48
#define COMMA_UART 43
const char START_CHAR = 43;
const char STOP_CHAR = 45;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef enum recMesState{
    OUT_OF_STATE = 0,
    ROVER_ID = 1,
    SEQUENCE_COUNT = 2,
    MESSAGE_LENGTH = 3,
    MESSAGE_BODY = 4,
    CHECKSUM = 5,
} rxStateType;

typedef enum piCommand{
    MOVE_COMMAND = 0, //direction
    READ_COMMAND = 1,   //direction
    PICKUP_COMMAND = 2, //direction
    STREAM_START = 3,//sensor
    STREAM_STOP = 4, //sensor
} piCommandType;

typedef enum piSpecifier{
    NORTH_MOVE = 0,
    EAST_MOVE = 1,
    SOUTH_MOVE = 2,
    WEST_MOVE = 3,
    LINE_SENSOR = 4,
    COLOR_SENSOR = 5,
    DISTANCE_SENSOR = 6,    
}piSpecifierType;

typedef enum piFlags {
    COMMAND_RECEIVED = 0,
    COMMAND_FINISHED = 1,
    EVENT_ALERT = 2,
}piFlagsType;

//receive format - "command specifier"
//send format - "flag command data"
//flag - RECIVED, FINISHED, EVENT
typedef struct {
    SYS_MODULE_OBJ usartHandle;
    char rxBuff[WIFLY_MAX_MSG_LEN];
    unsigned int rxMsgLen;
    QueueHandle_t rxMsgQ;

    SemaphoreHandle_t txBufferSemaphoreHandle;
    QueueHandle_t toSendQ;
    uint32_t sendSequenceCount;
    uint8_t txSequenceCount;
    
    char rxStateBuff[WIFLY_MAX_MSG_LEN];
    unsigned int rxBuffLen;
    rxStateType rxState;
    uint8_t rxSequenceCount;
    unsigned int givenMessageLength;
    bool stateFinished;
    uint8_t checkSum;
    
    
    
} WIFLY_DATA;


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
    void WIFLY_Initialize ( void )

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
    WIFLY_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void WIFLY_Initialize ( void );


//void messAnalysis (StandardQueueMessage msg, char *mesCheckSum, char *mesCount, char *mCount);

/*******************************************************************************
  Function:
    void WIFLY_Tasks ( void )

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
    WIFLY_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void WIFLY_Tasks( void );


#endif /* _WIFLY_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

