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
    
    /*
     * Port/Pin configuration:
     */
    
    //clear bits for dbgOutputVal
    int mask = 0x00FF;
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_E, mask);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_10);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_5);
    
    //clear pins for dbgOutputLoc
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_1);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_8);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_1);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_6);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_9);
    
    /*
     * USART configuration:
     */
    
    dbgUsartHandle = DRV_USART_Open(DBG_USART_INDEX, DRV_IO_INTENT_WRITE);

}
// *****************************************************************************
/* Function:
    void dbgOutputVal(unsigned char outVal)

  Summary:
    Output an 8-bit value over 8 PIC i/o lines
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Takes a char and outputs it over i/o lines  30-37

  Preconditions:
    None.

  Parameters:
    outVal          - char to but output

  Returns:
    None.

  Example:
    <code>
    // Where inChar, is the char to be output
    unsigned char inChar = 'a';
    dbgOutputVal(inChar);
	</code>

  Remarks:
    Make sure that in the harmoney config that
    the following lines are set GPIO_OUT - low;
    pin #   Board pin #     Pin Id
    5       30              RE7
    4       31              RE6
    3       32              RE5
    100     33              RE4
    99      34              RE3
    98      35              RE2
    94      36              RE1
    93      37              RE0
    70      38              RD10 - extra line
    82      39              RD5  - ground
*/

void dbgOutputVal(unsigned char outVal){
    PORTS_DATA_MASK mask = (PORTS_DATA_MASK)0x00FF;
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_E, mask);
    SYS_PORTS_Set(PORTS_ID_0, PORT_CHANNEL_E, outVal, mask);
}

/*
 * Sends a single character out of the UART
 */
void dbgUARTVal(unsigned char outVal){
    // Write a byte to the UART
    DRV_USART_WriteByte(dbgUsartHandle, outVal);
}


// *****************************************************************************
/* Function:
    void dbgOutputLoc(unsigned char outVal)

  Summary:
    Output an 8-bit value over 8 PIC i/o lines
	<p><b>Implementation:</b> Dynamic</p>

  Description:
    Takes a char and outputs it over i/o lines  46-53

  Preconditions:
    None.

  Parameters:
    outVal          - char to but output

  Returns:
    None.

  Remarks:
    Make sure that in the harmoney config that
    the following lines are set GPIO_OUT - low;
    pin #   Board pin #     Pin Id
    88      46              RF1
    83      47              RD6
    68      48              RD8
    71      49              RD11
    11      50              RG7
    12      51              RG8
    10      52              RG6
    14      53              RG9
*/
/*
 * Sends a value to a different set of PIC i/o lines than those used by 
 * dbgOutputVal. The value should be a constant defined in dbgLocationType
 */
void dbgOutputLoc(unsigned char outVal){
    bool RF1 = (outVal & 0x80);
    bool RD6 = (outVal & 0x40);
    bool RD8 = (outVal & 0x20);
    bool RD11 = (outVal & 0x10);
    bool RG7 = (outVal & 0x08);
    bool RG8 = (outVal & 0x04);
    bool RG6 = (outVal & 0x02);
    bool RG9 = (outVal & 0x01);
    PORTS_DATA_MASK mask_F = (PORTS_DATA_MASK)0x0001;
    PORTS_DATA_MASK mask_D = (PORTS_DATA_MASK)0x0940;
    PORTS_DATA_MASK mask_G = (PORTS_DATA_MASK)0x0430;
    
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_F, mask_F);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_D, mask_D);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_G, mask_G);
    
    
    /*
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_1);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_8);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_6);
    SYS_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_9);
    */
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_1, RF1);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6, RD6);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_8, RD8);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, RD11);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7, RG7);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, RG8);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_6, RG6);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_9, RG9);
}

void dbgFatalError(dbgErrorType errorType)
{
    int messageIndex = 0;
    taskENTER_CRITICAL();
    dbgOutputVal(errorType);
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
