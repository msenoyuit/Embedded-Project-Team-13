/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motor_control.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "motor_control.h"
#include "motor_control_public.h"
#include "encoders.h"
#include "motors.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define MOTOR_CONTROL_UPDATE_FREQUENCY_MS 50
#define MAX_SIGNAL 65535   /* = 2^16 - 1; In timer roll-overs;  */
#define MAX_SPEED 520      /* In counts/s; approximate measured value */
#define SIGNAL_CHANGE_PER_ERROR_PER_S 500
#define SIGNAL_CHANGE_PER_ERROR 8

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

MOTOR_CONTROL_DATA motorControlData;

static float lSpeedDesired, rSpeedDesired;
static TickType_t last_motor_control_update;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

static void motorControlUpdate(float lSpeed, float rSpeed) {
    int newLSignal, newRSignal;
    TickType_t current_time = xTaskGetTickCount();
    int elapsed_ms = ((current_time - last_motor_control_update) *
                      portTICK_PERIOD_MS);
    int signal_change_per_error = (SIGNAL_CHANGE_PER_ERROR_PER_S * elapsed_ms /
                                   1000);
    last_motor_control_update = current_time;
    
    newLSignal = getLeftMotorSignal();
    newLSignal += (int)(lSpeedDesired - lSpeed) * SIGNAL_CHANGE_PER_ERROR;

    newRSignal = getRightMotorSignal();
    newRSignal += (int)(rSpeedDesired - rSpeed) * SIGNAL_CHANGE_PER_ERROR;
    
    if (newLSignal > MAX_SIGNAL) {
        newLSignal = MAX_SIGNAL;
    } else if (newLSignal < -MAX_SIGNAL) {
        newLSignal = -MAX_SIGNAL;
    }

    if (newRSignal > MAX_SIGNAL) {
        newRSignal = MAX_SIGNAL;
    } else if (newRSignal < -MAX_SIGNAL) {
        newRSignal = -MAX_SIGNAL;
    }

    if ((lSpeedDesired - lSpeed) / lSpeedDesired <= 0.05 &&
        (rSpeedDesired - rSpeed) / rSpeedDesired <= 0.05) {
        SYS_PORTS_PinSet(0, PORT_CHANNEL_A, 3);
    } else {
        SYS_PORTS_PinClear(0, PORT_CHANNEL_A, 3);
    }

    setLeftMotorSignal(newLSignal);
    setRightMotorSignal(newRSignal);
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


BaseType_t motorControlSendMsgToQFromISR(StandardQueueMessage * message,
                                        BaseType_t * higherPriorityTaskWoken) {
    return sendStandardQueueMessageToBackFromISR(motorControlData.queue,
                                                 message,
                                                 higherPriorityTaskWoken);
}
    
BaseType_t motorControlSendMsgToQ(StandardQueueMessage * message,
                                        TickType_t time) {
    return sendStandardQueueMessageToBack(motorControlData.queue, message,
                                          time);
}

// Motor Speed Message Functions *******************************************
struct StandardQueueMessage makeMotorSpeeds(float left, float right) {
    StandardQueueMessage msg = {
        .type = MESSAGE_MOTOR_SPEEDS,
        .motorSpeeds.leftSpeed = left,
        .motorSpeeds.rightSpeed = right,
    };
    return msg;
}

struct StandardQueueMessage makeMotorSpeedsReport(float left, float right) {
    StandardQueueMessage msg = {
        .type = MESSAGE_MOTOR_SPEEDS_REPORT,
        .motorSpeeds.leftSpeed = left,
        .motorSpeeds.rightSpeed = right,
    };
    return msg;
}

float getLeftSpeed(struct StandardQueueMessage * msg) {
    checkMessageType2(msg, MESSAGE_MOTOR_SPEEDS, MESSAGE_MOTOR_SPEEDS_REPORT);
    return msg->motorSpeeds.leftSpeed;
}

float getRightSpeed(struct StandardQueueMessage * msg) {
    checkMessageType2(msg, MESSAGE_MOTOR_SPEEDS, MESSAGE_MOTOR_SPEEDS_REPORT);
    return msg->motorSpeeds.rightSpeed;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MOTOR_CONTROL_Initialize ( void )

  Remarks:
    See prototype in motor_control.h.
 */

void MOTOR_CONTROL_Initialize ( void ) {
    motorControlData.queue = xQueueCreate(MOTOR_CONTROL_QUEUE_LEN,
                                         sizeof(StandardQueueMessage)); 
    if(motorControlData.queue == NULL) {
        dbgFatalError(DBG_ERROR_MOTOR_CONTROL_INIT);
    }

    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_C, 14);
    SYS_PORTS_Clear(PORTS_ID_0, PORT_CHANNEL_G, 1);

    // Initialize the output compare modules for PWM output
    DRV_OC0_Enable();
    DRV_OC1_Enable();
    // Start the timer that drives the output compare modules
    DRV_TMR0_Start();

    lSpeedDesired = rSpeedDesired = 0;
    setLeftMotorSignal(0);
    setRightMotorSignal(0);

    last_motor_control_update = xTaskGetTickCount();

    encodersInit();
}

/******************************************************************************
  Function:
    void MOTOR_CONTROL_Tasks ( void )

  Remarks:
    See prototype in motor_control.h.
 */

void MOTOR_CONTROL_Tasks ( void ) {
    StandardQueueMessage receivedMessage;
    
    dbgOutputLoc(DBG_MOTOR_CONTROL_TASK_BEFORE_QUEUE_RECEIVE);
    standardQueueMessageReceive(motorControlData.queue, &receivedMessage,
                                portMAX_DELAY);
    dbgOutputLoc(DBG_MOTOR_CONTROL_TASK_AFTER_QUEUE_RECEIVE);

    switch(receivedMessage.type) {
    case MESSAGE_MOTOR_SPEEDS:
        lSpeedDesired = getLeftSpeed(&receivedMessage);
        rSpeedDesired = getRightSpeed(&receivedMessage);
        break;
    case MESSAGE_MOTOR_SPEEDS_REPORT:
        motorControlUpdate(getLeftSpeed(&receivedMessage),
                           getRightSpeed(&receivedMessage));
        break;
    default:
        ;
        // pass
    }
}

 

/*******************************************************************************
 End of File
 */
