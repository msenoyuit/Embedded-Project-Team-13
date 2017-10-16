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

#include <math.h>

#include "motor_control.h"
#include "motor_control_public.h"
#include "encoders.h"


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

#define MOTOR_CONTROL_UPDATE_FREQUENCY_MS 50
#define MAX_SIGNAL 65535   /* = 2^16 - 1; In timer roll-overs;  */
#define SIGNAL_DELTA MAX_SIGNAL/8 /* Amount to increment/decrement signal by */

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

static int lSpeedDesired, rSpeedDesired;

static int getLeftMotorSignal();
static int getRightMotorSignal();
static void setLeftMotorSignal(int signal);
static void setRightMotorSignal(int signal);

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* Implements a simple bang-bang-like speed control. Increases the signal by
 * SIGNAL_DELTA if we're below the desired speed, otherwise decreases the signal
 * by SIGNAL_DELTA.
 */
static void motorControlCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    int lSpeed, rSpeed;
    lSpeed = getLeftEncoderSpeedISR();
    rSpeed = getRightEncoderSpeedISR();

    int newLSignal, newRSignal;
    newLSignal = getLeftMotorSignal();
    if (lSpeed < lSpeedDesired) {
        newLSignal += SIGNAL_DELTA;
    } else {
        newLSignal -= SIGNAL_DELTA;
    }

    newRSignal = getRightMotorSignal();
    if (rSpeed < rSpeedDesired) {
        newRSignal += SIGNAL_DELTA;
    } else {
        newRSignal -= SIGNAL_DELTA;
    }

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

    setLeftMotorSignal(newLSignal);
    setRightMotorSignal(newRSignal);
        
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
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

struct StandardQueueMessage makeEncoderReading(EncoderId encoder, int counts) {
    StandardQueueMessage msg = {
        .type = MESSAGE_ENCODER_READING,
        .encoderReading.encoder = encoder,
        .encoderReading.counts = counts,
    };
    return msg;
}

EncoderId getEncoderId(const struct StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_ENCODER_READING);
    return msg->encoderReading.encoder;
}

int getEncoderCount(const struct StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_ENCODER_READING);
    return msg->encoderReading.counts;
}

struct StandardQueueMessage makeMotorSpeeds(int left, int right) {
    StandardQueueMessage msg = {
        .type = MESSAGE_MOTOR_SPEEDS,
        .motorSpeeds.leftSpeed = left,
        .motorSpeeds.rightSpeed = right,
    };
    return msg;
}

int getLeftSpeed(struct StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_MOTOR_SPEEDS);
    return msg->motorSpeeds.leftSpeed;
}

int getRightSpeed(struct StandardQueueMessage * msg) {
    checkMessageType(msg, MESSAGE_MOTOR_SPEEDS);
    return msg->motorSpeeds.rightSpeed;
}

// Motor Signal get/set*********************************************************
// Should only be directly set/read by the following
static int lSignal, rSignal;

static int getLeftMotorSignal(void) {
    return rSignal;
}

static int getRightMotorSignal(void) {
    return lSignal;
}

static void setLeftMotorAbsSignal(unsigned int signal) {
    DRV_OC0_PulseWidthSet(signal);
}

static void setLeftMotorDir(bool forward) {
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, 14, forward);
}

static void setLeftMotorSignal(int signal) {
    setLeftMotorDir(signal > 0);
    setLeftMotorAbsSignal((unsigned int) abs(signal));
    lSignal = signal;
}

static void setRightMotorAbsSignal(unsigned int signal) {
    DRV_OC1_PulseWidthSet(signal);
}

static void setRightMotorDir(bool forward) {
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, 1, forward);
}

static void setRightMotorSignal(int signal) {
    setRightMotorDir(signal > 0);
    setRightMotorAbsSignal((unsigned int) abs(signal));
    rSignal = signal;
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

    motorControlData.motorControlTimer =
        xTimerCreate("Motor Control Timer",
                     pdMS_TO_TICKS(MOTOR_CONTROL_UPDATE_FREQUENCY_MS), pdTRUE,
                     ( void * ) 0, motorControlCallback);
    if(motorControlData.motorControlTimer == NULL) {
        dbgFatalError(DBG_ERROR_MOTOR_CONTROL_INIT);
    }
    // Start the timer
    if(xTimerStart(motorControlData.motorControlTimer, 0) != pdPASS) {
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
    
    StandardQueueMessage msg;
    msg.type = MESSAGE_WIFLY_MESSAGE;
    switch(receivedMessage.type) {
    case MESSAGE_MOTOR_SPEEDS:
        lSpeedDesired = getLeftSpeed(&msg);
        rSpeedDesired = getRightSpeed(&msg);
        break;
    }
}

 

/*******************************************************************************
 End of File
 */
