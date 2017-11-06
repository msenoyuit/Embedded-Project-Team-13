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

static MotorSpeeds speedsDesired = {.speeds={0,0}};
static TickType_t last_motor_control_update;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

static void motorControlUpdate(MotorSpeeds speeds) {
    MotorSignals newSignals;
    
    TickType_t current_time = xTaskGetTickCount();
    int elapsed_ms = ((current_time - last_motor_control_update) *
                      portTICK_PERIOD_MS);
    int signal_change_per_error = (SIGNAL_CHANGE_PER_ERROR_PER_S * elapsed_ms /
                                   1000);
    last_motor_control_update = current_time;

    newSignals = getMotorSignals();
    int i = 0;
    for (; i < 2; i++) {
        newSignals.signals[i] += (int)(speedsDesired.speeds[i] -
                                       speeds.speeds[i] *
                                       SIGNAL_CHANGE_PER_ERROR);
        if (newSignals.signals[i] > MAX_SIGNAL) {
            newSignals.signals[i] = MAX_SIGNAL;
        } else if (newSignals.signals[i] < -MAX_SIGNAL) {
            newSignals.signals[i] = -MAX_SIGNAL;
        }
    }

    if ((speedsDesired.speeds[LEFT_SIDE] - speeds.speeds[LEFT_SIDE]) /
        speedsDesired.speeds[LEFT_SIDE] <= 0.05 &&
        (speedsDesired.speeds[RIGHT_SIDE] - speeds.speeds[RIGHT_SIDE]) /
        speedsDesired.speeds[RIGHT_SIDE] <= 0.05) {
        SYS_PORTS_PinSet(0, PORT_CHANNEL_A, 3);
    } else {
        SYS_PORTS_PinClear(0, PORT_CHANNEL_A, 3);
    }

    setMotorSignals(newSignals);
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
struct StandardQueueMessage makeMotorSpeeds(MotorSpeeds speeds) {
    StandardQueueMessage msg = {
        .type = MESSAGE_MOTOR_SPEEDS,
        .motorSpeeds = speeds,
    };
    return msg;
}

struct StandardQueueMessage makeMotorSpeedsReport(MotorSpeeds speeds) {
    StandardQueueMessage msg = makeMotorSpeeds(speeds);
    msg.type = MESSAGE_MOTOR_SPEEDS_REPORT;
    return msg;
}

MotorSpeeds getSpeeds(struct StandardQueueMessage * msg) {
    checkMessageType2(msg, MESSAGE_MOTOR_SPEEDS, MESSAGE_MOTOR_SPEEDS_REPORT);
    return msg->motorSpeeds;
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
        speedsDesired = getSpeeds(&receivedMessage);
        
        break;
    case MESSAGE_MOTOR_SPEEDS_REPORT:
        motorControlUpdate(getSpeeds(&receivedMessage));
        break;
    default:
        ;
        // pass
    }
}

 

/*******************************************************************************
 End of File
 */
