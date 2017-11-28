#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

#include "encoders.h"
#include "queue_utils.h"
#include "debug.h"
#include "motors.h"

#define SPEED_DELTA_T_MS 10
#define SPEED_FILTER_RATIO 0.7
#define COUNTS_PER_REVOLUTION = (298 * 12) // 298:1 gear ratio, 12 CPR encoder
// Approximation, with pi = 3, r = 1.5 * 3/2
#define ENCODER_TICKS_PER_INCH (COUNTS_PER_REVOLUTION / 3 * 3 / 2)

static volatile EncoderCounts encoderCounts = {.counts={0,0}};
static volatile EncoderCounts encoderLastCounts = {.counts={0,0}};
static volatile MotorSpeeds speeds = {.speeds={0,0}};

// Timer for encoder speed callback
static TimerHandle_t encoderSpeedTimer;

// Functions for Reading Encoder Inputs ****************************************
static bool getLeftA() {
    return (SYS_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_E) >> 8) & 1;
}

static bool getLeftB() {
    return (SYS_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_A) >> 9) & 1;
}

static bool getRightA() {
    return (SYS_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_E) >> 9) & 1;
}

static bool getRightB() {
    return (SYS_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_A) >> 7) & 1;
}

// Interrupt Handlers **********************************************************
// Handlers for when the A channel goes high. These are registered in
// system_interrupt.c in the middle of the pregenerated code
void lEncoderIsr() {
    if (getLeftA() != getLeftB()) {
        encoderCounts.counts[LEFT_SIDE]++;
    } else {
        encoderCounts.counts[LEFT_SIDE]--;
    }
}

void rEncoderIsr() {
    if (getRightA() == getRightB()) {
        encoderCounts.counts[RIGHT_SIDE]++;
    } else {
        encoderCounts.counts[RIGHT_SIDE]--;
    }
}

static void encoderSpeedCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    EncoderCounts counts = getEncoderCountsISR();

    int i = 0;
    for (; i < 2; i++) { // For left and right
        float newSpeed = (counts.counts[i] - encoderLastCounts.counts[i]) *
            1000 / SPEED_DELTA_T_MS;
        // Simple lowpass filter
        speeds.speeds[i] = SPEED_FILTER_RATIO * speeds.speeds[i] +
            (1 - SPEED_FILTER_RATIO) * newSpeed;
        encoderLastCounts.counts[i] = counts.counts[i];
    }

    StandardQueueMessage msg = makeMotorSpeedsReport(speeds);
    if(motorControlSendMsgToQFromISR(&msg, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_ENCODER_ISR);
    }
    
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

// Initialization **************************************************************
void encodersInit(void) {
    // Set up callback for calculating speed
    encoderSpeedTimer = xTimerCreate("Encoder speed timer",
                                     pdMS_TO_TICKS(SPEED_DELTA_T_MS),
                                     pdTRUE, ( void * ) 0,
                                     encoderSpeedCallback);
    if(encoderSpeedTimer == NULL) {
        dbgFatalError(DBG_ERROR_ENCODER_INIT);
    }
    if(xTimerStart(encoderSpeedTimer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_ENCODER_INIT);
    }
}

// External Access Functions ***************************************************
// Enter critical before reading to ensure it's not changed during read
EncoderCounts getEncoderCounts(void) {
    taskENTER_CRITICAL();
    EncoderCounts result = encoderCounts;
    taskEXIT_CRITICAL();
    return result;
}

EncoderCounts getEncoderCountsISR(void) {
    UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    EncoderCounts result = encoderCounts;
    taskEXIT_CRITICAL_FROM_ISR(mask);
    return result;
}

MotorSpeeds getEncoderSpeeds(void) {
    taskENTER_CRITICAL();
    MotorSpeeds result = speeds;
    taskEXIT_CRITICAL();
    return result;
}

MotorSpeeds getEncoderSpeedsISR(void) {
    UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    MotorSpeeds result = speeds;
    taskEXIT_CRITICAL_FROM_ISR(mask);
    return result;
}
