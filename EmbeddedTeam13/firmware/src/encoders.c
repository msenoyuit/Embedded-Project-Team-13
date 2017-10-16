#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

#include "encoders.h"
#include "queue_utils.h"
#include "debug.h"

#define SPEED_CALC_DELTA_T_MS 10
#define ENCODER_TICKS_PER_INCH 300 /* TODO: Measure; This is a guess! */
// Used by speed calculations. These should only ever be modified by the
// encoderSpeedCallback
static volatile int lEncoderLastCount, rEncoderLastCount;
static volatile int lEncoderSpeed, rEncoderSpeed;
// Encoder counts. These should only be modified by the encoder ISRs.
static volatile int lEncoderCount, rEncoderCount;
// Timer for encoder speed callback
static TimerHandle_t encoderSpeedTimer;

// Functions for Reading Encoder Inputs ****************************************
static bool getLeftA() {
    return (SYS_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_E) >> 8) & 1;
}
// NOT YET DETERMINED! For now we give the A reading, which will make it look
// like we are always going the same direction
static bool getLeftB() {
    return (SYS_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_E) >> 8) & 1;
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
    if (getLeftA() == getLeftB()) {
        lEncoderCount++;
    } else {
        lEncoderCount--;
    }
}

void rEncoderIsr() {
    if (getRightA() == getRightB()) {
        rEncoderCount++;
    } else {
        rEncoderCount--;
    }
}

static void encoderSpeedCallback(TimerHandle_t timer) {
    int lCount, rCount;
    lCount = getLeftEncoderCountISR();
    rCount = getRightEncoderCountISR();

    lEncoderSpeed = (lCount - lEncoderLastCount) * 1000 / SPEED_CALC_DELTA_T_MS;
    rEncoderSpeed = (rCount - rEncoderLastCount) * 1000 / SPEED_CALC_DELTA_T_MS;

    lEncoderLastCount = lCount;
    rEncoderLastCount = rCount;
}

// Initialization **************************************************************
void encodersInit(void) {
    // Initialize file-static variables
    lEncoderCount = rEncoderCount = lEncoderLastCount = rEncoderLastCount = 0;
    lEncoderSpeed = rEncoderSpeed = 0;

    // Set up callback for calculating speed
    encoderSpeedTimer = xTimerCreate("Encoder speed timer",
                                     pdMS_TO_TICKS(SPEED_CALC_DELTA_T_MS),
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
int getLeftEncoderCount(void) {
    taskENTER_CRITICAL();
    int count = lEncoderCount;
    taskEXIT_CRITICAL();
    return count;
}

int getRightEncoderCount(void) {
    taskENTER_CRITICAL();
    int count = rEncoderCount;
    taskEXIT_CRITICAL();
    return count;
}

int getLeftEncoderCountISR(void) {
    UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    int result = lEncoderCount;
    taskEXIT_CRITICAL_FROM_ISR(mask);
    return result;
}

int getRightEncoderCountISR(void) {
    UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    int result = rEncoderCount;
    taskEXIT_CRITICAL_FROM_ISR(mask);
    return result;
}

int getLeftEncoderSpeed(void) {
    taskENTER_CRITICAL();
    int speed = lEncoderSpeed;
    taskEXIT_CRITICAL();
    return speed;
}

int getRightEncoderSpeed(void) {
    taskENTER_CRITICAL();
    int speed = rEncoderSpeed;
    taskEXIT_CRITICAL();
    return speed;
}

int getLeftEncoderSpeedISR(void) {
    UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    int speed = lEncoderSpeed;
    taskEXIT_CRITICAL_FROM_ISR(mask);
    return speed;
}

int getRightEncoderSpeedISR(void) {
    UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    int speed = rEncoderSpeed;
    taskEXIT_CRITICAL_FROM_ISR(mask);
    return speed;
}
