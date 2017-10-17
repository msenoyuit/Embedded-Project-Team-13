#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

#include "encoders.h"
#include "queue_utils.h"
#include "debug.h"
#include "motors.h"

#define SPEED_DELTA_T_MS 10
#define SPEED_FILTER_RATIO 0.9
#define COUNTS_PER_REVOLUTION = 298 * 12 /* 298:1 gear ratio, 12 CPR encoder */
#define ENCODER_TICKS_PER_INCH 300 /* TODO: Measure; This is a guess! */
// Used by speed calculations. These should only ever be modified by the
// encoderSpeedCallback
static volatile int lEncoderLastCount, rEncoderLastCount;
static volatile size_t countBufPos;

static volatile float lEncoderSpeed, rEncoderSpeed;
// Encoder counts. These should only be modified by the encoder ISRs.
static volatile int lEncoderCount, rEncoderCount;
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
    int lCount = getLeftEncoderCountISR();
    int rCount = getRightEncoderCountISR();

    int lNewSpeed = (lCount - lEncoderLastCount) * 1000 / SPEED_DELTA_T_MS;
    int rNewSpeed = (rCount - rEncoderLastCount) * 1000 / SPEED_DELTA_T_MS;

    // Simple lowpass filter
    lEncoderSpeed = SPEED_FILTER_RATIO * lEncoderSpeed +
        (1 - SPEED_FILTER_RATIO) * lNewSpeed;
    rEncoderSpeed = SPEED_FILTER_RATIO * rEncoderSpeed +
        (1 - SPEED_FILTER_RATIO) * rNewSpeed;

    lEncoderLastCount = lCount;
    rEncoderLastCount = rCount;
}

// Initialization **************************************************************
void encodersInit(void) {
    // Initialize file-static variables
    lEncoderCount = rEncoderCount = 0;
    lEncoderLastCount = rEncoderLastCount = 0;
    lEncoderSpeed = rEncoderSpeed = 0;
    countBufPos = 0;

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
    float speed = lEncoderSpeed;
    taskEXIT_CRITICAL();
    return (int)speed;
}

int getRightEncoderSpeed(void) {
    taskENTER_CRITICAL();
    float speed = rEncoderSpeed;
    taskEXIT_CRITICAL();
    return (int)speed;
}

int getLeftEncoderSpeedISR(void) {
    UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    float speed = lEncoderSpeed;
    taskEXIT_CRITICAL_FROM_ISR(mask);
    return (int)speed;
}

int getRightEncoderSpeedISR(void) {
    UBaseType_t mask = taskENTER_CRITICAL_FROM_ISR();
    float speed = rEncoderSpeed;
    taskEXIT_CRITICAL_FROM_ISR(mask);
    return (int)speed;
}
