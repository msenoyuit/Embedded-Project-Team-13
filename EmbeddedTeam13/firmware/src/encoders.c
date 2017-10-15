#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

#include "encoders.h"
#include "queue_utils.h"
#include "debug.h"

#define SPEED_CALC_DELTA_T_MS 50
#define ENCODER_TICKS_PER_INCH 300 /* TODO: Measure; This is a guess! */
static volatile int lEncoderLastPos, rEncoderLastPos;
static volatile int lEncoderSpeed, rEncoderSpeed;

static volatile int lEncoderCount, rEncoderCount;

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

// Initialization **************************************************************
void encodersInit(void) {
    lEncoderCount = rEncoderCount = 0;
}

// External Access Functions ***************************************************
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
