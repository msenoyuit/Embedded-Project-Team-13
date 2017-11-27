#include <math.h>

#include "motors.h"

// Motor Signal get/set*********************************************************
// Should only be directly set/read by the following
static int lSignal, rSignal;

int getLeftMotorSignal(void) {
    return lSignal;
}

int getRightMotorSignal(void) {
    return rSignal;
}

static void setLeftMotorAbsSignal(unsigned int signal) {
    DRV_OC0_PulseWidthSet(signal);
}

static void setRightMotorAbsSignal(unsigned int signal) {
    DRV_OC1_PulseWidthSet(signal);
}

static void setLeftMotorDir(bool forward) {
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, 14, forward);
}

static void setRightMotorDir(bool forward) {
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, 1, forward);
}

void setLeftMotorSignal(int signal) {
    setLeftMotorDir(signal > 0);
    setLeftMotorAbsSignal((unsigned int) abs(signal));
    lSignal = signal;
}

void setRightMotorSignal(int signal) {
    setRightMotorDir(signal > 0);
    setRightMotorAbsSignal((unsigned int) abs(signal));
    rSignal = signal;
}
