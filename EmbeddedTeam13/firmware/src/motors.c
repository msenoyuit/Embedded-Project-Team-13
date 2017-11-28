#include <math.h>

#include "motors.h"

// Motor Signal get/set*********************************************************
// Should only be directly set/read by the following
static MotorSignals signals = {.signals={0,0}};

MotorSignals getMotorSignals(void) {
    return signals;
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

void setMotorSignals(MotorSignals new_signals) {
    setLeftMotorDir(new_signals.signals[LEFT_SIDE] > 0);
    setLeftMotorAbsSignal((unsigned int) abs(new_signals.signals[LEFT_SIDE]));

    setRightMotorDir(new_signals.signals[RIGHT_SIDE] > 0);
    setRightMotorAbsSignal((unsigned int) abs(new_signals.signals[RIGHT_SIDE]));
    
    signals = new_signals;
}
