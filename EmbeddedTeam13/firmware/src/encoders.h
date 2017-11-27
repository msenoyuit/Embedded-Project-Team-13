#ifndef ENCODERS_H
#define	ENCODERS_H

#include "FreeRTOS.h"
#include "motor_data_types.h"

void encodersInit(void);

EncoderCounts getEncoderCounts(void);
EncoderCounts getEncoderCountsISR(void);

MotorSpeeds getEncoderSpeeds(void);
MotorSpeeds getEncoderSpeedsISR(void);

void lEncoderIsr();
void rEncoderIsr();

#endif	/* ENCODERS_H */
