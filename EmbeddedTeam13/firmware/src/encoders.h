#ifndef ENCODERS_H
#define	ENCODERS_H

#include "FreeRTOS.h"

void encodersInit(void);

int getLeftEncoderCount(void);
int getRightEncoderCount(void);
int getLeftEncoderCountISR(void);
int getRightEncoderCountISR(void);

float getLeftEncoderSpeed(void);
float getRightEncoderSpeed(void);
float getLeftEncoderSpeedISR(void);
float getRightEncoderSpeedISR(void);

void lEncoderIsr();
void rEncoderIsr();

#endif	/* ENCODERS_H */
