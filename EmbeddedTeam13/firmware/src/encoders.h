#ifndef ENCODERS_H
#define	ENCODERS_H

#include "FreeRTOS.h"

void encodersInit(void);

int getLeftEncoderCount(void);
int getRightEncoderCount(void);
int getLeftEncoderCountISR(void);
int getRightEncoderCountISR(void);

int getLeftEncoderSpeed(void);
int getRightEncoderSpeed(void);
int getLeftEncoderSpeedISR(void);
int getRightEncoderSpeedISR(void);

void lEncoderIsr();
void rEncoderIsr();

#endif	/* ENCODERS_H */
