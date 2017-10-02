#ifndef ENCODERS_H
#define	ENCODERS_H

#include "FreeRTOS.h"

void encodersInit(void);

unsigned int getLeftEncoderCount(void);
void setLeftEncoderCount(unsigned int count);
unsigned int getRightEncoderCount(void);
void setRightEncoderCount(unsigned int count);

BaseType_t getLeftEncoderCountISR(unsigned int * result,
                                  BaseType_t * higherPriorityTaskWoken);
BaseType_t setLeftEncoderCountISR(unsigned int count,
                                  BaseType_t * higherPriorityTaskWoken);
BaseType_t getRightEncoderCountISR(unsigned int * result,
                                   BaseType_t * higherPriorityTaskWoken);
BaseType_t setRightEncoderCountISR(unsigned int count,
                                   BaseType_t * higherPriorityTaskWoken);

#endif	/* ENCODERS_H */

