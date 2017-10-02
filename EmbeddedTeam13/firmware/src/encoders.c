#include "FreeRTOS.h"
#include "timers.h"
#include "semphr.h"

#include "encoders.h"
#include "queue_utils.h"
#include "debug.h"

#define L_ENCODER_FREQUENCY_MS 5
#define R_ENCODER_FREQUENCY_MS 5

static unsigned int lEncoderCount, rEncoderCount;
static SemaphoreHandle_t encoderSemaphore;
static TimerHandle_t lEncoderTimer, rEncoderTimer;

static void encoderCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreTakeFromISR(encoderSemaphore, &higherPriorityTaskWoken);
    switch ((EncoderId) pvTimerGetTimerID(timer)) {
    case L_ENCODER:
        lEncoderCount++;
        break;
    case R_ENCODER:
        rEncoderCount++;
        break;
    default:
        dbgFatalError(DBG_ERROR_ENCODER_ISR);
        break;
    }
    
    xSemaphoreGiveFromISR(encoderSemaphore, &higherPriorityTaskWoken);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void encodersInit(void) {
    lEncoderCount = rEncoderCount = 0;

    lEncoderTimer = xTimerCreate("Left encoder timer",
                                 pdMS_TO_TICKS(L_ENCODER_FREQUENCY_MS), pdTRUE,
                                 ( void * ) L_ENCODER, encoderCallback);
    if(lEncoderTimer == NULL) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }

    rEncoderTimer = xTimerCreate("Right encoder timer",
                                 pdMS_TO_TICKS(R_ENCODER_FREQUENCY_MS), pdTRUE,
                                 ( void * ) R_ENCODER, encoderCallback);
    if(lEncoderTimer == NULL) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    
    encoderSemaphore = xSemaphoreCreateBinary();
    if (encoderSemaphore == NULL) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    xSemaphoreGive(encoderSemaphore);
}

unsigned int getLeftEncoderCount(void) {
    xSemaphoreTake(encoderSemaphore, portMAX_DELAY);
    int count = lEncoderCount;
    xSemaphoreGive(encoderSemaphore);
    return count;
}

void setLeftEncoderCount(unsigned int count) {
    xSemaphoreTake(encoderSemaphore, portMAX_DELAY);
    lEncoderCount = count;
    xSemaphoreGive(encoderSemaphore);
}

unsigned int getRightEncoderCount(void) {
    xSemaphoreTake(encoderSemaphore, portMAX_DELAY);
    int count = rEncoderCount;
    xSemaphoreGive(encoderSemaphore);
    return count;
}

void setRightEncoderCount(unsigned int count) {
    xSemaphoreTake(encoderSemaphore, portMAX_DELAY);
    rEncoderCount = count;
    xSemaphoreGive(encoderSemaphore);
}

BaseType_t getLeftEncoderCountISR(unsigned int * result,
                                    BaseType_t * higherPriorityTaskWoken) {
    if (!xSemaphoreTakeFromISR(encoderSemaphore, higherPriorityTaskWoken)) {
        return pdFALSE;
    }
    *result = lEncoderCount;
    return xSemaphoreGiveFromISR(encoderSemaphore, higherPriorityTaskWoken);
}

BaseType_t setLeftEncoderCountISR(unsigned int count,
                                  BaseType_t * higherPriorityTaskWoken) {
    if (!xSemaphoreTakeFromISR(encoderSemaphore, higherPriorityTaskWoken)) {
        return pdFALSE;
    }
    lEncoderCount = count;
    return xSemaphoreGiveFromISR(encoderSemaphore, higherPriorityTaskWoken);
}

BaseType_t getRightEncoderCountISR(unsigned int * result,
                                     BaseType_t * higherPriorityTaskWoken) {
    if (!xSemaphoreTakeFromISR(encoderSemaphore, higherPriorityTaskWoken)) {
        return pdFALSE;
    }
    *result = rEncoderCount;
    return xSemaphoreGiveFromISR(encoderSemaphore, higherPriorityTaskWoken);
}

BaseType_t setRightEncoderCountISR(unsigned int count,
                                   BaseType_t * higherPriorityTaskWoken) {
    if (!xSemaphoreTakeFromISR(encoderSemaphore, higherPriorityTaskWoken)) {
        return pdFALSE;
    }
    rEncoderCount = count;
    return xSemaphoreGiveFromISR(encoderSemaphore, higherPriorityTaskWoken);
}
