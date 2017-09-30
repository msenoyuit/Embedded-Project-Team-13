#include "line_sensor.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "drive_control_public.h"
#include "debug.h"

#define LINE_READ_FREQUENCY_MS 50
static TimerHandle_t line_timer;

static void lineTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    // Pretend to read sensor
    StandardQueueMessage msg = makeLineReading(0);
    if(driveControlSendMsgToQFromISR(&msg, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_LINE_RUN);
    }

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void lineSensorInit(void) {
    /* Configure Timer */
    line_timer = xTimerCreate("Line Timer",
                              pdMS_TO_TICKS(LINE_READ_FREQUENCY_MS),
                              pdTRUE, ( void * ) 0, lineTimerCallback);
    if(line_timer == NULL) {
        dbgFatalError(DBG_ERROR_LINE_INIT);
    }

    // Enable the ADC
    DRV_ADC_Open();

    // Start the timer
    if(xTimerStart(line_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_LINE_INIT);
    }
}
