#include "FreeRTOS.h"
#include "timers.h"

#include "ir_sensor.h"

#include "master_control_public.h"
#include "debug.h"

#define IR_READ_FREQUENCY_MS 50

static TimerHandle_t ir_timer;
static uint64_t ADC_avg;

static void irTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    
    //Indicate life
    SYS_PORTS_PinToggle(0, PORT_CHANNEL_A, 3);
    
    // Start getting the next ADC reading
    PLIB_ADC_SampleAutoStartEnable(ADC_ID_1);
    
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/*******************************************************************************
  Function:
    void IR_ADC_Average ( void )
 */

void IR_ADC_Average (void) {
    ADC_avg = 0;
    int i;
    for (i = 0; i < 16; i++) {
        ADC_avg += PLIB_ADC_ResultGetByIndex(ADC_ID_1, i);
    }
    ADC_avg /= 16;
    ADC_avg = 625881/(ADC_avg*200 - 3413);
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    StandardQueueMessage msg = makeDistanceReading(ADC_avg);
    dbgOutputLoc(DBG_ISR_BEFORE_QUEUE_SEND);
    if(masterControlSendMsgToQFromISR(&msg, &higherPriorityTaskWoken) != pdTRUE) {
        dbgFatalError(DBG_ERROR_IR_RUN);
    }
    dbgOutputLoc(DBG_ISR_AFTER_QUEUE_SEND);
}


void irSensorInit(void) {
    /* Configure Timer */
    ir_timer = xTimerCreate("IR Timer", pdMS_TO_TICKS(IR_READ_FREQUENCY_MS),
                         pdTRUE, ( void * ) 0, irTimerCallback);
    if(ir_timer == NULL) {
        dbgFatalError(DBG_ERROR_IR_INIT);
    }
    // Start the timer
    if(xTimerStart(ir_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_IR_INIT);
    }

    // Enable the ADC
    DRV_ADC_Open();

}
