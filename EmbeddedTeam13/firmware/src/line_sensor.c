#include "line_sensor.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "drive_control_public.h"
#include "debug.h"

/*#define LINE_READ_FREQUENCY_MS 50
#define STM8S105C4_ADDRESS 18
#define TX_DATA_LENGTH 1
#define RX_DATA_LENGTH 16
static TimerHandle_t line_timer;
static DRV_HANDLE drvI2CHandle; // I2C Driver Handle
static DRV_I2C_BUFFER_HANDLE txBuffer; // Transmit Buffer Handle
static DRV_I2C_BUFFER_HANDLE rxBuffer; // Receive Buffer Handle
static uint8_t txData = 0; // Holds the transmit data
static uint8_t rxData[RX_DATA_LENGTH];  // Hold the received data
static SYS_STATUS i2c_status;
static DRV_I2C_BUFFER_EVENT i2cOpStatus;
static int line_reading = 0;

static void lineTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    
    StandardQueueMessage msg = makeLineReading(line_reading);
    if(driveControlSendMsgToQFromISR(&msg, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_LINE_RUN);
    }

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}*/

void lineSensorInit(void) {
    /* Configure Timer */
    /*line_timer = xTimerCreate("Line Timer",
                              pdMS_TO_TICKS(LINE_READ_FREQUENCY_MS),
                              pdTRUE, ( void * ) 0, lineTimerCallback);
    if(line_timer == NULL) {
        dbgFatalError(DBG_ERROR_LINE_INIT);
    }
    
    if(xTimerStart(line_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_LINE_INIT);
    }*/
}
