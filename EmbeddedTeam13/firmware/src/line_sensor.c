#include "line_sensor.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "drive_control_public.h"
#include "debug.h"

#define LINE_READ_FREQUENCY_MS 50
#define STM8S105C4_ADDRESS 0x51
#define TX_DATA_LENGTH 1
#define RX_DATA_LENGTH 1
static TimerHandle_t line_timer;
static DRV_HANDLE drvI2CHandle; // I2C Driver Handle
static DRV_I2C_BUFFER_HANDLE txBuffer; // Transmit Buffer Handle
static DRV_I2C_BUFFER_HANDLE rxBuffer; // Receive Buffer Handle
static uint8_t txData = 3; // Holds the transmit data
static uint8_t rxData[RX_DATA_LENGTH] = {0};  // Hold the received data

static void lineTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    int line_reading = 0;
    
    // Add I2C read and write functions
    
    // Parse receive data
    line_reading = rxData[0];
    
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

    // Enable the I2C Module
    /*drvI2CHandle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);
    
    if (drvI2CHandle == DRV_HANDLE_INVALID) {
       dbgFatalError(DBG_ERROR_COLOR_INIT);
    }*/
    
    // Start the timer
    if(xTimerStart(line_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_LINE_INIT);
    }
}
