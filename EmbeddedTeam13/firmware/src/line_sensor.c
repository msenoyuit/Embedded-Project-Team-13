#include "line_sensor.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "drive_control_public.h"
#include "debug.h"

#define LINE_READ_FREQUENCY_MS 50
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

void LineBufferEventHandler(DRV_I2C_BUFFER_EVENT event,
                                DRV_I2C_BUFFER_HANDLE bufferHandle,
                                uintptr_t context ) {
    switch(event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE: // parse the received data
            line_reading = 1;
            break;

        case DRV_I2C_BUFFER_EVENT_ERROR: // set line reading to eliminate error
            line_reading = 2;
            break;
        
        default:
            break;
    }
}

static void lineTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    /*i2c_status = DRV_I2C_Status(sysObj.drvI2C0);
    
    // Receive 16 bytes of data from the color sensor if I2C is ready
    if (i2c_status == SYS_STATUS_READY) { 
        line_reading = 3;
        rxBuffer = DRV_I2C_Receive(drvI2CHandle, 18, &rxData[0], RX_DATA_LENGTH, NULL);
        if (rxBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
            dbgFatalError(DBG_ERROR_COLOR_RUN);
        }
    }*/
    
    StandardQueueMessage msg = makeLineReading(line_reading);
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
    /*
    // Enable the I2C Module
    drvI2CHandle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);
    if (drvI2CHandle == DRV_HANDLE_INVALID) {
       dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    
    DRV_I2C_BufferEventHandlerSet(drvI2CHandle, LineBufferEventHandler, i2cOpStatus);
    */
    // Start the timer
    if(xTimerStart(line_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_LINE_INIT);
    }
}
