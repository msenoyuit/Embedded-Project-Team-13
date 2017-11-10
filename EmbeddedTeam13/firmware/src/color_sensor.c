#include "color_sensor.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "master_control_public.h"
#include "debug.h"

#define COLOR_READ_FREQUENCY_MS 50
#define TCS34725_ADDRESS 82
#define TX_DATA_LENGTH 1
#define RX_DATA_LENGTH 32
static TimerHandle_t color_timer;
static DRV_HANDLE drvI2CHandle; // I2C Driver Handle
static DRV_I2C_BUFFER_HANDLE txBuffer; // Transmit Buffer Handle
static DRV_I2C_BUFFER_HANDLE rxBuffer; // Receive Buffer Handle
static uint8_t txData = 3; // Holds the transmit data
static uint8_t rxData[RX_DATA_LENGTH];  // Hold the received data
static SYS_STATUS i2c_status;
static DRV_I2C_BUFFER_EVENT i2cOpStatus;
static uint16_t clear, red, green, blue;

void ColorBufferEventHandler(DRV_I2C_BUFFER_EVENT event,
                                DRV_I2C_BUFFER_HANDLE bufferHandle,
                                uintptr_t context );

void ColorBufferEventHandler(DRV_I2C_BUFFER_EVENT event,
                                DRV_I2C_BUFFER_HANDLE bufferHandle,
                                uintptr_t context ) {
    switch(event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE: // parse the received data
            clear = (rxData[20] + (rxData[21] << 8));
            red = (rxData[22] + (rxData[23] << 8));
            green = (rxData[24] + (rxData[25] << 8));
            blue = (rxData[26] + (rxData[27] << 8));
            break;

        case DRV_I2C_BUFFER_EVENT_ERROR: // set data to indicate error
            clear = 1;
            red = 1;
            green = 1;
            blue = 1;
            break;
        
        default:
            break;
    }
}

static void colorTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    
    // Check status of I2C
    i2c_status = DRV_I2C_Status(sysObj.drvI2C0);
    
    // Receive 32 bytes of data from the color sensor if I2C is ready
    if (i2c_status == SYS_STATUS_READY) {   
        rxBuffer = DRV_I2C_Receive(drvI2CHandle, 82, &rxData[0], RX_DATA_LENGTH, NULL);
        /*if (rxBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
            dbgFatalError(DBG_ERROR_COLOR_RUN);
        }*/
    }
    
    StandardQueueMessage msg = makeColorReading(red, green, blue, clear);
    if(masterControlSendMsgToQFromISR(&msg, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_COLOR_RUN);
    }

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void colorSensorInit(void) {
    /* Configure Timer */
    color_timer = xTimerCreate("Color Timer",
                            pdMS_TO_TICKS(COLOR_READ_FREQUENCY_MS),
                            pdTRUE, ( void * ) 0, colorTimerCallback);
    if(color_timer == NULL) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    
    // Enable the I2C Module
    drvI2CHandle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);
    if (drvI2CHandle == DRV_HANDLE_INVALID) {
       dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    
    DRV_I2C_BufferEventHandlerSet(drvI2CHandle, ColorBufferEventHandler, i2cOpStatus);
    
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, TCS34725_ADDRESS, &txData, TX_DATA_LENGTH, NULL);
    if (txBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
            dbgFatalError(DBG_ERROR_COLOR_RUN);
    }
    
    // Start the timer
    if(xTimerStart(color_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }    
}
