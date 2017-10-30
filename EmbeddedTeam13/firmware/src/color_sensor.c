#include "color_sensor.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "master_control_public.h"
#include "debug.h"

#define COLOR_READ_FREQUENCY_MS 50
#define TCS34725_ADDRESS 0x52
#define TX_DATA_LENGTH 1
#define RX_DATA_LENGTH 32
static TimerHandle_t color_timer;
static DRV_HANDLE drvI2CHandle; // I2C Driver Handle
static DRV_I2C_BUFFER_HANDLE txBuffer; // Transmit Buffer Handle
static DRV_I2C_BUFFER_HANDLE rxBuffer; // Receive Buffer Handle
static uint8_t txData = 3; // Holds the transmit data
static uint8_t rxData[RX_DATA_LENGTH];  // Hold the received data

static void colorTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    
    int red = 0, green = 0, blue = 0, clear = 2;

    //clear = 4;
    
    // Initializes the color sensor so that's it's ready to transmit
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, TCS34725_ADDRESS, &txData, TX_DATA_LENGTH, NULL);
    
    if (txBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
        dbgFatalError(DBG_ERROR_COLOR_RUN);
    }
    
    // Receive 32 bytes of data from the color sensor
    rxBuffer = DRV_I2C_Receive(drvI2CHandle, TCS34725_ADDRESS, &rxData, RX_DATA_LENGTH, NULL);
    
    if (rxBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
        dbgFatalError(DBG_ERROR_COLOR_RUN);
    }
    
    /*if (rxBuffer != (DRV_I2C_BUFFER_HANDLE)NULL) {*/
        // Parse RGBC data from receive buffer
        //clear = 6;
     
        clear = rxData[20] & (rxData[21] << 8);
        red = rxData[22] & (rxData[23] << 8);
        green = rxData[24] & (rxData[25] << 8);
        blue = rxData[26] & (rxData[27] << 8);
    //}
    
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

    // Enable the ADC
    DRV_ADC_Open();
    
    // Enable the I2C Module
    drvI2CHandle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_BLOCKING);
    
    if (drvI2CHandle == DRV_HANDLE_INVALID) {
       dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    
    // Initializes the color sensor so that's it's ready to transmit
    /*txBuffer = DRV_I2C_Transmit(drvI2CHandle, TCS34725_ADDRESS, &txData, TX_DATA_LENGTH, NULL);
    
    if (txBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
        dbgFatalError(DBG_ERROR_COLOR_RUN);
    }
    
    rxBuffer = DRV_I2C_Receive(drvI2CHandle, TCS34725_ADDRESS, &rxData, RX_DATA_LENGTH, NULL);
    
    if (rxBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
        dbgFatalError(DBG_ERROR_COLOR_RUN);
    }*/
    
    // Start the timer
    if(xTimerStart(color_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
}
