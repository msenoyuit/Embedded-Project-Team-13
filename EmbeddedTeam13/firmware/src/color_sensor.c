#include "color_sensor.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "master_control_public.h"
#include "debug.h"

#define COLOR_READ_FREQUENCY_MS 50
#define TCS34725_ADDRESS 0x29
#define ACK_DATA_LENGTH 1
#define RX_DATA_LENGTH 4
static TimerHandle_t color_timer;
static DRV_HANDLE drvI2CHandle; // I2C Driver Handle
static DRV_I2C_BUFFER_HANDLE hReadyBuffer; // Ready Buffer Handle
static DRV_I2C_BUFFER_HANDLE hTxBuffer; // Transmit Buffer Handle
static DRV_I2C_BUFFER_HANDLE hAckBuffer; // Acknoledge Buffer Handle
static DRV_I2C_BUFFER_HANDLE hRxBuffer; // Receive Buffer Handle
static uint8_t ackData = 0; // Hold the acknowledge polling data
static uint8_t RxData[RX_DATA_LENGTH] = {0};  // Hold the received data

static void colorTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    int red = 0, green = 0, blue = 0, clear = 0;

    // Open I2C Connection
    /*drvI2CHandle = DRV_HANDLE_INVALID;
    drvI2CHandle = DRV_I2C_Open(I2C_ID_1, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_BLOCKING);
    
    // Checks if I2C connection was successfully opened
    if (drvI2CHandle != (DRV_HANDLE(NULL))) {
        // Transmit read request to I2C
        hAckBuffer = DRV_I2C_Transmit(drvI2CHandle, TCS34725_ADDRESS, (void *)&ackData, ACK_DATA_LENGTH, (void*)NULL);
        
        if(hAckBuffer != (DRV_I2C_BUFFER_HANDLE)NULL) {
            hRxBuffer = DRV_I2C_Receive(drvI2CHandle, TCS34725_ADDRESS, (void *)&RxData[0], RX_DATA_LENGTH, (void*)NULL);
            
        }
        // else indicate that error has occurred
        
        DRV_I2C_Close(drvI2CHandle);
    }*/
    // else indicate that error has occurred
    
    // Parse received data
    red = (int)RxData[0];
    green = (int)RxData[1];
    blue = (int)RxData[2];
    clear = (int)RxData[3];
    
    // Pretend to read sensor
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

    // Start the timer
    if(xTimerStart(color_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
}
