#include "line_sensor.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "drive_control_public.h"
#include "debug.h"

#define LINE_READ_FREQUENCY_MS 50
#define STM8S105C4_ADDRESS 0x51
#define ACK_DATA_LENGTH 1
#define RX_DATA_LENGTH 1
static TimerHandle_t line_timer;
static DRV_HANDLE drvI2CHandle; // I2C Driver Handle
static DRV_I2C_BUFFER_HANDLE hReadyBuffer; // Ready Buffer Handle
static DRV_I2C_BUFFER_HANDLE hTxBuffer; // Transmit Buffer Handle
static DRV_I2C_BUFFER_HANDLE hAckBuffer; // Acknoledge Buffer Handle
static DRV_I2C_BUFFER_HANDLE hRxBuffer; // Receive Buffer Handle
static uint8_t ackData = 0; // Hold the acknowledge polling data
static uint8_t RxData = 0; // HOld the received data

static void lineTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    int line_reading = 0;
    
    // Open I2C Connection
    /*drvI2CHandle = DRV_HANDLE_INVALID;
    drvI2CHandle = DRV_I2C_Open(I2C_ID_1, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_BLOCKING);
    
    // Checks if I2C connection was successfully opened
    if (drvI2CHandle != (DRV_HANDLE(NULL))) {
        // Transmit read request to I2C
        hAckBuffer = DRV_I2C_Transmit(drvI2CHandle, STM8S105C4_ADDRESS, (void *)&ackData, ACK_DATA_LENGTH, (void*)NULL);
        
        if(hAckBuffer != (DRV_I2C_BUFFER_HANDLE)NULL) {
            hRxBuffer = DRV_I2C_Receive(drvI2CHandle, STM8S105C4_ADDRESS, (void *)&RxData, RX_DATA_LENGTH, (void*)NULL);
            
        }
        // else indicate that error has occurred
        
        DRV_I2C_Close(drvI2CHandle);
    }*/
    // else indicate that error has occurred
    
    // Parse receive data
    line_reading = (int) RxData;
    
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
