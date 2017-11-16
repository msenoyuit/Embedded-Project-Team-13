#include "color_sensor.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "master_control_public.h"
#include "debug.h"

#define COLOR_READ_FREQUENCY_MS 50
#define LINE_READ_FREQUENCY_MS 50
#define TCS34725_ADDRESS 82
#define STM8S105C4_ADDRESS 18
#define TX_DATA_LENGTH 1
#define RX_DATA_LENGTH 32
#define RX2_DATA_LENGTH 16
static TimerHandle_t color_timer;
static TimerHandle_t line_timer;

static DRV_HANDLE drvI2CHandle; // I2C Driver Handle
static DRV_I2C_BUFFER_HANDLE txBuffer; // Transmit Buffer Handle
static DRV_I2C_BUFFER_HANDLE rxBuffer; // Receive Buffer Handle
static SYS_STATUS i2c_status; // Hold the current status of the I2C bus
static DRV_I2C_BUFFER_EVENT i2cOpStatus;

static uint8_t txData = 3; // Holds the transmit data
static uint8_t rxData[RX_DATA_LENGTH];  // Hold data received from the color sensor
static uint8_t rx2Data[RX2_DATA_LENGTH]; // Hold data received from the line sensor
static uint8_t line_data; // Hold the final computed line sesnor values
static uint16_t clear, red, green, blue; // Hold the final computed RGBC value

// LineBufferEventHandler(DRV_I2C_BUFFER_EVENT event,
                                //DRV_I2C_BUFFER_HANDLE bufferHandle,
                                //uintptr_t context );
// Check sample code for function to interpret raw sensor data
// If error set all values high

// lineTimerCallback(TimerHandle_t timer)
// Check if I2C is ready
// Receive from line sensor
// Send IR sensor data to drive control thread

/*void lineBufferEventHandler(DRV_I2C_BUFFER_EVENT event,
                                DRV_I2C_BUFFER_HANDLE bufferHandle,
                                uintptr_t context );

void lineBufferEventHandler(DRV_I2C_BUFFER_EVENT event,
                                DRV_I2C_BUFFER_HANDLE bufferHandle,
                                uintptr_t context ) {
    switch(event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE: // parse the received data
            line_data = 2;
            break;
        
        case DRV_I2C_BUFFER_EVENT_ERROR: // set data to indicate error
            line_data = 1;
            break;
        
        default:
            break; 
    }
}*/

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
            line_data = 2;
            break;

        case DRV_I2C_BUFFER_EVENT_ERROR: // set data to indicate error
            clear = 1;
            red = 1;
            green = 1;
            blue = 1;
            line_data = 1;
            break;
        
        default:
            break;
    }
}

static void lineTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    
    // Check status of I2C
    i2c_status = DRV_I2C_Status(sysObj.drvI2C0);
    
    // Receive 16 bytes of data from the line sensor if the I2C bus is ready
    if (i2c_status == SYS_STATUS_READY) {   
        rxBuffer = DRV_I2C_Receive(drvI2CHandle, STM8S105C4_ADDRESS, &rx2Data[0], RX2_DATA_LENGTH, NULL);
        /*if (rxBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
            dbgFatalError(DBG_ERROR_LINE_RUN);
        }*/
    }
    
    StandardQueueMessage msg = makeLineReading(line_data);
    if(driveControlSendMsgToQFromISR(&msg, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_LINE_RUN);
    }

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

static void colorTimerCallback(TimerHandle_t timer) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    
    // Check status of I2C
    i2c_status = DRV_I2C_Status(sysObj.drvI2C0);
    
    // Receive 32 bytes of data from the color sensor if I2C bus is ready
    if (i2c_status == SYS_STATUS_READY) {   
        rxBuffer = DRV_I2C_Receive(drvI2CHandle, TCS34725_ADDRESS, &rxData[0], RX_DATA_LENGTH, NULL);
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
    // Configure color sensor timer
    color_timer = xTimerCreate("Color Timer",
                            pdMS_TO_TICKS(COLOR_READ_FREQUENCY_MS),
                            pdTRUE, ( void * ) 0, colorTimerCallback);
    if(color_timer == NULL) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    
    // Configure line sensor timer
    line_timer = xTimerCreate("Line Timer",
                            pdMS_TO_TICKS(LINE_READ_FREQUENCY_MS),
                            pdTRUE, ( void * ) 0, lineTimerCallback);
    
    // Enable the I2C Module
    drvI2CHandle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);
    if (drvI2CHandle == DRV_HANDLE_INVALID) {
       dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    
    DRV_I2C_BufferEventHandlerSet(drvI2CHandle, ColorBufferEventHandler, i2cOpStatus);
    
    // Transmit a 0x03 byte to the color sensor to initial color readings
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, TCS34725_ADDRESS, &txData, TX_DATA_LENGTH, NULL);
    if (txBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
            dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    
    // Read one sequence from the line sensor
    rxBuffer = DRV_I2C_Receive(drvI2CHandle, STM8S105C4_ADDRESS, &rx2Data[0], RX2_DATA_LENGTH, NULL);
    if (rxBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    
    
    // Start the color sensor timer
    if(xTimerStart(color_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
    
    // Star line sensor timer
    if(xTimerStart(line_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
}
