#include "color_sensor.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "master_control_public.h"
#include "debug.h"

#define COLOR_READ_FREQUENCY_MS 50
#define TCS34725_ADDRESS 82
//#define STM8S105C4_ADDRESS 18
#define LINE_SENSOR_ADDRESS 124
#define TX_DATA_LENGTH 1
#define TX2_DATA_LENGTH 2
#define RX_DATA_LENGTH 32
#define RX2_DATA_LENGTH 2
static TimerHandle_t color_timer;
static DRV_HANDLE drvI2CHandle; // I2C driver handle
static DRV_I2C_BUFFER_HANDLE txBuffer; // Transmit buffer handle
static DRV_I2C_BUFFER_HANDLE rxBuffer; // Receive buffer handle for color data
static DRV_I2C_BUFFER_HANDLE rx2Buffer; // Recevie buffer handle for line data
static SYS_STATUS i2c_status; // Hold the current status of the I2C bus
static DRV_I2C_BUFFER_EVENT i2cOpStatus;
static uint8_t txData = 3; // Holds the transmit data
static uint8_t tx2Data[14] = {0x7D, 0x12, 0x7D, 0x34, 0x0F, 0xFF, 0x0E, 0xFC, 0x10, 0x01, 0x10, 0x00, 0x13, 0x11};
static uint8_t rxData[RX_DATA_LENGTH];  // Hold data received from the color sensor
static uint8_t rx2Data[RX2_DATA_LENGTH]; // Hold data received from the line sensor
static uint8_t line_data = 0; // Hold the final computed line sesnor values
static uint16_t clear, red, green, blue; // Hold the final computed RGBC value
//static uint8_t count = 0;

void ColorBufferEventHandler(DRV_I2C_BUFFER_EVENT event,
                                DRV_I2C_BUFFER_HANDLE bufferHandle,
                                uintptr_t context );

void readSensors();

void ColorBufferEventHandler(DRV_I2C_BUFFER_EVENT event,
                                DRV_I2C_BUFFER_HANDLE bufferHandle,
                                uintptr_t context ) {
    switch(event)
    {
        case DRV_I2C_BUFFER_EVENT_COMPLETE: // parse the received color data
            clear = (rxData[20] + (rxData[21] << 8));
            red = (rxData[22] + (rxData[23] << 8));
            green = (rxData[24] + (rxData[25] << 8));
            blue = (rxData[26] + (rxData[27] << 8));
            break;

        case DRV_I2C_BUFFER_EVENT_ERROR: // set data to indicate error
            line_data = 1;
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
    
    readSensors();
    
    // Send color readings to master control thread
    StandardQueueMessage msg = makeColorReading(red, green, blue, clear);
    if(masterControlSendMsgToQFromISR(&msg, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_COLOR_RUN);
    }

    // Send line readings to drive control thread
    msg = makeLineReading(line_data);
    if(driveControlSendMsgToQFromISR(&msg, &higherPriorityTaskWoken)
       != pdTRUE) {
        dbgFatalError(DBG_ERROR_LINE_RUN);
    }
    
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void readSensors() {
    drvI2CHandle = DRV_I2C_Open(DRV_I2C_INDEX_0, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);
    DRV_I2C_BufferEventHandlerSet(drvI2CHandle, ColorBufferEventHandler, i2cOpStatus);
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, LINE_SENSOR_ADDRESS, &tx2Data[10], TX2_DATA_LENGTH, NULL);
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, LINE_SENSOR_ADDRESS, &tx2Data[13], TX_DATA_LENGTH, NULL);
    rxBuffer = DRV_I2C_Receive(drvI2CHandle, LINE_SENSOR_ADDRESS, &line_data, 1, NULL);
    rxBuffer = DRV_I2C_Receive(drvI2CHandle, TCS34725_ADDRESS, &rxData[0], RX_DATA_LENGTH, NULL);
    DRV_I2C_Close(drvI2CHandle);
}

void colorSensorInit(void) {
    // Configure  timer
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
    
    // Test is this is needed here
    DRV_I2C_BufferEventHandlerSet(drvI2CHandle, ColorBufferEventHandler, i2cOpStatus);
    
    // Send values to initialize line sensor for data read
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, LINE_SENSOR_ADDRESS, &tx2Data[0], TX2_DATA_LENGTH, NULL);
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, LINE_SENSOR_ADDRESS, &tx2Data[2], TX2_DATA_LENGTH, NULL);
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, LINE_SENSOR_ADDRESS, &tx2Data[12], TX_DATA_LENGTH, NULL);
    rxBuffer = DRV_I2C_Receive(drvI2CHandle, LINE_SENSOR_ADDRESS, &rx2Data[0], RX2_DATA_LENGTH, NULL);
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, LINE_SENSOR_ADDRESS, &tx2Data[4], TX2_DATA_LENGTH, NULL);
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, LINE_SENSOR_ADDRESS, &tx2Data[6], TX2_DATA_LENGTH, NULL);
    //txBuffer = DRV_I2C_Transmit(drvI2CHandle, LINE_SENSOR_ADDRESS, &tx2Data[8], TX2_DATA_LENGTH, NULL);
    /*if (txBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }*/
    
    // Send value to initialize color sensor for data read
    txBuffer = DRV_I2C_Transmit(drvI2CHandle, TCS34725_ADDRESS, &txData, TX_DATA_LENGTH, NULL);
    /*if (txBuffer == (DRV_I2C_BUFFER_HANDLE)NULL) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }*/
    
    DRV_I2C_Close(drvI2CHandle);
    
    // Start the color sensor timer
    if(xTimerStart(color_timer, 0) != pdPASS) {
        dbgFatalError(DBG_ERROR_COLOR_INIT);
    }
}