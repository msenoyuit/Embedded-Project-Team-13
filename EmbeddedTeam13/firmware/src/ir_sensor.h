/* 
 * File:   ir_sensor.h
 * Author: nyle
 *
 * Created on September 27, 2017, 12:45 PM
 */

#ifndef IR_SENSOR_H
#define	IR_SENSOR_H

void irSensorInit(void);
void IR_ADC_Average (void);
static void irTimerCallback(TimerHandle_t timer);

#endif	/* IR_SENSOR_H */

