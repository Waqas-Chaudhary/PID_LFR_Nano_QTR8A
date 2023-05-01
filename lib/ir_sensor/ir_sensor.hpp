#ifndef IR_SENSOR_HPP_
#define IR_SENSOR_HPP_

#include <Arduino.h>
#include "QTRSensors.h"

void ir_sensors_init(void);

uint16_t ir_sensors_read_line(void);

#endif // IR_SENSOR_HPP_