#include "ir_sensor.hpp"

static QTRSensors QTR;

#define EMITTER_PIN  7 // Emitter pin for QTR8A IR Array
#define SENSOR_PINS  (const uint8_t[]) { A7, A6, 4, 5, A3, A2, A1, A0}
#define SENSOR_COUNT sizeof(SENSOR_PINS)

void ir_sensors_init(void)
{
    QTR.setTypeAnalog();
    QTR.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
    QTR.setEmitterPin(EMITTER_PIN);

    for (uint16_t i = 0; i < 400; i++)
    {
        QTR.calibrate();
    }
}

uint16_t ir_sensors_read_line(void)
{
    uint16_t Sensor_Readings[8];
    return QTR.readLineWhite(Sensor_Readings);
}