// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
#ifndef OMRON_D6FPH_H
#define OMRON_D6FPH_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#define D6FPH_ADDRESS   0x6C
#define SENS_CTRL       0xD040
#define COMP_DATA1_H    0xD051
#define TMP_H           0xD061
#define BUFFER_0        0x07
#define CTRL_REG        0x0B
#define START_ADDRESS   0x00

#define SENS_CTRL_MS        2 
#define SENS_CTRL_DV_PWR    1
#define SENS_CTRL_VAL       (0x01 << SENS_CTRL_MS | 0x01 << SENS_CTRL_DV_PWR)

#define SERIAL_CTRL_D_BYTE_CNT3     5
#define SERIAL_CTRL_REQ             3
#define SERIAL_CTRL_R_WZ            2
#define SERIAL_CTRL_VAL   (0x01 << SERIAL_CTRL_R_WZ | 0x01 << SERIAL_CTRL_REQ | 0x01 << SERIAL_CTRL_D_BYTE_CNT3)

#if defined(__AVR__) || defined(ESP8266)
typedef enum {
    I2C_ERROR_OK=0,
    I2C_ERROR_LENGTH,
    I2C_ERROR_ADDRESS_SENT_NACK,
    I2C_ERROR_DATA_SENT_NACK,
    I2C_ERROR_OTHER
} i2c_err_t;
#endif

enum sensorModels{
    MODEL_0025AD1 = 0,
    MODEL_0505AD3,
    MODEL_5050AD3
};

class Omron_D6FPH
{
public:
	Omron_D6FPH(void);
	boolean begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = D6FPH_ADDRESS, sensorModels sensorModel = MODEL_5050AD3);
    boolean begin(sensorModels sensorModel); 
    void setSensorModel(sensorModels sensorModel = MODEL_5050AD3);
    boolean isConnected();
    float getPressure();
    float getTemperature();
private:
    sensorModels _sensorModel;
    boolean init();
    boolean readRegister(uint8_t reg, uint16_t *value);
    boolean executeMcuMode();
    TwoWire *_i2cPort;
    uint8_t _i2cAddress;  
    uint16_t _rangeMode;
    uint16_t _rangeModeSubVal;
    uint8_t _rangeModeMulVal;
};

#endif