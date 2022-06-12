// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
#include "Omron_D6FPH.h"

// Hack - add missing defines
#define I2C_ERROR_OK (0)

Omron_D6FPH::Omron_D6FPH(void){
  // Constructor
}

boolean Omron_D6FPH::begin(sensorModels sensorModel){
  return begin(Wire, D6FPH_ADDRESS, sensorModel);
}

boolean Omron_D6FPH::begin(TwoWire &wirePort, uint8_t deviceAddress, sensorModels sensorModel){
  setSensorModel(sensorModel);
  _i2cPort = &wirePort;
  _i2cAddress = deviceAddress;
  if(isConnected()){
      return init();
  }
  return false;
}

void Omron_D6FPH::setSensorModel(sensorModels sensorModel){
    _sensorModel = sensorModel;
    switch (_sensorModel)
    {
    case MODEL_0025AD1:
        _rangeMode = 250;
        _rangeModeMulVal = 1;
        _rangeModeSubVal = 0;
        break;
    case MODEL_0505AD3:
        _rangeMode = 50;
        _rangeModeMulVal = 2;
        _rangeModeSubVal = 50;
        break;
    default:
        _rangeMode = 500;
        _rangeModeMulVal = 2;
        _rangeModeSubVal = 500;
        break;
    }
}

/**
 * Initialization after power up
 * Write 00h to the Control Register (Bh) to load NVM trim values, 
 * but keep MCU in non-reset state
 */
boolean Omron_D6FPH::init(){
  _i2cPort->beginTransmission(_i2cAddress);
  _i2cPort->write(CTRL_REG);
  _i2cPort->write(0x00);
  return (_i2cPort->endTransmission() == I2C_ERROR_OK);
}

boolean Omron_D6FPH::isConnected(){
    _i2cPort->beginTransmission((uint8_t)_i2cAddress);
    return _i2cPort->endTransmission() == I2C_ERROR_OK;
}

/**
 * Execute MCU mode after desired configurations are set
 * Write 06h(MS=1 & MCU_on) to the SENS_CTRL Register (D040h).
 */
boolean Omron_D6FPH::executeMcuMode(){
  _i2cPort->beginTransmission(_i2cAddress); 
  _i2cPort->write(START_ADDRESS);  
  _i2cPort->write(highByte(SENS_CTRL));  
  _i2cPort->write(lowByte(SENS_CTRL));  
  _i2cPort->write(0x18);  
  _i2cPort->write(SENS_CTRL_VAL);  
  return (_i2cPort->endTransmission() == I2C_ERROR_OK);
}

float Omron_D6FPH::getPressure(){
    if(executeMcuMode()){
        delay(33);
        _i2cPort->beginTransmission(_i2cAddress);
        _i2cPort->write(START_ADDRESS);
        _i2cPort->write(highByte(COMP_DATA1_H));
        _i2cPort->write(lowByte(COMP_DATA1_H));
        _i2cPort->write(SERIAL_CTRL_VAL);
        if (_i2cPort->endTransmission() == I2C_ERROR_OK){
            uint16_t value;
            if(readRegister(BUFFER_0, &value)){
                return (float)((value - 1024.00) * _rangeMode * _rangeModeMulVal / 60000L) - _rangeModeSubVal;
            }
        }
    }
    return NAN;
}

float Omron_D6FPH::getTemperature(){
    if(executeMcuMode()){
        delay(33);
        _i2cPort->beginTransmission(_i2cAddress);
        _i2cPort->write(START_ADDRESS);   
        _i2cPort->write(highByte(TMP_H));
        _i2cPort->write(lowByte(TMP_H));
        _i2cPort->write(SERIAL_CTRL_VAL);
        if (_i2cPort->endTransmission() == I2C_ERROR_OK){
            uint16_t value;
            if(readRegister(BUFFER_0, &value)){
                int temp = round((float)(value - 10214) / 3.739);
                return (temp/10.0); 
            }
        }
    }
    return NAN;
}

boolean Omron_D6FPH::readRegister(uint8_t reg, uint16_t *value) {
    _i2cPort->beginTransmission(_i2cAddress);
    _i2cPort->write(reg);
    if(_i2cPort->endTransmission() == I2C_ERROR_OK){
        _i2cPort->requestFrom(_i2cAddress, (uint8_t)2);
        *value = ((_i2cPort->read() << 8) | _i2cPort->read());
        return true;
    }
    return false;
}