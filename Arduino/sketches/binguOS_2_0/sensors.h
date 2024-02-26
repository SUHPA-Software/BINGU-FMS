#ifndef BINGU_SENSORS_H
#define BINGU_SENSORS_H

/* Import local files */
#include "globals.h"

//-----------------------------------------------------------------------------
// FUNCTION DEFENITIONS & VARIABLES
//-----------------------------------------------------------------------------

byte* i2cscan();
void getLIDARdistance(sensordata_s &sd);
void getPitotPressure(sensordata_q &sd);
void getRelativeHumidity(sensordata_RH &sd);
void getStaticPressure(sensordata_P &sd);
void vl5begin();

//-----------------------------------------------------------------------------
// FUNCTION CODE
//-----------------------------------------------------------------------------


byte* i2cscan()
{
  static byte i2cavailable[128];  // Make persistent outside of function scope
  byte statuscode, address;
  int nDevices = 0;

  // i2c addresses are 7 bits, scan all 128 possibilities
  for (address = 0; address < 128; ++address) {
    i2cavailable[address] = 0;
    // Refer to Arduino reference on 'Wire.endTransmission()' for more
    Wire.beginTransmission(address);
    statuscode = Wire.endTransmission();
    // Record the address into an array of available addresses
    if (statuscode == 0) {
      i2cavailable[nDevices] = address;
      ++nDevices;
    }
  }
  return i2cavailable;
}


void getLIDARdistance(sensordata_s &sd)
{
  if (flag_VL53L1Xready) {
    int distance = vl5.distance();
    int estdistance = KF_VL5z_m.updateEstimate(distance);
    sd.distance_m = (float)estdistance / 1000;
  }
  else {
    sd.distance_m = NAN;
  }
}


void getPitotPressure(sensordata_q &sd)
{
  // Read the selected sensor
  if (d6f.isConnected()) {
    sd.q_Pa = max(0, d6f.getPressure()); // Ignore negative pressures
    sd.Tpackage_C = d6f.getTemperature();
  }
  else {
    sd.q_Pa = NAN;
    sd.Tpackage_C = NAN;
  }
}


void getRelativeHumidity(sensordata_RH &sd)
{
  // Save humidity and temperature data
  sd.RH_percent = htu.readHumidity();
  sd.Tpackage_C = htu.readTemperature();
}


void getStaticPressure(sensordata_P &sd)
{
  float pressure = bmp.readPressure();
  float temperature = bmp.readTemperature();
  if ((pressure > 0) && (temperature < 179)) {
    // Save pressure and temperature data
    sd.P_Pa = KF_D6Fq_Pa.updateEstimate(pressure);
    sd.Tpackage_C = temperature;
  }
  else {
    sd.P_Pa = NAN;
    sd.Tpackage_C = NAN;
  }
}


void vl5begin()
{
  byte *scanner = i2cscan();

  flag_VL53L1Xready = false;
  for (int i = 0; i < 128; ++i) {
    // VL53L1X is really annoying and hangs if not present on I2C when begin is called
    if (scanner[i] == 0x29) {
      flag_VL53L1Xready = true;
      vl5.begin();
      vl5.startRanging();
      vl5.setTimingBudget(50);
    }
  }
}


#endif //BINGU_SENSORS_H