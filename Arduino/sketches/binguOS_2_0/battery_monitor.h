#ifndef BINGU_BATTERY_H
#define BINGU_BATTERY_H

/* Import local files */
#include "globals.h"

//-----------------------------------------------------------------------------
// FUNCTION DEFENITIONS & VARIABLES
//-----------------------------------------------------------------------------

float* getBatteryVoltage();
float getBatteryPerc(float battCells_V[]);

//-----------------------------------------------------------------------------
// FUNCTION CODE
//-----------------------------------------------------------------------------

float* getBatteryVoltage()
{
  static float battCells_V[3];  // Make persistent outside of function scope

  // Force analog read values to be unsigned, weirdness with floats!!
  unsigned int analog0val = analogRead(pin_V1S);
  unsigned int analog1val = analogRead(pin_V2S); 
  unsigned int analog2val = analogRead(pin_V3S); 

  // Determine the voltage at the battery sensing pin  
  float vsense1_V = 3.3 * analog0val / 1023;
  float vsense2_V = 3.3 * analog1val / 1023;
  float vsense3_V = 3.3 * analog2val / 1023;

  // Determine the voltage across each of the "probed" points
  float battProbepoint3_V = vsense3_V * (R9_MOhm + R10_MOhm) / R9_MOhm;
  float battProbepoint2_V = vsense2_V * (R11_MOhm + R12_MOhm) / R11_MOhm;
  float battProbepoint1_V = vsense1_V * (R13_MOhm + R14_MOhm) / R13_MOhm;

  // Solve for voltage across each cell
  battCells_V[0] = battProbepoint1_V;
  battCells_V[1] = battProbepoint2_V - battProbepoint1_V;
  battCells_V[2] = battProbepoint3_V - battProbepoint2_V;

  return battCells_V;
}

//TODO update primary LED to flash on low battery
float getBatteryPerc(float battCells_V[]) {
    // Yaseen found some random equation online for 3S battery voltage vs capacity
    float batteryvoltage_V = battCells_V[0] + battCells_V[1] + battCells_V[2];
    return (batteryvoltage_V - 10.604848) / 0.019036;
}

#endif //BINGU_BATTERY_H