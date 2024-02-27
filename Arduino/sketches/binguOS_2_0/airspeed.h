#ifndef BINGU_AIRSPEED_H
#define BINGU_AIRSPEED_H

/* Import local files */
#include "globals.h"

//-----------------------------------------------------------------------------
// FUNCTION DEFENITIONS & VARIABLES
//-----------------------------------------------------------------------------

void calc_sensor_readings();
float calcPvapoursaturated(float T_C);
void set_airspeed_LEDs();
void write_airspeed_LEDs(bool r, bool y, bool g);

//TODO setup constants as per flight testing
const float v_stall = 1.5;
const float v_rotate = 1.5;
const float pitchGradient = 1;

float Patm_Pa, Tatm_C, RH_percent;
float vias_mps;
float vcas_mps;
float Pvapoursaturated_Pa;
float Pvapour_Pa;
float Pdryair_Pa;
float Ratm_JpKpkg;
float cpatm_JpKpkg;
float cvatm_JpKpkg;
float gammaatm;
float rhoatm_kgpm3;
float speedofsound_mps;
float Machnumber;
float vtas_mps;

//-----------------------------------------------------------------------------
// FUNCTION CODE
//-----------------------------------------------------------------------------

void calc_sensor_readings() {
  Patm_Pa = data_P0.P_Pa;
  Tatm_C = data_P0.Tpackage_C - 8.3; /* calibration guess lol*/
  RH_percent = data_RH0.RH_percent;
  
  vias_mps = pow(2 * data_q0.q_Pa / rhoSL_kgpm3, 0.5);
  vcas_mps = vias_mps;
  Pvapoursaturated_Pa = calcPvapoursaturated(Tatm_C);
  Pvapour_Pa = Pvapoursaturated_Pa * (RH_percent / 100);
  Pdryair_Pa = Patm_Pa - Pvapour_Pa;
  Ratm_JpKpkg = Patm_Pa * R0_JpKpmol
    / (Pdryair_Pa * Wdryair_kgpmol + Pvapour_Pa * Wvapour_kgpmol);
  cpatm_JpKpkg = R0_JpKpmol / 2 * (7 / Wdryair_kgpmol + 8 / Wvapour_kgpmol);
  cvatm_JpKpkg = R0_JpKpmol / 2 * (5 / Wdryair_kgpmol + 6 / Wvapour_kgpmol);
  gammaatm = cpatm_JpKpkg / cvatm_JpKpkg;
  rhoatm_kgpm3 = Patm_Pa / Ratm_JpKpkg / (Tatm_C + 273.15);
  speedofsound_mps = pow(gammaatm * Ratm_JpKpkg * (Tatm_C + 273.15), 0.5);
  Machnumber = pow(2.0 / (gammaatm - 1.0) * (pow(1 + data_q0.q_Pa / Patm_Pa, (gammaatm - 1.0) / gammaatm) - 1.0), 0.5);
  vtas_mps = Machnumber * speedofsound_mps;

}


float calcPvapoursaturated(float T_C)
{
  // Use Teten's equation to estimate the saturation pressure of vapour
  float varA = 17.27, varB = 237.3;
  if (T_C > 35) {
    #if DEBUG_ENABLE
    Serial.println(F("calcPvapoursaturated not modelled for use in T_C > 35!"));
    #endif //DEBUG_ENABLE
  }
  else if (T_C < 0) {
    // Murray estimation for conditions below zero
    varA = 21.875;
    varB = 265.5;
  }
  return 610.78 * pow(e, varA * T_C / (varB + T_C));  
}

/* TODO WIP
void set_airspeed_LEDs() {
  //need pitch and speed
  float maxPitch = ;
  //TODO set conditions
  if (pitch > pitchGradient * (speed - v_stall)) {
    // RED = TOO SLOW: Stalling
    write_airspeed_LEDs(HIGH,LOW,LOW);
  } else if (pitch > pitchGradient * (speed - v_rotate)) {
    // RED/YELLOW = -VE RATE OF CLIMB: About to Stall pitch down
    write_airspeed_LEDs(HIGH,HIGH,LOW);
  } else if (pitch > pitchGradient * (speed - v_rotate) + 10) {
    // YELLOW = ABOVE STALL: Level Flight
    write_airspeed_LEDs(LOW,HIGH,LOW);
  } else if (pitch < pitchGradient * (speed - v_rotate) + 10) {
    // YELLOW/GREEN = GOOD SPEED: Can pitch up v. slowly
    write_airspeed_LEDs(LOW,HIGH,HIGH);
  } else {
    // GREEN = FAST: Can Pitch Up
    //TODO check current code => Should not occur
    write_airspeed_LEDs(LOW,LOW,HIGH);
  }
  
}

void write_airspeed_LEDs(bool r, bool y, bool g) {
  digitalWrite(pin_Red,r);
  digitalWrite(pin_Yellow,y);
  digitalWrite(pin_Green,g);
}
*/

#endif //BINGU_AIRSPEED_H