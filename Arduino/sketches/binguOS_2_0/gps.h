#ifndef BINGU_GPS_H
#define BINGU_GPS_H

/* Import local files */
#include "globals.h"

//-----------------------------------------------------------------------------
// FUNCTION DEFENITIONS & VARIABLES
//-----------------------------------------------------------------------------

static void smartDelay(unsigned long ms);
void getGPSdata(sensordata_GPS &sd);

//-----------------------------------------------------------------------------
// FUNCTION CODE
//-----------------------------------------------------------------------------

#if DEBUG_ENABLE
static void smartDelay(unsigned long ms) {delay(ms);}
#else //DEBUG_ENABLE
// This custom version of delay() ensures that the gps object
// is being "fed". Originally written by Mikal Hart
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < ms);
}
#endif //DEBUG_ENABLE

void getGPSdata(sensordata_GPS &sd)
{
  if (true) {
    // Number of connected satellites
    if (!gps.satellites.isValid()) {
      sd.satellites = -1;
    }
    else {
      sd.satellites = gps.satellites.value();
    }
  
    // Latitude and Longitude
    if (!gps.location.isValid()) {
      sd.lat = NAN;
      sd.lng = NAN;
    }
    else {
      sd.lat = gps.location.lat();
      sd.lng = gps.location.lng();
    }
  
    // GPS date and time
    TinyGPSDate d = gps.date;
    TinyGPSTime t = gps.time;
 
    if (!d.isValid() || (d.age() > 1500)) {
      sd.year = -1;
      sd.month = -1;
      sd.day = -1;
    }
    else {
      sd.year = d.year();
      sd.month = d.month();
      sd.day = d.day();
    }
 
    if (!t.isValid()) {
      sd.hour = -1;
      sd.minute = -1;
      sd.second = -1;
    }
    else {
      // Compensate for old fixes on time using the age of the fix
      sd.hour = (t.hour() + t.age() / 1000 / 60 / 60) % 24;
      sd.minute = (t.minute() + t.age() / 1000 / 60) % 60;
      sd.second = (t.second() + t.age() / 1000) % 60;
    }

    // GPS speed and course
    if (!gps.speed.isValid()) {
      sd.speed = NAN;
    }
    else {
      sd.speed = gps.speed.mps();
    }
    if (!gps.course.isValid()) {
      sd.course = NAN;
    }
    else {
      sd.course = gps.course.deg();
    }
  }
}

#endif //BINGU_GPS_H