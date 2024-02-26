#ifndef BINGU_SDCARD_H
#define BINGU_SDCARD_H

/* Import built-ins */
#include <SD.h>                 // For R/W operations

/* Import local files */
#include "airspeed.h"
#include "globals.h"
#include "gps.h"

//-----------------------------------------------------------------------------
// FUNCTION DEFENITIONS & VARIABLES
//-----------------------------------------------------------------------------
void setup_sd();
void loop_sdcard_write();
void updateSDreadwrite();
void getS4FlipFlop();


File datalog;


//-----------------------------------------------------------------------------
// FUNCTION CODE
//-----------------------------------------------------------------------------

void setup_sd() {
    flag_SDready = SD.begin(pin_SDCS);
}

void loop_sdcard_write() {
    if (
    (flag_SDready == true) && // ...the SD card is ready to write to
    (flag_S4flipflop == true) // ...the user wants to write to SD
    ) {

    // If enough time has passed since the last log, write data to SD
    if (tlast_SDwritten_ms + 1000 / SDlograte_Hz < tlast_cyclestart_ms) {

      // Log all of the data!
      datalog.print(F("System Uptime[ms] "));
      datalog.print(tlast_cyclestart_ms);
      datalog.print(",");
      datalog.print(F("System VBATTcell0[V] "));
      datalog.print(battCells_V[0]);
      datalog.print(",");
      datalog.print(F("System VBATTcell1[V] "));
      datalog.print(battCells_V[1]);
      datalog.print(",");
      datalog.print(F("System VBATTcell2[V] "));
      datalog.print(battCells_V[2]);
      datalog.print(",");

      datalog.print(F("Airdata rho[kg/m3] "));
      datalog.print(rhoatm_kgpm3);
      datalog.print(",");
      datalog.print(F("Airdata T[C] "));
      datalog.print(Tatm_C);
      datalog.print(",");
      datalog.print(F("Airdata VCAS[m/s] "));
      datalog.print(vcas_mps);
      datalog.print(",");
      datalog.print(F("Airdata VTAS[m/s] "));
      datalog.print(vtas_mps);
      datalog.print(",");

      datalog.print(F("BMP280_0 P[Pa] "));
      datalog.print(data_P0.P_Pa);
      datalog.print(",");
      datalog.print(F("BMP280_0 Tpackage[C] "));
      datalog.print(data_P0.Tpackage_C);
      datalog.print(",");
  
      datalog.print(F("HTU21DF_0 RH[%] "));
      datalog.print(data_RH0.RH_percent);
      datalog.print(",");
      datalog.print(F("HTU21DF_0 Tpackage[C] "));
      datalog.print(data_RH0.Tpackage_C);
      datalog.print(",");
  
      datalog.print(F("D6F-PH0025AD1_0 q[Pa] "));
      datalog.print(data_q0.q_Pa);
      datalog.print(",");
      datalog.print(F("D6F-PH0025AD1_0 Tpackage[C] "));
      datalog.print(data_q0.Tpackage_C);
      datalog.print(",");

      datalog.print(F("VL53L1X_0 s[m] "));
      datalog.print(data_s0.distance_m);
      datalog.print(",");
      
      datalog.print(F("GPS_0 satellites[-] "));
      datalog.print(data_GPS0.satellites);
      datalog.print(",");
      datalog.print(F("GPS_0 latitude[deg] "));
      datalog.print(data_GPS0.lat, 10);
      datalog.print(",");
      datalog.print(F("GPS_0 longitude[deg] "));
      datalog.print(data_GPS0.lng, 10);
      datalog.print(",");
      datalog.print(F("GPS_0 year[-] "));
      datalog.print(data_GPS0.year);
      datalog.print(",");
      datalog.print(F("GPS_0 month[-] "));
      datalog.print(data_GPS0.month);
      datalog.print(",");
      datalog.print(F("GPS_0 day[-] "));
      datalog.print(data_GPS0.day);
      datalog.print(",");
      datalog.print(F("GPS_0 hours[-] "));
      datalog.print(data_GPS0.hour);
      datalog.print(",");
      datalog.print(F("GPS_0 minutes[-] "));
      datalog.print(data_GPS0.minute);
      datalog.print(",");
      datalog.print(F("GPS_0 seconds[-] "));
      datalog.print(data_GPS0.second);
      datalog.print(",");
      datalog.print(F("GPS_0 speed[m/s] "));
      datalog.print(data_GPS0.speed, 2);
      datalog.print(",");
      datalog.print(F("GPS_0 course[deg] "));
      datalog.print(data_GPS0.course, 1);
      datalog.print(",");
 
      datalog.println(";");
    }

    // Every 5 seconds, flush any data to the SD card (improves SD lifetime)
    if (tlast_SDflushed_ms + 5000 < tlast_cyclestart_ms) {
      datalog.flush();
      smartDelay(100);
    }

    //TODO swap this for battery as SD activity light already implemented
    // Every 2 seconds, blink the green LED to indicate SD activity
    if (tnext_LED2expiry_ms < tlast_cyclestart_ms) {
      digitalWrite(pin_LED, HIGH);
      smartDelay(25);
      digitalWrite(pin_LED, LOW);
      tnext_LED2expiry_ms = tlast_cyclestart_ms + 2000;
    }
  }
}

void updateSDreadwrite()
{
  // If S4's flipflop state is false, close any files and leave the function
  getS4FlipFlop();
  if (!flag_S4flipflop) {
    return;
  }

  // If the SD card is ready and no filename is prepared, make one
  if ((flag_SDready == true) && (!datalog)) {
    snprintf(logfilename, sizeof(logfilename), "log000.csv");
    for(int i=0; i<1000; ++i) {
      logfilename[3] = '0' + (i / 100);
      logfilename[4] = '0' + ((i % 100) / 10);
      logfilename[5] = '0' + ((i % 100) % 10);
      // Create file if it does not exist (do not open existing)
      if (!SD.exists(logfilename)) {
        break;
      }
    }
    // Attempt to create a new file, unready the SD on failure
    datalog = SD.open(logfilename, FILE_WRITE);
    if (!datalog) {
      flag_SDready = false;
    }
  }
  // Else if the SD card and file object is ready, make sure SD is alive
  else if ((flag_SDready == true) && (datalog)) {
    if (!SD.exists(logfilename)) {
      flag_SDready = false;
      datalog.close();
    }
  }
  // Else the SD card is not ready, try to ready it
  else if (flag_SDready == false) {
    if (SD.begin(pin_SDCS)) {
      flag_SDready = true;
    }
    else {
      flag_S4flipflop = false;
    }
  }
}

void getS4FlipFlop()
{
  // If S4 (held-high) falls to ground, record falling edge
  if ((digitalRead(pin_S4) == LOW) && (flag_S4fallingedge == false)) {
    flag_S4fallingedge = true;
  }
  // If S4 is pulled back to high, record rising edge and toggle flip-flop
  if ((digitalRead(pin_S4) == HIGH) && (flag_S4fallingedge == true)) {
    // Set fallingedge to false and toggle the state of S4's toggle
    flag_S4fallingedge = false;
    flag_S4flipflop = !flag_S4flipflop;
  }
}


#endif //BINGU_SDCARD_H