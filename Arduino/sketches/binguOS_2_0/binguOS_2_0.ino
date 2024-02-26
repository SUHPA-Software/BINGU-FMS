/* binguOS, written by Yaseen Reza 28/07/2022 */
/* updated for atmega328pb by Robert Aries 12/02/2024 */
//-----------------------------------------------------------------------------
// DEPENDENCIES
//-----------------------------------------------------------------------------
/* Import built-ins */
#include <Wire.h>               // For I2C enabled devices

/* Import downloaded */
#include <Adafruit_BMP280.h>    // Static pressure sensor (w/ temperature)
#include <Adafruit_HTU21DF.h>   // Relative humidity sensor (w/ temperature)
#include <Adafruit_VL53L1X.h>   // LIDAR altimeter sensor (w/ temperature)
#include <Omron_D6FPH.h>        // Dynamic pressure sensor (w/ temperature)
#include <SimpleKalmanFilter.h> // Uni-dimensional Kalman filtering
#include <TinyGPSPlus.h>

/* Import local files */
#include "airspeed.h"
#include "battery_monitor.h"
#include "debug_variables.h"
#include "globals.h"
#include "gps.h"
#include "sd_card.h"
#include "sensors.h"

/* Function prototypes (needed if default arguments are given) */
// Nothing here yet!

//-----------------------------------------------------------------------------
// Setup - Run once
//-----------------------------------------------------------------------------
void setup() {
  #if DEBUG_ENABLE
    // Start serial debug connection
    Serial.begin(115200);
  #else //DEBUG_ENABLE
    // Start a serial connection for the GPS module
    Serial.begin(GPSBaud);
  #endif //DEBUG_ENABLE

  // Setup digital I/O pins
  pinMode(pin_LED, OUTPUT);
  pinMode(pin_V1S, INPUT);  // Strictly speaking, used for analog input
  pinMode(pin_V2S, INPUT);  // Strictly speaking, used for analog input
  pinMode(pin_V3S, INPUT);  // Strictly speaking, used for analog input
  pinMode(pin_S4, INPUT);

  // Setup SD card
  setup_sd();

  // Setup I2C communication
  Wire.begin();
  Wire.setClock(400000);

  vl5begin();
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  d6f.begin(MODEL_0025AD1);
  //TODO check these inits can be removed
  //d6f.begin(MODEL_0025AD1);
  //d6f.begin(MODEL_0025AD1);
  htu.begin();
}

//-----------------------------------------------------------------------------
// Loop - Repeat forever
//-----------------------------------------------------------------------------
void loop() {

  // Read system diagnostic information
  tlast_cyclestart_ms = millis();
  scanner = i2cscan();
  battCells_V = getBatteryVoltage();

  // Read sensor data
  getGPSdata(data_GPS0);
  getLIDARdistance(data_s0);
  getPitotPressure(data_q0);
  getRelativeHumidity(data_RH0);
  getStaticPressure(data_P0);
  // Run data processing routines
  updateSDreadwrite();

  // IMU calculations
  float pitch = NAN;
  float roll = NAN;
  float z_m = data_s0.distance_m;

  // Other calculations
  calc_sensor_readings();

  // Write to the SD card if...
  loop_sdcard_write();

  // If a message sent with high priority has expired, draw the homescreen
  //TODO draw LEDs

  // DEBUGGING PRINTS GO BELOW THIS POINT
  #if DEBUG_ENABLE
  Serial.print("Satellites: ");
  Serial.print(data_GPS0.satellites);
  Serial.print(" Latitude: ");
  Serial.print(data_GPS0.lat, 10);
  Serial.print(" Longitude: ");
  Serial.print(data_GPS0.lng, 10);
  Serial.print(" Seconds: ");
  Serial.println(data_GPS0.second);
  #endif //DEBUG_ENABLE
}
