#ifndef BINGU_GLOBALS_H
#define BINGU_GLOBALS_H

//-----------------------------------------------------------------------------
// DEFINITIONS - Global constants
//-----------------------------------------------------------------------------
#define DEBUG_ENABLE 1          //Determines whether to use the GPS module as they share SPI pins.
                                //TODO bit-bang SPI for use on RX1, TX1 to allow GPS & debug

/* Mathematical constants */
const float e = 2.71828;
const float pi = 3.141592;

/* Scientific constants */
const float R0_JpKpmol = 8.31446;       // The universal gas constant R

/* Standard conditions */
const float Wvapour_kgpmol = 0.01802;   // Molar masses
const float Wdryair_kgpmol = 0.02897;
const float PSL_Pa = 101325.0;          // Sea-level conditions
const float TSL_C = 15.0;
const float rhoSL_kgpm3 = 1.225;
const float hSL_m = 0;

/* Arduino pin definitions (BINGU Schematic v0.4.pdf) */
const int pin_RX0 = 30;
const int pin_TX0 = 31;
const int pin_RX1 = 12;
const int pin_TX1 = 13;
const int pin_SDCS = 23;              // CS1
//TODO remove most of airspeed calculation
const int pin_LED = 11;
const int pin_V1S = 22;
const int pin_V2S = 19;
const int pin_V3S = 26;
const int pin_SDA0 = SDA;             // Pin 27
const int pin_SCL0 = SCL;             // Pin 28
const int pin_S4 = 32;

/* BINGU hardware definitions (BINGU Schematic v0.4.pdf) */
const float R9_MOhm = 0.33;
const float R10_MOhm = 1.00;
const float R11_MOhm = 0.33;
const float R12_MOhm = 0.56;
const float R13_MOhm = 0.33;
const float R14_MOhm = 0.12;

/* Other hardware definitions */
constexpr float d6f_halfangle_rad radians(9);
constexpr float d6f_rotoffset_rad radians(210);

/* BinguOS software definitions */
const int SDlograte_Hz = 4;
const float D6F_mu = 3; // Measurement uncertainty (and seed for estimation uncertainty)
const float D6F_pv = 0.8;  // Process Variance 0.001~1, how fast the measurement moves
const float VL5_mu = 800;
const float VL5_pv = 0.8;
static const uint32_t GPSBaud = 9600;

//-----------------------------------------------------------------------------
// DEFINITIONS - Global variables and functions
//-----------------------------------------------------------------------------
/* BinguOS */
char logfilename[16];
bool flag_lowbattery = false;
bool flag_SDready = false;
bool flag_S4fallingedge = false;
bool flag_S4flipflop = false;
bool flag_VL53L1Xready = false;
unsigned long tlast_cyclestart_ms;
unsigned long tlast_SDflushed_ms = 0;
unsigned long tlast_SDwritten_ms = 0;
unsigned long tnext_LED2expiry_ms;

/* System Diagnosic Values*/
byte* scanner;
float* battCells_V;

/* Custom datatypes */
struct sensordata_P {
  float P_Pa;             // Static pressure measurement
  float Tpackage_C;       // Sensor package temperature
} data_P0;
struct sensordata_q {
  float q_Pa;             // Dynamic pressure measurement
  float Tpackage_C;       // Sensor package temperature
  float offsetX_rad;      // Sensor offset from the longitudinal axis in yaw
  float offsetY_rad;      // Sensor offset from the longitudinal axis in pitch
} data_q0;
struct sensordata_RH {
  float RH_percent;       // Relative humidity measurement
  float Tpackage_C;       // Sensor package temperature
} data_RH0;
struct sensordata_s {
  float distance_m;       // LiDAR time of flight measurement
} data_s0;
struct sensordata_GPS {
  int satellites;         // Number of satellites found
  float lat;              // Latitude
  float lng;              // Longitude
  int year;               // Year
  int month;              // Month
  int day;                // Day
  int hour;               // Hour
  int minute;             // Minute
  int second;             // Second
  float speed;            // GPS speed
  float course;           // GPS heading track
} data_GPS0;

/* IMU Calculations */

/* Custom classes */
Adafruit_BMP280 bmp;
Adafruit_HTU21DF htu;
Adafruit_VL53L1X vl5;
SimpleKalmanFilter KF_D6Fq_Pa(D6F_mu, D6F_mu, D6F_pv);
SimpleKalmanFilter KF_VL5z_m(VL5_mu, VL5_mu, VL5_pv);
Omron_D6FPH d6f;
TinyGPSPlus gps;

#endif //BINGU_GLOBALS_H