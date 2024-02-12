/* binguOS, written by Yaseen Reza 28/07/2022 */
/* updated for atmega328pb by Robert Aries 12/02/2024 */
//-----------------------------------------------------------------------------
// DEPENDENCIES
//-----------------------------------------------------------------------------
/* Import built-ins */
#include <LiquidCrystal.h>      // For the LCD display
#include <SD.h>                 // For R/W operations
#include <Wire.h>               // For I2C enabled devices

/* Import downloaded */
#include <Adafruit_BMP280.h>    // Static pressure sensor (w/ temperature)
#include <Adafruit_HTU21DF.h>   // Relative humidity sensor (w/ temperature)
#include <Adafruit_VL53L1X.h>   // LIDAR altimeter sensor (w/ temperature)
#include <Omron_D6FPH.h>        // Dynamic pressure sensor (w/ temperature)
#include <SimpleKalmanFilter.h> // Uni-dimensional Kalman filtering
#include <TinyGPSPlus.h>

/* Import custom */
// Nothing here!

//-----------------------------------------------------------------------------
// DEFINITIONS - Global constants
//-----------------------------------------------------------------------------
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
const int pin_RX0 = RXD;              // Pin 30
const int pin_TX0 = TXD;              // Pin 31
const int pin_RX1 = 12;
const int pin_TX1 = 13;
const int pin_LED2 = 11;
const int pin_SDCS = 23;              // CS1
//TODO remove most of airspeed calculation
const int pin_LED = LED_BUILTIN;      // Pin 13
const int pin_V1S = ADC7;             // Pin 22
const int pin_V2S = ADC6;             // Pin 19
const int pin_V3S = ADC3;             // Pin 26
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
const char splash[17] = "binguOS V2.0";
const int SDlograte_Hz = 4;
const int LCDcolumns = 16;
const int LCDrows = 1;
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
unsigned long tnext_LCDexpiry_ms;
unsigned long tnext_LED2expiry_ms;

/* Custom LCD characters */
byte char_alpha[8] = {
  B00000,
  B01001,
  B10101,
  B10010,
  B10010,
  B01101,
  B00000,
};
byte char_beta[8] = {
  B01100,
  B10010,
  B11110,
  B10001,
  B11110,
  B10000,
  B10000,
};

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
} data_q0, data_q1, data_q2;
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

/* Custom classes */
Adafruit_BMP280 bmp;
Adafruit_HTU21DF htu;
Adafruit_VL53L1X vl5;
File datalog;
SimpleKalmanFilter KF_D6Fq_Pa(D6F_mu, D6F_mu, D6F_pv);
SimpleKalmanFilter KF_VL5z_m(VL5_mu, VL5_mu, VL5_pv);
LiquidCrystal lcd(
  pin_LCD_RS, pin_LCD_EN, pin_LCD_D4,
  pin_LCD_D5, pin_LCD_D6, pin_LCD_D7
  );
Omron_D6FPH d6f;
TinyGPSPlus gps;

/* Function prototypes (needed if default arguments are given) */
// Nothing here yet!

//-----------------------------------------------------------------------------
// Setup - Run once
//-----------------------------------------------------------------------------
void setup() {

  // Start serial debug connection and LCD interface
  Serial.begin(115200);
  lcd.begin(LCDcolumns, LCDrows);
  lcd.createChar(0, char_alpha);
  lcd.createChar(1, char_beta);

  // Start a serial connection for the GPS module
  Serial1.begin(GPSBaud);

  // Setup digital I/O pins
  pinMode(pin_LED2, OUTPUT);
  pinMode(pin_S2, OUTPUT);
  pinMode(pin_S3, OUTPUT);
  pinMode(pin_LED, OUTPUT);
  pinMode(pin_V1S, INPUT);  // Strictly speaking, used for analog input
  pinMode(pin_V2S, INPUT);  // Strictly speaking, used for analog input
  pinMode(pin_V3S, INPUT);  // Strictly speaking, used for analog input
  pinMode(pin_S4, INPUT);

  // Setup SD card
  flag_SDready = SD.begin(pin_SDCS);

  // Setup I2C communication
  Wire.setSCL(pin_SCL0);
  Wire.setSDA(pin_SDA0);
  Wire.begin();
  Wire.setClock(400000);

  vl5begin();
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  digitalWrite(pin_S2, LOW);
  digitalWrite(pin_S3, LOW);
  d6f.begin(MODEL_0025AD1);
  digitalWrite(pin_S2, HIGH);
  d6f.begin(MODEL_0025AD1);
  digitalWrite(pin_S3, HIGH);
  d6f.begin(MODEL_0025AD1);
  htu.begin();

  // Save cartesian coordinates of pitot positions from polar coordinate setup
  calcPitotpositions(data_q0, data_q1, data_q2);
}

//-----------------------------------------------------------------------------
// Loop - Repeat forever
//-----------------------------------------------------------------------------
void loop() {

  // Read system diagnostic information
  tlast_cyclestart_ms = millis();
  byte *scanner = i2cscan();
  float *battCells_V = getBatteryVoltage();

  // Read sensor data
  getGPSdata(data_GPS0);
  getLIDARdistance(data_s0);
  getPitotPressure(data_q0, 0);
  getPitotPressure(data_q1, 1);
  getPitotPressure(data_q2, 2);
  getRelativeHumidity(data_RH0);
  getStaticPressure(data_P0);
  // Run data processing routines
  float AoA_rad, AoS_rad, q_Pa;
  updateIconBattery(battCells_V);
  updateIconDatalogging();
  updateIconGPS(data_GPS0.satellites);
  updatePitotSolution(data_q0, data_q1, data_q2, AoA_rad, AoS_rad, q_Pa);
  updateSDreadwrite();

  // IMU calculations
  float pitch = NAN;
  float roll = NAN;
  float z_m = data_s0.distance_m;

  // Other calculations
  float Patm_Pa, Tatm_C, RH_percent;
  Patm_Pa = data_P0.P_Pa;
  Tatm_C = data_P0.Tpackage_C - 8.3; /* calibration guess lol*/
  RH_percent = data_RH0.RH_percent;
  
  float vias_mps = pow(2 * q_Pa / rhoSL_kgpm3, 0.5);
  float vcas_mps = vias_mps;
  float Pvapoursaturated_Pa = calcPvapoursaturated(Tatm_C);
  float Pvapour_Pa = Pvapoursaturated_Pa * (RH_percent / 100);
  float Pdryair_Pa = Patm_Pa - Pvapour_Pa;
  float Ratm_JpKpkg = Patm_Pa * R0_JpKpmol
    / (Pdryair_Pa * Wdryair_kgpmol + Pvapour_Pa * Wvapour_kgpmol);
  float cpatm_JpKpkg = R0_JpKpmol / 2 * (7 / Wdryair_kgpmol + 8 / Wvapour_kgpmol);
  float cvatm_JpKpkg = R0_JpKpmol / 2 * (5 / Wdryair_kgpmol + 6 / Wvapour_kgpmol);
  float gammaatm = cpatm_JpKpkg / cvatm_JpKpkg;
  float rhoatm_kgpm3 = Patm_Pa / Ratm_JpKpkg / (Tatm_C + 273.15);
  float speedofsound_mps = pow(gammaatm * Ratm_JpKpkg * (Tatm_C + 273.15), 0.5);
  float Machnumber = pow(2.0 / (gammaatm - 1.0) * (pow(1 + q_Pa / Patm_Pa, (gammaatm - 1.0) / gammaatm) - 1.0), 0.5);
  float vtas_mps = Machnumber * speedofsound_mps;

  // Write to the SD card if...
  if (
    (flag_SDready == true) && // ...the SD card is ready to write to
    (flag_S4flipflop == true) // ...the user wants to write to SD
    ) {

    // If enough time has passed since the last log, write data to SD
    if (tlast_SDwritten_ms + 1000 / SDlograte_Hz < tlast_cyclestart_ms) {

      // Log all of the data!
      datalog.print("System Uptime[ms] ");
      datalog.print(tlast_cyclestart_ms);
      datalog.print(",");
      datalog.print("System VBATTcell0[V] ");
      datalog.print(battCells_V[0]);
      datalog.print(",");
      datalog.print("System VBATTcell1[V] ");
      datalog.print(battCells_V[1]);
      datalog.print(",");
      datalog.print("System VBATTcell2[V] ");
      datalog.print(battCells_V[2]);
      datalog.print(",");

      datalog.print("Airdata rho[kg/m3] ");
      datalog.print(rhoatm_kgpm3);
      datalog.print(",");
      datalog.print("Airdata T[C] ");
      datalog.print(Tatm_C);
      datalog.print(",");
      datalog.print("Airdata AoA[deg] ");
      datalog.print(degrees(AoA_rad));
      datalog.print(",");
      datalog.print("Airdata AoS[deg] ");
      datalog.print(degrees(AoS_rad));
      datalog.print(",");
      datalog.print("Airdata VCAS[m/s] ");
      datalog.print(vcas_mps);
      datalog.print(",");
      datalog.print("Airdata VTAS[m/s] ");
      datalog.print(vtas_mps);
      datalog.print(",");

      datalog.print("BMP280_0 P[Pa] ");
      datalog.print(data_P0.P_Pa);
      datalog.print(",");
      datalog.print("BMP280_0 Tpackage[C] ");
      datalog.print(data_P0.Tpackage_C);
      datalog.print(",");
  
      datalog.print("HTU21DF_0 RH[%] ");
      datalog.print(data_RH0.RH_percent);
      datalog.print(",");
      datalog.print("HTU21DF_0 Tpackage[C] ");
      datalog.print(data_RH0.Tpackage_C);
      datalog.print(",");
  
      datalog.print("D6F-PH0025AD1_0 q[Pa] ");
      datalog.print(data_q0.q_Pa);
      datalog.print(",");
      datalog.print("D6F-PH0025AD1_0 Tpackage[C] ");
      datalog.print(data_q0.Tpackage_C);
      datalog.print(",");
      datalog.print("D6F-PH0025AD1_1 q[Pa] ");
      datalog.print(data_q1.q_Pa);
      datalog.print(",");
      datalog.print("D6F-PH0025AD1_1 Tpackage[C] ");
      datalog.print(data_q1.Tpackage_C);
      datalog.print(",");
      datalog.print("D6F-PH0025AD1_2 q[Pa] ");
      datalog.print(data_q2.q_Pa);
      datalog.print(",");
      datalog.print("D6F-PH0025AD1_2 Tpackage[C] ");
      datalog.print(data_q2.Tpackage_C);
      datalog.print(",");

      datalog.print("VL53L1X_0 s[m] ");
      datalog.print(data_s0.distance_m);
      datalog.print(",");
      
      datalog.print("GPS_0 satellites[-] ");
      datalog.print(data_GPS0.satellites);
      datalog.print(",");
      datalog.print("GPS_0 latitude[deg] ");
      datalog.print(data_GPS0.lat, 10);
      datalog.print(",");
      datalog.print("GPS_0 longitude[deg] ");
      datalog.print(data_GPS0.lng, 10);
      datalog.print(",");
      datalog.print("GPS_0 year[-] ");
      datalog.print(data_GPS0.year);
      datalog.print(",");
      datalog.print("GPS_0 month[-] ");
      datalog.print(data_GPS0.month);
      datalog.print(",");
      datalog.print("GPS_0 day[-] ");
      datalog.print(data_GPS0.day);
      datalog.print(",");
      datalog.print("GPS_0 hours[-] ");
      datalog.print(data_GPS0.hour);
      datalog.print(",");
      datalog.print("GPS_0 minutes[-] ");
      datalog.print(data_GPS0.minute);
      datalog.print(",");
      datalog.print("GPS_0 seconds[-] ");
      datalog.print(data_GPS0.second);
      datalog.print(",");
      datalog.print("GPS_0 speed[m/s] ");
      datalog.print(data_GPS0.speed, 2);
      datalog.print(",");
      datalog.print("GPS_0 course[deg] ");
      datalog.print(data_GPS0.course, 1);
      datalog.print(",");
 
      datalog.println(";");
    }

    // Every 5 seconds, flush any data to the SD card (improves SD lifetime)
    if (tlast_SDflushed_ms + 5000 < tlast_cyclestart_ms) {
      datalog.flush();
      smartDelay(100);
    }

    // Every 2 seconds, blink the green LED to indicate SD activity
    if (tnext_LED2expiry_ms < tlast_cyclestart_ms) {
      digitalWrite(pin_LED2, HIGH);
      smartDelay(25);
      digitalWrite(pin_LED2, LOW);
      tnext_LED2expiry_ms = tlast_cyclestart_ms + 2000;
    }
  }

  // If a message sent with high priority has expired, draw the homescreen
  if (tlast_cyclestart_ms > tnext_LCDexpiry_ms) {

    lcd.clear();

    // Display airspeed
    char displayvtas_mps[4];
    zeroPadFloat(displayvtas_mps, sizeof(displayvtas_mps), vtas_mps, 1);
    lcd.setCursor(0, 0);
    lcd.print("V");
    lcd.print(displayvtas_mps);

    // Display angle of attack
    char displayaoa_deg[3];
    zeroPadFloat(displayaoa_deg, sizeof(displayaoa_deg), degrees(AoA_rad), 1);
    lcd.setCursor(5, 0);
    lcd.write(byte(0)); // Alpha
    lcd.print(displayaoa_deg);

    // Display altitude
    char displayaltitude_m[4];
    zeroPadFloat(displayaltitude_m, sizeof(displayaltitude_m), z_m, 1);
    lcd.setCursor(9, 0);
    lcd.print("z");
    lcd.print(displayaltitude_m);

    // Draw extra icons
    lcd.setCursor(13, 0);
    lcd.write(byte(4)); // GPS signal
    lcd.write(byte(3)); // SD card
    lcd.write(byte(2)); // Battery
  }

  // DEBUGGING PRINTS GO BELOW THIS POINT

  Serial.print("Satellites: ");
  Serial.print(data_GPS0.satellites);
  Serial.print(" Latitude: ");
  Serial.print(data_GPS0.lat, 10);
  Serial.print(" Longitude: ");
  Serial.print(data_GPS0.lng, 10);
  Serial.print(" Seconds: ");
  Serial.println(data_GPS0.second);
}

//-----------------------------------------------------------------------------
// Support functions
//-----------------------------------------------------------------------------

byte * i2cscan(void)
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


void calcPitotpositions(sensordata_q &sd0, sensordata_q &sd1, sensordata_q &sd2)
{
  // For a unit-length pitot arm, the distance of tip from origin is the bloom height
  float bloomheight = sin(d6f_halfangle_rad);
  sd0.offsetY_rad = asin(bloomheight * sin((2 * pi * 0 / 3) + d6f_rotoffset_rad));
  sd0.offsetX_rad = asin(bloomheight * cos((2 * pi * 0 / 3) + d6f_rotoffset_rad));
  sd1.offsetY_rad = asin(bloomheight * sin((2 * pi * 1 / 3) + d6f_rotoffset_rad));
  sd1.offsetX_rad = asin(bloomheight * cos((2 * pi * 1 / 3) + d6f_rotoffset_rad));
  sd2.offsetY_rad = asin(bloomheight * sin((2 * pi * 2 / 3) + d6f_rotoffset_rad));
  sd2.offsetX_rad = asin(bloomheight * cos((2 * pi * 2 / 3) + d6f_rotoffset_rad));
}


float calcPvapoursaturated(float T_C)
{
  // Use Teten's equation to estimate the saturation pressure of vapour
  float varA = 17.27, varB = 237.3;
  if (T_C > 35) {
    Serial.println("calcPvapoursaturated not modelled for use in T_C > 35!");
  }
  else if (T_C < 0) {
    // Murray estimation for conditions below zero
    varA = 21.875;
    varB = 265.5;
  }
  return 610.78 * pow(e, varA * T_C / (varB + T_C));  
}


float * getBatteryVoltage(void)
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


void getPitotPressure(sensordata_q &sd, int choice012)
{
  // Set the default choice of D6F sensor to read from
  digitalWrite(pin_S2, LOW);
  digitalWrite(pin_S3, LOW);

  // If 1 or 2 is selected in the input, change observed sensor
  switch (choice012) {
    case 0:
      break;
    case 1:
      digitalWrite(pin_S2, HIGH);
      break;
    case 2:
      digitalWrite(pin_S2, HIGH);
      digitalWrite(pin_S3, HIGH);
      break;
    default:
      Serial.println("Valid choices for getPitotPressure choice012 = [0, 1, 2]");
      break;
  }
  smartDelay(2);  // Delay between switching GPIO and 

  // Read the selected sensor
  if (d6f.isConnected()) {
    sd.q_Pa = max(0, d6f.getPressure()); // Ignore negative pressures
    sd.Tpackage_C = d6f.getTemperature();
  }
  else {
    sd.q_Pa = NAN;
    sd.Tpackage_C = NAN;
  }
  smartDelay(2);

  // Set all the pins back to low for consistency
  digitalWrite(pin_S2, LOW);
  digitalWrite(pin_S3, LOW);
}


void getRelativeHumidity(sensordata_RH &sd)
{
  // Save humidity and temperature data
  sd.RH_percent = htu.readHumidity();
  sd.Tpackage_C = htu.readTemperature();
}


void getS4FlipFlop(void)
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


void getStaticPressure(sensordata_P &sd)
{
  float pressure = bmp.readPressure();
  float temperature = bmp.readTemperature();
  if ((pressure > 0) && (temperature < 179)) {
    // Save pressure and temperature data
    sd.P_Pa = pressure;
    sd.Tpackage_C = temperature;
  }
  else {
    sd.P_Pa = NAN;
    sd.Tpackage_C = NAN;
  }
}


void setDisplayExpiry(unsigned long t_ms)
{
  // Set a system time after which the homescreen may resume
  tnext_LCDexpiry_ms = millis() + t_ms;
}


// This custom version of delay() ensures that the gps object
// is being "fed". Originally written by Mikal Hart
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}


void updateIconBattery(float battCells_V[])
{
    byte char_battery[8];
    // Empty and full icons
    byte char_battery0[8] = {
      B01110,
      B11011,
      B10001,
      B10001,
      B10001,
      B10001,
      B10001,
    };
    byte char_battery1[8] = {
      B01110,
      B11111,
      B11111,
      B11111,
      B11111,
      B11111,
      B11111,
    };

  // Found some random equation online for 3S battery voltage vs capacity
  auto fbatt3s_percent = [](float batt_V) { return (batt_V - 10.604848) / 0.019036; };
  float batteryvoltage_V = battCells_V[0] + battCells_V[1] + battCells_V[2];
  float battery_percent = fbatt3s_percent(batteryvoltage_V);

  // Draw a pretty icon, starting with the empty version
  // Note: char_x's size is 8 (7 element array + terminator)
  // Note: sizeof(char_x)-n is the number of rows to be modified
  int batterylevels = 6;
  for (unsigned int i = 0; i < sizeof(char_battery)-1; ++i) {
    char_battery[i] = char_battery0[i];
  }
  for (int i = 0; i < batterylevels; ++i) {
    if (battery_percent >= 100 - (i + 1) * 75 / batterylevels) {
      char_battery[i+1] = char_battery1[i+1];
    }
  }    
  lcd.createChar(2, char_battery);
}

void updateIconDatalogging(void)
{
  byte char_datalogging[8];
  // Inactive and active icons
  byte char_datalogging0[8] = {
    B11110,
    B11111,
    B01110,
    B10101,
    B11011,
    B10101,
    B01110,
  };

  byte char_datalogging1[8] = {
    B11110,
    B11111,
    B11110,
    B11111,
    B11111,
    B10001,
    B11111,
  };
  
  // Draw a pretty icon, starting with the empty version
  // Note: char_x's size is 8 (7 element array + terminator)
  // Note: sizeof(char_x)-n is the number of rows to be modified
  for (unsigned int i = 0; i < sizeof(char_datalogging)-1; ++i) {
    if (flag_SDready == false) {
      char_datalogging[i] = char_datalogging0[i];
    }
    else {
      char_datalogging[i] = char_datalogging1[i];
    }
  }
  lcd.createChar(3, char_datalogging);
}


void updateIconGPS(int satellites)
{
  byte char_GPSconnection[8];
  byte char_GPSconnection0[8] = {
    B00001,
    B00011,
    B00101,
    B01001,
    B10001,
    B10001,
    B10001,
  };
  byte char_GPSconnection1[8] = {
    B00001,
    B00011,
    B00101,
    B01001,
    B11001,
    B11001,
    B11001,
  };
  byte char_GPSconnection2[8] = {
    B00001,
    B00011,
    B00101,
    B01101,
    B11101,
    B11101,
    B11101,
  }; 
  byte char_GPSconnection3[8] = {
    B00001,
    B00011,
    B00111,
    B01111,
    B11111,
    B11111,
    B11111,
  };
  for (unsigned int i = 0; i < sizeof(char_GPSconnection)-1; ++i) {
    switch (satellites) {
    case -1:
      char_GPSconnection[i] = char_GPSconnection0[i];
      break;
    case 0:
      char_GPSconnection[i] = char_GPSconnection0[i];
      break;
    case 1:
      char_GPSconnection[i] = char_GPSconnection1[i];
      break;
    case 2:
      char_GPSconnection[i] = char_GPSconnection1[i];
      break;
    case 3:
      char_GPSconnection[i] = char_GPSconnection2[i];
      break;
    default:
      char_GPSconnection[i] = char_GPSconnection3[i];
      break;
    }
  }
  lcd.createChar(4, char_GPSconnection);
}


void updatePitotSolution(sensordata_q sd0, sensordata_q sd1, sensordata_q sd2, float &AoA_rad, float &AoS_rad, float &qtrue_Pa)
{
  // Return a qtrue_Pa value if any of the 3 sensors cannot be read
  float Sigmaq_Pa = 0;
  int nsdvalid = 0;
  if (!isnan(sd0.q_Pa)) {
    Sigmaq_Pa += KF_D6Fq_Pa.updateEstimate(sd0.q_Pa);
    ++nsdvalid;
  }
  if (!isnan(sd1.q_Pa)) {
    Sigmaq_Pa += KF_D6Fq_Pa.updateEstimate(sd1.q_Pa);
    ++nsdvalid;
  }
  if (!isnan(sd2.q_Pa)) {
    Sigmaq_Pa += KF_D6Fq_Pa.updateEstimate(sd2.q_Pa);
    ++nsdvalid;
  }
  if (nsdvalid == 0) {
    AoA_rad = NAN;
    AoS_rad = NAN;
    qtrue_Pa = NAN;
    return;
  }
  else if (nsdvalid < 3) {
    AoA_rad = NAN;
    AoS_rad = NAN;
    qtrue_Pa = Sigmaq_Pa / nsdvalid / cos(d6f_halfangle_rad); // Average q
    return;
  }

  // Kalman filter the pressures and use the smoother estimates
  float est0_q_Pa = KF_D6Fq_Pa.updateEstimate(sd0.q_Pa);
  float est1_q_Pa = KF_D6Fq_Pa.updateEstimate(sd1.q_Pa);
  float est2_q_Pa = KF_D6Fq_Pa.updateEstimate(sd2.q_Pa);

  // Resolve dynamic pressures into vars defined in Yaseen's derivation
  float varA = est0_q_Pa * cos(sd1.offsetY_rad) * cos(sd1.offsetX_rad);
  varA = varA - est1_q_Pa * cos(sd0.offsetY_rad) * cos(sd0.offsetX_rad);
  
  float varB = est1_q_Pa * cos(sd0.offsetY_rad) * sin(sd0.offsetX_rad);
  varB = varB - est0_q_Pa * cos(sd1.offsetY_rad) * sin(sd1.offsetX_rad);
  
  float varC = est1_q_Pa * sin(sd0.offsetY_rad) * cos(sd0.offsetX_rad);
  varC = varC - est0_q_Pa * sin(sd1.offsetY_rad) * cos(sd1.offsetX_rad);
  
  float varD = est1_q_Pa * sin(sd0.offsetY_rad) * sin(sd0.offsetX_rad);
  varD = varD - est0_q_Pa * sin(sd1.offsetY_rad) * sin(sd1.offsetX_rad);
  
  float varE = est0_q_Pa * cos(sd2.offsetY_rad) * cos(sd2.offsetX_rad);
  varE = varE - est2_q_Pa * cos(sd0.offsetY_rad) * cos(sd0.offsetX_rad);
  
  float varF = est2_q_Pa * cos(sd0.offsetY_rad) * sin(sd0.offsetX_rad);
  varF = varF - est0_q_Pa * cos(sd2.offsetY_rad) * sin(sd2.offsetX_rad);

  float varG = est2_q_Pa * sin(sd0.offsetY_rad) * cos(sd0.offsetX_rad);
  varG = varG - est0_q_Pa * sin(sd2.offsetY_rad) * cos(sd2.offsetX_rad);
  
  float varH = est2_q_Pa * sin(sd0.offsetY_rad) * sin(sd0.offsetX_rad);
  varH = varH - est0_q_Pa * sin(sd2.offsetY_rad) * sin(sd2.offsetX_rad);

  // Solve quadratic formula for +- solutions to Angle of Sideslip
  float qA = varD * varF - varB * varH;
  float qB = varA * varH + varB * varG - varC * varF - varD * varE;
  float qC = varC * varE - varA * varG;
  float AoS1_rad = atan(-qB + pow(pow(qB, 2)- 4 * qA * qC, 0.5)) / (2 * qA);
  float AoS2_rad = atan(-qB - pow(pow(qB, 2)- 4 * qA * qC, 0.5)) / (2 * qA);

  // Angle of Sideslip solution is that with the smaller magnitude
  if (abs(AoS1_rad) < abs(AoS2_rad))
    AoS_rad = AoS1_rad;
  else
    AoS_rad = AoS2_rad;

  // Angle of Attack solution is easily solved for
  AoA_rad = atan((varA - tan(AoS_rad) * varB) / (varC - tan(AoS_rad) * varD));

  // True dynamic pressure is recovered by correcting for AoA/AoS/offset errors
  qtrue_Pa = est0_q_Pa;
  qtrue_Pa *= 1 / cos(AoA_rad + sd0.offsetY_rad);
  qtrue_Pa *= 1 / cos(AoS_rad + sd0.offsetX_rad);
}


void updateSDreadwrite(void)
{
  // If S4's flipflop state is false, close any files and leave the function
  getS4FlipFlop();
  if (!flag_S4flipflop) {
    if (strlen(datalog.name()) > 0) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(">");
      lcd.print(datalog.name());
      datalog.close();
      setDisplayExpiry(3000);
    }
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


void zeroPadFloat(char* targetarray, int targetarraysize, float float2pad, int dpposition)
{
  char absvaluestring[targetarraysize];
  int idx_valuestart = 0, leadingdigits = 0;

  if (float2pad < 0) {
    targetarray[0] = '-';
    idx_valuestart = 1;
  }

  // If no. of leading digits exceeds available chars, return asterisks
  if (float2pad > 0) {
    leadingdigits = (int)log10(float2pad) + 1;
  }
  else if (float2pad < 0) {
    leadingdigits = (int)log10(-float2pad) + 1;
    // If -1 < number < 0, don't need to display leading zero
    if (float2pad > -1) {
      dpposition = leadingdigits;
    }
    else {
      dpposition = leadingdigits + 1;
    }
    
  }
  else {
    dpposition = targetarraysize - 1;
  }
  if (leadingdigits > targetarraysize-1) {
    for (int i = 0; i < (targetarraysize-1); ++i) {
      targetarray[i] = '*';
    }
  }
  // Else write placeholder zeros and then populate with the float value
  else {
    int npads = max(0, min(dpposition, targetarraysize-1) - leadingdigits);
    // DO NOT USE abs() HERE - I know pow looks ugly, but it stops -0.0 appearing
    snprintf(absvaluestring, sizeof(absvaluestring), "%f", pow(pow(float2pad, 2), 0.5));
    for (int i = idx_valuestart; i < (targetarraysize-1); ++i) {
      if (i < npads) {
        targetarray[i] = '0';
      }
      else {
        targetarray[i] = absvaluestring[i-npads];
      }
    }
  }
  // Null terminate the string (final character of the array)
  targetarray[targetarraysize-1] = '\0';
}

void vl5begin(void)
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
