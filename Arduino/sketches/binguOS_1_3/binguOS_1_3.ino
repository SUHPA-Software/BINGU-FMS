/* binguOS, written by Yaseen Reza 17/06/2022 */
//-----------------------------------------------------------------------------
// DEPENDENCIES
//-----------------------------------------------------------------------------
/* Import built-ins */
#include <LiquidCrystal.h>      // For the LCD display
#include <SD.h>                 // For R/W operations
#include <SoftwareSerial.h>     // For UART communication with the GPS
#include <Wire.h>               // For I2C enabled devices

/* Import downloaded */
#include <Adafruit_BMP280.h>    // Static pressure sensor (w/ temperature)
#include <Adafruit_HTU21DF.h>   // Relative humidity sensor (w/ temperature)
#include <Adafruit_VL53L1X.h>   // LIDAR altimeter sensor (w/ temperature)
#include <MPU6050_6Axis_MotionApps20.h>            // 6 DoF Acc/gyro sensor (w/ temperature)
#include <Omron_D6FPH.h>        // Dynamic pressure sensor (w/ temperature)
#include <SimpleKalmanFilter.h> // Uni-dimensional Kalman filtering
#include <TinyGPSPlus.h>        // Global Positioning System

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

/* Arduino pin definitions (BINGU Schematic v0.2.pdf) */
const int pin_RX1 = 0;
const int pin_TX1 = 1;
const int pin_LED2 = 5;
const int pin_SDCS = BUILTIN_SDCARD;  // Pin 10
const int pin_S2 = 11;
const int pin_S3 = 12;
const int pin_LED = LED_BUILTIN;      // Pin 13
const int pin_V1S = A0;               // Pin 14
const int pin_V2S = A1;               // Pin 15
const int pin_V3S = A2;               // Pin 16
const int pin_SDA0 = SDA;             // Pin 18
const int pin_SCL0 = SCL;             // Pin 19
const int pin_LCD_RS = 22;
const int pin_LCD_EN = 23;
const int pin_S4 = 33;
const int pin_LCD_D4 = 38;
const int pin_LCD_D5 = 39;
const int pin_LCD_D6 = 40;
const int pin_LCD_D7 = 41;

/* BINGU hardware definitions (BINGU Schematic v0.2.pdf) */
const float R1_MOhm = 0.33;
const float R2_MOhm = 1.00;
const float R3_MOhm = 0.33;
const float R4_MOhm = 0.56;
const float R5_MOhm = 0.33;
const float R6_MOhm = 0.12;

/* Other hardware definitions */
constexpr float d6f_halfangle_rad radians(6);
constexpr float d6f_rotoffset_rad radians(60);

/* BinguOS software definitions */
const char splash[17] = "binguOS V1.2";
const int SDlograte_Hz = 4;
const int LCDcolumns = 16;
const int LCDrows = 1;
const int GPSbaudrate = 4800;
const float D6F_mu = 3; // Measurement uncertainty (and seed for estimation uncertainty)
const float D6F_pv = 0.8;  // Process Variance 0.001~1, how fast the measurement moves 

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

/* Custom classes */
Adafruit_BMP280 bmp;
Adafruit_HTU21DF htu;
Adafruit_VL53L1X vl5;
File datalog;
SimpleKalmanFilter KF_D6Fq_Pa(D6F_mu, D6F_mu, D6F_pv);
LiquidCrystal lcd(
  pin_LCD_RS, pin_LCD_EN, pin_LCD_D4,
  pin_LCD_D5, pin_LCD_D6, pin_LCD_D7
  );
MPU6050 mpu;
Omron_D6FPH d6f;
SoftwareSerial ss(pin_RX1, pin_TX1);
TinyGPSPlus gps;

/* Function prototypes (needed if default arguments are given) */
// Nothing here yet!

/* 6DoF sensor stuffs */
#include "I2Cdev.h"
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//-----------------------------------------------------------------------------
// Setup - Run once
//-----------------------------------------------------------------------------
void setup() {

  // Start serial debug connection and LCD interface
  Serial.begin(115200);
  lcd.begin(LCDcolumns, LCDrows);
  lcd.createChar(0, char_alpha);
  lcd.createChar(1, char_beta);

  // Setup GPS up with a smart timer for the splash screen
  ss.begin(GPSbaudrate);

  // Setup digital I/O pins
  pinMode(pin_LED2, OUTPUT);
  pinMode(pin_S2, OUTPUT);
  pinMode(pin_S3, OUTPUT);
  pinMode(pin_LED, OUTPUT);
  pinMode(pin_V1S, INPUT);
  pinMode(pin_V2S, INPUT);
  pinMode(pin_V3S, INPUT);
  pinMode(pin_S4, INPUT);

  // Setup SD card
  flag_SDready = SD.begin(pin_SDCS);

  // Setup I2C communication
  Wire.setSCL(pin_SCL0);
  Wire.setSDA(pin_SDA0);
  Wire.begin();

  vl5begin();
  bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  d6f.begin(MODEL_0025AD1);
  htu.begin();
  beginMPU();


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
  updatePitotSolution(data_q0, data_q1, data_q2, AoA_rad, AoS_rad, q_Pa);
  updateSDreadwrite();

  // IMU calculations
  float pitch, roll;
  float z_m = 0;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    pitch = -asin(-2.0*(q.x*q.z - q.w*q.y));
    roll = -atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    Serial.print("euler\t");
    Serial.print(degrees(pitch));
    Serial.print("\t");
    Serial.println(degrees(roll));
    z_m = data_s0.distance_m * cos(pitch) * cos(roll);
  }
  else {
    pitch = NAN;
    roll = NAN;
    z_m = NAN;
  }

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
      datalog.print(", ");
      datalog.print("BMP280_0 Tpackage[C] ");
      datalog.print(data_P0.Tpackage_C);
      datalog.print(", ");
  
      datalog.print("HTU21DF_0 RH[%] ");
      datalog.print(data_RH0.RH_percent);
      datalog.print(", ");
      datalog.print("HTU21DF_0 Tpackage[C] ");
      datalog.print(data_RH0.Tpackage_C);
      datalog.print(", ");
  
      datalog.print("D6F-PH0025AD1_0 q[Pa] ");
      datalog.print(data_q0.q_Pa);
      datalog.print(", ");
      datalog.print("D6F-PH0025AD1_0 Tpackage[C] ");
      datalog.print(data_q0.Tpackage_C);
      datalog.print(", ");
      datalog.print("D6F-PH0025AD1_1 q[Pa] ");
      datalog.print(data_q1.q_Pa);
      datalog.print(", ");
      datalog.print("D6F-PH0025AD1_1 Tpackage[C] ");
      datalog.print(data_q1.Tpackage_C);
      datalog.print(", ");
      datalog.print("D6F-PH0025AD1_2 q[Pa] ");
      datalog.print(data_q2.q_Pa);
      datalog.print(", ");
      datalog.print("D6F-PH0025AD1_2 Tpackage[C] ");
      datalog.print(data_q2.Tpackage_C);
      datalog.print(", ");

      datalog.print("MPU6050_0 pitch[deg] ");
      datalog.print(degrees(pitch));
      datalog.print(", ");
      datalog.print("MPU6050_0 roll[deg] ");
      datalog.print(degrees(roll));
      datalog.print(", ");

      datalog.print("VL53L1X_0 s[m] ");
      datalog.print(data_s0.distance_m);
      datalog.print(", ");
      datalog.println(";");
    }

    // Every 10 seconds, flush any data to the SD card (improves SD lifetime)
    if (tlast_SDflushed_ms + 10000 < tlast_cyclestart_ms) {
      datalog.flush();
    }

    // Every 4 seconds, blink the green LED to indicate SD activity
    if (tnext_LED2expiry_ms < tlast_cyclestart_ms) {
      digitalWrite(pin_LED2, HIGH);
      delay(25);
      digitalWrite(pin_LED2, LOW);
      tnext_LED2expiry_ms = tlast_cyclestart_ms + 4000;
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
    char displayaoa_deg[4];
    zeroPadFloat(displayaoa_deg, sizeof(displayaoa_deg), degrees(AoA_rad), 1);
    lcd.setCursor(5, 0);
    lcd.write(byte(0)); // Alpha
    lcd.print(displayaoa_deg);

    // Display altitude
    char displayaltitude_m[4];
    zeroPadFloat(displayaltitude_m, sizeof(displayaltitude_m), z_m, 1);
    lcd.setCursor(10, 0);
    lcd.print("z");
    lcd.print(displayaltitude_m);

    // Draw extra icons
    lcd.setCursor(14, 0);
    lcd.write(byte(3)); // SD card
    lcd.write(byte(1)); // Battery
  }
  
  delay(50);
}

//-----------------------------------------------------------------------------
// Support functions
//-----------------------------------------------------------------------------

void beginMPU(void)
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

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
  float battProbepoint3_V = vsense3_V * (R1_MOhm + R2_MOhm) / R1_MOhm;
  float battProbepoint2_V = vsense2_V * (R3_MOhm + R4_MOhm) / R3_MOhm;
  float battProbepoint1_V = vsense1_V * (R5_MOhm + R6_MOhm) / R5_MOhm;

  // Solve for voltage across each cell
  battCells_V[0] = battProbepoint1_V;
  battCells_V[1] = battProbepoint2_V - battProbepoint1_V;
  battCells_V[2] = battProbepoint3_V - battProbepoint2_V;

  return battCells_V;
}


void getLIDARdistance(sensordata_s &sd)
{
  if (flag_VL53L1Xready) {
    int distance = vl5.distance();
    sd.distance_m = (float)distance / 1000;
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

  // Read the selected sensor
  if (d6f.isConnected()) {
    sd.q_Pa = max(0, d6f.getPressure()); // Ignore negative pressures
    sd.Tpackage_C = d6f.getTemperature();
  }
  else {
    sd.q_Pa = NAN;
    sd.Tpackage_C = NAN;
  }

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
  for (unsigned int i = 0; i < sizeof(char_battery)-1; ++i) {
    char_battery[i] = char_battery0[i];
  }
  for (int i = 0; i < 6; ++i) {
    if (battery_percent >= 100 - (i + 1) * 75 / 6) {
      char_battery[i+1] = char_battery1[i+1];
    }
  }    
  lcd.createChar(1, char_battery);
}

void updateIconDatalogging(void) {
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
  double varA = est0_q_Pa * cos(sd1.offsetY_rad) * cos(sd1.offsetX_rad);
  varA = varA - est1_q_Pa * cos(sd0.offsetY_rad) * cos(sd0.offsetX_rad);
  
  double varB = est1_q_Pa * cos(sd0.offsetY_rad) * sin(sd0.offsetX_rad);
  varB = varB - est0_q_Pa * cos(sd1.offsetY_rad) * sin(sd1.offsetX_rad);
  
  double varC = est1_q_Pa * sin(sd0.offsetY_rad) * cos(sd0.offsetX_rad);
  varC = varC - est0_q_Pa * sin(sd1.offsetY_rad) * cos(sd1.offsetX_rad);
  
  double varD = est1_q_Pa * sin(sd0.offsetY_rad) * sin(sd0.offsetX_rad);
  varD = varD - est0_q_Pa * sin(sd1.offsetY_rad) * sin(sd1.offsetX_rad);
  
  double varE = est0_q_Pa * cos(sd2.offsetY_rad) * cos(sd2.offsetX_rad);
  varE = varE - est2_q_Pa * cos(sd0.offsetY_rad) * cos(sd0.offsetX_rad);
  
  double varF = est2_q_Pa * cos(sd0.offsetY_rad) * sin(sd0.offsetX_rad);
  varF = varF - est0_q_Pa * cos(sd2.offsetY_rad) * sin(sd2.offsetX_rad);

  double varG = est2_q_Pa * sin(sd0.offsetY_rad) * cos(sd0.offsetX_rad);
  varG = varG - est0_q_Pa * sin(sd2.offsetY_rad) * cos(sd2.offsetX_rad);
  
  double varH = est2_q_Pa * sin(sd0.offsetY_rad) * sin(sd0.offsetX_rad);
  varH = varH - est0_q_Pa * sin(sd2.offsetY_rad) * sin(sd2.offsetX_rad);

  // Solve quadratic formula for +- solutions to Angle of Sideslip
  double qA = varD * varF - varB * varH;
  double qB = varA * varH + varB * varG - varC * varF - varD * varE;
  double qC = varC * varE - varA * varG;
  double AoS1_rad = atan(-qB + pow(pow(qB, 2)- 4 * qA * qC, 0.5)) / (2 * qA);
  double AoS2_rad = atan(-qB - pow(pow(qB, 2)- 4 * qA * qC, 0.5)) / (2 * qA);

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
