/*  binguOS, written by Yaseen Reza 13/06/2022  */
#include <SD.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_VL53L1X.h> 
#include <LiquidCrystal.h>
#include <Omron_D6FPH.h>
#include <TinyGPSPlus.h>

// Pin Definitions
const int SERIAL1_RX = 0;
const int SERIAL1_TX = 1;
const int LED2 = 5;
const int SD_CHIPSELECT = BUILTIN_SDCARD;
const int SW2_D6F = 11;
const int SW3_D6F = 12;
const int VSENSE1 = A0;
const int VSENSE2 = A1;
const int VSENSE3 = A2;
const int I2C_SCL = SCL;
const int I2C_SDA = SDA;
const int LCD_RS = 22;
const int LCD_EN = 23;
const int SW4_PUSH = 33;
const int LCD_D4 = 38;
const int LCD_D5 = 39;
const int LCD_D6 = 40;
const int LCD_D7 = 41;

// Science constants
const float e = 2.71828;
const float pi = 3.14159;
const float R0_JpKpmol = 8.31446;
const float rhoSL_kgpm3 = 1.225;
const float Wvapour_kgpmol = 0.01802;
const float Wdryair_kgpmol = 0.02897;

// Teensy & BINGU motherboard
const char _splash[17] = "binguOS V1.1";
char filename[32];
unsigned long LED2_expirytime_ms = 0;
bool lowbattflag = false;
const long RST1_Ohm = 330000;
const long RST2_Ohm = 1000000;
const long RST3_Ohm = 330000;
const long RST4_Ohm = 560000;
const long RST5_Ohm = 330000;
const long RST6_Ohm = 120000;
unsigned long SD_loginterval_ms = 200;
unsigned long SD_lastlogtime_ms;
bool SD_ready = false;
bool SW4_fallingedge = false;
bool SW4_toggle = false;

// BMP280 Barometric pressure sensor
struct BMPsensor {
  float P_Pa;
  float T_C;
} bmpsensor0;

// D6F-PH0025AD1 dynamic pressure sensing
#define D6F_ADDR 0x6C  // D6F-PH I2C client address at 7bit expression
int d6favailable = 0;
constexpr float d6f_halfangle radians(6);
constexpr float d6f_rotoffset radians(60);
struct D6Fsensor {
  double q_Pa;
  float T_C;
  double theta_rad;
  double psi_rad;
} d6fsensor0, d6fsensor1, d6fsensor2;

// NEO-6m GPS
const int GPS_BAUD = 9600;

// HD44780 LCD
const int LCD_COLS = 16;
const int LCD_ROWS = 1;  // Only using 1 row gives much higher display contrast
unsigned long LCD_msgexpirytime_ms;
char LCD_textline1[17];
byte char_alpha[8] = {
  B00000,
  B01001,
  B10101,
  B10010,
  B10010,
  B01101,
  B00000,
};
byte char_battery[8] = {
  B01110,
  B11011,
  B10001,
  B10001,
  B10001,
  B10001,
  B10001,
};

// HTU-21DF Humidity sensor
struct HTUsensor {
  float RH;
  float T_C;
} htusensor0;

// MPU-6050 Accelerometer and gyroscope
float MPU_offsets_gyro[3];
struct MPUsensor {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float T_C;
} mpusensor0;

// Object Assignment
Adafruit_BMP280 bmp;
Adafruit_BNO055 bno;
Adafruit_HTU21DF htu;
Adafruit_VL53L1X vl5;
Adafruit_MPU6050 mpu;
File datalog;
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
Omron_D6FPH d6f;
SoftwareSerial ss(SERIAL1_RX, SERIAL1_TX);
TinyGPSPlus gps;

// Prototyping
void beginBMP(void);
void beginMPU(void);
void printLCD(const char *str, int skiprows=0);
void setLCDexpiry(unsigned long t_ms=3000);
void updateBMP(struct BMPsensor *sensor);
void updateD6F(struct D6Fsensor *sensor0, struct D6Fsensor *sensor1, struct D6Fsensor *sensor2);
void updateHTU(struct HTUsensor *sensor);
void updateMPU(struct MPUsensor *sensor);
void updateSW4(void);

//------------------------------------------------------------------------------
// Initialise attached devices
//------------------------------------------------------------------------------

void setup() {

  // Open serial communications and initialise LCD display
  Serial.begin(115200);
  lcd.createChar(0, char_alpha);
  lcd.begin(LCD_COLS, LCD_ROWS);
  printLCD(_splash);
  delay(2000);

  // Assign useful pins
  Serial.println("Assigning pin modes...");
  pinMode(LED2, OUTPUT);
  pinMode(SW2_D6F, OUTPUT);
  pinMode(SW3_D6F, OUTPUT);
  pinMode(SW4_PUSH, INPUT);

  Serial.println("\nInitialising serial peripheral interface:");
  printLCD("Sensors starting");

  // Initialise SD library and card
  if (SD.begin(SD_CHIPSELECT)) {
    printLCD("SD Card   [ OK ]");
    SD_ready = true;
  }
  else {
    printLCD("SD Card   [FAIL]");
    delay(1000);
  }

  Serial.println("\nInitialising software serial:");

  // Open a software serial link to the GPS module
  ss.begin(GPS_BAUD);
  while (!ss) {
    delay(1000); // wait for serial connection and gps stream.
  }
  if (gps.charsProcessed() > 10) {
    printLCD("NEO-6M GPS[ OK ]");
  }
  else {
    printLCD("NEO-6M GPS[FAIL]");
    delay(1000);
  }

  Serial.println("\nInitialising I2C devices:");
  
  // Set I2C communications pins (18 and 19 are default for wire1)
  Wire.setSCL(I2C_SCL);
  Wire.setSDA(I2C_SDA);
  Wire.begin();
  
  // Initialise barometric sensor
  beginBMP();

  // Initialise dynamic pressure sensors
  d6f.begin(MODEL_0025AD1);
  // Sensor 0
  digitalWrite(SW2_D6F, LOW);
  digitalWrite(SW3_D6F, LOW);
  if (d6f.isConnected()) {
    d6favailable = d6favailable + 1;
  }
  d6fsensor0.theta_rad = d6f_halfangle * sin((2 * pi * 0 / 3) + d6f_rotoffset);
  d6fsensor0.psi_rad = d6f_halfangle * cos((2 * pi * 0 / 3) + d6f_rotoffset);
  // Sensor 1
  digitalWrite(SW2_D6F, HIGH);
  if (d6f.isConnected()) {
    d6favailable = d6favailable + 1;
  }
  d6fsensor1.theta_rad = d6f_halfangle * sin((2 * pi * 1 / 3) + d6f_rotoffset);
  d6fsensor1.psi_rad = d6f_halfangle * cos((2 * pi * 1 / 3) + d6f_rotoffset);
  // Sensor 2
  digitalWrite(SW3_D6F, HIGH);
  if (d6f.isConnected()) {
    d6favailable = d6favailable + 1;
  }
  d6fsensor2.theta_rad = d6f_halfangle * sin((2 * pi * 2 / 3) + d6f_rotoffset);
  d6fsensor2.psi_rad = d6f_halfangle * cos((2 * pi * 2 / 3) + d6f_rotoffset);
  // Clean-up
  digitalWrite(SW2_D6F, LOW);
  digitalWrite(SW3_D6F, LOW);
  if (d6favailable == 3) {
    printLCD("D6F-PH0025[ OK ]");
  }
  else if ((0 < d6favailable) && (d6favailable < 3)) {
    printLCD("D6F-PH0025[WARN]");
    delay(1000);
  }
  else {
    printLCD("D6F-PH0025[FAIL]");
    delay(1000);
  }

  // Initialise humidity sensor
  if (htu.begin()) {
    printLCD("HTU21D-F  [ OK ]");
  }
  else {
    printLCD("HTU21D-F  [FAIL]");
    delay(1000);
  }

  // Initialise accelerometer/gyroscope
  beginMPU();

  /*
  // Initialise time of flight sensor
  if (vl5.begin(0x29, &Wire)) {
    printLCD("VL53L1X   [ OK ]");
  }
  else {
    printLCD("VL53L1X   [FAIL]");
    delay(1000);
  }*/

  Serial.print("\n");
  printLCD("STARTUP COMPLETE");
  delay(1000);
  Serial.print("\n");
}

//------------------------------------------------------------------------------
// Main Program Loop
//------------------------------------------------------------------------------

void loop() {

  // Try to initialise hotplugged sensors
  //-------------------------------------
  // Start BMP if missing
  if (bmp.getStatus() == 0) {
    beginBMP();
  }
  // Count D6F-PH0025AD1s if they are added
  d6favailable = 0;
  digitalWrite(SW2_D6F, LOW);
  digitalWrite(SW3_D6F, LOW);
  if (d6f.isConnected()) {
    d6favailable = d6favailable + 1;
  }
  digitalWrite(SW2_D6F, HIGH);
  if (d6f.isConnected()) {
    d6favailable = d6favailable + 1;
  }
  digitalWrite(SW3_D6F, HIGH);
  if (d6f.isConnected()) {
    d6favailable = d6favailable + 1;
  }
  digitalWrite(SW2_D6F, LOW);
  digitalWrite(SW3_D6F, LOW);

  // Update all peripherals
  //-----------------------
  updateBMP(&bmpsensor0);
  updateD6F(&d6fsensor0, &d6fsensor1, &d6fsensor2);
  updateHTU(&htusensor0);
  updateSW4();
  
  // Calculate battery stuffs
  //-------------------------
  // Find the voltage being sensed from the LiPo input
  float vsense1_V = 3.3 * analogRead(VSENSE1) / 1023;
  float vsense2_V = 3.3 * analogRead(VSENSE2) / 1023;
  float vsense3_V = 3.3 * analogRead(VSENSE3) / 1023;

  // Compute the original cell voltages
  float cell0_V = (RST1_Ohm + RST2_Ohm) / RST2_Ohm * vsense1_V;
  float cell1_V = (RST3_Ohm + RST4_Ohm) / RST4_Ohm * vsense2_V;
  float cell2_V = (RST5_Ohm + RST6_Ohm) / RST6_Ohm * vsense3_V;

  // If cells are all below 0.1 V, it's probably plugged into a computer and not a battery
  if ((cell0_V < 0.1) && (cell1_V < 0.1) && (cell2_V < 0.1)) {
    cell0_V = 4.2;
    cell1_V = 4.2;
    cell2_V = 4.2;
  }

  // Calculate battery percentage using some random eq. from online
  float batt_V = cell0_V + cell1_V + cell2_V;
  auto fbatt_percent = [](float batt_V) { return max(0, (batt_V - 10.604848) / 0.019036); };
  float batt_percent = fbatt_percent(batt_V);

  // If battery is about to die (any cells close to 20% charge), warn the user!
  if ((cell0_V <= 3.73) || (cell1_V <= 3.73) || (cell2_V <= 3.73) || (batt_percent < 25)) {
    printLCD("LOBATT, TURN OFF");
    while(1); // Make the program stuck on purpose
  }
  // Else if battery is below 35%
  else if ((batt_percent < 35) && (lowbattflag == false)) {
    printLCD("*LOBATT WARNING*");
    setLCDexpiry(20000);  // 20 seconds of warning
  }
  // Else if all the batteries are above 3.8 V, set lowbattflag to false
  else if ((batt_percent >= 40) && (lowbattflag == true)) {
    lowbattflag = false;
  }

  // Draw a pretty battery icon
  for (int i=1; i<sizeof(char_battery)-1; ++i) {
    if (batt_percent >= 100 - i * 75/(sizeof(char_battery)-2)) {
      char_battery[i] = B11111;
    }
    else if ((batt_percent < 90) && (i == 1)) {
      char_battery[i] = B11011;
    }
    else {
      char_battery[i] = B10001;
    }
  }    
  lcd.createChar(1, char_battery);

  // Calculate mathsy stuffs
  //------------------------
  // Assign values to air data quantities, using defaults if needed
  float Patm_Pa, RH_percent, T_C;
  // ATMOSPHERIC PRESSURE
  if (!isnan(bmpsensor0.P_Pa)) {
    Patm_Pa = bmpsensor0.P_Pa;
  }
  else {
    Patm_Pa = 101325.0; // Assign sea-level conditions
  }
  // ATMOSPHERIC RELATIVE HUMIDITY
  if (!isnan(htusensor0.RH)) {
    RH_percent = htusensor0.RH;
  }
  else {
    RH_percent = 40;  // Assign some humidity I chose
  }
  // ATMOSPHERIC TEMPERATURE
  if (!isnan(htusensor0.T_C)) {
    T_C = htusensor0.T_C;
  }
  else if (!isnan(bmpsensor0.T_C)) {
    T_C = bmpsensor0.T_C;
  }
  else if (!isnan(d6fsensor0.T_C)) {
    T_C = d6fsensor0.T_C;
  }
  else if (!isnan(d6fsensor1.T_C)) {
    T_C = d6fsensor1.T_C;
  }
  else if (!isnan(d6fsensor2.T_C)) {
    T_C = d6fsensor2.T_C;
  }
  else {
    T_C = 15.0; // Assign sea-level conditions
  }

  // Use Yaseen's derivation to find angle of attack/sideslip and dynamic pressure
  double alpha_rad = 0;
  double beta_rad = 0;
  double q_Pa = 0;
  // Check if all D6F-PH0025AD1s are available
  if (d6favailable == 3) {
    double yA = d6fsensor0.q_Pa * cos(d6fsensor1.theta_rad) * cos(d6fsensor1.psi_rad);
    yA = yA - d6fsensor1.q_Pa * cos(d6fsensor0.theta_rad) * cos(d6fsensor0.psi_rad);
    
    double yB = d6fsensor1.q_Pa * cos(d6fsensor0.theta_rad) * sin(d6fsensor0.psi_rad);
    yB = yB - d6fsensor0.q_Pa * cos(d6fsensor1.theta_rad) * sin(d6fsensor1.psi_rad);
    
    double yC = d6fsensor1.q_Pa * sin(d6fsensor0.theta_rad) * cos(d6fsensor0.psi_rad);
    yC = yC - d6fsensor0.q_Pa * sin(d6fsensor1.theta_rad) * cos(d6fsensor1.psi_rad);
    
    double yD = d6fsensor1.q_Pa * sin(d6fsensor0.theta_rad) * sin(d6fsensor0.psi_rad);
    yD = yD - d6fsensor0.q_Pa * sin(d6fsensor1.theta_rad) * sin(d6fsensor1.psi_rad);
    
    double yE = d6fsensor0.q_Pa * cos(d6fsensor2.theta_rad) * cos(d6fsensor2.psi_rad);
    yE = yE - d6fsensor2.q_Pa * cos(d6fsensor0.theta_rad) * cos(d6fsensor0.psi_rad);
    
    double yF = d6fsensor2.q_Pa * cos(d6fsensor0.theta_rad) * sin(d6fsensor0.psi_rad);
    yF = yF - d6fsensor0.q_Pa * cos(d6fsensor2.theta_rad) * sin(d6fsensor2.psi_rad);

    double yG = d6fsensor2.q_Pa * sin(d6fsensor0.theta_rad) * cos(d6fsensor0.psi_rad);
    yG = yG - d6fsensor0.q_Pa * sin(d6fsensor2.theta_rad) * cos(d6fsensor2.psi_rad);
    
    double yH = d6fsensor2.q_Pa * sin(d6fsensor0.theta_rad) * sin(d6fsensor0.psi_rad);
    yH = yH - d6fsensor0.q_Pa * sin(d6fsensor2.theta_rad) * sin(d6fsensor2.psi_rad);

    double quad_a = yD * yF - yB * yH;
    double quad_b = yA * yH + yB * yG - yC * yF - yD * yE;
    double quad_c = yC * yE - yA * yG;
    double beta1_rad = atan(-quad_b + pow(pow(quad_b, 2)- 4 * quad_a * quad_c, 0.5)) / (2 * quad_a);
    double beta2_rad = atan(-quad_b - pow(pow(quad_b, 2)- 4 * quad_a * quad_c, 0.5)) / (2 * quad_a);

    if (abs(beta1_rad) < abs(beta2_rad)) {
      beta_rad = beta1_rad;
    }
    else {
      beta_rad = beta2_rad;
    }
    alpha_rad = atan((yA - tan(beta_rad) * yB) / (yC - tan(beta_rad) * yD));
    q_Pa = d6fsensor0.q_Pa / cos(alpha_rad + d6fsensor0.theta_rad) / cos(beta_rad + d6fsensor0.psi_rad);
  } 
  // Else at least one D6F is available, take the average dynamic pressure
  else if ((0 < d6favailable) && (d6favailable < 3)) {
    double Sigmaq_Pa = 0;
    if (d6fsensor0.q_Pa > 0) {
      Sigmaq_Pa = Sigmaq_Pa + d6fsensor0.q_Pa;
    }
    if (d6fsensor1.q_Pa > 0) {
      Sigmaq_Pa = Sigmaq_Pa + d6fsensor1.q_Pa;
    }
    if (d6fsensor2.q_Pa > 0) {
      Sigmaq_Pa = Sigmaq_Pa + d6fsensor2.q_Pa;
    }
    q_Pa = Sigmaq_Pa / d6favailable / cos(d6f_halfangle);
  }
  
  // Use Bernoulli's principle to find the indicated airspeed (IAS)
  float vias_mps = pow(2 * q_Pa / rhoSL_kgpm3, 0.5);

  // Use a calibration mapping to find the calibrated airspeed (CAS)
  float vcas_mps = vias_mps;

  // Use Teten's Eq. to find the vapour saturation pressure
  float Pvapoursaturated_Pa = 610.78 * pow(e, 17.27 * T_C / (237.3 + T_C));

  // Work out the vapour partial pressure
  float Pvapour_Pa = Pvapoursaturated_Pa * (RH_percent / 100);

  // Use Dalton's law to work out the dry air partial pressure
  float Pdryair_Pa = Patm_Pa - Pvapour_Pa;

  // Use gas mixing theory to find the specific gas constant
  float Ratm_JpKpkg = Patm_Pa * R0_JpKpmol / (Pdryair_Pa * Wdryair_kgpmol + Pvapour_Pa * Wvapour_kgpmol);

  // Use gas mixing theory to find the specific heat ratio
  float cpatm_JpKpkg = R0_JpKpmol / 2 * (7 / Wdryair_kgpmol + 8 / Wvapour_kgpmol);
  float cvatm_JpKpkg = R0_JpKpmol / 2 * (5 / Wdryair_kgpmol + 6 / Wvapour_kgpmol);
  float gammaatm = cpatm_JpKpkg / cvatm_JpKpkg;

  // Use ideal gas equations to work out air density
  float rhoatm_kgpm3 = Patm_Pa / Ratm_JpKpkg / (T_C + 273.15);

  // Work out the speed of sound and Mach number
  float speedofsound_mps = pow(gammaatm * Ratm_JpKpkg * (T_C + 273.15), 0.5);
  float Machnumber = pow(2.0 / (gammaatm - 1.0) * (pow(1 + q_Pa / Patm_Pa, (gammaatm - 1.0) / gammaatm) - 1.0), 0.5);

  // Work out the true airspeed (TAS)
  float vtas_mps = Machnumber * speedofsound_mps;

  // Work out the equivalent airspeed (EAS)
  float veas_mps = vtas_mps / pow(rhoSL_kgpm3 / rhoatm_kgpm3, 0.5);

  // Check if the user wants to write to the SD card
  //------------------------------------------------
  // If SW4 has been toggled on and SD card is available
  if ((SW4_toggle == true) && (SD_ready == true)) {

    // If a file is currently not open, make one and open it
    if (strlen(datalog.name()) == 0) {

      // Make a new filename
      strncpy(filename, "bingulog000.csv", sizeof(filename));
      for(int i=0; i<1000; ++i) {
        filename[8] = '0' + (i / 100);
        filename[9] = '0' + ((i % 100) / 10);
        filename[10] = '0' + ((i % 100) % 10);
        // Create file if it does not exist (do not open existing)
        if (!SD.exists(filename)) {
          Serial.print("Writing to: ");
          Serial.println(filename);
          break;
        }
      }
   
      // Open the new file and commit to the datalog object
      datalog = SD.open(filename, FILE_WRITE);
      if (datalog) {
        printLCD("Starting datalog");
        setLCDexpiry();
      }
      else {
        printLCD("Can't open file!");
        setLCDexpiry();
        SD_ready = false;
      }
 
    }
    // Else a datalog object already exists, make sure the SD is still alive
    else if (!SD.exists(filename)) {
      // Stop SD writing activity
      SD_ready = false;
      datalog.close();
    }
 
  }
  // Else if SW4 has been toggled on and SD isn't ready, try to ready it
  else if ((SW4_toggle == true) && (SD_ready == false)) {
    if (SD.begin(SD_CHIPSELECT)) {
      printLCD("SD Card   [ OK ]");
      setLCDexpiry();
      SD_ready = true;
    }
    // SD couldn't be readied, stop checking for readiness
    else {
      printLCD("SD Card   [FAIL]");
      setLCDexpiry();
      SW4_toggle = false;
    }
  }
  // Else if SW4 has been toggled off and a file is open, time to close it
  else if ((SW4_toggle == false) && (strlen(datalog.name()) > 0)) {
    datalog.close();
    snprintf(LCD_textline1, sizeof(LCD_textline1), ">%s", filename);
    printLCD(LCD_textline1);
    setLCDexpiry(4000);
  }

  // The user wants to write to the SD card (and it is ready)
  //---------------------------------------------------------
  // The SD card has been confirmed writable, begin writing if log interval time has elapsed
  if ((SW4_toggle == true) && (SD_ready == true)) {
    if (millis() > SD_loginterval_ms + SD_lastlogtime_ms) {
 
      // Log all the data!
      datalog.print("SystemUptime [ms] ");
      datalog.print(millis());
      datalog.print(", ");
      datalog.print("BATTcell_0 [V] ");
      datalog.print(cell0_V);
      datalog.print(", ");
      datalog.print("BATTcell_1 [V] ");
      datalog.print(cell1_V);
      datalog.print(", ");
      datalog.print("BATTcell_2 [V] ");
      datalog.print(cell2_V);
      datalog.print(", ");
      datalog.flush();

      datalog.print("BMP280_0 P[Pa] ");
      datalog.print(bmpsensor0.T_C);
      datalog.print(", ");
      datalog.print("BMP280_0 T[C] ");
      datalog.print(bmpsensor0.T_C);
      datalog.print(", ");
      datalog.flush();

      datalog.print("HTU21DF_0 RH[%] ");
      datalog.print(htusensor0.RH);
      datalog.print(", ");
      datalog.print("HTU21DF_0 T[C] ");
      datalog.print(htusensor0.T_C);
      datalog.print(", ");
      datalog.flush();

      datalog.print("D6F-PH0025AD1_0 q[Pa] ");
      datalog.print(d6fsensor0.q_Pa);
      datalog.print(", ");
      datalog.print("D6F-PH0025AD1_0 T[C] ");
      datalog.print(d6fsensor0.T_C);
      datalog.print(", ");
      datalog.flush();

      datalog.print("D6F-PH0025AD1_1 q[Pa] ");
      datalog.print(d6fsensor1.q_Pa);
      datalog.print(", ");
      datalog.print("D6F-PH0025AD1_1 T[C] ");
      datalog.print(d6fsensor1.T_C);
      datalog.print(", ");
      datalog.flush();

      datalog.print("D6F-PH0025AD1_2 q[Pa] ");
      datalog.print(d6fsensor2.q_Pa);
      datalog.print(", ");
      datalog.print("D6F-PH0025AD1_2 T[C] ");
      datalog.print(d6fsensor2.T_C);
  
      // Finish with the SD card
      datalog.println(";");
      datalog.flush();
      SD_lastlogtime_ms = millis();
    }

    // While the SD card is logging, blink LED2
    if (millis() > LED2_expirytime_ms + 4000) {
      digitalWrite(LED2, HIGH);
      delay(25);
      digitalWrite(LED2, LOW);
      LED2_expirytime_ms = millis();
    }
  }

  // Update the Pilot's LCD
  //-----------------------
  // If all pop-up messages have expired, display the homescreen
  if (millis() > LCD_msgexpirytime_ms) {

    char str1[6];
    char str2[6];
    char str3[4];
 
    // Add true airspeed
    if (vtas_mps >= 10) {
      snprintf(str1, sizeof(str1), "V%.1f", vtas_mps);
    }
    else {
      snprintf(str1, sizeof(str1), "V0%.1f", vtas_mps);
    }
    // Add angle of attack
    if (degrees(alpha_rad) >= 10) {
      snprintf(str2, sizeof(str2), "A %.0f", degrees(alpha_rad));
    }
    else if (degrees(alpha_rad) >= 0) {
      snprintf(str2, sizeof(str2), "A %.1f", degrees(alpha_rad));
    }
    else if (degrees(alpha_rad) <= 10) {
      snprintf(str2, sizeof(str2), "A-%.0f", abs(degrees(alpha_rad)));
    }
    else {
      snprintf(str2, sizeof(str2), "A-%.1f", abs(degrees(alpha_rad)));
    }
    // Add humidity
    snprintf(str3, sizeof(str3), "H%.0f", min(htusensor0.RH, 99));
    
    snprintf(LCD_textline1, sizeof(LCD_textline1), "%s %s %s", str1, str2, str3);
    printLCD(LCD_textline1);
    lcd.setCursor(6, 0);
    lcd.write(byte(0));
    lcd.setCursor(15, 0);
    lcd.write(byte(1));
  }

  // Limit the clock cycle to a maximum possible refresh of 20 Hz
  delay(50);
}

//------------------------------------------------------------------------------
// Support Functions
//------------------------------------------------------------------------------

void beginBMP(void)
{
  if (bmp.begin()) {
    printLCD("BMP280    [ OK ]");
  }
  else if (bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
    printLCD("BMP280    [ OK ]");
  }
  else {
    printLCD("BMP280    [FAIL]");
    delay(1000);
    return;
  }
}

void beginMPU(void)
{
  if (mpu.begin()) {
    printLCD("MPU-6050  [ OK ]");
  }
  else {
    printLCD("MPU-6050  [FAIL]");
    delay(1000);
    return;
  }

  // Define the operating conditions (Refer to library basic_readings example)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // +-8G max
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // +-500 deg/s max
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);   // halfpower bandwidth

  // Calibrate the gyroscope
  
  printLCD("Calibrating GYRO");
  
  int N_samples = 5000;
  for( int index = 0; index < 3; index += 1 ) {
    MPU_offsets_gyro[index] = 0; // Set gyroscope offsets to zero
  }
  for( int count = 0; count < N_samples; count += 1 ) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float gyro[] = { g.gyro.x, g.gyro.y, g.gyro.z };
    for( int index = 0; index < 3; index += 1 ) {
      MPU_offsets_gyro[index] += float( gyro[index] );  // Sum gyro drift
    } 
  }
  for( int index = 0; index < 3; index += 1 ) {
    MPU_offsets_gyro[index] *= 1 / N_samples; // Save mean gyro drift
  }
  printLCD("GYRO Calibrated!");

  // Calibrate the accelerometer

  if ((SD_ready == true) || ((SD_ready == false) && SD.begin(SD_CHIPSELECT))) {

    // Try to read an existing file before deciding to create new calibration data
    File calibrationdata;
    strncpy(filename, "_calibrate_MPU-6050.conf", sizeof(filename));
    if (SD.exists(filename)) {
      /*
       * // Imagine saving calibration data like accx \n <max(accx)> \n <min(accx)> \n accy \n <max(accy)> \n <min(accy)>
       */
      calibrationdata = SD.open(filename, FILE_READ);
      /*
       * Imagine running a while loop that breaks on bad file size or on bad calibration data
      if (calibrationdata.size() > 0) {
        ; // Do something
      }
      */
    }
    
    if(!SD.exists(filename)) {
      // Prepare a file to log
      calibrationdata = SD.open(filename, FILE_WRITE);
      /*
       * // Create temporary lambda functions to assist with accelerometer calibration
       * auto f_calgx = [](float rawgx_degps) { rawgx_degps - MPU_offsets_gyro[0] };
       * auto f_calgy = [](float rawgy_degps) { rawgy_degps - MPU_offsets_gyro[1] };
       * auto f_calgz = [](float rawgz_degps) { rawgz_degps - MPU_offsets_gyro[2] };
       */ 
       ;  // Do something
    }

    // Clean-up
    calibrationdata.close();
  }
}

void printLCD(const char *str, int skiprows)
{
  /*
  This function wraps (no /t or tabbing) text output on an LCD display.
  Depends on definition of lcd object and the display size as LCD_COLS, LCD_ROWS.
  */
  int locspace = LCD_COLS;  // Record the last place a whitespace
  int locstart = 0;
  int slen = strlen(str);

  // Debugging
  Serial.print("LCD >>> ");
  Serial.println(str);

  // Iterate over each of the rows
  for (int i=0; i<LCD_ROWS; ++i) {

    // If a skip argument was passed, do not update some of the rows
    if (i < skiprows) {
      continue;
    }

    // Move LCD cursor to column 0, row i
    lcd.setCursor(0, i);
 
    // If text fits on a line, nothing to worry about
    if (slen <= LCD_COLS) {
      locspace = LCD_COLS;
    }
    // Else if text needs to wrap, find index of a good EOL by looking for a space in text
    else if (i+1 != LCD_ROWS) {
      for (int j=locstart+LCD_COLS; j>locstart; --j) {
        if (isSpace(str[j])){
          locspace = j;
          break;
        }
      }
    }
    // Else we're on the last row of the display (wrapping is not possible)
    else {
      locspace = LCD_ROWS * LCD_COLS;
    }

    // If returnline char is used, ignore the EOL wrap we just found and use the returnline char
    int locreturn = String(str).indexOf('\n', locstart);
    if ((locreturn < locspace) && (locreturn != -1)) {
      locspace = locreturn;
    }

    // Write to the display and fill empty gaps with spaces
    for (int j=locstart; j<locstart+LCD_COLS; ++j) {
      // If there is a valid character to be printed
      if (j < slen) {
        // Display overflow if in last row, last column, and not end of string
        if ((i + 1 == LCD_ROWS) && (j + 1 == locstart + LCD_COLS) && (j + 1 < slen)) {
          lcd.print('-');   // Display overflow character
        }
        // Else print as usual
        else {
          lcd.print(j<locspace ? str[j] : ' ');
        }
      }
      // No characters remain in the string, blank the remaining space
      else {
        lcd.print(' ');
      }
    }

    // Record the string index the printer should start from (in the display's next row)
    locstart = locspace + 1;
  }
  // Record the time the LCD was last updated
  LCD_msgexpirytime_ms = 0;
}

void setLCDexpiry(unsigned long t_ms)
{
  // Set the system time at which a pop-up message should expire
  LCD_msgexpirytime_ms = millis() + t_ms;
}

void updateBMP(struct BMPsensor *sensor)
{
  if ((bmp.readPressure() > 0) && (bmp.readTemperature() < 179)) {
    // Save pressure and temperature data
    sensor->P_Pa = bmp.readPressure();
    sensor->T_C = bmp.readTemperature();
  }
  else {
    sensor->P_Pa = NAN;
    sensor->T_C = NAN;
  }
}

void updateD6F(struct D6Fsensor *sensor0, struct D6Fsensor *sensor1, struct D6Fsensor *sensor2)
{
  // Save dynamic pressure and temperature data
  // Read sensor 0
  digitalWrite(SW2_D6F, LOW);
  digitalWrite(SW3_D6F, LOW);
  if (d6f.isConnected()) {
    sensor0->q_Pa = max(0, d6f.getPressure());
    sensor0->T_C = d6f.getTemperature();
  }
  else {
    sensor0->q_Pa = NAN;
    sensor0->T_C = NAN;
  }

  // Read sensor 1
  digitalWrite(SW2_D6F, HIGH);
  if (d6f.isConnected()) {
    sensor1->q_Pa = max(0, d6f.getPressure());
    sensor1->T_C = d6f.getTemperature();
  }
  else {
    sensor1->q_Pa = NAN;
    sensor1->T_C = NAN;
  }

  // Read sensor 2
  digitalWrite(SW3_D6F, HIGH);
  if (d6f.isConnected()) {
    sensor2->q_Pa = max(0, d6f.getPressure());
    sensor2->T_C = d6f.getTemperature();
  }
  else {
    sensor2->q_Pa = NAN;
    sensor2->T_C = NAN;
  }

  // Clean-up
  digitalWrite(SW2_D6F, LOW);
  digitalWrite(SW3_D6F, LOW);
}

void updateHTU(struct HTUsensor *sensor)
{
  // Save humidity and temperature data
  sensor->RH = htu.readHumidity();
  sensor->T_C = htu.readTemperature();
}

void updateMPU(struct MPUsensor *sensor)
{
  // Read raw sensor values
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float raw_ax = a.acceleration.x;
  float raw_ay = a.acceleration.y;
  float raw_az = a.acceleration.z;
  float raw_gx = g.gyro.x;
  float raw_gy = g.gyro.y;
  float raw_gz = g.gyro.z;

  // Calibrate sensor values
  float cal_ax = raw_ax;
  float cal_ay = raw_ay;
  float cal_az = raw_az;
  float cal_gx = raw_gx - MPU_offsets_gyro[0];
  float cal_gy = raw_gy - MPU_offsets_gyro[1];
  float cal_gz = raw_gz - MPU_offsets_gyro[2];

  // Reorient to new reference frame
  float new_ax = cal_ax;
  float new_ay = cal_ay;
  float new_az = cal_az;
  float new_gx = cal_gx;
  float new_gy = cal_gy;
  float new_gz = cal_gz;

  // Store calibrated values
  sensor->ax = new_ax;
  sensor->ay = new_ay;
  sensor->az = new_az;
  sensor->gx = new_gx;
  sensor->gy = new_gy;
  sensor->gz = new_gz;
  sensor->T_C = temp.temperature;
}

void updateSW4(void)
{
  // If SW4 (held-high) falls to ground, record falling edge
  if (digitalRead(SW4_PUSH) == LOW) {
    SW4_fallingedge = true;
  }
  // If SW4 is pulled back to high and a falling edge is still erroneously true
  if ((digitalRead(SW4_PUSH) == HIGH) && (SW4_fallingedge == true)) {
    // Set fallingedge to false and toggle the state of SW4's toggle
    SW4_fallingedge = false;
    SW4_toggle = !SW4_toggle;
  }
}
