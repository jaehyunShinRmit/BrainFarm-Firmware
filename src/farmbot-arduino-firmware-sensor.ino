/******************************************************************************
  # GPS/SD card code is adopted from
  https://github.com/sparkfun/GPS_Shield
  It uses the TinyGPS++ library to parse the NMEA strings sent by the GPS module,
  and prints interesting GPS information - comma separated - to a newly created
  file on the SD card.
  TinyGPS++ Library  - https://github.com/mikalhart/TinyGPSPlus/releases
  uSD card CS pin (pin 10 on SparkFun GPS Logger Shield)
  
  # 9DOF IMU
  The LSM9DS1 is a versatile 9DOF sensor.
  Hardware setup: This library supports communicating with the
  LSM9DS1 over either I2C.
  The pin-out is as follows:
  LSM9DS1 --------- Arduino
   SCL    --------- SCL (A5 on older 'Duinos')
   SDA    --------- SDA (A4 on older 'Duinos')
   VDD    --------- 3.3V
   GND    --------- GND

  # DC Motor Driver 2x15A_lite module.
  The pin-out is as follows:
  Driver  --------- Arduino
   M1_PWM --------- PIN5
   M2_PWM --------- PIN6
   M1_EN  --------- PIN4
   M2_EN  --------- PIN7
   +5v    --------- +5v
   GND    --------- GND
******************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "TinyGPS++.h"
#include "SparkFunLSM9DS1.h"
#include "Filters.h"
#include "Fuser.h"
#include "FarmbotSensor.h"
#include "Command.h"
//#include "GCodeProcessor.h" //need to uncomment it to use Gcode
#include "KCodeProcessor.h"
#include "TimerOne.h"
#include "TimerThree.h"
#include "pins.h"
#include "Config.h"
#include "MemoryFree.h"
#include "Debug.h"


FarmbotSensor Bot; // Frambot Sensor state variables
LSM9DS1 imu; //IMU
Fuser ekf;   //EKF
static char commandEndChar = 0x0A;
static KCodeProcessor *kCodeProcessor = new KCodeProcessor();
//static GCodeProcessor *gCodeProcessor = new GCodeProcessor(); //need to uncomment it to use Gcode

unsigned long lastAction;

char incomingChar = 0;
char incomingCommandArray[INCOMING_CMD_BUF_SIZE];
int  incomingCommandPointer = 0;
char commandChar[INCOMING_CMD_BUF_SIZE + 1];

// Blink led routine used for testing
bool blink = false;
void blinkLed()
{
  blink = !blink;
  digitalWrite(LED_PIN, blink);
}
// Initialize the low pass filter
#define ACCLOWPASSFILTER 1
#define MAGLOWPASSFILTER 1
float filterFrequency = 30;                       // filter frequency (Hz)
FilterOnePole AxLowpass(LOWPASS, filterFrequency);  // create a one pole (RC) lowpass filter
FilterOnePole AyLowpass(LOWPASS, filterFrequency);  // create a one pole (RC) lowpass filter
FilterOnePole AzLowpass(LOWPASS, filterFrequency);  // create a one pole (RC) lowpass filter
FilterOnePole MxLowpass(LOWPASS, filterFrequency);  // create a one pole (RC) lowpass filter
FilterOnePole MyLowpass(LOWPASS, filterFrequency);  // create a one pole (RC) lowpass filter
FilterOnePole MzLowpass(LOWPASS, filterFrequency);  // create a one pole (RC) lowpass filter
//////////////////
//  IMU Sensor  //
//////////////////

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M   0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
// Offsets applied to raw x/y/z values
float mag_offsets[3]            = { 37793.5207402, -6321.2426109, 6891.2431500 };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 0.97196, 0.00833, 0.00444 },
  { 0.00833, 1.04714, -0.00693 },
  { 0.00444, -0.00693, 1.00345 }
};
// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -11.41 //Bundoora 14/11/2017 //-8.58 Declination (degrees) in Boulder, CO.

//////////////////////////
// GPS and Logger shield//
//////////////////////////
TinyGPSPlus tinyGPS; // tinyGPSPlus object to be used throughout
#define GPS_BAUD 9600 // GPS module's default baud rate
// If you're using an Arduino Uno, Mega, RedBoard, or any board that uses the
// 0/1 UART for programming/Serial monitor-ing, use SoftwareSerial:
// Set gpsPort to either ssGPS if using SoftwareSerial or Serial1 if using an
// Arduino with a dedicated hardware serial port
#define gpsPort Serial1  // Alternatively, use Serial1 on the Leonardo
// Keep in mind, the SD library has max file name lengths of 8.3 - 8 char prefix,
// and a 3 char suffix.
// Our log files are called "logXX.csv, so "gpslog99.csv" is our max file.
#define ARDUINO_USD_CS 10 // uSD card CS pin (pin 10 on SparkFun GPS Logger Shield)
#define LOG_FILE_PREFIX "log" // Name of the log file.
#define MAX_LOG_FILES 100 // Number of log files that can be made
#define LOG_FILE_SUFFIX "csv" // Suffix of the log file
#define LOG_COLUMN_COUNT 14
static unsigned long lastPrint = 0; // Keep track of print time
char logFileName[13]; // Char string to store the log file name
// Data to be logged:
char * log_col_names[LOG_COLUMN_COUNT] = {
  "ms", "E(mm)", "N(mm)", "V(mm/s)","P(rad)","DP(rad/s)", "A(mm/s^2)","Heading(IMU)","roll(deg)", "pitch(deg)", "latitude" ,"longitude","Speed(GPS)","Heading(GPS)"
}; // log_col_names is printed at the top of the file.
char * log_col_names_raw[LOG_COLUMN_COUNT] = {
  "ms", "ax(m/s^2)", "ay(m/s^2)", "az(mm/s^2)","gx(rad/s)","gy(rad/s)", "gy(rad/s)","Roll(deg)","pitch(deg)", "heading(deg)", "latitude" ,"longitude","Speed(GPS)","Heading(GPS)"
}; // log_col_names is printed at the top of the file.

bool oneshot = false;
/////////////////////////////////
// Motor       //
//Arduino Mega	USE TIMER1 for PWM 11, 12, 13	
//                  TIMER3 for PWM 2, 3, 5
//So we use PWM 6,7 to control motor 
//Since we use Interrupt routine with TIMER1 and TIMER3
/////////////////////////////////
const int PIN_LEFTRIGHT_JOYSTICK  = 6;     //M1 
const int PIN_FORNTBACK_JOYSTICK  = 7;     //M2 
const int Stop       = 131;
int Positive         = 220;
int Negative         = 70;
int counter          = 0;

//////////////////////
// Log Rate Control //
//////////////////////
unsigned long lastLog = 0; // Global var to keep of last time we logged

////////////////////////////
// Sketch Output Settings //
////////////////////////////
//#define PRINT_CALCULATED
#define SENSORUPDATE_SPEED 50  // 50ms -> 20hz update rate
#define MAXINDEXUIO 10*300     // 300s
#define MAXINDEXURL 10*300     // 300s


bool interruptBusy = false;
int interruptSecondTimer = 0;
double measurement[5];
float a = 6378137.0;
void interrupt(void) {
  
  if (!debugInterrupt) {
    interruptSecondTimer++;
    
    if (interruptBusy == false) {
      interruptBusy = true;
      // triggered once per 100 ms
          
      if (interruptSecondTimer >= 100000 / MOVEMENT_INTERRUPT_SPEED) {
        interruptSecondTimer = 0;
        updateOriantation();
        Bot.latitude  = (double)tinyGPS.location.lat();
        Bot.longitude = (double)tinyGPS.location.lng();
        Bot.speed = (double)(tinyGPS.speed.mph());
        // Logging Raw data 
        if(KCurrentState::getInstance()->getisRawdataLogging()){
          logRawData();
        }
        // Running Kalman filtering - Absolute Optimization
        if (KCurrentState::getInstance()->getisKalmanFiltering()) {
          measurement[2] = (double)Bot.heading;
          measurement[3] = (double)Bot.gz;
          measurement[4] = (double)Bot.ax;
          ekf.step(measurement);

          Bot.E  = ekf.getX(0); //mm
          Bot.N  = ekf.getX(1); //mm
          Bot.P  = ekf.getX(2); //deg or rad
          Bot.V  = ekf.getX(3); //mm/s
          Bot.dP = ekf.getX(4); // deg/s or rad/s
          Bot.A  = ekf.getX(5); //mm/s^2

          measurement[0] = (double)Bot.E;
          measurement[1] = (double)Bot.N;

          // Serial.print("Time: ");
          // Serial.println(millis());
          // Serial.print(Bot.roll_deg);
          // Serial.print(",");
          // Serial.print(Bot.pitch_deg);
          // Serial.print(",");
          // Serial.println( Bot.heading_deg);

          if (tinyGPS.location.isUpdated()) {
            double X, Y, dX, dY, GE, GN;
            //if we assume, the earth is sperical shape,and altitude is 0
            X = a * cos(Bot.latitude) * cos(Bot.longitude);
            Y = a * cos(Bot.latitude) * sin(Bot.longitude);
            dX = X - Bot.initLat;
            dY = Y - Bot.initLon;
            GE = dX * -sin(Bot.longitude) + dY * cos(Bot.longitude);
            GN = dX * -sin(Bot.latitude) * cos(Bot.longitude) + dY * cos(Bot.latitude) * sin(Bot.longitude);

            // Conduct sensor fusion GPS and IMU
            // Bot.E GE
            measurement[0] = GE;
            measurement[1] = GN;
          }
          if(LOGGING){
           logFarmbot();
          }
        }
        ////Updating Robot Oriantation
        if (KCurrentState::getInstance()->getisUpdatingInitialOrientation()) {
          int indexUIO = 0;
          if (indexUIO++ < MAXINDEXUIO) {
            InitialOriantationUpdate(indexUIO);
          }
          else {
            KCurrentState::getInstance()->setisUpdatingInitialOrientation(false);
            indexUIO = 0;
            KCurrentState::getInstance()->setInitRoll(Bot.initLat);
            KCurrentState::getInstance()->setInitPitch(Bot.initPitch);
            KCurrentState::getInstance()->setInitHeading(Bot.initHeading);
          }
        }
        ////Updating Reference Location
        if (KCurrentState::getInstance()->getisUpdatingReferenceLocation()) { 
          int indexURL = 0;
          if (indexURL++ < MAXINDEXURL) {
            InitialGPSUpdate(indexURL);
          }
          else {
            KCurrentState::getInstance()->setisUpdatingReferenceLocation(false);
            indexURL = 0;
            KCurrentState::getInstance()->setInitLat(Bot.initLat);
            KCurrentState::getInstance()->setInitLon(Bot.initLon);
          }
        }
        //Initialize Kalmanfilter modelr. Updating Q,R
        if (KCurrentState::getInstance()->getisInitKalman()){ 
          KCurrentState::getInstance()->setisInitKalman(false);
          ekf.InitP();
          if(KCurrentState::getInstance()->getisUpdatingQ()){
              ekf.updateQ(KCurrentState::getInstance()->getRow(),KCurrentState::getInstance()->getCol(),KCurrentState::getInstance()->getCovQ());
          }
          if(KCurrentState::getInstance()->getisUpdatingR()){
              ekf.updateR(KCurrentState::getInstance()->getRow(),KCurrentState::getInstance()->getCol(),KCurrentState::getInstance()->getCovR());
          }
        }
          
        stateupdate();
      }
      interruptBusy = false;
    }
  }
}
bool interrupt3Busy = false;
int interrupt3SecondTimer = 0;
int movingTimer = 0;
void interrupt_motor(void) {

  
  if (interrupt3Busy == false) {
    interrupt3Busy = true;
    if(KCurrentState::getInstance()->getisAdvencing() && movingTimer < 20){
         movingTimer++;
         motoradvance();       //Move forward
         Serial.println("Advancing");
    }
    else{
        movingTimer=0;
        motorstop();
        KCurrentState::getInstance()->setisAdvencing(false);
    }
      //back_off ()          //Move backward
      //turn_L ()            //Turn Left
      //turn_R ()            //Turn Right
      
    interrupt3Busy = false;
  }
}
void setup()
{

  Serial.begin(115200);

  // GPS and SD card shield initialization
  gpsPort.begin(GPS_BAUD);
  Serial.println("Setting up SD card.");
  // see if the card is present and can be initialized:
  if (!SD.begin(ARDUINO_USD_CS))
  {
    Serial.println("Error initializing SD card.");
  }
  updateFileName(); // Each time we start, create a new file, increment the number
  printHeader(); // Print a header at the top of the new file

  // IMU initialization
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1);
  }

  // Start the interrupt
  // Interrupt management code library written by Paul Stoffregen
  // The default time 100 micro seconds
  Timer1.attachInterrupt(interrupt);
  Timer1.initialize(MOVEMENT_INTERRUPT_SPEED);
  Timer1.start();

  Timer3.attachInterrupt(interrupt_motor);
  Timer3.initialize(100000); //0.1 seconds == 100000microseconds
  Timer3.start();

  // Initialize the inactivity check
  lastAction = millis();
  Serial.print("R99 ARDUINO STARTUP COMPLETE\r\n");

  pinMode(PIN_LEFTRIGHT_JOYSTICK, OUTPUT);
  pinMode(PIN_FORNTBACK_JOYSTICK, OUTPUT);
}




void loop()
{
  // Updating IMU sensor  
  if ((lastPrint + SENSORUPDATE_SPEED) < millis()){
    updateFarmbot();
    lastPrint = millis(); // Update lastPrint time
  }
  // Report GPS connection 
  if(tinyGPS.location.isUpdated() && oneshot)
  {
    Serial.print("GPS connected\r\n");
    oneshot = false;
  }
  // Continue to "feed" the tinyGPS object:
  while (gpsPort.available())
    tinyGPS.encode(gpsPort.read());
  
  //waitng command from Raspberry PI/Laptop
  if (Serial.available()) {

    // Save current time stamp for timeout actions
    lastAction = millis();

    // Get the input and start processing on receiving 'new line'
    incomingChar = Serial.read();

    // Filter out emergency stop.
    if (!(incomingChar == 69 || incomingChar == 101))
    {
      incomingCommandArray[incomingCommandPointer] = incomingChar;
      incomingCommandPointer++;
    }
    else
    {
      KCurrentState::getInstance()->setEmergencyStop();
    }

    // If the string is getting to long, cap it off with a new line and let it process anyway
    if (incomingCommandPointer >= INCOMING_CMD_BUF_SIZE - 1)
    {
      incomingChar = '\n';
      incomingCommandArray[incomingCommandPointer] = incomingChar;
      incomingCommandPointer++;
    }

    if (incomingChar == '\n' || incomingCommandPointer >= INCOMING_CMD_BUF_SIZE)
    {

      //char commandChar[incomingCommandPointer + 1];
      for (int i = 0; i < incomingCommandPointer - 1; i++)
      {
        commandChar[i] = incomingCommandArray[i];
      }
      commandChar[incomingCommandPointer - 1] = '\0';

      if (incomingCommandPointer > 1)
      {

        // Report back the received command
        Serial.print(COMM_REPORT_CMD_ECHO);
        Serial.print(" ");
        Serial.print("*");
        Serial.print(commandChar);
        Serial.print("*");
        Serial.print("\r\n");

        // Create a command and let it execute
        Command *command = new Command(commandChar);

        // Log the values if needed for debugging
        if (LOGGING || debugMessages)
        {
          command->print();
        }

        if (command->getCodeEnum() > 300) //K command starts from 311
        {
          kCodeProcessor->execute(command);
        }
        else { //G command
          //gCodeProcessor->execute(command);
        }

        free(command);

      }

      incomingCommandPointer = 0;
    }
  }
}

void updateFarmbot()
{
  updateIMU();
  float Ax, Ay, Az, Mx, My, Mz;

  if (ACCLOWPASSFILTER) {
    AxLowpass.input(imu.calcAccel(imu.ax));
    AyLowpass.input(imu.calcAccel(imu.ay));
    AzLowpass.input(imu.calcAccel(imu.az));
    Ax = AxLowpass.output();
    Ay = AyLowpass.output();
    Az = AzLowpass.output();
  }
  else {
    Ax = imu.calcAccel(imu.ax);
    Ay = imu.calcAccel(imu.ay);
    Az = imu.calcAccel(imu.az);
  }
  if (MAGLOWPASSFILTER) {
    MxLowpass.input(imu.calcMag(imu.mx));
    MyLowpass.input(imu.calcMag(imu.my));
    MzLowpass.input(imu.calcMag(imu.mz));
    Mx = MxLowpass.output();
    My = MyLowpass.output();
    Mz = MzLowpass.output();
  }
  else {
    Mx = imu.calcMag(imu.mx);
    My = imu.calcMag(imu.my);
    Mz = imu.calcMag(imu.mz);
  }

  Bot.ax = (Ax * 9.81); // converting m/s^2
  Bot.ay = (Ay * 9.81); // converting m/s^2
  Bot.az = (Az * 9.81); // converting m/s^2
  Bot.gx =  imu.calcGyro(imu.gx);
  Bot.gy =  imu.calcGyro(imu.gy);
  Bot.gz =  imu.calcGyro(imu.gz);
  Bot.mx =  Mx * 100000;
  Bot.my =  My * 100000;
  Bot.mz =  Mz * 100000;
}

void updateOriantation(){
  float x =  Bot.mx - mag_offsets[0];
  float y =  Bot.my - mag_offsets[1];
  float z =  Bot.mz - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Pitch and roll
  float roll  = atan2(Bot.ay, Bot.az);
  float pitch = atan2(-Bot.ax, Bot.ay * sin(roll) + Bot.az * cos(roll));
  float roll_deg = roll * 180.0 / M_PI;
  float pitch_deg = pitch * 180.0 / M_PI;

  // Tilt compensated magnetic sensor measurements
  float mx_comp = mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll);
  float my_comp = mz * sin(roll) - my * cos(roll);

  // Arctangent of y/x
  float heading = atan2(my_comp, mx_comp);
  float heading_deg = heading * 180.0 / M_PI;
  if (heading_deg  < 0.0)
    heading_deg += 360.0;

  // oriantation in radian
  Bot.roll  = roll;
  Bot.pitch = pitch;
  Bot.heading = heading;

  // oridantation in degree
  Bot.roll_deg = (long) roll_deg;
  Bot.pitch_deg = (long) pitch_deg;
  Bot.heading_deg = (long) heading_deg;
}

void InitialOriantationUpdate(int n){
  updateOriantation();
  Bot.initRoll    = ((Bot.initRoll * n) + Bot.roll) / (n + 1);
  Bot.initPitch   = ((Bot.initPitch * n) + Bot.pitch) / (n + 1);
  Bot.initHeading = ((Bot.initHeading * n) + Bot.heading) / (n + 1);
}

void InitialGPSUpdate(int n){
  Bot.initLat  = ((Bot.initLat * n) + (double)tinyGPS.location.lat()) / (n + 1);
  Bot.initLon  = ((Bot.initLon * n) + (double)tinyGPS.location.lng()) / (n + 1);
}

int updateIMU(){
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
}
byte logRawData(){
  File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file
  if (logFile)
  {
    logFile.print(millis(), 5);
    logFile.print(',');
    logFile.print(Bot.ax, 5);
    logFile.print(',');
    logFile.print(Bot.ay, 5);
    logFile.print(',');
    logFile.print(Bot.az, 5);
    logFile.print(',');
    logFile.print(Bot.gx, 5);
    logFile.print(',');
    logFile.print(Bot.gy, 5);
    logFile.print(',');
    logFile.print(Bot.gz, 5);
    logFile.print(',');
    logFile.print(Bot.roll_deg, 5);
    logFile.print(',');
    logFile.print(Bot.pitch_deg, 5);
    logFile.print(',');
    logFile.print(Bot.heading_deg, 5);
    logFile.print(',');
    logFile.print(tinyGPS.location.lng(), 6);
    logFile.print(',');
    logFile.print(tinyGPS.location.lat(), 6);
    logFile.print(',');
    logFile.print(tinyGPS.speed.mph(), 2);
    logFile.print(',');
    logFile.print(tinyGPS.course.deg(),2);
    logFile.println();
    logFile.close();

    return 1; // Return success
  }
  else{
    Serial.print("The log file has been open - please check the SD card\r\n");
    KCurrentState::getInstance()->setisRawdataLogging(false);
  }
  return 0; // If we failed to open the file, return fail
}

byte logFarmbot(){
  File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file
  if (logFile)
  {
    logFile.print(millis(), 5);
    logFile.print(',');
    logFile.print(Bot.E, 5);
    logFile.print(',');
    logFile.print(Bot.N, 5);
    logFile.print(',');
    logFile.print(Bot.V, 5);
    logFile.print(',');
    logFile.print(Bot.P, 5);
    logFile.print(',');
    logFile.print(Bot.dP, 5);
    logFile.print(',');
    logFile.print(Bot.A, 5);
    logFile.print(',');
    logFile.print(Bot.heading_deg, 5);
    logFile.print(',');
    logFile.print(tinyGPS.location.lng(), 6);
    logFile.print(',');
    logFile.print(tinyGPS.location.lat(), 6);
    logFile.print(',');
    logFile.print(tinyGPS.speed.mph(), 2);
    logFile.print(',');
    logFile.print(tinyGPS.course.deg(),2);
    logFile.println();
    logFile.close();

    return 1; // Return success
  }

  return 0; // If we failed to open the file, return fail
}

// printHeader() - prints our eight column names to the top of our log file
void printHeader(){
  File logFile = SD.open(logFileName, FILE_WRITE); // Open the log file

  if (logFile) // If the log file opened, print our column names to the file
  {
    int i = 0;
    for (; i < LOG_COLUMN_COUNT; i++)
    {
      logFile.print(log_col_names[i]);
      if (i < LOG_COLUMN_COUNT - 1) // If it's anything but the last column
        logFile.print(','); // print a comma
      else // If it's the last column
        logFile.println(); // print a new line
    }
    logFile.close(); // close the file
  }
}

// updateFileName() - Looks through the log files already present on a card,
// and creates a new file with an incremented file index.
void updateFileName(){
  int i = 0;
  for (; i < MAX_LOG_FILES; i++)
  {
    memset(logFileName, 0, strlen(logFileName)); // Clear logFileName string
    // Set logFileName to "gpslogXX.csv":
    sprintf(logFileName, "%s%d.%s", LOG_FILE_PREFIX, i, LOG_FILE_SUFFIX);
    if (!SD.exists(logFileName)) // If a file doesn't exist
    {
      break; // Break out of this loop. We found our index
    }
    else // Otherwise:
    {
      Serial.print(logFileName);
      Serial.println(" exists"); // Print a debug statement
    }
  }
  Serial.print("File name: ");
  Serial.println(logFileName); // Debug print the file name
}


void motorstop(void)                    //Stop
{
  analogWrite(PIN_LEFTRIGHT_JOYSTICK, Stop);
  analogWrite(PIN_FORNTBACK_JOYSTICK, Stop);
}
void motoradvance()       //Move forward
{
  analogWrite(PIN_LEFTRIGHT_JOYSTICK, Stop);
  analogWrite(PIN_FORNTBACK_JOYSTICK, Positive);
}
void back_off ()          //Move backward
{
  analogWrite(PIN_LEFTRIGHT_JOYSTICK, Stop);
  analogWrite(PIN_FORNTBACK_JOYSTICK, Negative);
}
void turn_L ()            //Turn Left
{
  analogWrite(PIN_LEFTRIGHT_JOYSTICK, Negative);
  analogWrite(PIN_FORNTBACK_JOYSTICK, Stop);
}
void turn_R ()            //Turn Right
{
  analogWrite(PIN_LEFTRIGHT_JOYSTICK, Positive);
  analogWrite(PIN_FORNTBACK_JOYSTICK, Stop);
}

void stateupdate(void) {
  KCurrentState::getInstance()->setAX(Bot.ax);
  KCurrentState::getInstance()->setAY(Bot.ay);
  KCurrentState::getInstance()->setAZ(Bot.az);
  KCurrentState::getInstance()->setGX(Bot.gx);
  KCurrentState::getInstance()->setGY(Bot.gy);
  KCurrentState::getInstance()->setGZ(Bot.gz);
  KCurrentState::getInstance()->setMX(Bot.mx);
  KCurrentState::getInstance()->setMY(Bot.my);
  KCurrentState::getInstance()->setMZ(Bot.mz);
  KCurrentState::getInstance()->setRoll(Bot.roll);
  KCurrentState::getInstance()->setPitch(Bot.pitch);
  KCurrentState::getInstance()->setHeading(Bot.heading);
  KCurrentState::getInstance()->setLat(Bot.latitude);
  KCurrentState::getInstance()->setLog(Bot.longitude);
  KCurrentState::getInstance()->setSpeed(Bot.speed);
}

