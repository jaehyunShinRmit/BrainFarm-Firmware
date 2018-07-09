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

  # Joystick pin
  EFTRIGHT_JOYSTICK  --- PWM6;     //M1 
  FORNTBACK_JOYSTICK --- PWM7;     //M2 
******************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <SparkFunLSM9DS1.h>
#include <Filters.h>
#include "Fuser.h"
#include "BrainBotSensor.h"
#include "Command.h"
//#include "GCodeProcessor.h" //need to uncomment it to use Gcode
#include "KCodeProcessor.h"
#include "TimerOne.h"
#include "TimerThree.h"
#include "pins.h"
#include "Config.h"
#include "MemoryFree.h"
#include "Debug.h"


BrainBotSensor Bot; // Frambot Sensor state variables
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

//Joystick
int LeftRight1 = 0;  // variable to store the value coming from the sensor
int LeftRight2 = 0;  // variable to store the value coming from the sensor
int FrontBack1 = 0;  // variable to store the value coming from the sensor
int FrontBack2 = 0;  // variable to store the value coming from the sensor
int sensorPin8 = A8;    // select the input pin LeftRight1
int sensorPin9 = A9;    // select the input pin LeftRight2
int sensorPin10 = A10;    // select the input pin FrontBack1
int sensorPin11 = A11;    // select the input pin FrontBack2


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
#define gpsPort Serial1  // Alternatively, use Serial1 on the Leonardo
#define GPS_BAUD 9600 // GPS module's default baud rate
// If you're using an Arduino Uno, Mega, RedBoard, or any board that uses the
// 0/1 UART for programming/Serial monitor-ing, use SoftwareSerial:


#define ARDUINO_USD_CS 10 // uSD card CS pin (pin 10 on SparkFun GPS Logger Shield)


static unsigned long lastPrint = 0; // Keep track of print time
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
const int Stop       = 130;
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

#define TIMER1_RATE 10000      // 10ms
#define SAMPLING_RATE 10       // 100ms

bool interruptBusy = false;
int interruptSecondTimer = 0;
double measurement[5];
float a = 6378137.0;
void interrupt(void) {
  
  if (!debugInterrupt) {
    interruptSecondTimer++;
    
    if (interruptBusy == false) {
      interruptBusy = true;
      
      // triggered once per 10 ms
      if (interruptSecondTimer >= SAMPLING_RATE) {
        interruptSecondTimer = 0;
        updateOriantation();
        Bot.latitude  = (double)tinyGPS.location.lat();
        Bot.longitude = (double)tinyGPS.location.lng();
        Bot.speed = (double)(tinyGPS.speed.mph());
        // Logging Raw data 
        if(KCurrentState::getInstance()->getisRawdataLogging()){
          logRawData();
          SendingRawData();
        }
        if(KCurrentState::getInstance()->getisReinforceddataLogging()){
          logRawData();
          LoggingForReinforceLearing(); // send serial message to Main program 
        }
        /* 25 May 2018 Jaehyun Shin
        /  Decided to applying sensor fusion algorithm on Main program not in Arduino
        /  since the Arduino has only 6-7 decimal digits of precision in float type calculation.
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
         */ 
        stateupdate();
      }
      interruptBusy = false;
    }
  }
}
bool interrupt3Busy = false;
int interrupt3SecondTimer = 0;
int movingTimer = 0;
char *str;
void interrupt_motor(void) {
  if (interrupt3Busy == false) {
    interrupt3Busy = true;
    if(KCurrentState::getInstance()->getisMoving() && movingTimer < KCurrentState::getInstance()->getMovingDistance()*1000){
         movingTimer++;
        switch(KCurrentState::getInstance()->getMovingDirection()){
          case ADVANCE:
            motoradvance();       //Move forward
            break;
          case RETREAT:
            back_off();
            break;
          case LEFT:
            turn_L();
            break;
          case RIGHT:
            turn_R();
            break;
        }
    }
    else{
        if (movingTimer > 0){
          Serial.println("Robot has been stopped");
        }
        movingTimer=0;
        motorstop();
        KCurrentState::getInstance()->setisMoving(false);
        //KCurrentState::getInstance()->setisRawdataLogging(false);
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
  // KCurrentState::getInstance()->updateFileName(); // Each time we start, create a new file, increment the number
  // KCurrentState::getInstance()->printHeader(); // Print a header at the top of the new file

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
  Timer1.initialize(10000); //10 ms == 10000 microseconds
  Timer1.start();

  Timer3.attachInterrupt(interrupt_motor);
  Timer3.initialize(1000); //1 ms == 1000 microseconds
  Timer3.start();



  pinMode(PIN_LEFTRIGHT_JOYSTICK, OUTPUT);
  pinMode(PIN_FORNTBACK_JOYSTICK, OUTPUT);
  motorstop();

    // Initialize the inactivity check
  lastAction = millis();
  Serial.print("ARDUINO STARTUP COMPLETE\r\n");
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

byte updateIMU(){
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
  return 0;
}
byte logRawData(){
  File logFile = SD.open(KCurrentState::getInstance()->getlogFileName(), FILE_WRITE); // Open the log file
  if (logFile)
  {
    logFile.print(micros(), 5);
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
    logFile.print(Bot.mx, 5);
    logFile.print(',');
    logFile.print(Bot.my, 5);
    logFile.print(',');
    logFile.print(Bot.mz, 5);
    logFile.print(',');
    logFile.print(tinyGPS.location.lat(), 6);
    logFile.print(',');
    logFile.print(tinyGPS.location.lng(), 6);
    logFile.print(',');
    logFile.print(tinyGPS.speed.mph(), 2);
    logFile.print(',');
    logFile.print(tinyGPS.course.deg(),2);
    logFile.println();
    logFile.close();

    return 1; // Return success
  }
  else{
    Serial.print("The log file has not been opend - please check the SD card\r\n");
    KCurrentState::getInstance()->setisRawdataLogging(false);
  }
  return 0; // If we failed to open the file, return fail
}

byte logFarmbot(){
  File logFile = SD.open(KCurrentState::getInstance()->getlogFileName(), FILE_WRITE); // Open the log file
  if (logFile)
  {
    logFile.print(micros(), 5);
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

void LoggingForReinforceLearing (void) {
  FrontBack1 = analogRead(sensorPin8);
  FrontBack2 = analogRead(sensorPin9);
  LeftRight1 = analogRead(sensorPin10);
  LeftRight2 = analogRead(sensorPin11);
  FrontBack1 = map(FrontBack1, 0, 1023, 0, 255);
  FrontBack2 = map(FrontBack2, 0, 1023, 0, 255);
  LeftRight1 = map(LeftRight1, 0, 1023, 0, 255);
  LeftRight2 = map(LeftRight2, 0, 1023, 0, 255);
  
  Serial.print(FrontBack1);
  Serial.print(',');
  Serial.print(FrontBack2);
  Serial.print(',');
  Serial.print(LeftRight1);
  Serial.print(',');
  Serial.print(LeftRight2);
  Serial.print(',');
  Serial.print(Bot.ax,5);
  Serial.print(',');
  Serial.print(Bot.ay,5);
  Serial.print(',');
  Serial.print(Bot.az,5);
  Serial.print(',');
  Serial.print(Bot.gx,5);
  Serial.print(',');
  Serial.print(Bot.gy,5);
  Serial.print(',');
  Serial.print(Bot.gz,5);
  Serial.print(',');
  Serial.print(Bot.mx,5);
  Serial.print(',');
  Serial.print(Bot.my,5);
  Serial.print(',');
  Serial.print(Bot.mz,5);
  Serial.print(',');
  Serial.print(Bot.latitude, 6);
  Serial.print(',');
  Serial.print(Bot.longitude, 6);
  Serial.print(',');
  Serial.print(tinyGPS.altitude.meters());
  Serial.print(',');
  Serial.println(tinyGPS.course.deg());
} 

void SendingRawData(void){
  Serial.print(Bot.ax,5);
  Serial.print(',');
  Serial.print(Bot.ay,5);
  Serial.print(',');
  Serial.print(Bot.az,5);
  Serial.print(',');
  Serial.print(Bot.gx,5);
  Serial.print(',');
  Serial.print(Bot.gy,5);
  Serial.print(',');
  Serial.print(Bot.gz,5);
  Serial.print(',');
  Serial.print(Bot.mx,5);
  Serial.print(',');
  Serial.print(Bot.my,5);
  Serial.print(',');
  Serial.print(Bot.mz,5);
  Serial.print(',');
  Serial.print(tinyGPS.location.lat(), 6);
  Serial.print(',');
  Serial.print(tinyGPS.location.lng(), 6);
  Serial.print(',');
  Serial.print(tinyGPS.altitude.meters());
  Serial.print(',');
  Serial.println(tinyGPS.course.deg());
}