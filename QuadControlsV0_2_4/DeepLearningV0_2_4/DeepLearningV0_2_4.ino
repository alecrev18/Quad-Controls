#include <Servo.h>
#include <Wire.h>
#include <PID_v1.h>
#include <math.h>
#include "NNPID.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_LSM9DS0.h"
#include "Kalman.h"
#include "PinChangeInt.h"

//Sensor Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000); 

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

#define Rad_To_Deg 57.2958
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanz;

double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle;

uint32_t timer;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Receiver variables
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define channel1_pin 1
#define channel2_pin 2
#define channel3_pin 3
#define channel4_pin 4
#define channel5_pin 5
#define channel6_pin 6

int channel1 = 0;
int channel2 = 0;
int channel3 = 0;
int channel4 = 0;
int channel5 = 0;
int channel6 = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Servo firstESC; //Create as many Servo objects you want. You can controll 2 or more Servos at the same time
Servo secondESC;
Servo thirdESC;
Servo fourthESC;

int throttle = 0;

double Input[3]={0,0,0}; 
double PIDout[3] = {0,0,0};
double Output[4]={1060,1060,1060,1060};
double Setpoint[3]={0,0,0};

PID PitchPID(&kalAngleX, &PIDout[0], &Setpoint[0],1.55,0.009,0, DIRECT);//DELETE
PID RollPID(&kalAngleY, &PIDout[1], &Setpoint[1],1.55,0.009,0, DIRECT);
//PID YawPID(&Input[2], &Output[2], &Setpoint[2],5,0.5,0, DIRECT);

String streamRead = "";

//NNPID PitchNN(0.4);
//NNPID RollNN(0.4);
//NNPID YawNN(0.4);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  
  //Setpoint set to (0,0,0) right now
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  firstESC.attach(2);  // Check PIN NUMBER
  secondESC.attach(4); //""
  thirdESC.attach(6);  //""
  fourthESC.attach(8); //""
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  Serial.begin(9600); 

  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));

  displaySensorDetails();
  configureSensor();
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  double roll = atan2(accel.acceleration.y,accel.acceleration.z)*Rad_To_Deg;
  double pitch = atan(-accel.acceleration.x / sqrt(accel.acceleration.y * accel.acceleration.y + accel.acceleration.z * accel.acceleration.y))* Rad_To_Deg;
  
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  
  timer = micros();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
 
//  firstESC.writeMicroseconds(1060);//write output that the ESC likes Probably not nessecary
//  secondESC.writeMicroseconds(1060);
//  thirdESC.writeMicroseconds(1060);
//  fourthESC.writeMicroseconds(1060);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Receiver Setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println(F("Done with Setup"));
  delay(500);
  
}

void loop() {

//  if(Serial.available())//Uses Serial monitor to update setpoint if anything is there
//  {
//      Setpoint = Serial.parseFloat();
//  }

    sensors_event_t accel, mag, gyro, temp;

    lsm.getEvent(&accel, &mag, &gyro, &temp); 
    
    double dt = (double)(micros()-timer)/1000000;
    timer = micros();
  
    gyroX = gyro.gyro.x;
    gyroY = gyro.gyro.y;
    gyroZ = gyro.gyro.z;

    double gyroXrate = gyroX / 131.0;
    double gyroYrate = gyroY / 131.0;

    accX = accel.acceleration.x;
    accY = accel.acceleration.y;
    accZ = accel.acceleration.z;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  double roll = atan2(accY,accZ)*Rad_To_Deg;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ))* Rad_To_Deg;

  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);}
  else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
    
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;  
  
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

  channel1 = pulseIn(channel1_pin, HIGH, 20000);
  channel2 = pulseIn(channel2_pin, HIGH, 20000);
  channel3 = pulseIn(channel3_pin, HIGH, 20000);
  channel4 = pulseIn(channel4_pin, HIGH, 20000);
//  channel5 = pulseIn(channel5_pin, HIGH, 20000);
//  channel6 = pulseIn(channel6_pin, HIGH, 20000);
  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  

////    if(Setpoint[0]-Input[0]>0.5 || Setpoint[0]-Input[0]<-0.5)
//    {
//        PitchNN.updateSetpoint(Setpoint[0]);
//        PitchNN.updatePosition(Input[0]);
//        PitchNN.calculateOutputOfNeurons();
//        PitchNN.calculateInputOfNeurons();
//        PitchNN.updateWeights();
//    }
//
////        if(Setpoint[1]-Input[1]>0.5 || Setpoint[1]-Input[1]<-0.5)
//    {
//        RollNN.updateSetpoint(Setpoint[1]);
//        RollNN.updatePosition(Input[1]);
//        RollNN.calculateOutputOfNeurons();
//        RollNN.calculateInputOfNeurons();
//        RollNN.updateWeights();
//    }
//
////        if(Setpoint[2]-Input[2]>0.5 || Setpoint[2]-Input[2]<-0.5)
//    {
//        YawNN.updateSetpoint(Setpoint[2]);
//        YawNN.updatePosition(Input[2]);
//        YawNN.calculateOutputOfNeurons();
//        YawNN.calculateInputOfNeurons();
//        YawNN.updateWeights();
//    }

    PitchPID.Compute();
    RollPID.Compute();
//    YawPID.Compute();

//  Source: http://www.benripley.com/development/quadcopter-source-code-from-scratch/

  Output[0] = throttle+PIDout[0];//+YawNN.Output();
  Output[1] = throttle+PIDout[1];//-YawNN.Output();
  Output[2] = throttle-PIDout[0];//+YawNN.Output();
  Output[3] = throttle-PIDout[1];//-YawNN.Output();
  
  firstESC.writeMicroseconds(Output[0]);
  secondESC.writeMicroseconds(Output[1]);
  thirdESC.writeMicroseconds(Output[2]);
  fourthESC.writeMicroseconds(Output[3]);

    Serial.print(" Roll, ");
    Serial.print(kalAngleX);
    Serial.print(" Pitch, ");
    Serial.print(kalAngleY);
    Serial.print(" Channel1, ");
//    Serial.print(unChannel1InShared);
    Serial.println();
//    PitchNN.Printx();
//    PitchNN.Printu();
//    PitchNN.Printw();  

//delay(250);  
}

void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
//  Serial.println(F("------------------------------------"));
//  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
//  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
//  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
//  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
//  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
//  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
//  Serial.println(F("------------------------------------"));
//  Serial.println(F(""));
//
//  Serial.println(F("------------------------------------"));
//  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
//  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
//  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
//  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
//  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
//  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
//  Serial.println(F("------------------------------------"));
//  Serial.println(F(""));
//
//  Serial.println(F("------------------------------------"));
//  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
//  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
//  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
//  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
//  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
//  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
//  Serial.println(F("------------------------------------"));
//  Serial.println(F(""));
//
//  Serial.println(F("------------------------------------"));
//  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
//  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
//  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
//  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
//  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
//  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
//  Serial.println(F("------------------------------------"));
//  Serial.println(F(""));
  
  delay(500);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
//  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
//  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

