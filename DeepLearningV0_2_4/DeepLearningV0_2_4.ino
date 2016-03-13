#include <Servo.h>
#include <Wire.h>
#include "PID_v1.h"
#include <math.h>
#include "NNPID.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_LSM9DS0.h"
#include "Kalman.h"
#include "PinChangeInt.h"
#include "TinyGPS++.h"
//#include "TinyGPS.h"
#include "MatrixMath.h"
float pi = 3.1415926;

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
//Kalman Filter Initialization of Variables
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TinyGPSPlus gps;
double dLat, dLng;
//TinyGPS gps;
//float flat, flon;
//unsigned long fix_age;
unsigned long time;

int i = 1;
float cosLat = 0.0;  
int64_t Rearth = 6378137;
float RcovarianceMatrix[2][2] = {
         {2.21017383364137, 3.51637078682249},
         {3.51637078682249, 13.8032720993553},
  };
  float QcovarianceMatrix[4][4] = {
         {0.01,    0,       0,      0    },
         {0,       0.01,    0,      0    },
         {0,       0,       0.001,  0    },
         {0,       0,       0,      0.001},
  };
float Hmatrix[2][4] = {
      {1,0,0,0},
      {0,1,0,0},
  };
float HmatrixTranspose[4][2] = {
            {1.0, 0.0},
            {0.0, 1.0},
            {0.0, 0.0},
            {0.0, 0.0},
      };  
float PerrorCovariance[4][4] = {
         {0.001, 0,     0,    0   },
         {0,     0.001, 0,    0   },
         {0,     0,     0.02, 0   },
         {0,     0,     0,    0.02},
};
float delta_T = 0; //((float) (GPS_data[i][2] - GPS_data[i-1][2])) / 1000; //Time elapsed in seconds
float data[2] = {0,0}; //{(float) (GPS_data[i][0]-GPS_data[0][0])*pi/180*Rearth/10000000,(float) (GPS_data[i][1]-GPS_data[0][1])*pi/180*Rearth*cosLat/10000000};
float Xstate[4][1] = {
        {data[0]},
        {data[1]},
        {0.0    },
        {0.0    },
};
int64_t GPS_data[1][3] = {
        {0,0,0},
};

int64_t prevGPS_data[1][3] = {
        {0,0,0},
};
int64_t firstGPS_data[1][3] = {
        {0,0,0},
};

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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Kalman Filter Setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial1.begin(9600);
  time = millis();
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Kalman Filter Data Update
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  if(Serial1.available()) // might need to be a while loop as per source
  {
    if(gps.encode(Serial1.read()))
    {
      time = millis();
      prevGPS_data[0][0] = GPS_data[0][0];
      prevGPS_data[0][1] = GPS_data[0][1];
      prevGPS_data[0][2] = GPS_data[0][2];
      displayInfo();
//      if (gps.location.isValid())
//      {
//        dLat = gps.location.lat();
//        dLng = gps.location.lng();
//      }
      //gps.f_get_position(&flat, &flon, &fix_age);
      GPS_data[0][0] = dLat; //dLat becomes flat if using TinyGPS
      GPS_data[0][1] = dLng; //dLng becomes flon if using TinyGPS
      GPS_data[0][2] = (uint64_t) time;
    
      if ((GPS_data[0][0] > 0) && (GPS_data[0][1] < 0)) //Test for valid GPS data in Continental U.S. 
      {
        Kalman();
      }       
    }
  }
  else if ((i > 1) && (millis() > time + 500)) //Tests if GPS is not available for more than 1 second
  {
    Serial.println(F("No GPS detected: check wiring."));
    KalmanNoData();
  }
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

void Kalman(){
          if (i == 1){ //Initialize cosLat            
            firstGPS_data[0][0] = GPS_data[0][0];
            firstGPS_data[0][1] = GPS_data[0][1];
            firstGPS_data[0][2] = GPS_data[0][2]; 
            cosLat = cos((float) firstGPS_data[0][0]/10000000*pi/180);     
          }              
          delta_T = ((float) (GPS_data[0][2] - prevGPS_data[0][2])) / 1000; //Time elapsed in seconds
          float Amatrix[4][4] = {
                 {1, 0,  delta_T, 0      },
                 {0, 1,  0,       delta_T},
                 {0, 0,  1,       0      },
                 {0, 0,  0,       1      },
          };          
          float nextXstateEstimate[4][1] = {
                {0},
                {0},
                {0},
                {0},
          };    
          Matrix.Multiply((float*) Amatrix, (float*) Xstate, 4, 4, 1, (float*) nextXstateEstimate);           
          /*************Line 78 of Matlab code*************/
          float PerrorCovarianceEstimate[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float AmatrixTranspose[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float IntermediateProductMatrix[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float IntermediateSumMatrix[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };          
          Matrix.Transpose((float*) Amatrix, 4 ,4, (float *) AmatrixTranspose);
          Matrix.Multiply((float*) Amatrix, (float*) PerrorCovariance, 4,4,4, (float*) IntermediateProductMatrix);
          Matrix.Multiply((float*) IntermediateProductMatrix, (float*) AmatrixTranspose, 4,4,4, (float*) IntermediateSumMatrix);
          Matrix.Add((float*) IntermediateSumMatrix, (float*) QcovarianceMatrix, 4, 4, (float*) PerrorCovarianceEstimate);
          /*************Line 81 of Matlab code*************/
          float KalmanGain[4][2] = {
                {0.0, 0.0},
                {0.0, 0.0},
                {0.0, 0.0},
                {0.0, 0.0},
          };    
          float IntermediateProductMatrix2[2][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float IntermediateProductMatrix3[2][2] = {
                {0.0, 0.0},
                {0.0, 0.0}, 
          };
          float IntermediateQuotientMatrix[4][2] = {
                {0.0, 0.0},
                {0.0, 0.0},
                {0.0, 0.0},
                {0.0, 0.0},
          };
          float IntermediateSumMatrix2[2][2] = {
                {0.0, 0.0}, 
                {0.0, 0.0},
          };
          Matrix.Multiply((float*) Hmatrix, (float*) PerrorCovarianceEstimate, 2,4,4, (float*) IntermediateProductMatrix2);
          Matrix.Multiply((float*) IntermediateProductMatrix2, (float*) HmatrixTranspose, 2,4,2, (float*) IntermediateProductMatrix3);
          Matrix.Add((float*) IntermediateProductMatrix3, (float*) RcovarianceMatrix, 2, 2, (float*) IntermediateSumMatrix2);
          int flag = Matrix.Invert((float*) IntermediateSumMatrix2, 2); //The argument matrix gets inverted; no need to create temp matrix.
          if (flag == 0){
            Serial.println("CANT INVERT");
          }
          Matrix.Multiply((float*) PerrorCovarianceEstimate, (float*) HmatrixTranspose, 4,4,2, (float*) IntermediateQuotientMatrix);
          Matrix.Multiply((float*) IntermediateQuotientMatrix, (float*) IntermediateSumMatrix2, 4,2,2, (float*) KalmanGain);
          data[0] = (float) (GPS_data[0][0]-firstGPS_data[0][0])*pi/180*Rearth/10000000;
          data[1] = (float) (GPS_data[0][1]-firstGPS_data[0][1])*pi/180*Rearth*cosLat/10000000;     
          float ZkTranspose[2][1] = { //We only need the transposed version of this, so we do it right here.
                {data[0]},
                {data[1]},
                }; 
          float IntermediateProductMatrix4[2][1] = {
                {0.0}, 
                {0.0},
          };
          float IntermediateProductMatrix5[4][1] = {
                {0.0}, 
                {0.0},
                {0.0}, 
                {0.0},
          };
          float IntermediateSubtractionMatrix[2][1] = {
                {0.0}, 
                {0.0},
          };
          Matrix.Multiply((float*) Hmatrix, (float*) nextXstateEstimate, 2,4,1, (float*) IntermediateProductMatrix4);
          Matrix.Subtract((float*) ZkTranspose, (float*) IntermediateProductMatrix4, 2, 1, (float*) IntermediateSubtractionMatrix);
          Matrix.Multiply((float*) KalmanGain, (float*) IntermediateSubtractionMatrix, 4,2,1, (float*) IntermediateProductMatrix5); //Reuse intermediate matrix because it has appropriate dimensions
          Matrix.Add((float*) nextXstateEstimate, (float*) IntermediateProductMatrix5, 4,1, (float*) Xstate); //NO NEED TO TRANSPOSE X STATE. WE DID THAT IN MATLAB FOR CONVENIENCE
          /*************Line 84 of Matlab code*************/
          Matrix.Multiply((float*) KalmanGain, (float*) Hmatrix, 4,2,4, (float*) IntermediateProductMatrix); //Reuse this intermediate matrix because it's 4x4 and we need 4x4
          float IdentityMatrix[4][4] = {
                {1,0,0,0},
                {0,1,0,0},
                {0,0,1,0},
                {0,0,0,1},
          };
          Matrix.Subtract((float*) IdentityMatrix, (float*) IntermediateProductMatrix, 4,4, (float*) IntermediateQuotientMatrix); //Reuse this intermediate matrix because it's 4x4 and we need 4x4.
          Matrix.Multiply((float*) IntermediateQuotientMatrix, (float*) PerrorCovarianceEstimate, 4,4,4, (float*) PerrorCovariance);              
          Serial.print(i);Serial.print(",");Serial.print(Xstate[0][0]);Serial.print(",");Serial.print(Xstate[1][0]);Serial.print(",");Serial.print(Xstate[2][0]);Serial.print(",");Serial.print(Xstate[3][0]);Serial.print(",");Serial.print(ZkTranspose[0][0]);Serial.print(",");Serial.print(ZkTranspose[1][0]);Serial.print(",");Serial.print((int32_t) GPS_data[0][0]);Serial.print(",");Serial.print((int32_t) GPS_data[0][1]);Serial.print(",");Serial.println((int32_t) GPS_data[0][2]);
          i = i + 1; 
        
}

void KalmanNoData(){
          uint64_t currentTime = (int32_t) millis();  
          delta_T = (float) (currentTime - time) / 1000; //Time elapsed in seconds
          float Amatrix[4][4] = {
                 {1, 0,  delta_T, 0      },
                 {0, 1,  0,       delta_T},
                 {0, 0,  1,       0      },
                 {0, 0,  0,       1      },
          };          
          float nextXstateEstimate[4][1] = {
                {0},
                {0},
                {0},
                {0},
          };    
          Matrix.Multiply((float*) Amatrix, (float*) Xstate, 4, 4, 1, (float*) nextXstateEstimate);
          Xstate[0][0] = nextXstateEstimate[0][0];
          Xstate[1][0] = nextXstateEstimate[1][0];
          Xstate[2][0] = nextXstateEstimate[2][0];
          Xstate[3][0] = nextXstateEstimate[3][0];
                    
          /*************Line 78 of Matlab code*************/
          float AmatrixTranspose[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float IntermediateProductMatrix[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };
          float IntermediateSumMatrix[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          }; 
          float PerrorCovarianceEstimate[4][4] = {
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0, 0.0},
          };         
          Matrix.Transpose((float*) Amatrix, 4 ,4, (float *) AmatrixTranspose);
          Matrix.Multiply((float*) Amatrix, (float*) PerrorCovariance, 4,4,4, (float*) IntermediateProductMatrix);
          Matrix.Multiply((float*) IntermediateProductMatrix, (float*) AmatrixTranspose, 4,4,4, (float*) IntermediateSumMatrix);
          Matrix.Add((float*) IntermediateSumMatrix, (float*) QcovarianceMatrix, 4, 4, (float*) PerrorCovarianceEstimate);
          for (int j = 0; j < 4; j++){
            for (int k = 0; k < 4; k++){
              PerrorCovariance[j][k] = PerrorCovarianceEstimate[j][k];
            }
          }          
          Serial.print(i);Serial.print(",");Serial.print(Xstate[0][0]);Serial.print(",");Serial.print(Xstate[1][0]);Serial.print(",");Serial.print(Xstate[2][0]);Serial.print(",");Serial.print(Xstate[3][0]);Serial.print(",");Serial.print(9999);Serial.print(",");Serial.print(9999);Serial.print(",");Serial.print(-1.0);Serial.print(",");Serial.print(-1.0);Serial.print(",");Serial.println((uint32_t)currentTime);
          i = i + 1;  
          time = millis();       
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    dLat = gps.location.lat();
    Serial.print(dLat, 6);
    Serial.print(F(","));
    dLng = gps.location.lng();
    Serial.print(dLng, 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Time:  "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

