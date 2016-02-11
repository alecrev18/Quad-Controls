#include <Servo.h>
#include <Wire.h>
#include <PID_v1.h>
#include <math.h>
#include "NNPID.h"

//Sensor Stuff ///////////////////////////////////////////////////////////////////////////////////////////////////

#define byte uint8_t

int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1};

#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13 

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Servo firstESC; //Create as many Servo objects you want. You can controll 2 or more Servos at the same time
Servo secondESC;
Servo thirdESC;
Servo fourthESC;

int throttle = 0;

double Input[3]={0,0,0}; 
double Output[3]={0,0,0};
double Setpoint[3]={0,0,0};

PID PitchPID(&Input[0], &Output[0], &Setpoint[0],1.55,0.009,0, DIRECT);//DELETE
PID RollPID(&Input[1], &Output[1], &Setpoint[1],1.55,0.009,0, DIRECT);
PID YawPID(&Input[2], &Output[2], &Setpoint[2],5,0.5,0, DIRECT);

int Motor1 = 0;
int Motor2 = 0;
int Motor3 = 0;
int Motor4 = 0;

String streamRead = "";

float buff[3]= {0,0,0};
float accum;

int Chan1 = 2;      //Pitch from Remote
int Chan1IN = 1460;
int Chan2 = 3;      //Roll
int Chan2IN = 1460;
int Chan3 = 4;      //Throttle
int Chan3IN = 0;
int Chan4 = 5;      //Yaw
int Chan4IN = 1460;

int gain=1;

//NNPID PitchNN(0.4);
//NNPID RollNN(0.4);
//NNPID YawNN(0.4);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  Input[0] = ToDeg(pitch);
  Input[1] = ToDeg(roll);
  Input[2] = ToDeg(yaw);
  
  //Setpoint set to (0,0,0) right now
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  firstESC.attach(8);  // Check PIN NUMBER
  secondESC.attach(9); //""
  thirdESC.attach(10);  //""
  fourthESC.attach(11); //""

  pinMode(Chan1, INPUT);
  pinMode(Chan2, INPUT);
  pinMode(Chan3, INPUT);
  pinMode(Chan4, INPUT);

  PitchPID.SetMode(AUTOMATIC);
  RollPID.SetMode(AUTOMATIC);
  YawPID.SetMode(AUTOMATIC);

  PitchPID.SetOutputLimits(-255, 255);
  RollPID.SetOutputLimits(-255, 255);
  YawPID.SetOutputLimits(-255, 255);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  Serial.begin(115200); 

  pinMode (STATUS_LED,OUTPUT);  // Status LED
  
  I2C_Init();

  digitalWrite(STATUS_LED,LOW);
  delay(1500);

  Accel_Init();
  Compass_Init();
  Gyro_Init();

  delay(20);

  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;
    
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

  delay(2000);
  digitalWrite(STATUS_LED,HIGH);

  timer=millis();
  delay(20);
  counter=0;

  firstESC.writeMicroseconds(1060);//write output that the ESC likes Probably not nessecary
  secondESC.writeMicroseconds(1060);
  thirdESC.writeMicroseconds(1060);
  fourthESC.writeMicroseconds(1060);
  
  delay(1000);
}

void loop() {

//  if(Serial.available())//Uses Serial monitor to update setpoint if anything is there
//  {
//      Setpoint = Serial.parseFloat();
//  }
  
  if((millis()-timer)>=30)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***


//    accum = 0;//This is where I average the sensor data 
//              //Probably a better way to do this
//    for( int i=2; i>1; i++)
//    {
//      buff[i] = buff[i-1];
//      accum = accum+buff[i];
//    }
//    buff[0]= (float) ToDeg(pitch);
//    accum = accum+buff[0];
//    
//    Input = accum/3;
//    Input = (float) ToDeg(pitch);

//    if(Setpoint-Input>0.5 || Setpoint-Input<-0.5)
//    {
//        PitchNN.updateSetpoint(Setpoint);
//        PitchNN.updatePosition(Input);
//        PitchNN.calculateOutputOfNeurons();
//        PitchNN.calculateInputOfNeurons();
//        PitchNN.updateWeights();
//    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Input[0] = ToDeg(pitch);
    Input[1] = ToDeg(roll);
    Input[2] = ToDeg(yaw);

    Chan1IN = pulseIn(Chan1,HIGH);
    Chan2IN = pulseIn(Chan2,HIGH);
    Chan3IN = pulseIn(Chan3,HIGH);
    Chan4IN = pulseIn(Chan4,HIGH);

    Setpoint[0] = ((double)Chan2IN-1460)/10;
    Setpoint[1] = ((double)Chan1IN-1460)/10;
    Setpoint[2] = ((double)Chan4IN-1460)/10;

    throttle = (int)Chan3IN;// - 1060;

//    if(throttle <0)
//    {
//      throttle = 0;
//    }
//    throttle = 2*throttle;
//
//    for(int i=0;i<3;i++)
//    {
//      if(Setpoint[i]<-100 && Setpoint[i]>100)
//      {
//        Setpoint[i] =0; 
//      }
//    }

    PitchPID.Compute();
    RollPID.Compute();
    YawPID.Compute();
    
  }
  
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

//  Source: http://www.benripley.com/development/quadcopter-source-code-from-scratch/

  gain =20;
  
//  Output[0] = throttle-PitchNN.Output()*gain;//+YawNN.Output();
//  Output[1] = throttle-RollNN.Output()*gain-PitchNN.Output()*gain;//-YawNN.Output();
//  Output[2] = throttle+PitchNN.Output()*gain;//+YawNN.Output();
//  Output[3] = throttle+RollNN.Output()*gain+PitchNN.Output()*gain;//-YawNN.Output();

  Motor1 = throttle - Output[0]+Output[1];
  Motor2 = throttle - Output[0]-Output[1];
  Motor3 = throttle + Output[1]+Output[0];
  Motor4 = throttle - Output[1]+Output[0];
  
  
  firstESC.writeMicroseconds(Motor1);
  secondESC.writeMicroseconds(Motor2);
  thirdESC.writeMicroseconds(Motor3);
  fourthESC.writeMicroseconds(Motor4);


  Serial.print(ToDeg(pitch));
  Serial.print(",");
  Serial.print(Output[0]);
  Serial.print(",");
  Serial.print(Setpoint[0]);
  Serial.println();
  
//    Serial.print(" Pitch, ");
//    Serial.print(ToDeg(pitch));
//    Serial.print(" Input ");
//    Serial.print(Input);
//    Serial.print(" Setpoint ");
//    Serial.print(Setpoint);
//    Serial.print(" Output ");
//    Serial.print(Output);
//    Serial.println();
//    PitchNN.Printx();
//    PitchNN.Printu();
//    PitchNN.Printw();    
}



