
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <IBusBM.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

///*
//RC Self Balancing Golf Bag Robot 
//
//--------------
//|Description:|
//--------------
//This code drives a self balancing golf bag robot.
//
//
//--------------
//|CONNECTIONS:|
//--------------
//
//Motor Controlers: RioRand 400W 6-60V PWM DC Brushless Electric Motor Speed Controller with Hall
//---------------------------------------

//
//RC Interface: FR Sky FS-iA6B
//---------------------------------------
//IBUS Interface
//
//
//AUTHOR: Ryan McGuire
//DATE: 6/4/2020
//
// */



//-------------------------------------------------------------------
//-------------------------- PID Variables --------------------------
//-------------------------------------------------------------------

double Pk1 = 15;          // balancing PID
double Ik1 = 0  ;          //Increase I to get more response at small errors
double Dk1 = 1.5;           //Increase this after D is close

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - balancing IMU

float SetpointTrim1 = -13.75;

//-------------------------------------------------------------------
//-------------------------------------------------------------------



//-------------------------------------------------------------------
//-------------------------- MPU Variables --------------------------
//-------------------------------------------------------------------

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//-------------------------------------------------------------------
//-------------------------------------------------------------------


//-------------------------------------------------------------------
//------------------------ Fifo Managment ---------------------------
//-------------------------------------------------------------------

//10/13/18 gfp: added to support Homer Creutz's algorithm testing
const int PACKET_SIZE = 28;
//const int LOOP_DELAY_MSEC = 100;
const int LOOP_DELAY_MSEC = 360;
const int INT_MONITOR_PIN = 50;
const int MAX_FIFO_BYTES = 1024;
const int MAX_LINE_LENGTH = 80; //added 10/20/19
long int startMsec = 0; //added 10/20/19
int GetPacketLoopCount,OuterGetPacketLoopCount;
 
//-------------------------------------------------------------------
//-------------------------------------------------------------------

//-------------------------------------------------------------------
//------------------------- Radio Variables -------------------------
//-------------------------------------------------------------------

IBusBM IBus;    // IBus object

int ch1; // Steering
int steer;

int ch2; // Throttle
int throt;

int ch3; // unused

int ch4; // unused

int ch5; // KillSwitch
int kill;

int ch6; // Debug
int dbg;

int ch7; //Angle Offset
float OffsetTrim;

int ch8; // Debug speed


//-------------------------------------------------------------------
//-------------------------------------------------------------------


//-------------------------------------------------------------------
//------------------------ Motion Variables -------------------------
//-------------------------------------------------------------------

float pitch;
float roll;
long velocity;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//-------------------------------------------------------------------
//-------------------------------------------------------------------


//-------------------------------------------------------------------
//------------------------ Motor Variables --------------------------
//-------------------------------------------------------------------

int LMot;
int LPin = 44;
int LDirPin = 42;

int RMot;
int RPin = 45;
int RDirPin = 43;

//-------------------------------------------------------------------
//-------------------------------------------------------------------

//-------------------------------------------------------------------
//----------------------- Timing Variables --------------------------
//-------------------------------------------------------------------

unsigned long currentMillis;

long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timers
unsigned long count;

int loopTime;
int previousLooptime;

unsigned long previousSafetyMillis;

//-------------------------------------------------------------------
//-------------------------------------------------------------------


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

int IMUdataReady = 0;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
    IMUdataReady = 1;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif



    Serial.begin(115200);
    
    //Setup Radio Connection
    IBus.begin(Serial1);    // iBUS object connected to serial0 RX pin

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
  
    //MPU Calibration
    mpu.setXGyroOffset(-741);
    mpu.setYGyroOffset(-56);
    mpu.setZGyroOffset(-3);
    mpu.setXAccelOffset(2599);
    mpu.setYAccelOffset(-1934);
    mpu.setZAccelOffset(1900);
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

    // Setup PID controllers
    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-127, 127);
    PID1.SetSampleTime(10);

    pinMode(LPin, OUTPUT); // Set our output pins as such
    pinMode(RPin, OUTPUT);
    pinMode(LDirPin, OUTPUT);
    pinMode(RDirPin, OUTPUT);

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
            previousMillis = currentMillis; 
    
            // measure loop time / optional
            loopTime = currentMillis - previousLooptime;
            //Serial.print("time: ");
            //Serial.print(loopTime);
            //Serial.print("\t");
            previousLooptime = currentMillis;

            //Call the function to recieve the remote values
            RadioRead();

            

            

            // see if IMU data is ready based on the interrupt
            if (IMUdataReady == 1) {
                readAngles();             // call the function on the other tab to get the IMU data
            }

            pitch = (ypr[1] * 180/M_PI);           
            roll = (ypr[2] * 180/M_PI)+2;      

            OffsetTrim=mapfloat(ch7,1000,2000,-3,3);
            throt=mapfloat(ch2,1000,2000,-6,6);
            steer=map(ch1,1000,2000,-1,1);
            kill=map(ch5,1000,2000,0,1);

           
            
            Setpoint1=SetpointTrim1+OffsetTrim+throt;
            //Serial.println(Setpoint1);           
            Input1 = roll;
            PID1.Compute();



            //Disable motors if off by more than 10 degrees.  USEFULL FOR TUNING SAFETY
            if(!kill){
              RMot=0;
              LMot=0;
            }
            else{
              if(steer==0){
                RMot=Output1;
                LMot=Output1;
              }
              if(steer==1){
                RMot=0;
                LMot=Output1;
              }
              if(steer==-1){
                RMot=Output1;
                LMot=0;
              }

            }
            PWMDrive(LPin,RPin,LDirPin,RDirPin,LMot,RMot);

            

//            Serial.print(roll);
//            Serial.print(" ");
//            Serial.print(Setpoint1);
//            Serial.print(" ");
//            Serial.print(Input1);
//            Serial.print(" ");
//            Serial.println(Output1);
    }
}





















//
//#include "I2Cdev.h"
//
//#include "MPU6050_6Axis_MotionApps20.h"
//
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//    #include "Wire.h"
//#endif
//
//
//MPU6050 mpu;
//
//#define OUTPUT_READABLE_YAWPITCHROLL
//
//#define LED_PIN 13 
//bool blinkState = false;
//
//
////Output Variables
////----------------
//int LMot;
//int LPin = 45;
//int LDirPin = 43;
//
//int RMot;
//int RPin = 44;
//int RDirPin = 42;
//
//
//
//// MPU control/status vars
//bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
//uint8_t fifoBuffer[64]; // FIFO storage buffer
//
//// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//
//// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
////*************************************************************************************************************************************************************************
////                                                 Variables
//
//float OldP = 0;         // Previous value used to calculate change in P //  DELTA P
//float P = 0;            //  Proportional component
//float I = 0;            //  Integral        just the sum of P over time
//float OldI = 0;         //  previous value of I for calculation of Delta I
//float D = 0;            //  Differential       D = P - OldP
//float bp = -60;         // balance point
//float pwm = 0;          // value of Pulse Width Modulation  to ENA ENB
//long a = 0;             // L298N to IN 1 to 4
//long b = 0;             //
//const int PinR1 = 5;    //  arduino  pin 5 to l298  pin IN4
//const int PinR2 = 6;    //  arduino  pin 6 to l298  pin IN3
//const int PinL1 = 7;    //  arduino  pin 7 to l298  pin IN1
//const int PinL2 = 8;    //  arduino  pin 8 to l298  pin IN2
//const int PwmR  = 9;    //  arduino  pin 9 to l298  pin ENB
//const int PwmL  = 10;   //  arduino  pin 10 to l298  pin ENA
////****************************************************************************************************************************************************
//
//
//// ================================================================
//// ===               INTERRUPT DETECTION ROUTINE                ===
//// ================================================================
//
//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//void dmpDataReady() {
//    mpuInterrupt = true;
//}
//
//
//
//
//
//
//
//
//
//
//
//// ================================================================
//// ===                      INITIAL SETUP                       ===
//// ================================================================
//
//void setup() {
////*********************************************************************************************************************************************************
//// arduino to l298 pins hopefully self explanatory
//      pinMode(PinR1,OUTPUT);
//      pinMode(PinR2,OUTPUT);
//      pinMode(PinL1,OUTPUT);
//      pinMode(PinL2,OUTPUT);
//
//  //Output Pins
//  pinMode(LPin, OUTPUT); // Set our output pins as such
//  pinMode(RPin, OUTPUT);
//  pinMode(LDirPin, OUTPUT);
//  pinMode(RDirPin, OUTPUT);
//
//      
////**********************************************************************************************************************************************************
//    // join I2C bus (I2Cdev library doesn't do this automatically)
//    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//        Wire.begin();
//        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
//    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//        Fastwire::setup(400, true);
//    #endif
//
//    // initialize serial communication
//    Serial.begin(57600);
//    while (!Serial); // 
//
//    // initialize device
//    Serial.println(F("Initializing I2C devices..."));
//    mpu.initialize();
//
//    // verify connection
//    Serial.println(F("Testing device connections..."));
//    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//
//
//// load and configure the DMP
//    Serial.println(F("Initializing DMP..."));
//    devStatus = mpu.dmpInitialize();
//
//    // supply your own gyro offsets here, scaled for min sensitivity
// //****************************************************************************************************************************************
// //  use calibration program to get your own values
//    mpu.setXGyroOffset(44);//(220);
//    mpu.setYGyroOffset(-21);//(76);
//    mpu.setZGyroOffset(-30);//(-85);
//    mpu.setXAccelOffset(-1875);//(1788); // 1688 factory default for my test chip
//    mpu.setYAccelOffset(-1426);
//    mpu.setZAccelOffset(2215);
// //****************************************************************************************************************************************
//    // make sure it worked (returns 0 if so)
//    if (devStatus == 0) {
//        // turn on the DMP, now that it's ready
//        Serial.println(F("Enabling DMP..."));
//        mpu.setDMPEnabled(true);
//
//        // enable Arduino interrupt detection
//        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
//        attachInterrupt(0, dmpDataReady, RISING);
//        mpuIntStatus = mpu.getIntStatus();
//
//        // set our DMP Ready flag so the main loop() function knows it's okay to use it
//        Serial.println(F("DMP ready! Waiting for first interrupt..."));
//        dmpReady = true;
//
//        // get expected DMP packet size for later comparison
//        packetSize = mpu.dmpGetFIFOPacketSize();
//    } else {
//        // ERROR!
//        // 1 = initial memory load failed
//        // 2 = DMP configuration updates failed
//        // (if it's going to break, usually the code will be 1)
//        Serial.print(F("DMP Initialization failed (code "));
//        Serial.print(devStatus);
//        Serial.println(F(")"));
//    }
//
//    // configure LED for output
//    pinMode(LED_PIN, OUTPUT);
//
//}
//
//
//
//// ================================================================
//// ===                    MAIN PROGRAM LOOP                     ===
//// ================================================================
//
//void loop() {
//    // if programming failed, don't try to do anything
//    if (!dmpReady) return;
//
//    // wait for MPU interrupt or extra packet(s) available
//    while (!mpuInterrupt && fifoCount < packetSize) {
//        // other program behavior stuff here
//   
//               
//     }
//
//    // reset interrupt flag and get INT_STATUS byte
//    mpuInterrupt = false;
//    mpuIntStatus = mpu.getIntStatus();
//
//    // get current FIFO count
//    fifoCount = mpu.getFIFOCount();
//
//    // check for overflow (this should never happen unless our code is too inefficient)
//    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
//        // reset so we can continue cleanly
//        mpu.resetFIFO();
//        Serial.println(F("FIFO overflow!"));
//
//    // otherwise, check for DMP data ready interrupt (this should happen frequently)
//    } else if (mpuIntStatus & 0x02) {
//        // wait for correct available data length, should be a VERY short wait
//        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//
//        // read a packet from FIFO
//        mpu.getFIFOBytes(fifoBuffer, packetSize);
//       
//        // track FIFO count here in case there is > 1 packet available
//        // (this lets us immediately read more without waiting for an interrupt)
//        fifoCount -= packetSize;
//
//        #ifdef OUTPUT_READABLE_YAWPITCHROLL      
//          // display Euler angles in degrees
//          mpu.dmpGetQuaternion(&q, fifoBuffer);
//          mpu.dmpGetGravity(&gravity, &q);
//          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//         
//   
////*********************************************************************************************************************************************************************
////                                                                    
//// PID control based on Pseudocode from https://en.wikipedia.org/wiki/PID_controller
//// and the balance point idea from https://www.youtube.com/user/jmhrvy1947
//
//          OldP =P;                     // save value of P
//          P = (ypr[2] * 1000) + bp;    // update P from MPU add bp to correct for balance
//          OldI = I;                    // save old I
//          I = I + (P * 0.05) ;        
//          I = I + ((I - OldI)*2  );     // calulate new I
//          if( I >  250 ) I =  250;           // LIMIT  Stop I building up too high
//          if( I < -250 ) I = -250;           // or too low value
//          D = P - OldP;                      //  D differential   change in P
//          pwm = ( P * 1 ) + ( I  ) + ( D * 10 ) ; // P I D  
//         
//          a = 0;
//          b = 0;
//
//          
//
//          
//
//
//
//          
////          if(pwm < 0){    
////            a = 0;
////            b = 1;
////             bp = bp - 0.01;      
////            digitalWrite(13, 0);
////          }
////          if(pwm > 0){    
////            a = 1;
////            b = 0;
////            bp = bp + 0.01;          
////            digitalWrite(13, 1);                                  
////          }
////          /////////////////////////////
////          // remove sign from PWM as - value has no meaning        
////          pwm  = abs(pwm);                  
////          if ( pwm < 0) pwm = 0;
////          if ( pwm > 255) pwm = 255;
//
//          PWMDrive(LPin,RPin,LDirPin,RDirPin,pwm,pwm);
//          
//
//
////
////            if(abs(ypr[2]) < abs(1.1)){
////              analogWrite(PwmR, pwm);
////              digitalWrite(PinR1, a);
////              digitalWrite(PinR2 ,b);
////             
////              analogWrite(PwmL ,pwm);
////              digitalWrite(PinL1 ,a);
////              digitalWrite(PinL2 ,b);
////              }
////           else{
////              analogWrite(PwmR , 0);
////              analogWrite(PwmL , 0);
////              I = 0;
////              bp = -98;
////              delay(1000);
////}
//           
//         
// //********************************************************************************************************************************************************          
//        #endif        
//    }
//
//
//}
//
//
//
//
//
////
//////#include <MPU6050.h>
////#include "MPU6050_6Axis_MotionApps20.h"
////#include <I2Cdev.h>
////#include <IBusBM.h>
////#include "wire.h"
////
////#include <Servo.h>     //Servo library
////#include "CytronMotorDriver.h" //Motor Shield Library
////
/////*
////RC Arduino Robot FR Sky interface
////
////--------------
////|Description:|
////--------------
////This code connects an FRSKY Reciever to 2 DC Motors for controlling a robot.
////
////
////--------------
////|CONNECTIONS:|
////--------------
////
////Motor Shield: Cytron SHIELD-MDD10
////---------------------------------------
////Arduino D3  - Motor Driver PWM 1 Input
////Arduino D4  - Motor Driver DIR 1 Input
////Arduino D9 - Motor Driver PWM 2 Input
////Arduino D10 - Motor Driver DIR 2 Input
////
////RC Interface: FR Sky FS-iA6B
////---------------------------------------
////Arduino D5  - Steering
////Arduino D6  - Throttle
////Arduino D7  - Elevator (Unused)
////
////
////AUTHOR: Ryan McGuire
////DATE: 6/4/2020
////
//// */
////
////
//////Timer Variables
//////---------------
////unsigned long currentMillis;
////long previousMillis = 0;    // set up timers
////long interval = 10;        // time constant for timers
////unsigned long count;
////int loopTime;
////int previousLooptime;
////unsigned long previousSafetyMillis;
//// 
////
////
//////Input Variables
//////---------------
////int ch1; // Steering
////int steer;
////
////int ch2; // Throttle
////int throt;
////
////int ch3; // unused
////
////int ch4; // unused
////
////int ch5; // KillSwitch
////int kill;
////
////int ch6; // Debug
////int dbg;
////
////int ch7; // Debug speed
////int pause;
////
//////Output Variables
//////----------------
////int LMot;
////int LPin = 45;
////int LDirPin = 43;
////
////int RMot;
////int RPin = 44;
////int RDirPin = 42;
////
//////Constants
//////----------------
////int deadband=50;
////
////
///////Radio
//////----------------
////IBusBM IBus;    // IBus object
////
////
////#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
////#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
////bool blinkState = false;
////
////
////
//////MPU Stuff
//////----------------
////MPU6050 mpu;
////
////// MPU control/status vars
////bool dmpReady = false;  // set true if DMP init was successful
////uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
////uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
////uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
////uint16_t fifoCount;     // count of all bytes currently in FIFO
////uint8_t fifoBuffer[64]; // FIFO storage buffer
////
////// orientation/motion vars
////Quaternion q;           // [w, x, y, z]         quaternion container
////VectorInt16 aa;         // [x, y, z]            accel sensor measurements
////VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
////VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
////VectorFloat gravity;    // [x, y, z]            gravity vector
////float euler[3];         // [psi, theta, phi]    Euler angle container
////float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
////
////
////float OldP = 0;         // Previous value used to calculate change in P //  DELTA P
////float P = 0;            //  Proportional component
////float I = 0;            //  Integral        just the sum of P over time
////float OldI = 0;         //  previous value of I for calculation of Delta I
////float D = 0;            //  Differential       D = P - OldP
////float bp = -60;         // balance point
////
////// ================================================================
////// ===               INTERRUPT DETECTION ROUTINE                ===
////// ================================================================
////
////volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
////void dmpDataReady() {
////    mpuInterrupt = true;
////}
////
////////MPU6050 MPU();
//////MPU6050 accelgyro;
////////MPU6050 MPU(0x69); // <-- use for AD0 high
//////int IMUdataReady = 0;
//////volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//////// MPU control/status vars
//////bool dmpReady = false;  // set true if DMP init was successful
//////uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//////uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//////uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//////uint16_t fifoCount;     // count of all bytes currently in FIFO
//////uint8_t fifoBuffer[64]; // FIFO storage buffer
//////
////////Angle Variables
////////----------------
//////int16_t ax, ay, az;
//////int16_t gx, gy, gz;
////////Accelerometer
//////float accAngle;
////////Gyro
//////int16_t gyroX, gyroRate;
//////float gyroAngle=0;
////////
////////Timming
//////unsigned long currTime, prevTime=0, loopTime;
////
////
////void setup() {
////
////  //Input Pins
////  pinMode(22, INPUT); // Set our input pins as such
////  pinMode(23, INPUT);
////  pinMode(41, INPUT); // Set our input pins as such
////  pinMode(24, INPUT);
////
////  //Output Pins
////  pinMode(LPin, OUTPUT); // Set our output pins as such
////  pinMode(RPin, OUTPUT);
////  pinMode(LDirPin, OUTPUT);
////  pinMode(RDirPin, OUTPUT);
////  
////  //Serial Setup
////  Serial.begin(19200); // Pour a bowl of Serial
////  IBus.begin(Serial1);    // iBUS object connected to serial0 RX pin
////  dbg=0;
////
////  
////    // initialize device
////    Serial.println(F("Initializing I2C devices..."));
////    mpu.initialize();
////    pinMode(INTERRUPT_PIN, INPUT);
////
////    // verify connection
////    Serial.println(F("Testing device connections..."));
////    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
////
////    // wait for ready
////    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
////    while (Serial.available() && Serial.read()); // empty buffer
////    while (!Serial.available());                 // wait for data
////    while (Serial.available() && Serial.read()); // empty buffer again
////
////    // load and configure the DMP
////    Serial.println(F("Initializing DMP..."));
////    devStatus = mpu.dmpInitialize();
////
////    // supply your own gyro offsets here, scaled for min sensitivity
////    mpu.setXGyroOffset(220);
////    mpu.setYGyroOffset(76);
////    mpu.setZGyroOffset(-85);
////    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
////
////    // make sure it worked (returns 0 if so)
////    if (devStatus == 0) {
////        // Calibration Time: generate offsets and calibrate our MPU6050
////        mpu.CalibrateAccel(6);
////        mpu.CalibrateGyro(6);
////        mpu.PrintActiveOffsets();
////        // turn on the DMP, now that it's ready
////        Serial.println(F("Enabling DMP..."));
////        mpu.setDMPEnabled(true);
////
////        // enable Arduino interrupt detection
////        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
////        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
////        Serial.println(F(")..."));
////        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
////        mpuIntStatus = mpu.getIntStatus();
////
////        // set our DMP Ready flag so the main loop() function knows it's okay to use it
////        Serial.println(F("DMP ready! Waiting for first interrupt..."));
////        dmpReady = true;
////
////        // get expected DMP packet size for later comparison
////        packetSize = mpu.dmpGetFIFOPacketSize();
////    } else {
////        // ERROR!
////        // 1 = initial memory load failed
////        // 2 = DMP configuration updates failed
////        // (if it's going to break, usually the code will be 1)
////        Serial.print(F("DMP Initialization failed (code "));
////        Serial.print(devStatus);
////        Serial.println(F(")"));
////    }
////
////
////
////
////
////
////
////  
//////
//////    //GYRO STUFF_____________________________________________________
//////      // join I2C bus (I2Cdev library doesn't do this automatically)
//////    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//////        Wire.begin();
//////    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//////        Fastwire::setup(400, true);
//////    #endif
//////
//////
//////
//////    // initialize device
//////    Serial.println("Initializing I2C devices...");
//////    accelgyro.initialize();
//////
//////    // verify connection
//////    Serial.println("Testing device connections...");
//////    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
//////
//////
//////    // enable Arduino interrupt detection
//////    attachInterrupt(0, dmpDataReady, RISING);
//////    mpuIntStatus = accelgyro.getIntStatus();
//////
//////
//////
//////    // use the code below to change accel/gyro offset values
//////    /*
//////    Serial.println("Updating internal sensor offsets...");
//////    // -76  -2359 1688  0 0 0
//////    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//////    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//////    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//////    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//////    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//////    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//////    Serial.print("\n");
//////    accelgyro.setXGyroOffset(220);
//////    accelgyro.setYGyroOffset(76);
//////    accelgyro.setZGyroOffset(-85);
//////    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//////    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//////    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//////    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//////    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//////    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//////    Serial.print("\n");
//////    */
////}
////
//////  // initialize IMU 
//////  accelgyro.initialize();
//////  devStatus = accelgyro.dmpInitialize();
//////
//////  // supply your own gyro offsets here, scaled for min sensitivity
//////  accelgyro.setXGyroOffset(176);
//////  accelgyro.setYGyroOffset(-31);
//////  accelgyro.setZGyroOffset(57);
//////  accelgyro.setXAccelOffset(-2424);
//////  accelgyro.setYAccelOffset(272);
//////  accelgyro.setZAccelOffset(987);
//////
//////  // make sure it worked (returns 0 if so)
//////  if (devStatus == 0) {
//////      // turn on the DMP, now that it's ready
//////      accelgyro.setDMPEnabled(true);
//////
//////      // enable Arduino interrupt detection
//////      attachInterrupt(0, dmpDataReady, RISING);
//////      mpuIntStatus = accelgyro.getIntStatus();
//////
//////      // get expected DMP packet size for later comparison
//////      packetSize = accelgyro.dmpGetFIFOPacketSize();
//////    } else {
//////        // ERROR!
//////        // 1 = initial memory load failed
//////        // 2 = DMP configuration updates failed
//////        // (if it's going to break, usually the code will be 1)
//////        Serial.print(F("DMP Initialization failed (code "));
//////        Serial.print(devStatus);
//////        Serial.println(F(")"));
//////    }
//////}
////
////
//////
////// 
//////}
////
////void loop() {
////    // if programming failed, don't try to do anything
////    if (!dmpReady) return;
////
////    // wait for MPU interrupt or extra packet(s) available
////    while (!mpuInterrupt && fifoCount < packetSize) {
////        if (mpuInterrupt && fifoCount < packetSize) {
////          // try to get out of the infinite loop 
////          fifoCount = mpu.getFIFOCount();
////        }  
////        // other program behavior stuff here
////        // .
////        // .
////        // .
////        // if you are really paranoid you can frequently test in between other
////        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
////        // while() loop to immediately process the MPU data
////        // .
////        // .
////        // .
////    }
////
////    // reset interrupt flag and get INT_STATUS byte
////    mpuInterrupt = false;
////    mpuIntStatus = mpu.getIntStatus();
////
////    // get current FIFO count
////    fifoCount = mpu.getFIFOCount();
////  if(fifoCount < packetSize){
////          //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
////      // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
////  }
////    // check for overflow (this should never happen unless our code is too inefficient)
////    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
////        // reset so we can continue cleanly
////        mpu.resetFIFO();
////      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
////        Serial.println(F("FIFO overflow!"));
////
////    // otherwise, check for DMP data ready interrupt (this should happen frequently)
////    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
////


//////  
//////    //Get the loop start time
//////    currentMillis = millis();
//////
//////    //Only enter the loop every 10ms    
//////    if (currentMillis - previousMillis >= 10) {  // start timed event
//////      previousMillis = currentMillis; 
//////      
//////      // measure loop time / optional
//////      
//////      loopTime = currentMillis - previousLooptime;
//////      //Serial.print("time: ");
//////      //Serial.print(loopTime);
//////      //Serial.print("\t");
//////      previousLooptime = currentMillis;    
//////
//////  
//////      //Call the function to read the mpu
//////      readAngles();
//////    
//////      //Call the function to recieve the remote values
//////      RemoteRecieve();
//////      
//////      //Call the function to add the Radio Deadbands
//////      RemoteDeadband();
//////    
//////      //Call the Function to mix the steering
//////      mixing(steer,throt,LMot,RMot);
//////    
//////      //Call the Function to Drive the Motors
//////      PWMDrive(LPin,RPin,LDirPin,RDirPin,RMot,LMot);
//////      
//////      //Call the debuging function
//////      debugging();
//////      //  delay(pause);
//////      //   // loop delay adjustable by remote
//////
//////    }// end of timed event   
////}
////
////
////
////
////void debugging(){
////  /*
////   * This function handles all of the debugging outputs.  
////   * There is flexability in turning it on or off using a switch on the controller.
////   * The delay for the serial terminal is adjustable as well and allows for better readability.
////   * Using the serial port for debugging really slows evertyhing down so it needs to be disabled for optimal performance.
////   */
////
////  //Debug Switch
////  if(ch6>1800)
////    {dbg=1;
////    pause=5;
////    }
////  if(ch6<1800)
////    {dbg=0;
////    pause=5;
////    }
////
////  /*
////   * If debugging is on this outputs all of the values read from the channels.
////   * This is used to evaluate issues with the RC connection.
////   */
////   
////  if(dbg){
////    Serial.println("CHANNEL VALUES:");
////    Serial.print("Ch1  : ");
////    Serial.println(ch1);
////    Serial.print("Ch2  : ");
////    Serial.println(ch2);
////    Serial.print("Ch3  : ");
////    Serial.println(ch3);
////    Serial.print("Ch4  : ");
////    Serial.println(ch4);
////    Serial.print("Ch5  : ");
////    Serial.println(ch5);
////    Serial.print("Ch6  : ");
////    Serial.println(ch6);
////    Serial.print("Ch7  : ");
////    Serial.println(ch7);
////    Serial.println(" ");
////    }
////
////
////    if(dbg){
////      Serial.print("Steer: ");
////      Serial.println(steer);
////     
////      Serial.print("Throt: ");
////      Serial.println(throt);
////
////      Serial.print("Pause: ");
////      Serial.println(pause);
////
////      Serial.print("Debug: ");
////      Serial.println(dbg);       
////    }
////
////    if(dbg){
////      Serial.print("L    : ");
////      Serial.println(LMot);
////      Serial.print("R    : ");
////      Serial.println(RMot);
////    }
////
////
//////      Serial.print("a/g:\t");
//////      Serial.print(ax); Serial.print("\t");
//////      Serial.print(ay); Serial.print("\t");
//////      Serial.print(az); Serial.print("\t");
//////      Serial.print(gx); Serial.print("\t");
//////      Serial.print(gy); Serial.print("\t");
//////      Serial.println(gz);
////
////    
////    if(dbg){
////      /*
////      *Delay Knob
////      *This uses the value of channel 7 and adjusts the serial Delay.  
////      *This is used for debugging purpuses and should normally be very low. 
////      */
////      if(ch7<1000)
////        {ch7=1000;
////        }
////      pause=map(ch7, 1000,2200,0,5000);
////    }
////  
////
////
////
////  
////
////  
////}
