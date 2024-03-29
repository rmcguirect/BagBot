#include <Servo.h>
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
//Motor Controler: RioRand 400W 6-60V PWM DC Brushless Electric Motor Speed Controller with Hall
//---------------------------------------
//Left PWM Pin :44
//Right PWM Pin:45
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
Servo LeftMotor;


int RMot;
int RPin = 45;
Servo RightMotor;



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

    LeftMotor.attach(LPin);
    RightMotor.attach(RPin);
    
    //Setup Radio Connection
    IBus.begin(Serial3);    // iBUS object connected to serial3 RX pin

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
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
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    // Setup PID controllers
    PID1.SetMode(AUTOMATIC);              
    PID1.SetOutputLimits(-127, 127);
    PID1.SetSampleTime(10);

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
            Serial.println(" ");
            Serial.println("Roll: ");
            Serial.println(roll);
            LMot=0;
            RMot=0;
            ServoPWMDrive(LeftMotor,RightMotor,LMot,RMot);
            
            
        }
}

            
