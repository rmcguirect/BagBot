#include <SoftwareSerial.h>
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

double Pk1 =10;          // balancing PID
double Ik1 = 0  ;          //Increase I to get more response at small errors
double Dk1 = 2;           //Increase this after P is close

double Setpoint1, Input1, Output1;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup - balancing IMU

float SetpointTrim1 = -3;

//-------------------------------------------------------------------
//-------------------------------------------------------------------



//-------------------------------------------------------------------
//-------------------------- MPU Variables --------------------------
//-------------------------------------------------------------------

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 z
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






//-------------------------------------------------------------------
//---------------------- Hoverboard Serial --------------------------
//-------------------------------------------------------------------
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   38400       // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)


SoftwareSerial HoverSerial(10,11);         // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t  start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;





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

    HoverSerial.begin(HOVER_SERIAL_BAUD);
    pinMode(LED_BUILTIN, OUTPUT);
    
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

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
  // Check for new data availability in the Serial buffer
  if (HoverSerial.available()) {
    incomingByte    = HoverSerial.read();                                 // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;   // Construct the start frame    
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
    Serial.print(incomingByte);
    return;
  #endif      
  
  // Copy received data
  if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
    p     = (byte *)&NewFeedback;
    *p++  = incomingBytePrev;
    *p++  = incomingByte;
    idx   = 2;  
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++  = incomingByte; 
    idx++;
  } 
  
  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {    
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);
  
    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      
      // Print data to built-in Serial
      Serial.print("1: ");   Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;  // Reset the index (it prevents to enter in this if condition in the next cycle)
  }
  
  // Update previous states
  incomingBytePrev  = incomingByte;
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

unsigned long iTimeSend = 0;
int iTestMax = SPEED_MAX_TEST;
int Speed=0;
int iTest = 0;


void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    unsigned long timeNow = millis();   
    currentMillis = millis();

        //Start loop every 10ms
        if (currentMillis - previousMillis >= 10) {
            previousMillis = currentMillis; 
    
                //Call the function to recieve the remote values
                RadioRead();
    
                //Recieve data from hoverboard
                Receive();       
                
                //Recieve Data from IMU
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
              }
//
//            Serial.println(" ");
//            Serial.println("Roll: ");
//            Serial.println(roll);
//            Serial.println("Setpoint: ");
//            Serial.println(Setpoint1);
//            Serial.println("Output: ");
//            Serial.println(Output1);



            Speed=map(Output1,-130,130,-1000,1000);

            //Disable motors if off by more than 10 degrees.  USEFULL FOR TUNING SAFETY

//            //EXPERIMENTAL
//            throt=mapfloat(ch2,1000,2000,-1000,1000);
//            steer=map(ch1,1000,2000,-1000,1000);
//            Send(steer, throt);
            
            if(!kill){
              Speed=0;
            }

           

            
            Send(0, Speed);
            

            
            

            
            
        
}

            
