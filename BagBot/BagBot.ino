
#include <MPU6050.h>
#include <I2Cdev.h>
#include <IBusBM.h>
#include "wire.h"

#include <Servo.h>     //Servo library
#include "CytronMotorDriver.h" //Motor Shield Library

/*
RC Arduino Robot FR Sky interface

--------------
|Description:|
--------------
This code connects an FRSKY Reciever to 2 DC Motors for controlling a robot.


--------------
|CONNECTIONS:|
--------------

Motor Shield: Cytron SHIELD-MDD10
---------------------------------------
Arduino D3  - Motor Driver PWM 1 Input
Arduino D4  - Motor Driver DIR 1 Input
Arduino D9 - Motor Driver PWM 2 Input
Arduino D10 - Motor Driver DIR 2 Input

RC Interface: FR Sky FS-iA6B
---------------------------------------
Arduino D5  - Steering
Arduino D6  - Throttle
Arduino D7  - Elevator (Unused)


AUTHOR: Ryan McGuire
DATE: 6/4/2020

 */


 


//Input Variables
//---------------
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

int ch7; // Debug speed
int pause;

int16_t ax, ay, az;
int16_t gx, gy, gz;

//Accelerometer
float accAngle;
//Gyro
int16_t gyroX, gyroRate;
float gyroAngle=0;

//Timming
unsigned long currTime, prevTime=0, loopTime;


//Output Variables
//----------------
int LMot;
int LPin = 45;
int LDirPin = 43;

int RMot;
int RPin = 44;
int RDirPin = 42;



//Constants
//----------------
int deadband=50;

IBusBM IBus;    // IBus object


//Gyro Stuff
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high


void setup() {

  
  pinMode(22, INPUT); // Set our input pins as such
  pinMode(23, INPUT);
  pinMode(41, INPUT); // Set our input pins as such
  pinMode(24, INPUT);
  
  pinMode(LPin, OUTPUT); // Set our output pins as such
  pinMode(RPin, OUTPUT);
  pinMode(LDirPin, OUTPUT);
  pinMode(RDirPin, OUTPUT);
  
  
  Serial.begin(19200); // Pour a bowl of Serial
  IBus.begin(Serial1);    // iBUS object connected to serial0 RX pin
  dbg=1;



  //GYRO STUFF_____________________________________________________
      // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif



    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76  -2359 1688  0 0 0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */

 
}

void loop() {
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  

  //Read the GY521
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //Acceleration angle
  accAngle = atan2(ax, az)*RAD_TO_DEG;
  if(isnan(accAngle));
  else
    Serial.println(accAngle);

  //Gyro Angle
  gyroRate = map(gx, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*loopTime/1000;
  Serial.println(gyroAngle);

  
  //This is where ibus data is read from the serial port.
  ch1=IBus.readChannel(0); // get latest value for servo channel 1
  ch2=IBus.readChannel(1); // get latest value for servo channel 2
  ch3=IBus.readChannel(2); // get latest value for servo channel 2
  ch4=IBus.readChannel(3); // get latest value for servo channel 2
  ch5=IBus.readChannel(4); // get latest value for servo channel 5
  ch6=IBus.readChannel(5); // get latest value for servo channel 6
  ch7=IBus.readChannel(6); // get latest value for servo channel 7



  //Read KillSwitch
  if (ch5>1800 || ch5<0) //Check for killswitch and failsafe.
    { Serial.println("Failsafe");
    }else{

      //Add  Deadbands
      if (ch1>1500-deadband && ch1<1500+deadband) 
        {ch1=1500;}  \
      steer=map(ch1, 1000,2000,-127,127);

      if (ch2>1500-deadband && ch2<1500+deadband)
        {ch2=1500;}
      throt=map(ch2, 1000,2000,-127,127);

      
    

     
      mixing(steer,throt,LMot,RMot);

      PWMDrive(LPin,RPin,LDirPin,RDirPin,RMot,LMot);
      
  }
  
  


  
  debugging();
  delay(pause);
   // I put this here just to make the terminal 
}




void debugging(){
  /*
   * This function handles all of the debugging outputs.  
   * There is flexability in turning it on or off using a switch on the controller.
   * The delay for the serial terminal is adjustable as well and allows for better readability.
   * Using the serial port for debugging really slows evertyhing down so it needs to be disabled for optimal performance.
   */

  //Debug Switch
  if(ch6>1800)
    {dbg=0;
    pause=5;
    }
    else{dbg=1;}

  /*
   * If debugging is on this outputs all of the values read from the channels.
   * This is used to evaluate issues with the RC connection.
   */
   
  if(dbg){
    Serial.println("CHANNEL VALUES:");
    Serial.print("Ch1  : ");
    Serial.println(ch1);
    Serial.print("Ch2  : ");
    Serial.println(ch2);
    Serial.print("Ch3  : ");
    Serial.println(ch3);
    Serial.print("Ch4  : ");
    Serial.println(ch4);
    Serial.print("Ch5  : ");
    Serial.println(ch5);
    Serial.print("Ch6  : ");
    Serial.println(ch6);
    Serial.print("Ch7  : ");
    Serial.println(ch7);
    Serial.println(" ");
    }


    if(dbg){
      Serial.print("Steer: ");
      Serial.println(steer);
     
      Serial.print("Throt: ");
      Serial.println(throt);

      Serial.print("Pause: ");
      Serial.println(pause);

      Serial.print("Debug: ");
      Serial.println(dbg);       
    }

    if(dbg){
      Serial.print("L    : ");
      Serial.println(LMot);
      Serial.print("R    : ");
      Serial.println(RMot);
    }


      Serial.print("a/g:\t");
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.println(gz);

    
    if(dbg){
      /*
      *Delay Knob
      *This uses the value of channel 7 and adjusts the serial Delay.  
      *This is used for debugging purpuses and should normally be very low. 
      */
      if(ch7<1000)
        {ch7=1000;
        }
      pause=map(ch7, 1000,2200,0,5000);
    }
  



  

  
}
