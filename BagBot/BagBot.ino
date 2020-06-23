#include <IBusBM.h>
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




//Output Variables
//----------------
int LMot;
int RMot;
CytronMD motor1(PWM_DIR, 9, 8);   // PWM1 = Pin 9 , DIR1 = Pin 8.
CytronMD motor2(PWM_DIR, 11, 13); // PWM2 = Pin 11, DIR2 = Pin 13.


//Constants
//----------------
int deadband=50;

IBusBM IBus;    // IBus object

//Servp Variables TEMPORARY
Servo servo_test;      //initialize a servo object for the connected servo            
int angle = 0;    
unsigned long starttime=0;


void setup() {

  
  pinMode(22, INPUT); // Set our input pins as such
  pinMode(23, INPUT);
  pinMode(41, INPUT); // Set our input pins as such
  pinMode(24, INPUT);


  servo_test.attach(31); 
  
  Serial.begin(19200); // Pour a bowl of Serial
  IBus.begin(Serial1);    // iBUS object connected to serial0 RX pin
  dbg=1;

 
}

void loop() {


  
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
      

    
      //Throttle Control
      motor1.setSpeed(LMot);
      motor2.setSpeed(RMot);
    
      angle = map(steer, -500, 500, 0, 179);     // scaling the potentiometer value to angle value for servo between 0 and 180) 
      servo_test.write(angle);                   //command to rotate the servo to the specified angle 

      if(dbg){
        Serial.print("Ang  : ");
        Serial.println(angle);}
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
