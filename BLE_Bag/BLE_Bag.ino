
//_____________________________________________________
//________________________IMU__________________________
//_____________________________________________________
#include <Arduino_LSM9DS1.h>
#include <ArduinoBLE.h>


float Ax,Ay,Az;
float Gx,Gy,Gz;
int accelX=1;
int accelY=1;
int CtrlX,CtrlY;
bool CtrlKill;
int looptime;



int oldBatteryLevel = 0;  // last battery level reading from analog input




void Imu_Init()
{
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
};

void Imu_accel(float &x, float &y, float &z)
{
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
  }
};


void Imu_gyro(float &x, float &y, float &z)
{
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
  }
};

void Imu_Loop()
{
  //Check Accelerometer
  Imu_accel(Ax,Ay,Az);
  accelX = (1+Ax)*100;
  accelY = (1+Ay)*100;

  //Check Gyro  
  Imu_gyro(Gx,Gy,Gz);
  
};

//_____________________________________________________
//______________________IMU END________________________
//_____________________________________________________




//_____________________________________________________
//________________________BLE__________________________
//_____________________________________________________


BLEDevice central;

BLEUnsignedCharCharacteristic  customXChar("286e3be7-136d-4a2d-b209-2fd2e8582b5f", BLERead | BLENotify);
BLEUnsignedCharCharacteristic  customYChar("d2352bdb-5a3e-40d7-bfb9-5374f857d517", BLERead | BLENotify);


// Debug Service
BLEService DebugService("947477fb-accc-481a-be54-7811e3130223");
BLEByteCharacteristic RequestDebug("75e05785-6421-48f6-aa4c-61f01e8ab4cb",BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic LoopTimeDB("1250c35b-2981-4812-95d9-3e022ef5e952",BLERead | BLEWrite | BLENotify);
BLEByteCharacteristic Message("e58285e9-1985-40d2-90b1-a3d95518e361",BLERead | BLEWrite | BLENotify);

// PID Service
BLEService PIDService("902243be-0018-11eb-adc1-0242ac120002");
BLECharCharacteristic PChar("a2fa3960-0018-11eb-adc1-0242ac120002",BLERead | BLEWrite | BLENotify);
BLECharCharacteristic IChar("a7337320-0018-11eb-adc1-0242ac120002",BLERead | BLEWrite | BLENotify);
BLECharCharacteristic DChar("aad4a616-0018-11eb-adc1-0242ac120002",BLERead | BLEWrite | BLENotify);

// Control Service
BLEService ControlService("b7e18f72-0018-11eb-adc1-0242ac120002");
BLEIntCharacteristic XChar("b386edc8-0018-11eb-adc1-0242ac120002",BLERead | BLEWrite);
BLEIntCharacteristic YChar("afd24560-0018-11eb-adc1-0242ac120002 ",BLERead | BLEWrite);
BLEIntCharacteristic KillChar("bc60cc7a-0018-11eb-adc1-0242ac120002",BLERead | BLEWrite | BLENotify);



bool Ble_Init()
{
  
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
    }
  
    
  /* Set a local name for the BLE device
  This name will appear in advertising packets
  and can be used by remote devices to identify this BLE device
  The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("Bag Bot");

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  

  //Setup the Control Service
  BLE.setAdvertisedService(ControlService); // add the service UUID
  ControlService.addCharacteristic(XChar); //
  ControlService.addCharacteristic(YChar); //
//  ControlService.addCharacteristic(KillChar); //

  BLE.addService(ControlService);

  
  
//  ControlReadyChar.setEventHandler(BLEWritten, ControlWritten);
//  ControlReadyChar.setValue(0);


  //Setup the Debug Service
  BLE.setAdvertisedService(DebugService); // add the service UUID
  
  DebugService.addCharacteristic(LoopTimeDB); //
  DebugService.addCharacteristic(Message); //
  DebugService.addCharacteristic(RequestDebug);
  BLE.addService(DebugService);

  //RequestDebug.setEventHandler(BLEWritten, DebugWrite);
  //RequestDebug.setValue(0);






  

  //Start advertising BLE.  It will start continuously transmitting BLE
  //advertising packets and will be visible to remote BLE central devices
  //until it receives a new connection 
  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");

  // wait for a BLE central
  bool BleReady=0;
  
  while (BleReady==0){
    BleReady=Ble_Loop();
  };
  

  

  
};

bool Ble_Loop()
{
  Serial.println("BLE Loop");
  //Check for a central
  central = BLE.central();
  
  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);
    return 1;
  }
  return 0;
};


//_____________________________________________________
//______________________BLE END________________________
//_____________________________________________________





void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //while (!Serial);
  Serial.println("Started");

  

  //Initialize IMU
  Imu_Init();

  //Initialize BLE
  Ble_Init();
};

void loop() {
  
  while (central.connected()) {
      long currentMillis = millis();
      long previousMillis;
      // if 200ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 100) {
        
        looptime=currentMillis - previousMillis;
        previousMillis = currentMillis;
        //Update IMU
        Imu_Loop();
        //DebugWrite();
        
        //TEMP CODE
        Serial.println(XChar.value());

      }
  }

  
  Serial.println("Disconnected");

  // wait for a BLE central
  bool BleReady=0;
  while (BleReady==0){
    BleReady=Ble_Loop();
  };
};
      

void blePeripheralConnectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void blePeripheralDisconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}


void ControlWritten(BLEDevice central, BLECharacteristic characteristic) 
{
//    Serial.print(XChar.value());
//    CtrlX=XChar.value();
//    CtrlY=YChar.value();
//    CtrlKill=KillChar.value();
//
//    DebugWrite(CtrlX);

};

//void DebugWrite(BLEDevice central, BLECharacteristic characteristic) 


void DebugWrite(int msg) 
{
      Serial.println(looptime);
      LoopTimeDB.writeValue(looptime);
      Message.writeValue(msg);
      
};
