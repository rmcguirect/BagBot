
//_____________________________________________________
//________________________IMU__________________________
//_____________________________________________________
#include <Arduino_LSM9DS1.h>


float Ax,Ay,Az;
float Gx,Gy,Gz;
int accelX=1;
int accelY=1;

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
  
}

//_____________________________________________________
//______________________IMU END________________________
//_____________________________________________________




//_____________________________________________________
//________________________BLE__________________________
//_____________________________________________________
#include <ArduinoBLE.h>

BLEDevice central;

bool Ble_Init()
{
  
  // Create the Service and charecteristics For the IMU
  BLEService customService("1101");
  BLEUnsignedIntCharacteristic  customXChar("2101", BLERead | BLENotify);
  BLEUnsignedIntCharacteristic  customYChar("2101", BLERead | BLENotify);

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
  BLE.setAdvertisedService(customService); // add the service UUID
  customService.addCharacteristic(customXChar); //
  customService.addCharacteristic(customYChar); //
  BLE.addService(customService);
  customXChar.writeValue(accelX);
  customYChar.writeValue(accelY);

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
  while (!Serial);
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
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;

        //Update IMU
        Imu_Loop();
      }
  }

  
  Serial.println("Disconnected");

    // wait for a BLE central
  bool BleReady=0;
  
  while (BleReady==0){
    BleReady=Ble_Loop();
  };
      

  


};
