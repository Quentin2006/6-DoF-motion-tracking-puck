#include <Wire.h>
#include <WiFi.h>
#include <ArduinoBLE.h>

#include <MPU6050.h>

MPU6050 mpu;
BLEService myService("fff0");

void setup() {
  Serial.begin(115200);
  while (!Serial);


  // NOTE: SETUP BLUETOOTH

  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }
  Serial.println("Bluetooth® Low Energy Central - LED control");


  BLE.setDeviceName("Test");

  BLE.advertise();

  Serial.println("advertising ...");


  // NOTE: CONENCT MPU6050
  //
  // Wire.begin(4, 5);
  //
  // Serial.println("Initing MPU6050...");
  // mpu.initialize();
}


void loop() {

  BLE.poll();

  // mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Serial.print("Accel: ");
  // Serial.print(ax);
  // Serial.print(", ");
  // Serial.print(ay);
  //
  // Serial.print("Gyro: ");
  // Serial.print(gx);
  // Serial.print(", ");
  // Serial.print(gy);
  // Serial.print(", ");
  // Serial.println(gz);
}
