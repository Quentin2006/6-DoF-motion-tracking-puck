#include <Arduino.h>
#include "mpu6050.h"
#include "config.h"

Mpu6050 mpu;

void setup() {
  Serial.begin(115200);
  Serial.flush();
  Wire.begin(SDA_PIN, SCL_PIN);
}

void loop() {
  vec3 accel, gyro;
  mpu.readAccel(accel);
  mpu.readGyro(gyro);
  Serial.printf("accel: %f, %f, %f\n", accel.x, accel.y, accel.z);
  Serial.printf("gyro: %f, %f, %f\n", gyro.x, gyro.y, gyro.z);
}


  
  
  
  
               
  


  
  
  
  
               
  
