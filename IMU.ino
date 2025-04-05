#include <Arduino.h>
#include "BNO055.h"

// Create a global BNO055 object
BNO055 myBNO;  // uses default Wire (SDA/SCL) and default address (0x28)

void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!myBNO.begin()) {
    Serial.println("BNO055 begin() failed");
    while (1);
  }
  Serial.println("BNO055 is now in IMU mode (X,Y only, gravity-free).");
}

void loop() {
  // For example, update every 20ms
  delay(20);
  myBNO.update();

  float ax = myBNO.getAccX();
  float ay = myBNO.getAccY();
  float vx = myBNO.getVelX();
  float vy = myBNO.getVelY();

  Serial.print("LinAcc X,Y: ");
  Serial.print(ax, 2); Serial.print(", ");
  //Serial.print(ay, 2);

  //Serial.print("  Vel X,Y: ");
  //Serial.print(vx, 2); Serial.print(", ");
  //Serial.print(vy, 2);

  Serial.println();
}

