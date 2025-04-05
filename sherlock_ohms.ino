#include "Robot.h"

Robot robot;

unsigned long last_print_time = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for serial monitor

    robot.begin();
    Serial.println("Robot initialized.");
}

void loop() {
    robot.update();
    robot.updateSoftStop();

    // Print wheel angular velocities every 500ms
    unsigned long now = millis();
    if (now - last_print_time > 500) {
        last_print_time = now;

        float* velocities = robot.getWheelAngularVelocities();
        Serial.print("Wheel Angular Velocities [rad/s]: ");
        for (int i = 0; i < 4; ++i) {
            Serial.print(velocities[i], 3);
            Serial.print(" ");
        }

        if (robot.isSoftStopping()) {
            Serial.print(" (Soft stopping...)");
        } else if (robot.isStopped()) {
            Serial.print(" (Stopped)");
        }

        Serial.println();
    }

    // Listen for keyboard commands via serial
    if (Serial.available()) {
        char c = Serial.read();

        if (c == 's') {
            Serial.println("Initiating soft stop...");
            robot.initiateSoftStop(5.0); // 5 rad/s² deceleration
        }

        if (c == 'e') {
            Serial.println("EMERGENCY STOP triggered!");
            robot.emergencyStopAll();
        }

        if (c == 'f') {
            Serial.println("Driving forward.");
            robot.setWheelVelocities(5.0, -5.0, 5.0, -5.0);
        }
    }

    delay(10); // ~100 Hz loop
}
