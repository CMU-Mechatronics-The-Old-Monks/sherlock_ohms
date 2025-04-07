#include "Robot.h"
#include "DataManager.h"

Robot robot;

// Data communication setup
DataManager rx_data(4);  // expecting 4 floats from Pi
DataManager tx_data(4);  // optional: send back 4 floats

unsigned long last_print_time = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for Serial connection

    robot.begin();
    robot.enableWheels();
    Serial.println("Robot initialized.");
}

void loop() {
    robot.update();
    //robot.updateSoftStop();

    //robot.setWheelVelocities(2.0,2.0,2.0,2.0);

    // Check for incoming velocity commands from Pi
    if (rx_data.receiveData(Serial)) {
        float wheel_cmds[4];
        rx_data.parseData(wheel_cmds, 4);

        robot.setWheelVelocities(
            wheel_cmds[0],
            wheel_cmds[1],
            wheel_cmds[2],
            wheel_cmds[3]
        );
        //robot.setWheelVelocities(2.0,2.0,2.0,2.0);

        // Optional: echo back wheel velocities (filtered/actual)
        // float* feedback = robot.getWheelAngularVelocities();
        // std::vector<float> fb(feedback, feedback + 4);
        // tx_data.packAndTransmitData(fb, Serial);
        
        
        // Echo the exact same commands back to the Pi
        std::vector<float> echoed(wheel_cmds, wheel_cmds + 4);
        tx_data.packAndTransmitData(echoed, Serial);
      }

    

    delay(10); // ~100 Hz loop
}


// #include "Robot.h"

// Robot robot;

// unsigned long last_update_time = 0;
// int phase = 0;

// void setup() {
//     Serial.begin(115200);
//     while (!Serial);
//     Serial.println("Running motor test pattern...");
//     robot.begin();
//     robot.enableWheels();
// }

// //////////////////////////////////////////////////////////////////////

// void loop() {
//     robot.update();

//     // Simple time-based phase switch every 2 seconds
//     unsigned long now = millis();
//     if (now - last_update_time > 2000) {
//         last_update_time = now;

//         // Sweep through a test pattern
//         switch (phase) {
//             case 0:
//                 Serial.println("Phase 0: All wheels forward");
//                 robot.setWheelVelocities(3.0, 3.0, 3.0, 3.0);
//                 break;
//             case 1:
//                 Serial.println("Phase 1: All wheels backward");
//                 robot.setWheelVelocities(-3.0, -3.0, -3.0, -3.0);
//                 break;

//             // case 2:
//             //     Serial.println("Phase 2: Rotate in place CW");
//             //     robot.setWheelVelocities(3.0, -3.0, 3.0, -3.0);
//             //     break;
//             // case 3:
//             //     Serial.println("Phase 3: Rotate in place CCW");
//             //     robot.setWheelVelocities(-3.0, 3.0, -3.0, 3.0);
//             //     break;

//             // case 4:
//             //     Serial.println("Phase 4: Soft stop");
//             //     robot.initiateSoftStop(3.0);  // soft deceleration
//             //     break;
//             // case 5:
//             //     Serial.println("Phase 5: Emergency stop");
//             //     //robot.emergencyStopAll();
//             //     break;
//             case 3:
//                 Serial.println("Done!");
//                 robot.setWheelVelocities(0.0, 0.0, 0.0, 0.0);
//                 //while (1);  // Halt
//         }

//         float* pwm = robot.getWheelPWMValues();
//         float* vel = robot.getWheelAngularVelocities();
//         float* pos = robot.getWheelAngles();
//         Serial.print("PWM Outputs: ");
//         for (int i = 0; i < 4; ++i) {
//             Serial.print(pwm[i], 1);
//             Serial.print(" ");
//         }
//         Serial.print("  wheel velocities: ");
//         for (int i = 0; i < 4; ++i) {
//             Serial.print(vel[i], 1);
//             Serial.print(" ");
//         }
//         Serial.print("  wheel angles: ");
//         for (int i = 0; i < 4; ++i) {
//             Serial.print(pos[i], 1);
//             Serial.print(" ");
//         }
//         Serial.println();

//         phase++;
//     }

//     //robot.updateSoftStop();
//     delay(10); // 100Hz update
// }

//////////////////////////////////////////////////////////////////////

// void loop() {
//     robot.update();
//     //robot.updateSoftStop();
    
//     float target = 3.0; // for example

//     robot.setWheelVelocity(target);

//     float pwm = robot.getWheelPWMValue();
//     float velocity = robot.getWheelAngularVelocity();
    

//     Serial.print(pwm);
//     Serial.print('\t');         // Tab separator (better for Serial Plotter)
//     Serial.print(velocity);
//     Serial.print('\t');
//     Serial.println(target);     // Final value + newline

//     // Optionally toggle speed, test soft stop, etc...
//     delay(50);
// }