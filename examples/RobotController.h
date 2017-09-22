#include <Arduino.h>
#include "../src/RobotController.h"
#include "../src/VarSpeedServo.h"
#include "../src/Kinematic.h"
#include "TimerOne.h"


#define pin_servo_0 3
#define pin_servo_1 4
#define pin_servo_2 5
#define pin_servo_3 6
AD }
6 }
};

RobotController *_RobotController;
VarSpeedServo  *servos[6];
Kinematic Kinematic(geometry);

void logicalToPhysicalAngles(float angles[6]) {
    // angles[2] += angles[1];
}

void physicalToLogicalAngles(float angles[6]) {
    // angles[2] -= angles[1];
}

void updateServos() {
    for (size_t i = 0; i < 6; i++) {
        servos[i]->process(20);
    }

    // important: move to loop when debugging (Serial.print needs ISR)
    // todo use volatile and ATOMIC on angle buffer and stuff
    _RobotController->process();
}

void setup()
{
    Serial.begin(9600);
    delay(4000);

    for (size_t i = 0; i < 6; i++) {
        servos[i] = new VarSpeedServo(servoConfig[i][0],
                                servoConfig[i][1],
                                servoConfig[i][2],
                                servoConfig[i][3],
                                servoConfig[i][4],
                                servoConfig[i][5],
                                servoConfig[i][6]);
    }

    _RobotController = new RobotController(servos,
                                           Kinematic,
                                           logicalAngleLimits,
                                           logicalToPhysicalAngles,
                                           physicalToLogicalAngles);

    float angles[6] = { 45 * DEG_TO_RAD, 0, 0, 0, 0, 0 };
    _RobotController->setMaxVelocity(5);
    _RobotController->setMovementMethod(RobotController::MOVEMENT_METHODS::P2P);
    _RobotController->setTargetLogicalAngles(angles);
    _RobotController->process();

    _RobotController->setMaxVelocity(5);
    float pose[6] = { 18, 0, 10, 0, PI, 0 };
    _RobotController->setTargetPose(pose);


    // init Timer and register callback
    Timer1.initialize(20 * 1000); // 20ms
    Timer1.attachInterrupt(updateServos);
}

void loop()
{}
