#include "main.h"
#include "okapi/api.hpp"
#include "gps.hpp"
#include "display.hpp"
#include "elliot.hpp"
#include "state.hpp"
#include "json.hpp"
#include <deque>
using namespace okapi;

Catapult::Catapult(MotorGroup& cata, pros::ADIDigitalIn& sensor): cata(cata), sensor(sensor) {}

void Catapult::beginTask() {
    pros::Task cataTask([](void* obj){((Catapult*)obj)->catapultTask();}, this);
}

void Catapult::catapultTask() {
    while(true) {
        if(headingToSwitch && sensor.get_value()) {
            headingToSwitch = false;
            cata.moveVelocity(0);
        }
        pros::delay(1);
    }
}

void Catapult::goToSwitch() {
    headingToSwitch = true;
    cata.moveVelocity(100);
}

void Catapult::setVelocity(double vel) {
    headingToSwitch = false;
    cata.moveVelocity(vel);
}

bool Catapult::isGoingToSwitch() {
    return headingToSwitch;
}

json& getGPSState() {
    auto &state = getState();
    if(state.find("gps") == state.end()) {
        printf("Defaults were applied for the GPS. Please run GPS calibration.\n");
        state["gps"] = {
            {"cpr", 885.050691498},
            {"cpi", 68},
            {"kP", 0},
            {"kI", 0},
            {"kD", 0},
            {"dT", 0}
        };
        saveState();
    }
    return state["gps"];
}

Elliot::Elliot():
left{3, 4},
right{-2, -1},
catapultMtr{5},
score{7},
intake{10},
catapultLimit{2},
camera{15},
gps{left, right, getGPSState()},
controller{CONTROLLER_MASTER},
leftSonic{'C', 'D'},
rightSonic{'E', 'F'},
catapult{catapultMtr, catapultLimit} {
    score.setBrakeMode(AbstractMotor::brakeMode::hold);
    left.setGearing(AbstractMotor::gearset::green);
    right.setGearing(AbstractMotor::gearset::green);
    left .setEncoderUnits(AbstractMotor::encoderUnits::counts);
    right.setEncoderUnits(AbstractMotor::encoderUnits::counts);
    catapultMtr.setGearing(AbstractMotor::gearset::red);
    catapultMtr.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    score.setGearing(AbstractMotor::gearset::green);
    intake.setGearing(AbstractMotor::gearset::green);
}

void Elliot::takeCoast() {
    usageGuard.take(TIMEOUT_MAX);
}

void Elliot::stop() {
    left.moveVelocity(0);
    right.moveVelocity(0);
    catapult.setVelocity(0);
    score.moveVelocity(0);
    intake.moveVelocity(0);
}

void Elliot::takeStopped() {
    takeCoast();
    stop();
}

void Elliot::giveDirect() {
    usageGuard.give();
}

void Elliot::give() {
    usageGuard.give();
    left .setBrakeMode(AbstractMotor::brakeMode::coast);
    right.setBrakeMode(AbstractMotor::brakeMode::coast);
}

void Elliot::beginTasks() {
    catapult.beginTask();
    gps.beginTask();
}

Elliot* elliot;
void createRobot() {
    if(!elliot) {
        elliot = new Elliot();
    }
}