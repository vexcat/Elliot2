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
            {"cpr", 442.5252999999999 * (360.0 / 900.0)},
            {"cpi", 68.5052 * (360.0 / 900.0)}
        };
        saveState();
    }
    return state["gps"];
}

json& getBaseState() {
    auto &state = getState();
    if(state.find("base") == state.end()) {
        printf("Defaults were applied for the base. Please change base PID.\n");
        state["base"] = {
            {"dist", {
                {"kP", 0},
                {"kI", 0},
                {"kD", 0},
            }},
            {"angle", {
                {"kP", 0},
                {"kI", 0},
                {"kD", 0},
            }},
            {"turn", {
                {"kP", 0},
                {"kI", 0},
                {"kD", 0},
            }}
        };
        saveState();
    }
    return state["base"];
}

json& getCameraState(pros::Vision& def) {
    auto &state = getState();
    if(state.find("cam") == state.end()) {
        printf("Defaults were applied for the camera.\n");
        state["cam"] = {
            {"auto", false},
            {"white", (int)def.get_white_balance()},
            {"exposure", (int)def.get_exposure()}
        };
        saveState();
    }
    return state["cam"];
}

Elliot::Elliot():
controller{CONTROLLER_MASTER},
camera{15},
camSettings{camera, getCameraState(camera)},
left{3, 4},
right{-2, -1},
catapultMtr{5},
score{7},
intake{10},
catapultLimit{2},
leftSonic{'C', 'D'},
rightSonic{'E', 'F'},
catapult{catapultMtr, catapultLimit},
gps{left, right, getGPSState()},
box{nullptr},
baseSettings{gps, box, getBaseState()} {
    score.setBrakeMode(AbstractMotor::brakeMode::hold);
    left.setGearing(AbstractMotor::gearset::green);
    right.setGearing(AbstractMotor::gearset::green);
    left .setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    right.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    catapultMtr.setGearing(AbstractMotor::gearset::red);
    catapultMtr.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    score.setGearing(AbstractMotor::gearset::green);
    intake.setGearing(AbstractMotor::gearset::blue);
}

void Elliot::takeCoast() {
    usageGuard.take(TIMEOUT_MAX);
}

void Elliot::stop() {
    box->base.stop();
    getRobot().baseSettings.loadState();
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
    stop();
}

void Elliot::beginTasks() {
    catapult.beginTask();
    gps.beginTask();
}

Elliot* elliot;
void createRobot() {
    if(elliot) delete elliot;
    elliot = new Elliot();
}