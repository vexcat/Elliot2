#include "main.h"
#include "okapi/api.hpp"
#include "gps.hpp"
#include "display.hpp"
#include "elliot.hpp"
#include "state.hpp"
#include "json.hpp"
#include <deque>
using namespace okapi;

void Puncher::loadState() {
    double oldTarget = 0;
    bool oldDisabled = true;
    if(controllerPtr) {
        oldTarget = controllerPtr->getTarget();
        oldDisabled = controllerPtr->isDisabled();
    }
    controllerPtr = std::make_shared<okapi::AsyncPosPIDController>(
        std::make_shared<Potentiometer>(angleSense),
        std::make_shared<MotorGroup>(angler),
        okapi::TimeUtilFactory::create(),
        puncherData["kP"].get<double>(),
        puncherData["kI"].get<double>(),
        puncherData["kD"].get<double>()
    );
    controllerPtr->flipDisable(oldDisabled);
    controllerPtr->setTarget(oldTarget);
    controllerPtr->startThread();
     lowTargetPosition = puncherData["low" ].get<double>();
    highTargetPosition = puncherData["high"].get<double>();
}

Puncher::Puncher(MotorGroup& ipuncher, MotorGroup& iangler, okapi::Potentiometer& iangleSense, json& ipuncherData):
puncher(ipuncher), angler(iangler), angleSense(iangleSense), puncherData(ipuncherData) {
    lastPuncherPosition = ipuncher.getPosition();
    loadState();
}

void Puncher::puncherTask() {
    while(true) {
        if(abs(puncher.getPosition() - puncher.getTargetPosition()) < 20 && puncher.getActualVelocity() < 5) {
            puncher.moveVoltage(0);
        }
        pros::delay(5);
    }
}
void Puncher::lowTarget() {
    controllerPtr->setTarget(lowTargetPosition);
    controllerPtr->flipDisable(false);
}
void Puncher::highTarget() {
    controllerPtr->setTarget(highTargetPosition);
    controllerPtr->flipDisable(false);
}
void Puncher::stopAutoControl() {
    controllerPtr->flipDisable(true);
}
void Puncher::shoot() {
    lastPuncherPosition -= 360;
    puncher.moveAbsolute(lastPuncherPosition, 200);
}
void Puncher::setVelocity(double vel) {
    puncher.moveVelocity(vel);
}
int Puncher::targetError() {
    return controllerPtr->getError();
}
void Puncher::toggleTarget() {
    if(controllerPtr->getTarget() == lowTargetPosition) {
        highTarget();
    } else {
        lowTarget();
    }
}
void Puncher::setLowTarget(int target) {
    if(controllerPtr->getTarget() == lowTargetPosition) {
        controllerPtr->setTarget(target);
    }
    lowTargetPosition = target;
    puncherData["low"] = target;
    saveState();
}
void Puncher::setHighTarget(int target) {
    if(controllerPtr->getTarget() == highTargetPosition) {
        controllerPtr->setTarget(target);
    }
    highTargetPosition = target;
    puncherData["high"] = target;
    saveState();
}
okapi::IterativePosPIDController::Gains Puncher::getGains() {   
    return {
        puncherData["kP"].get<double>(),
        puncherData["kI"].get<double>(),
        puncherData["kD"].get<double>(),
        0
    };
}
void Puncher::setGains(okapi::IterativePosPIDController::Gains gains) {
    puncherData["kP"] = gains.kP;
    puncherData["kI"] = gains.kI;
    puncherData["kD"] = gains.kD;
    loadState();
    saveState();
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

json& getPuncherState() {
    auto &state = getState();
    if(state.find("puncher") == state.end()) {
        printf("Defaults were applied for the puncher. Please change puncher PID.\n");
        state["puncher"] = {
            {"kP", 0.001},
            {"kI", 0.001},
            {"kD", 0.001},
            {"low", 800},
            {"high", 1200}
        };
        saveState();
    }
    return state["puncher"];
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
left{3, 4},
right{-2, -1},
puncherMtr{5},
angler{7},
intake{10},
scorer{6},
angleSense{'B'},
puncher{puncherMtr, angler, angleSense, getPuncherState()},
gps{left, right, getGPSState()},
base{nullptr},
baseSettings{left, right,
[&]() { return gps.inchToCounts(1); },
[&]() { return gps.radiansToCounts(1); },
base, getBaseState()} {
    angler.setBrakeMode(AbstractMotor::brakeMode::hold);
    angler.setGearing(AbstractMotor::gearset::green);
    left.setGearing(AbstractMotor::gearset::green);
    right.setGearing(AbstractMotor::gearset::green);
    left .setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    right.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    puncherMtr.setGearing(AbstractMotor::gearset::red);
    puncherMtr.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    intake.setGearing(AbstractMotor::gearset::green);
    scorer.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
}

void Elliot::takeCoast() {
    usageGuard.take(TIMEOUT_MAX);
}

void Elliot::stop() {
    base->stop();
    puncher.setVelocity(0);
    angler.moveVelocity(0);
    intake.moveVelocity(0);
    scorer.moveVelocity(0);
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
    gps.beginTask();
    puncher.beginTask();
}

Elliot* elliot;
void createRobot() {
    if(elliot) delete elliot;
    elliot = new Elliot();
}
