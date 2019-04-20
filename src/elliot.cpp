/**
 * @file ellliot.cpp
 * 
 * This file defines all devices present on Elliot.
 */
#include "main.h"
#include "okapi/api.hpp"
#include "gps.hpp"
#include "display.hpp"
#include "elliot.hpp"
#include "state.hpp"
#include "json.hpp"
#include "autoshoot.hpp"
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
    puncherTarget = ipuncher.getPosition();
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
    puncherTarget -= 360;
    puncher.moveAbsolute(puncherTarget, 200);
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

void defaults(const std::string& name, const json& data) {
    auto &state = getState();
    auto loc = state.find(name);
    if(loc == state.end()) {
        state[name] = data;
        printf("All default settings were applied for %s.\n", name.c_str());
        saveState();
    } else {
        for(auto &[k, v]: data.items()) {
            if(loc->find(k) == loc->end()) {
                state[name][k] = v;
                printf("Default setting was applied for %s.%s.\n", name.c_str(), k.c_str());
            }
        }
        saveState();
    }
}

json& getGPSState() {
    defaults("gps", {
        {"cpr", 442.5252999999999 * (360.0 / 900.0)},
        {"cpi", 68.5052 * (360.0 / 900.0)}
    });
    return getState()["gps"];
}

json& getBaseState() {
    defaults("base", {
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
        }},
        {"voltage", false}
    });
    return getState()["base"];
}

json& getPuncherState() {
    defaults("puncher", {
        {"kP", 0.001},
        {"kI", 0.001},
        {"kD", 0.001},
        {"low", 800},
        {"high", 1200}
    });
    return getState()["puncher"];
}

pros::Mutex Elliot::usageGuard;

Elliot::Elliot():
controller{CONTROLLER_MASTER},
left{12, 1},
right{-3, -4},
puncherMtr{5},
angler{8},
intake{10},
scorer{6},
angleSense{'A'},
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
    intake.setGearing(AbstractMotor::gearset::blue);
    scorer.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
    scorer.setGearing(AbstractMotor::gearset::red);
}

void Elliot::takeCoast() {
    usageGuard.take(TIMEOUT_MAX);
}

void Elliot::stop() {
    puncher.setVelocity(0);
    angler.moveVelocity(0);
    intake.moveVelocity(0);
    scorer.moveVelocity(0);
    left .setBrakeMode(AbstractMotor::brakeMode::coast);
    right.setBrakeMode(AbstractMotor::brakeMode::coast);
    base->stop();
    base->setMaxVelocity((int)getRobot().left.getGearing());
    base->setMaxVoltage(12000);
}

void Elliot::takeStopped() {
    takeCoast();
    getRobot().stop();
}

void Elliot::giveDirect() {
    usageGuard.give();
}

void Elliot::give() {
    getRobot().stop();
    usageGuard.give();
}

void Elliot::beginTasks() {
    gps.beginTask();
    puncher.beginTask();
    pros::Task shotTask(autoshootTask);
}

Elliot* elliot;
void createRobot() {
    if(elliot) delete elliot;
    elliot = new Elliot();
}
