#pragma once
#include "main.h"
#include "okapi/api.hpp"
#include "gps.hpp"
#include "display.hpp"
#include "base.hpp"
#include "debugging.hpp"
#include "ccpid_mod.hpp"
#include <deque>
using namespace okapi;

class Puncher {
    volatile bool shooting = false;
    int lowTargetPosition;
    int highTargetPosition;
    int lastPuncherPosition;
    okapi::MotorGroup &puncher;
    okapi::MotorGroup &angler;
    okapi::Potentiometer &angleSense;
    pros::Mutex reconstructionMutex;
    json& puncherData;
    std::shared_ptr<okapi::AsyncPosPIDController> controllerPtr;
    void loadState();
    void puncherTask();
    public:
    Puncher(okapi::MotorGroup& puncher, okapi::MotorGroup& angler, okapi::Potentiometer& angleSense, json& puncherData);
    void beginTask() {
        pros::Task([](void* me) {((Puncher*)me)->puncherTask();}, (void*)this);
    }
    void lowTarget();
    void highTarget();
    void stopAutoControl();
    void shoot();                 // Move motor 1 revolution
    void setVelocity(double vel); // Manually move puncher, implicitly call stopAutoControl()
    int targetError();            // Angler's error to its target
    void toggleTarget();
    okapi::IterativePosPIDController::Gains getGains();
    void setGains(okapi::IterativePosPIDController::Gains gains);
    void setLowTarget(int target);
    void setHighTarget(int target);
    int getLowTarget() { return lowTargetPosition; }
    int getHighTarget() { return highTargetPosition; }
};

class Elliot {
	int multiplier = 1;
    friend void createRobot();
    pros::Mutex usageGuard;
    public:
    long opctrlBegin;
    pros::Controller controller;
	MotorGroup left;
	MotorGroup right;
	MotorGroup puncherMtr;
	MotorGroup angler;
	MotorGroup intake;
    MotorGroup arm;
    okapi::Potentiometer angleSense;
    Puncher puncher;
    GPS gps;
    std::unique_ptr<Elliot2CCPID> base;
    BaseSettings baseSettings;
    Elliot();
    Elliot(const Elliot&) = delete;
    Elliot& operator=(const Elliot&) = delete;
    void stop();
    void takeCoast();
    void takeStopped();
    void give();
    void giveDirect();
    void drive(pros::Controller&);
    void beginTasks();
};
void createRobot();
extern Elliot* elliot;
inline Elliot& getRobot() { return *elliot; }
Elliot& getRobot();
