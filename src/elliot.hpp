#pragma once
#include "main.h"
#include "okapi/api.hpp"
#include "gps.hpp"
#include "display.hpp"
#include "camera.hpp"
#include "base.hpp"
#include "debugging.hpp"
#include "ccpid_mod.hpp"
#include <deque>
using namespace okapi;

class Puncher {
    volatile bool shooting = false;
    int lowTargetPosition = 800;
    int highTargetPosition = 1200;
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

struct BaseBox {
    okapi::Elliot2CCPID base;
    BaseBox(
  const TimeUtil &itimeUtil,
  const std::shared_ptr<ChassisModel> &imodel,
  std::unique_ptr<IterativePosPIDController> idistanceController,
  std::unique_ptr<IterativePosPIDController> iangleController,
  std::unique_ptr<IterativePosPIDController> iturnController,
  const AbstractMotor::GearsetRatioPair igearset,
  const ChassisScales &iscales):
    base(itimeUtil, imodel, std::move(idistanceController), std::move(iangleController), std::move(iturnController), igearset, iscales) {}
};

class Elliot {
	int multiplier = 1;
    friend void createRobot();
    pros::Mutex usageGuard;
    public:
    long opctrlBegin;
    pros::Controller controller;
    pros::Vision camera;
    CameraSettings camSettings;
	MotorGroup left;
	MotorGroup right;
	MotorGroup puncherMtr;
	MotorGroup angler;
	MotorGroup intake;
    MotorGroup arm;
    okapi::Potentiometer angleSense;
    pros::ADIUltrasonic leftSonic;
    pros::ADIUltrasonic rightSonic;
    Puncher puncher;
    GPS gps;
    BaseBox* box;
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
