#pragma once
#include "main.h"
#include "okapi/api.hpp"
#include "gps.hpp"
#include "display.hpp"
#include "camera.hpp"
#include "base.hpp"
#include "debugging.hpp"
#include <deque>
using namespace okapi;

class Catapult {
    volatile bool headingToSwitch = false;
    okapi::MotorGroup &cata;
    pros::ADIDigitalIn &sensor;
    public:
    Catapult(okapi::MotorGroup& cata, pros::ADIDigitalIn& sensor);
    void catapultTask();
    void goToSwitch();
    void setVelocity(double vel);
    bool isGoingToSwitch();
    void beginTask();
};

struct BaseBox {
    okapi::ChassisControllerPID base;
    BaseBox(okapi::MotorGroup &l, okapi::MotorGroup &r, okapi::IterativePosPIDController::Gains d, okapi::IterativePosPIDController::Gains a, okapi::IterativePosPIDController::Gains t, okapi::AbstractMotor::GearsetRatioPair gear, okapi::ChassisScales s):
    base(okapi::ChassisControllerFactory::create(l, r, d, a, t, gear, s)) {}
};

class Elliot {
	int multiplier = 1;
	double speedMultiplier = 1.0;
	int lastR1 = 0;
    friend void createRobot();
    pros::Mutex usageGuard;
    public:
    long opctrlBegin;
    pros::Controller controller;
    pros::Vision camera;
    CameraSettings camSettings;
	MotorGroup left;
	MotorGroup right;
	MotorGroup catapultMtr;
	MotorGroup score;
	MotorGroup intake;
    MotorGroup arm;
    pros::ADIDigitalIn catapultLimit;
    pros::ADIUltrasonic leftSonic;
    pros::ADIUltrasonic rightSonic;
    Catapult catapult;
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