#pragma once
#include "main.h"
#include "okapi/api.hpp"
#include "gps.hpp"
#include "display.hpp"
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
};

class Elliot {
	int multiplier = 1;
	double speedMultiplier = 200;
	int lastR1 = 0;
    friend void createRobot();
    pros::Mutex usageGuard;
    void resetMtrs();
    public:
	MotorGroup left;
	MotorGroup right;
	MotorGroup catapultMtr;
	MotorGroup score;
	MotorGroup intake;
    pros::ADIDigitalIn catapultLimit;
    Catapult catapult;
    GPS gps;
    Elliot();
    Elliot(const Elliot&) = delete;
    Elliot(Elliot&&) = delete;
    Elliot& operator=(const Elliot&) = delete;
    Elliot& operator=(Elliot&&) = delete;
    void stop();
    void takeCoast();
    void takeStopped();
    void give();
    void giveDirect();
    void drive(pros::Controller&);
};
void createRobot();
Elliot& getRobot();