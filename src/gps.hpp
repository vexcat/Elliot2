#pragma once
#include "main.h"
#include "okapi/api.hpp"
#include "json.hpp"
using json = nlohmann::json;
using namespace okapi;

struct RoboPosition {
	double x;
	double y;
	double o;
};

struct PIDGains {
    double kP;
    double kI;
    double kD;
};

class GPS {
    RoboPosition position = {0, 0, 0};
    //Counts Per Radian (the important one)
    double cpr;
    //Counts Per Inch
    double cpi;
    //PID Gains
    PIDGains gains;
    //Delta Time - How often to apply PID in ms
    int dT;
    pros::Mutex daemonLock;
    json& data;
    public:
    MotorGroup& left;
    MotorGroup& right;
    GPS(MotorGroup& leftSide, MotorGroup& rightSide, json& idata);

    void setCPR(double newCPR);

    void setCPI(double newCPI);

    double countsToRadians(double counts) { return counts / cpr; }
    double radiansToCounts(double radians) { return radians * cpr; }

    double countsToInch(double counts) { return counts / cpi; }
    double inchToCounts(double inches) { return inches * cpi; }

    RoboPosition getPosition();

    void setPosition(RoboPosition pos);
    
    void gpsDaemon();

	double radius2(double r, double l);
	void addPosDelta(RoboPosition& robot, double L, double R);

    PIDGains getPIDGains() { return gains; }
    void setPIDGains(PIDGains gains);

    int getDeltaTime() { return dT; }
    void setDeltaTime(int delta);
    
    void beginTask();
};

double periodicallyEfficient(double n, double p = PI * 2);