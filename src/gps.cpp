#include "main.h"
#include "gps.hpp"
#include "okapi/api.hpp"
#include "state.hpp"
#include "json.hpp"
#include <deque>
using namespace okapi;
using namespace std;

double periodicallyEfficient(double n, double p) {
    while(n < -p/2) n += p;
    while(n >  p/2) n -= p;
    return n;
}

GPS::GPS(MotorGroup& leftSide, MotorGroup& rightSide, json& idata): left(leftSide), right(rightSide), data(idata) {}
void GPS::beginTask() {
    pros::Task daemon([](void* obj){((GPS*)obj)->gpsDaemon();}, this);
}

void GPS::setCPR(double newCPR) {
    daemonLock.take(TIMEOUT_MAX);
    cpr = newCPR;
    data["cpr"] = newCPR;
    saveState();
    daemonLock.give();
}

void GPS::setCPI(double newCPI) {
    daemonLock.take(TIMEOUT_MAX);
    cpi = newCPI;
    data["cpi"] = newCPI;
    saveState();
    daemonLock.give();
}

RoboPosition GPS::getPosition() {
    RoboPosition ret;
    daemonLock.take(TIMEOUT_MAX);
    ret = position;
    daemonLock.give();
    return ret;
}

void GPS::setPosition(RoboPosition pos) {
    daemonLock.take(TIMEOUT_MAX);
    position = pos;
    daemonLock.give();
}

void GPS::gpsDaemon() {
    uint32_t lastTime = pros::c::millis();
    //These two change how encoder smoothing happens.
    const int sampleCount = 2;
    const int dT = 5;
    pair<double, double> lastMeasurement;
    deque<pair<double, double>> samples; 
    for(int i = 0; i < sampleCount; i++) {
        samples.push_back({0.0f, 0.0f});
    }
    while(true) {
        samples.pop_front();
        samples.push_back({left.getPosition(), right.getPosition()});
        pair<double, double> currentMeasurement;
        for(auto &sample: samples) {
            currentMeasurement.first += sample.first;
            currentMeasurement.second += sample.second;
        }
        currentMeasurement.first /= sampleCount;
        currentMeasurement.second /= sampleCount;
        pair<double, double> delta;
        delta.first = currentMeasurement.first - lastMeasurement.first;
        delta.second = currentMeasurement.second - lastMeasurement.second;
        lastMeasurement = currentMeasurement;

        daemonLock.take(TIMEOUT_MAX);
        addPosDelta(position, delta.first, delta.second);
        daemonLock.give();
        pros::c::task_delay_until(&lastTime, dT);
    }
}

void GPS::addPosDelta(RoboPosition& robot, double L, double R) {
    //No motion
    if(R == 0 && L == 0) {
        return;
    }

    //Change in angle
    double dTheta = countsToRadians(R-L) / 2;

    //Straight/reverse (Infinite circle)
    if(abs(dTheta) < 0.0001) {
        robot.x += cos(robot.o) * L;
        robot.y += sin(robot.o) * R;
        return;
    }

    //Turning circle radius
    double r = radiansToCounts((R + L) / (R - L));
	
    //Rotation around turning circle by dTheta
    //The math here is quite compact, but there's a simple way to think about it.
    //The sin & cos with robot.o serve to undo the current rotation about the turning circle, while
    //the sin & cos with robot.o + dTheta serve to update it with the new dTheta.
    //(-sin, cos) means r units from the left of the robot
    //(sin, -cos) means r units from the right of the robot
    robot.x += (sin(robot.o + dTheta) - sin(robot.o)) * r;
    robot.y += (cos(robot.o) - cos(robot.o + dTheta)) * r;
    robot.o += dTheta;
}