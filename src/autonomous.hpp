#pragma once
#include "main.h"
#include "json.hpp"
#include "gps.hpp"
#include <string>
using json = nlohmann::json;
void runAuton(json::iterator loc, json::iterator end, bool isBlue);
void runAutonNamed(std::string name, bool isBlue);
void moveToSetpoint(RoboPosition pt, double velLimit, bool stayStraight, int extraTime, int turnExtraTime = 0);
void setBlue(bool);
bool getBlue();
void runMotion(json motionObject, RoboPosition& lastPos, bool isBlue);