#pragma once
#include "main.h"
#include "json.hpp"
#include "gps.hpp"
#include <string>
using json = nlohmann::json;
void runAuton(json& motionArray, bool isBlue);
void runAutonNamed(std::string name, bool isBlue);
void moveToSetpoint(RoboPosition pt, GPS& gps, double velLimit, bool goStraight);
void setBlue(bool);
bool getBlue();
void runMotion(json motionObject, RoboPosition& lastPos, bool isBlue);