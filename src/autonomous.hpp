#pragma once
#include "main.h"
#include "json.hpp"
#include "gps.hpp"
#include <string>
using json = nlohmann::json;
void runMotion(json motionObject, bool isBlue);
void runAuton(json& motionArray, bool isBlue);
void runAutonNamed(std::string name, bool isBlue);
void moveToSetpoint(RoboPosition pt, GPS& gps, int settleTime);
void setBlue(bool);
bool getBlue();