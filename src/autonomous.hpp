/**
 * @file autonomous.hpp
 * 
 * This file declares functions to interpret JSON objects as individual steps of an autonomous.
 */

#pragma once
#include "main.h"
#include "json.hpp"
#include "gps.hpp"
#include <string>
using json = nlohmann::json;

/**
 * Runs an auton from a given starting position in a list, to
 * an end position.
 * 
 * @param loc    Index at the start of auton
 * @param end    Index at the end of auton
 * @param isBlue Whether to invert turns
 * @see runMotion()
 */
void runAuton(json::iterator loc, json::iterator end, bool isBlue);

/**
 * Runs an auton saved on the SD card, by name.
 * Auton is loaded from getState()["autons"][name].
 * 
 * @param name   Name of autonomous to run
 * @param isBlue Whether to invert turns
 * @see runAuton()
 */
void runAutonNamed(std::string name, bool isBlue);

/**
 * Moves to a point using odometry data. This will first turn
 * towards the point, then move straight to it.
 * 
 * @param pt            Point to move to
 * @param velLimit      Velocity limit in [0, 1]
 * @param reverse       Whether to move straight to point in reverse/forward
 * @param extraTime     Extra time to let straight motion finish
 * @param turnExtraTime Extra time to let turn-to-face-point motion finish
 */
void moveToSetpoint(RoboPosition pt, double velLimit, bool stayStraight, int extraTime, int turnExtraTime = 0);

/**
 * Control whether autonomous motions will run as red or blue, inverting turns.
 */
void setBlue(bool);

/**
 * Check if turns are being inverted by the blue mode.
 */
bool getBlue();

/**
 * Run a single JSON motion. The offset parameter can be used to run motions
 * originally intended for a different area of the field, to tack together
 * autonomi. The isBlue parameter overrides the global blue mode, to invert
 * turns for the blue side.
 * 
 * @param motionObject JSON data to interpret
 * @param offset       x, y, and o to be added to motion data
 * @param isBlue       Whether to invert turns
 */
void runMotion(json motionObject, RoboPosition& offset, bool isBlue);
