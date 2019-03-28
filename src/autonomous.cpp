/**
 * @file autonomous.cpp
 * 
 * This file interprets JSON objects as individual steps of an autonomous.
 */

#include "main.h"
#include "display.hpp"
#include "autonomous.hpp"
#include "gps.hpp"
#include "state.hpp"
#include "elliot.hpp"
#include "debugging.hpp"
#include "pros/apix.h"
using namespace std;

///Whether to execute autonomous motions as on the blue side, or the red side.
bool isBlue;
void setBlue(bool blue) {
  isBlue = blue;
}
bool getBlue() {
  return isBlue;
}

//Move to a point, documented in .hpp.
void moveToSetpoint(RoboPosition pt, double velLimit, bool reverse, int extraTime, int turnExtraTime) {
  auto &bot = getRobot();
  auto &gps = bot.gps;
  auto &cha = *bot.base;
  //Stick to the speed limit
  cha.setMaxVelocity(velLimit * (int)bot.left.getGearing());
  //Get current position
  auto curPos = gps.getPosition();
  //Calculate distance
  double dx = pt.x - curPos.x;
  double dy = pt.y - curPos.y;
  double dist = sqrt(dx*dx + dy*dy);
  //Is it 0? We're done.
  if(dist == 0) return;
  //Calculate change in angle
  double dTheta = periodicallyEfficient(atan2(dy, dx) - curPos.o);
  //If it's behind the robot, flip and generate a negative distance.
  if(reverse) {
    dTheta = periodicallyEfficient(dTheta + PI);
    dist *= -1;
  }
  //Turn by dTheta radians CCW
  cha.turnAngle(dTheta * -(180.0 / PI) * okapi::degree);
  //Wait for turnExtraTime ms.
  pros::delay(turnExtraTime);
  //Move by dist degrees
  cha.moveDistance(dist);
  //Wait for extraTime ms.
  pros::delay(extraTime);
}

void runMotion(json motionObject, RoboPosition& offset, bool isBlue) {
  auto &bot = getRobot();
  //Get the type of motion
  auto type = motionObject["type"].get<std::string>();
  //Now, run the appropriate function for each type.
  //Move to point
  if(type == "position") {
    //Get the target position.
    RoboPosition target = {
      bot.gps.inchToCounts(motionObject["x"].get<double>()) + offset.x, //X in inches
      bot.gps.inchToCounts(motionObject["y"].get<double>()) + offset.y, //Y in inches
      0
    };
    //Now, apply the blue mode by reversing the target x.
    if(isBlue) target.x = bot.gps.inchToCounts(144) - target.x;
    //Move to point target
    moveToSetpoint(
      {
        target.x,
        target.y,
        0
      },
      motionObject["v" ].get<double>(),        //Max velocity
      motionObject["r" ].get<bool  >(),        //Reverse? 
      motionObject["t" ].get<double>() * 1000, //Extra straight time
      motionObject["rT"].get<double>() * 1000  //Extra turn time
    );
  }
  //Rotate to an absolute orientation
  if(type == "rotateTo") {
    double dTheta = motionObject["o"].get<double>();
    //Add offset orientation
    dTheta += offset.o;
    //Invert turn if on blue side
    dTheta = isBlue ? PI - dTheta : dTheta;
    //Subtract part of turn already completed
    dTheta -= bot.gps.getPosition().o;
    //Make periodically efficient, [-pi, pi] turns.
    dTheta = periodicallyEfficient(dTheta);
    //"v": Velocity limit in [0, 1]
    double velLimit = motionObject["v"].get<double>();
    bot.base->setMaxVelocity(velLimit * (int)bot.left.getGearing());
    //Turn by dTheta radians
    bot.base->turnAngle(dTheta * -(180 / PI) * okapi::degree);
    //Wait extra "t" seconds
    pros::delay(motionObject["t"].get<double>() * 1000);
  }
  //Move puncher to low target
  if(type == "low") {
    bot.puncher.lowTarget();
    pros::delay(1000 * motionObject["t"].get<double>());
  }
  //Move puncher to high target
  if(type == "high") {
    bot.puncher.highTarget();
    pros::delay(1000 * motionObject["t"].get<double>());
  }
  //Punch ball
  if(type == "punch") {
    bot.puncher.shoot();
    pros::delay(1000 * motionObject["t"].get<double>());
  }
  //Spin intake
  if(type == "intake") {
    //"v": Velocity to move intake by in [-1, 1]
    double v = motionObject["v"].get<double>();
    bot.intake.controllerSet(v);
    //If "t" is set, wait "t" seconds and stop the intake.
    double timing = motionObject["t"].get<double>();
    if(timing != 0) {
      pros::delay((int)(timing * 1000));
      bot.intake.controllerSet(0);
    }
  }
  //Delay by "t" seconds
  if(type == "delay") {
    pros::delay((int)(1000 * motionObject["t"].get<double>()));
  }
  //Brake Modes
  if(type == "hold") {
    bot. left.setBrakeMode(AbstractMotor::brakeMode::hold);
    bot.right.setBrakeMode(AbstractMotor::brakeMode::hold);
  }
  if(type == "coast") {
    bot. left.setBrakeMode(AbstractMotor::brakeMode::coast);
    bot.right.setBrakeMode(AbstractMotor::brakeMode::coast);
  }
  if(type == "short") {
    bot. left.setBrakeMode(AbstractMotor::brakeMode::brake);
    bot.right.setBrakeMode(AbstractMotor::brakeMode::brake);
  }
  //Set GPS position
  if(type == "origin") {
    //"x"/"y"/"o": X & Y in inches, orientation in radians.
    RoboPosition newPos = {
      bot.gps.inchToCounts(motionObject["x"].get<double>()),
      bot.gps.inchToCounts(motionObject["y"].get<double>()),
      motionObject["o"].get<double>()
    };
    //Offset should start with an orientation of 0, always.
    //This can be changed by the user with a "delta" command
    //but is rarely needed.
    offset = {
      newPos.x,
      newPos.y,
      0
    };
    //Offset does not respect blueMode, but the GPS position should.
    newPos.x = isBlue ? bot.gps.inchToCounts(144) - newPos.x : newPos.x;
    newPos.o = isBlue ? PI - newPos.o : newPos.o;
    bot.gps.setPosition(newPos);
  }
  //Set just the offset
  if(type == "delta") {
    //"x"/"y"/"o": X & Y in inches, orientation in radians.
    offset = {
      bot.gps.inchToCounts(motionObject["x"].get<double>()),
      bot.gps.inchToCounts(motionObject["y"].get<double>()),
      motionObject["o"].get<double>()
    };
  }
  //Directly control base motors, without PID.
  if(type == "direct") {
    bot.base->stop();
    //"l"/"r": Left & right velocities in [-1, 1]
    double l = motionObject["l"].get<double>() * (int)bot.left.getGearing();
    double r = motionObject["r"].get<double>() * (int)bot.right.getGearing();
    //Respect isBlue by swapping l & r.
    bot.left.moveVelocity(isBlue ? r : l);
    bot.right.moveVelocity(isBlue ? l : r);
    //If "t" is set: delay "t" seconds then stop base
    if(motionObject["t"].get<double>() != 0) {
      pros::delay(motionObject["t"].get<double>() * 1000);
      bot.left.moveVelocity(0);
      bot.right.moveVelocity(0);
    }
  }
  //Scorer motor control
  if(type == "scorer") {
    //"p": Position in degrees to move to, at 100rpm
    double position = motionObject["p"].get<double>();
    bot.scorer.moveAbsolute(position, 100);
    //"t": Wait time for scorer to start moving
    pros::delay(motionObject["t"].get<double>() * 1000);
  }
  //Straight Line
  if(type == "sline") {
    //"d": Distance to travel in inches
    double distance = bot.gps.inchToCounts(motionObject["d"].get<double>());
    //"v": Velocity limit in [0, 1]
    bot.base->setMaxVelocity(motionObject["v"].get<double>() * (int)bot.left.getGearing());
    //Move distance
    bot.base->moveDistance(distance);
    //"t": Extra time to wait after moving
    pros::delay(motionObject["t"].get<double>() * 1000);
  }
}

void runAuton(json::iterator loc, json::iterator end, bool isBlue) {
  auto &bot = getRobot();
  bot.scorer.tarePosition();
  auto oldBrake = bot.left.getBrakeMode();
  bot. left.setBrakeMode(AbstractMotor::brakeMode::coast);
  bot.right.setBrakeMode(AbstractMotor::brakeMode::coast);
  bot.scorer.setBrakeMode(AbstractMotor::brakeMode::coast);
  RoboPosition tracking = {0, 0, 0};
  for(; loc != end; loc++) {
    runMotion(*loc, tracking, isBlue);
  }
  bot. left.setBrakeMode(oldBrake);
  bot.right.setBrakeMode(oldBrake);
}

void runAutonNamed(std::string name, bool isBlue) {
  auto &autons = getState()["autons"];
  auto autonWithName = (autons.find(name));
  if(autonWithName != autons.end()) {
    runAuton(autonWithName->begin(), autonWithName->end(), isBlue);
    return;
  }
  printf("%s does not name an autonomous. Will stall.\n", name.c_str());
}

/**
 * autonomous() is a function called by PROS when competition
 * control state switches to autonomous mode. This implementation
 * will call runAutonNamed() with the name of the auton selected
 * on the brain screen's autonomous selector.
 */
void autonomous() {
  runAutonNamed(getSelectedAuton(), isBlue);
}
