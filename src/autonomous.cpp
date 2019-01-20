#include "main.h"
#include "display.hpp"
#include "autonomous.hpp"
#include "gps.hpp"
#include "state.hpp"
#include "elliot.hpp"
#include "debugging.hpp"
using namespace std;

int tryGo(GPS& gps, double L, double R, double velLimit) {
  double higher = max(abs(L), abs(R));
  if(higher == 0) return true;
  double scale = (velLimit * (int)gps.left.getGearing()) / higher;
  //Set velocities
  gps. left.moveVelocity(scale * L);
  gps.right.moveVelocity(scale * R);
  if(higher < gps.inchToCounts(4)) {
    //Measure initial positions
    double initial_left = gps.left.getPosition();
    double initial_right = gps.right.getPosition();
    //Now, re-adjust as the error closes in.
    while(true) {
      //Get errors, which should be smaller than higher.
      double lerr = higher - abs(gps. left.getPosition() -  initial_left);
      double rerr = higher - abs(gps.right.getPosition() - initial_right);
      double err = max(lerr, rerr);
      //If it's negative, return.
      if(err < 0) {
        break;
      }
      //Error still high? Slow down.
      double  leftVel = scale * L * (((err / higher) * 0.8 + 0.2));
      double rightVel = scale * R * (((err / higher) * 0.8 + 0.2));
      gps. left.moveVelocity( leftVel);
      gps.right.moveVelocity(rightVel);
      pros::delay(5);
    }
    gps.left.moveVelocity(0);
    gps.right.moveVelocity(0);
    while(gps.left.getActualVelocity() || gps.right.getActualVelocity()) {
      pros::delay(5);
    }
    return true;
  } else {
    //Find velocities
    //printf("Velocities used were %f,%f\n", scale * L, scale * R);
  }
  return false;
}

void moveToSetpoint(RoboPosition pt, GPS& gps, double velLimit, bool stayStraight) {
    //printf("moveToSetpoint was called with %f,%f on %f,%f.\n", pt.x, pt.y, gps.getPosition().x, gps.getPosition().y);
    //When sign of L going straight or dTheta*r turning changes, we're done.
    double initialSign = 0;
    double curSign = 0;
    while(true) {
        RoboPosition robot = gps.getPosition();
        auto dx = pt.x - robot.x;
        auto dy = pt.y - robot.y;
        auto denom = (2*dy*cos(robot.o) - 2*dx*sin(robot.o));
        double L;
        double R;
        if(abs(denom) < 0.001 || stayStraight) {
            //If denominator is 0, the turning radius is infinite.
            //We can compare the angle of the object relative to the robot to the robot's actual position to check forwards/backwards.
            auto spRAngle = periodicallyEfficient(atan2(dy, dx) - robot.o);
            auto dist = sqrt(dx*dx + dy*dy);
            //If it's within pi of robot.o, go forward.
            if(-PI/2 < spRAngle && spRAngle < PI/2) {
                L = dist;
                R = dist;
            } else {
                L = -dist;
                R = -dist;
            }
            curSign = L > 0 ? 1 : -1;
        } else {
            //What's the turning radius we need?
            auto r = (dx*dx + dy*dy) / denom;
            auto Cx = robot.x - sin(robot.o) * r;
            auto Cy = robot.y + cos(robot.o) * r;
            auto spAngle = atan2(pt.y-Cy, pt.x-Cx);
            auto rAngle = atan2(robot.y-Cy, robot.x-Cx);
            auto dTheta = periodicallyEfficient(spAngle - rAngle);
            //Find R/L
            L = dTheta * (r - getRobot().gps.radiansToCounts(1));
            R = dTheta * (r + getRobot().gps.radiansToCounts(1));
            curSign = (dTheta * r) > 0 ? 1 : -1;
            //printf("r = %f, dTheta = %f, L = %f, R = %f\n", r, dTheta, L, R);
        }
        if(max(abs(R), abs(L)) == 0) break;
        //Set initialSign
        if(initialSign == 0) {
            initialSign = curSign;
        }
        //Dance
        if(tryGo(gps, L, R, velLimit)) {
          break;
        }
        if(curSign != initialSign) {
          break;
        }
        pros::delay(5);
    }
    //brake
    gps.left.controllerSet(0);
    gps.right.controllerSet(0);
}

bool isBlue;
void setBlue(bool blue) {
  isBlue = blue;
  //printf("Blue: %d\n", isBlue);
}

bool getBlue() {
  return isBlue;
}

void runMotion(json motionObject, RoboPosition& lastPos, bool isBlue) {
  //set the brake mode in case it wasn't set before
  auto &bot = getRobot();
  //get the type of motion
  auto type = motionObject["type"].get<std::string>();
  //now, run the appropriate function for each type.
  if(type == "position") {
    double rX = motionObject["x"].get<double>();
    if(isBlue) {
      rX *= -1;
    }
    double rY = motionObject["y"].get<double>();
    lastPos.x += bot.gps.inchToCounts(rX);
    lastPos.y += bot.gps.inchToCounts(rY);
    moveToSetpoint({
      lastPos.x,
      lastPos.y,
      0
    }, bot.gps, motionObject["v"].get<double>(), motionObject["_goStraight"].get<bool>());
    pros::delay(motionObject["t"].get<double>() * 1000);
  }
  if(type == "rotateTo") {
    double dTheta = motionObject["o"].get<double>();
    dTheta = isBlue ? PI - dTheta : dTheta;
    dTheta -= bot.gps.getPosition().o;
    dTheta = periodicallyEfficient(dTheta);
    double initialSign = dTheta > 0 ? 1 : -1;
    double initialLeft = bot.left.getPosition();
    double initialRight = bot.right.getPosition();
    while(true) {
      double lDelta = bot.left.getPosition() - initialLeft;
      double rDelta = bot.right.getPosition() - initialRight;
      double measuredDTheta = bot.gps.countsToRadians(rDelta - lDelta) / 2;
      double velLimit = motionObject["v"].get<double>();
      if(tryGo(bot.gps, -bot.gps.radiansToCounts(dTheta - measuredDTheta), bot.gps.radiansToCounts(dTheta - measuredDTheta), velLimit)) {
        bot.gps.left.moveVelocity(0);
        bot.gps.right.moveVelocity(0);
        break;
      }
      pros::delay(2);
    }
  }
  if(type == "sline") {
    auto position = bot.gps.getPosition();
    double distance = motionObject["d"].get<double>();
    runMotion({
      {"x", cos(getBlue() ? PI - position.o: position.o) * distance},
      {"y", sin(getBlue() ? PI - position.o: position.o) * distance},
      {"t", motionObject["t"].get<double>()},
      {"v", motionObject["v"].get<double>()},
      {"_goStraight", true},
      {"type", "position"}
    }, lastPos, isBlue);
  }
  if(type == "scorer") {
    double v = motionObject["v"].get<double>() * (int)bot.score.getGearing();
    bot.score.controllerSet(v);
    double timing = motionObject["t"].get<double>();
    if(timing != 0) {
      pros::delay((int)(timing * 1000));
      bot.score.controllerSet(0);
    }
  }
  if(type == "catapult") {
    bot.catapult.setVelocity(motionObject["v"].get<double>() * (int)bot.catapultMtr.getGearing());
    double timing = motionObject["t"].get<double>();
    if(timing != 0) {
      pros::delay((int)(timing * 1000));
      bot.catapult.setVelocity(0);
    }
  }
  if(type == "intake") {
    double v = motionObject["v"].get<double>() * (int)bot.intake.getGearing();
    bot.intake.controllerSet(v);
    double timing = motionObject["t"].get<double>();
    if(timing != 0) {
      pros::delay((int)(timing * 1000));
      bot.intake.controllerSet(0);
    }
  }
  if(type == "shoot") {
    bot.catapult.setVelocity((int)bot.catapultMtr.getGearing());
    pros::delay(300);
    bot.catapult.setVelocity(0);
    bot.catapult.goToSwitch();
    //Bit of a hack, but whatever.
    while(bot.catapultMtr.getTargetVelocity()) {
      pros::delay(5);
    }
  }
  if(type == "delay") {
    pros::delay((int)(1000 * motionObject["t"].get<double>()));
  }
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
}

void runAuton(json& motionArray, bool isBlue) {
  auto loc = motionArray.begin();
  auto &bot = getRobot();
  bot.left .setBrakeMode(AbstractMotor::brakeMode::hold);
  bot.right.setBrakeMode(AbstractMotor::brakeMode::hold);
  //Process the first entry, an Origin.
  RoboPosition tracking = {
    bot.gps.inchToCounts((*loc)["x"].get<double>()),
    bot.gps.inchToCounts((*loc)["y"].get<double>()),
    (*loc)["o"].get<double>()
  };
  tracking.x = getBlue() ? (bot.gps.countsToInch(144) - tracking.x) : tracking.x;
  tracking.o = getBlue() ? PI - tracking.o : tracking.o;
  bot.gps.setPosition(tracking);
  loc++;
  for(; loc != motionArray.end(); loc++) {
    runMotion(*loc, tracking, isBlue);
  }
}

void runAutonNamed(std::string name, bool isBlue) {
  auto &autons = getState()["autons"];
  auto autonWithName = (autons.find(name));
  if(autonWithName != autons.end()) {
    runAuton(*autonWithName, isBlue);
    return;
  }
  printf("%s does not name an autonomous. Will stall.\n", name);
}

void autonomous() {
  runAutonNamed(getSelectedAuton(), isBlue);
}
