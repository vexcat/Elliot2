#include "main.h"
#include "display.hpp"
#include "autonomous.hpp"
#include "gps.hpp"
#include "state.hpp"
#include "elliot.hpp"
using namespace std;

void moveToSetpoint(RoboPosition pt, GPS& gps, int settleTime) {
    //When sign of L going straight or dTheta*r turning changes, we're done.
    double initialSign = 0;
    double curSign = 0;
    do {
        RoboPosition robot = gps.getPosition();
        auto dx = pt.x - robot.x;
        auto dy = pt.y - robot.y;
        auto denom = (2*dy*cos(robot.o) - 2*dx*sin(robot.o));
        double L;
        double R;
        if(abs(denom) < 0.0001) {
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
            curSign = L > 0 ? 1 : 0;
        } else {
            //What's the turning radius we need?
            auto r = (dx*dx + dy*dy) / denom;
            auto Cx = robot.x - sin(robot.o) * r;
            auto Cy = robot.y + cos(robot.o) * r;
            auto spAngle = atan2(pt.y-Cy, pt.x-Cx);
            auto rAngle = atan2(robot.y-Cy, robot.x-Cx);
            auto dTheta = periodicallyEfficient(spAngle - rAngle);
            //Find R/L
            L = dTheta * (r - 9);
            R = dTheta * (r + 9);
            curSign = (dTheta * r) ? 2 : -2;
        }
        if(max(abs(R), abs(L)) == 0) break;
        //Set initialSign
        if(initialSign == 0) {
            initialSign = curSign;
        }
        //Find velocities
        gps. left.moveVelocity(((int)gps. left.getGearing()) * (L / max(abs(R), abs(L))));
        gps.right.moveVelocity(((int)gps.right.getGearing()) * (R / max(abs(R), abs(L))));
        pros::delay(5);
    } while(curSign == initialSign);
    //brake
    gps.left.controllerSet(0);
    gps.right.controllerSet(0);
    pros::delay(settleTime);
}

bool isBlue;
void setBlue(bool blue) {
  isBlue = blue;
  printf("Blue: %d\n", isBlue);
}

bool getBlue() {
  return isBlue;
}

void runMotion(json motionObject, bool isBlue) {
  //set the brake mode in case it wasn't set before
  auto &bot = getRobot();
  //get the type of motion
  auto type = motionObject["type"].get<std::string>();
  //now, run the appropriate function for each type.
  if(type == "direct") {
    double l = motionObject["l"].get<double>() * (int)bot. left.getGearing();
    double r = motionObject["r"].get<double>() * (int)bot.right.getGearing();
    //If we're blue, swap left and right.
    if(isBlue) {
      bot.left .moveVelocity(r);
      bot.right.moveVelocity(l);
    } else {
      bot.left .moveVelocity(l);
      bot.right.moveVelocity(r);
    }
  }
  if(type == "position") {
    RoboPosition pos = bot.gps.getPosition();
    moveToSetpoint({
      bot.gps.inchToCounts((isBlue ? -1 : 1) * motionObject["x"].get<double>() + pos.x),
      bot.gps.inchToCounts(                    motionObject["y"].get<double>() + pos.y),
      0
    }, bot.gps, motionObject["t"].get<double>() * 1000);
  }
  if(type == "rotation") {
    double dTheta = periodicallyEfficient(motionObject["o"].get<double>());
    if(isBlue) dTheta *= -1;
    double initialSign = dTheta > 0 ? 1 : -1;
    double initialLeft = bot.left.getPosition();
    double initialRight = bot.right.getPosition();
    bot.left .moveVelocity(-initialSign * (int)bot.left.getGearing());
    bot.right.moveVelocity( initialSign * (int)bot.right.getGearing());
    while(true) {
      double lDelta = bot.left.getPosition() - initialLeft;
      double rDelta = bot.right.getPosition() - initialRight;
      double measuredDTheta = bot.gps.countsToRadians(rDelta - lDelta) / 2;
      if(abs(measuredDTheta) > abs(dTheta)) {
        bot.left.controllerSet(0);
        bot.right.controllerSet(0);
        pros::delay(motionObject["t"].get<double>() * 1000);
        break;
      }
      pros::delay(2);
    }
  }
  if(type == "sline") {
    double distance = bot.gps.inchToCounts(motionObject["d"].get<double>());
    double initialSign = distance > 0 ? 1 : -1;
    double initialLeft = bot.left.getPosition();
    double initialRight = bot.right.getPosition();
    bot.left.moveVelocity(initialSign * (int)bot.left.getGearing());
    bot.right.moveVelocity(initialSign * (int)bot.right.getGearing());
    while(true) {
      double measuredDistance = (bot.left.getPosition() - initialLeft + bot.right.getPosition() - initialRight) / 2;
      if(abs(measuredDistance) > abs(distance)) {
        bot.left .controllerSet(0);
        bot.right.controllerSet(0);
        pros::delay((int)(motionObject["t"].get<double>() * 1000));
        break;
      }
      pros::delay(2);
    }
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
}

void runAuton(json& motionArray, bool isBlue) {
  auto loc = motionArray.begin();
  auto &bot = getRobot();
  bot.left .setBrakeMode(AbstractMotor::brakeMode::hold);
  bot.right.setBrakeMode(AbstractMotor::brakeMode::hold);
  //Process the first entry, an Origin.
  RoboPosition origin = {
    (*loc)["x"].get<double>(),
    (*loc)["y"].get<double>(),
    (*loc)["o"].get<double>()
  };
  bot.gps.setPosition(origin);
  loc++;
  for(; loc != motionArray.end(); loc++) {
    runMotion(*loc, isBlue);
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
