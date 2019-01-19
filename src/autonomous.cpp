#include "main.h"
#include "display.hpp"
#include "autonomous.hpp"
#include "gps.hpp"
#include "state.hpp"
#include "elliot.hpp"
using namespace std;

int tryGo(GPS& gps, double L, double R) {
  double higher = max(abs(L), abs(R));
  double scale = (int)gps.left.getGearing() / higher;
  if(higher < 800) {
    //Measure initial positions
    double initial_left = gps.left.getPosition();
    double initial_right = gps.right.getPosition();
    //First, go full speed towards the targets.
    gps.left.moveRelative(L, scale * L);
    gps.right.moveRelative(R, scale * R);
    //Create a settle detector
    auto settleDetector = SettledUtilFactory::create();
    //Now, re-adjust as the error closes in.
    while(true) {
      //Get errors, which should be smaller than higher.
      double lerr = higher - abs(gps. left.getPosition() -  initial_left);
      double rerr = higher - abs(gps.right.getPosition() - initial_right);
      double err = max(lerr, rerr);
      //Pass this error to settleDetector. If it's true, return.
      if(settleDetector.isSettled(err)) {
        break;
      }
      //If it's negative, return. Just in case.
      if(err < 0) {
        printf("Err was less than 0. (?)\n");
        break;
      }
      //Error still high? Slow down.
      gps. left.modifyProfiledVelocity(scale * L * ((err / higher) * 0.8 + 0.2));
      gps.right.modifyProfiledVelocity(scale * L * ((err / higher) * 0.8 + 0.2));
      pros::delay(5);
    }
    return true;
  } else {
    //Find velocities
    gps. left.moveVelocity(scale * L);
    gps.right.moveVelocity(scale * R);
  }
  return false;
}

void moveToSetpoint(RoboPosition pt, GPS& gps, double velLimit) {
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
            curSign = L > 0 ? 1 : -1;
        } else {
            //What's the turning radius we need?
            auto r = (dx*dx + dy*dy) / denom;
            auto Cx = robot.x - sin(robot.o) * r;
            auto Cy = robot.y + cos(robot.o) * r;
            auto spAngle = atan2(pt.y-Cy, pt.x-Cx);
            auto rAngle = robot.o - PI/2;
            auto dTheta = periodicallyEfficient(spAngle - rAngle);
            //Find R/L
            L = dTheta * (r - 9);
            R = dTheta * (r + 9);
            curSign = (dTheta * r) > 0 ? 1 : -1;
        }
        if(max(abs(R), abs(L)) == 0) break;
        //Set initialSign
        if(initialSign == 0) {
            initialSign = curSign;
        }
        //Dance
        if(tryGo(gps, L, R)) {
          break;
        }
        pros::delay(5);
    } while(true);
    //brake
    gps.left.controllerSet(0);
    gps.right.controllerSet(0);
}

bool isBlue;
void setBlue(bool blue) {
  isBlue = blue;
  printf("Blue: %d\n", isBlue);
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
    double rX = (isBlue ? -1 : 1) * motionObject["x"].get<double>();
    double rY =                     motionObject["y"].get<double>();
    lastPos.x += bot.gps.inchToCounts(rX);
    lastPos.y += bot.gps.inchToCounts(rY);
    moveToSetpoint({
      lastPos.x,
      lastPos.y,
      0
    }, bot.gps, motionObject["t"].get<double>() * 1000);
  }
  if(type == "rotateTo") {
    double dTheta = motionObject["o"].get<double>();
    if(isBlue) dTheta *= -1;
    dTheta -= bot.gps.getPosition().o;
    dTheta = periodicallyEfficient(dTheta);
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
    runMotion({
      {"x", lastPos.x + cos(lastPos.o) * distance},
      {"y", lastPos.y + sin(lastPos.o) * distance},
      {"t", motionObject["t"].get<double>()},
      {"v", motionObject["v"].get<double>()}
    }, lastPos, isBlue);
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
    lastPos.x += cos(lastPos.o) * distance;
    lastPos.y += sin(lastPos.o) * distance;
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
    (*loc)["x"].get<double>(),
    (*loc)["y"].get<double>(),
    (*loc)["o"].get<double>()
  };
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
