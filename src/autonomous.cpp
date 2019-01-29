#include "main.h"
#include "display.hpp"
#include "autonomous.hpp"
#include "gps.hpp"
#include "state.hpp"
#include "elliot.hpp"
#include "debugging.hpp"
#include "pros/apix.h"
using namespace std;

double maxVel(GPS& gps) {
  return max(abs(gps.left.getActualVelocity()), abs(gps.right.getActualVelocity())) / (double)(int)gps.left.getGearing();
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
        if(tryGoAutomatic(gps, L, R, velLimit)) {
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

//Ball tracking function
void trackBall(double maxVel, double threshold, double oovThreshold, double attack) {
  auto &bot = getRobot();
  bot.intake.moveVelocity(200);
  while(true) {
    //Where's the ball?
    //Wait 400ms to definitively say there's no ball.
    pros::vision_object_s_t object;
    for(int i = 0; i < 4; i++) {
      pros::delay(50);
      object = bot.camera.get_by_sig(0, 1);
      if(object.signature != VISION_OBJECT_ERR_SIG) {
        break;
      }
    }
    if(object.signature == VISION_OBJECT_ERR_SIG) break;
    int centerThreshold = threshold/2;
    int half = VISION_FOV_WIDTH/2;
    //If the object is about to go out of view, stop.
    if(VISION_FOV_HEIGHT - object.top_coord < oovThreshold) {
      break;
    }
    //Too left
    if(object.left_coord < half-centerThreshold) {
      tryGoAccel(bot.gps, 0.75, 1.0, maxVel);
    }
    //Perfect
    if(half-centerThreshold < object.left_coord && object.left_coord < half+centerThreshold) {
      tryGoAccel(bot.gps, 1, 1, maxVel);
    }
    //Too right
    if(object.left_coord > half+centerThreshold) {
      tryGoAccel(bot.gps, 1, 0.75, maxVel);
    }
  }
  //Go forward, intake off.
  tryGoVeryAutomatic(bot.gps, attack, attack, maxVel);
  bot.intake.moveVelocity(0);
}

void runMotion(json motionObject, RoboPosition& offset, bool isBlue) {
  //set the brake mode in case it wasn't set before
  auto &bot = getRobot();
  //get the type of motion
  auto type = motionObject["type"].get<std::string>();
  //now, run the appropriate function for each type.
  if(type == "position") {
    //Get the target position.
    RoboPosition target = {
      bot.gps.inchToCounts(motionObject["x"].get<double>()) + offset.x,
      bot.gps.inchToCounts(motionObject["y"].get<double>()) + offset.y,
      0
    };
    //Now, apply the blue mode by reversing the target x.
    if(isBlue) target.x = bot.gps.inchToCounts(144) - target.x;
    moveToSetpoint({
      target.x,
      target.y,
      0
    }, bot.gps, motionObject["v"].get<double>(), motionObject["s"].get<bool>());
  }
  if(type == "rotateTo") {
    double dTheta = motionObject["o"].get<double>();
    dTheta = isBlue ? PI - dTheta : dTheta;
    dTheta -= bot.gps.getPosition().o;
    dTheta = periodicallyEfficient(dTheta);
    double velLimit = motionObject["v"].get<double>();
    tryGoVeryAutomatic(bot.gps, -bot.gps.radiansToCounts(dTheta), bot.gps.radiansToCounts(dTheta), velLimit);
  }
  if(type == "autoball") {
    trackBall(motionObject["v"].get<double>(), motionObject["c"].get<double>(), motionObject["d"].get<double>(), bot.gps.inchToCounts(motionObject["a"].get<double>()));
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
    bot.catapult.goToSwitch();
    while(bot.catapult.isGoingToSwitch()) {
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
  if(type == "origin") {
    //Offset does not respect blueMode, but the GPS position should.
    RoboPosition newPos = {
      bot.gps.inchToCounts(motionObject["x"].get<double>()),
      bot.gps.inchToCounts(motionObject["y"].get<double>()),
      motionObject["o"].get<double>()
    };
    offset = {
      newPos.x,
      newPos.y,
      0
    };
    newPos.x = isBlue ? bot.gps.inchToCounts(144) - newPos.x : newPos.x;
    newPos.o = isBlue ? PI - newPos.o : newPos.o;
    bot.gps.setPosition(newPos);
  }
  if(type == "delta") {
    offset = {
      bot.gps.inchToCounts(motionObject["x"].get<double>()),
      bot.gps.inchToCounts(motionObject["y"].get<double>()),
      motionObject["o"].get<double>()
    };
  }
}

void runAuton(json::iterator loc, json::iterator end, bool isBlue) {
  auto &bot = getRobot();
  auto oldBrake = bot.left.getBrakeMode();
  bot. left.setBrakeMode(AbstractMotor::brakeMode::coast);
  bot.right.setBrakeMode(AbstractMotor::brakeMode::coast);
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
  printf("%s does not name an autonomous. Will stall.\n", name);
}

void autonomous() {
  runAutonNamed(getSelectedAuton(), isBlue);
}
