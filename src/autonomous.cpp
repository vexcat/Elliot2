#include "main.h"
#include "display.hpp"
#include "autonomous.hpp"
#include "gps.hpp"
#include "state.hpp"
#include "elliot.hpp"
#include "debugging.hpp"
#include "pros/apix.h"
using namespace std;

//The goal of this class is mainly to limit acceleration by using PID.
//This acceleration limiting affects both sides of the robot equally.
class PIDController {
  MotorGroup &left;
  MotorGroup &right;
  PIDGains& gains;
  double lTarget;
  double rTarget;
  enum {
    FOLLOWING_NONE,
    FOLLOWING_LEFT,
    FOLLOWING_RIGHT
  } follow;
  IterativePosPIDController controller;
  double velLimit;
  public:
  PIDController(MotorGroup &outputLeft, MotorGroup &outputRight, PIDGains gains, double limit): left(outputLeft), right(outputRight), gains(gains),
  controller(IterativeControllerFactory::posPID(gains.kP, gains.kI, gains.kD)), velLimit(limit) {}

  void setTarget(double L, double R) {
    //For the sake of including both turns and forward motions, target will be the max of L & R.
    //moveToSetpoint is responsible for tuning R/L ratio.
    if(abs(L) > abs(R)) {
      follow = FOLLOWING_LEFT;
      controller.setTarget(L);
    } else {
      follow = FOLLOWING_RIGHT;
      controller.setTarget(R);
    }
    lTarget = L;
    rTarget = R;
    controller.setTarget(max(abs(L), abs(R)));
  }

  void stepError(double L, double R, bool farTarget = false) {
    //farTarget means L & R are not a target, but a movement ratio. Only accelerate, no deceleration.
    if(farTarget) {
      L *= 100000.0;
      R *= 100000.0;
    }
    //Set the target if none is set.
    if(follow == FOLLOWING_NONE) {
      setTarget(L, R);
    }
    //Now step with "absolute" position.
    stepAbs(lTarget - L, rTarget - R);
  }

  void stepAbs(double L, double R) {
    //Step controller based on follow.
    double controllerOutput;
    if(follow == FOLLOWING_LEFT) {
      controllerOutput = controller.step(L);
    } else if(follow == FOLLOWING_RIGHT) {
      controllerOutput = controller.step(R);
    }
    double higher = max(abs(lTarget - L), abs(rTarget - R));
    if(higher == 0) {
      left.moveVelocity(0);
      right.moveVelocity(0);
      return;
    }
    double scale = (velLimit * (int)left.getGearing()) / higher;
    left.moveVelocity(scale * (lTarget - L) * controllerOutput);
    right.moveVelocity(scale * (rTarget - R) * controllerOutput);
  }

  void reset() {
    follow = FOLLOWING_NONE;
    controller.reset();
  }

  void stopMtrs() {
    left.moveVelocity(0);
    right.moveVelocity(0);
  }

  bool done() {
    //Assume that if there is no movement past 60 units of error, we're done.
    if(abs(left.getActualVelocity()) < 5 && abs(right.getActualVelocity()) < 5 && abs(controller.getError()) < 60) return true;
    return false; 
  }
};

PIDController controllerFromGPS(GPS& gps, double velLimit) {
  return PIDController(gps.left, gps.right, gps.getPIDGains(), velLimit);
}

void moveToSetpoint(RoboPosition pt, GPS& gps, double velLimit, bool stayStraight, int extraTime) {
  auto controller = controllerFromGPS(gps, velLimit);
  //printf("moveToSetpoint was called with %f,%f on %f,%f.\n", pt.x, pt.y, gps.getPosition().x, gps.getPosition().y);
  //When sign of L going straight or dTheta*r turning changes, we're done.
  bool doneTimerStarted = false;
  uint32_t timeDone;
  auto lastTime = pros::millis();
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
      //printf("r = %f, dTheta = %f, L = %f, R = %f\n", r, dTheta, L, R);
    }
    if(max(abs(R), abs(L)) == 0) break;
    //Dance
    controller.stepError(L, R);
    //The controller thinks the motion is done. Activate the timed exit condition.
    if(controller.done()) {
      doneTimerStarted = true;
      timeDone = pros::millis();
    }
    //If the motion has been done for more than extraTime, break the loop.
    if(pros::millis() >= timeDone + extraTime) {
      break;
    }
    pros::Task::delay_until(&lastTime, gps.getDeltaTime());
  }
  //brake
  controller.stopMtrs();
}

bool isBlue;
void setBlue(bool blue) {
  isBlue = blue;
  //printf("Blue: %d\n", isBlue);
}

bool getBlue() {
  return isBlue;
}

void automaticControl(double L, double R, double velLimit, int extraTime = 0) {
  auto &gps = getRobot().gps;
  auto controller = controllerFromGPS(gps, velLimit);
  controller.setTarget(L + gps.left.getPosition(), R + gps.right.getPosition());
  bool finished = false;
  auto lastTime = pros::millis();
  do {
    controller.stepAbs(gps.left.getPosition(), gps.right.getPosition());
    pros::Task::delay_until(&lastTime, gps.getDeltaTime());
    finished = finished || controller.done();
    if(finished) extraTime -= gps.getDeltaTime();
  } while(!finished || extraTime > 0);
  controller.stopMtrs();
}

//Ball tracking function
void trackBall(double maxVel, double threshold, double oovThreshold, double attack, int extraTime) {
  auto &bot = getRobot();
  bot.intake.moveVelocity(200);
  auto controller = controllerFromGPS(bot.gps, maxVel);
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
      controller.stepError(0.75, 1.0, true);
    }
    //Perfect
    if(half-centerThreshold < object.left_coord && object.left_coord < half+centerThreshold) {
      controller.stepError(1.0, 1.0, true);
    }
    //Too right
    if(object.left_coord > half+centerThreshold) {
      controller.stepError(1.0, 0.75, true);
    }
  }
  //Go forward, intake off.
  automaticControl(attack, attack, maxVel, extraTime);
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
    }, bot.gps, motionObject["v"].get<double>(), motionObject["s"].get<bool>(), motionObject["t"].get<double>() * 1000);
  }
  if(type == "rotateTo") {
    double dTheta = motionObject["o"].get<double>();
    dTheta += offset.o;
    dTheta = isBlue ? PI - dTheta : dTheta;
    dTheta -= bot.gps.getPosition().o;
    dTheta = periodicallyEfficient(dTheta);
    double velLimit = motionObject["v"].get<double>();
    automaticControl(-bot.gps.radiansToCounts(dTheta), bot.gps.radiansToCounts(dTheta), velLimit, motionObject["t"].get<double>());
  }
  if(type == "autoball") {
    trackBall(
      motionObject["v"].get<double>(),
      motionObject["c"].get<double>(),
      motionObject["d"].get<double>(),
      bot.gps.inchToCounts(motionObject["a"].get<double>()),
      motionObject["t"].get<double>() * 1000);
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
