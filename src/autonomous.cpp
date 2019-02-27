#include "main.h"
#include "display.hpp"
#include "autonomous.hpp"
#include "gps.hpp"
#include "state.hpp"
#include "elliot.hpp"
#include "debugging.hpp"
#include "pros/apix.h"
using namespace std;

bool isBlue;
void setBlue(bool blue) {
  isBlue = blue;
}

bool getBlue() {
  return isBlue;
}

//Ball tracking function
void trackBall(double maxVel, double threshold, double oovThreshold, double attack, int extraTime) {
  auto &bot = getRobot();
  auto &cha = bot.box->base;
  cha.setMaxVelocity(maxVel * (int)bot.left.getGearing());
  //"Green card" signature
  bot.camera.print_signature(bot.camera.get_signature(1));
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
      //break;
    }
    //Too left
    if(object.left_coord < half-centerThreshold) {
      cha.driveVector(1.0, -0.3);
    }
    //Perfect
    if(half-centerThreshold < object.left_coord && object.left_coord < half+centerThreshold) {
      cha.driveVector(1.0, 0.0);
    }
    //Too right
    if(object.left_coord > half+centerThreshold) {
      cha.driveVector(1.0, 0.3);
    }
  }
  //Go forward, intake off.
  cha.moveDistance(attack);
  bot.intake.moveVelocity(0);
}

void moveToSetpoint(RoboPosition pt, double velLimit, bool reverse, int extraTime, int turnExtraTime) {
  auto &bot = getRobot();
  auto &gps = bot.gps;
  auto &cha = bot.box->base;
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
    },
    motionObject["v"].get<double>(),         //Max velocity
    motionObject["r"].get<bool>(),           //Reverse? 
    motionObject["t"].get<double>() * 1000,  //Extra straight time
    motionObject["rT"].get<double>() * 1000);//Extra turn time
  }
  if(type == "rotateTo") {
    double dTheta = motionObject["o"].get<double>();
    dTheta += offset.o;
    dTheta = isBlue ? PI - dTheta : dTheta;
    dTheta -= bot.gps.getPosition().o;
    dTheta = periodicallyEfficient(dTheta);
    double velLimit = motionObject["v"].get<double>();
    bot.box->base.setMaxVelocity(velLimit * (int)bot.left.getGearing());
    bot.box->base.turnAngle(dTheta * -(180 / PI) * okapi::degree);
    pros::delay(motionObject["t"].get<double>() * 1000);
  }
  if(type == "autoball") {
    trackBall(
      motionObject["v"].get<double>(),
      motionObject["c"].get<double>(),
      motionObject["d"].get<double>(),
      bot.gps.inchToCounts(motionObject["a"].get<double>()),
      motionObject["t"].get<double>() * 1000
    );
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
  if(type == "direct") {
    bot.box->base.stop();
    bot.left.moveVelocity(motionObject["l"].get<double>());
    bot.right.moveVelocity(motionObject["r"].get<double>());
    if(motionObject["t"].get<double>() != 0) {
      pros::delay(motionObject["t"].get<double>() * 1000);
      bot.left.moveVelocity(0);
      bot.right.moveVelocity(0);
    }
  }
  if(type == "arm") {
    double position = motionObject["p"].get<double>();
    bot.arm.moveAbsolute(position, 100);
    pros::delay(motionObject["t"].get<double>() * 1000);
  }
  if(type == "sline") {
    double distance = motionObject["d"].get<double>();
    bot.box->base.moveDistance(distance);
    pros::delay(motionObject["t"].get<double>() * 1000);
  }
}

void runAuton(json::iterator loc, json::iterator end, bool isBlue) {
  auto &bot = getRobot();
  bot.arm.tarePosition();
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
