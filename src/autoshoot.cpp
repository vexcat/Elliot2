#include "main.h"
#include "elliot.hpp"
#include "autoshoot.hpp"

void autoshoot(int toggleTime, int punchTime) {
  auto &puncher = getRobot().puncher;
  //Punch
  puncher.shoot();
  pros::delay(punchTime);
  //Toggle
  puncher.toggleTarget();
  pros::delay(toggleTime);
  //Punch
  puncher.shoot();
  pros::delay(punchTime);
}
void autoshootTask(void*) {
  auto &bot = getRobot();
  auto &ctrl = bot.controller;
  while(true) {
    bot.takeCoast();
    if(ctrl.get_digital_new_press(DIGITAL_A)) {
      auto prevBrakeMode = bot.left.getBrakeMode();
      bot.base->setBrakeMode(AbstractMotor::brakeMode::hold);
      bot.base->stop();
      autoshoot();
      bot.base->setBrakeMode(prevBrakeMode);
    }
    bot.giveDirect();
    pros::delay(5);
  }
}