#include "main.h"
#include "elliot.hpp"
#include "autoshoot.hpp"

bool autoshootActive = false;
void autoshootReal(bool giveInDelay, int toggleTime, int punchTime) {
  auto &bot = getRobot();
  auto &puncher = bot.puncher;
  //Punch
  puncher.shoot();
  bot.giveDirect();
  pros::delay(punchTime);
  //Toggle
  bot.takeCoast();
  puncher.toggleTarget();
  bot.giveDirect();
  pros::delay(toggleTime);
  //Punch
  bot.takeCoast();
  puncher.shoot();
  bot.giveDirect();
  pros::delay(punchTime);
  //Toggle
  bot.takeCoast();
  puncher.toggleTarget();
  pros::delay(toggleTime);
}

void autoshoot(int toggleTime, int punchTime) {
  autoshootReal(false, toggleTime, punchTime);
}

void autoshootTask(void*) {
  auto &bot = getRobot();
  auto &ctrl = bot.controller;
  while(true) {
    bot.takeCoast();
    if(ctrl.get_digital_new_press(DIGITAL_A)) {
      //Tell opcontrol autoshoot is active & wait for response
      autoshootActive = true;

      auto prevBrakeMode = bot.left.getBrakeMode();
      bot.base->setBrakeMode(AbstractMotor::brakeMode::hold);
      autoshootReal(true, 0, 800);
      bot.base->setBrakeMode(prevBrakeMode);
      
      //Restore user control
      autoshootActive = false;
    }
    bot.giveDirect();
    pros::delay(5);
  }
}