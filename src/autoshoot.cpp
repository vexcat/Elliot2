#include "main.h"
#include "elliot.hpp"
#include "autoshoot.hpp"

bool autoshootActive = false;
void autoshootReal(bool giveInDelay, int toggleTime, int punchTime) {
  auto &bot = getRobot();
  auto &puncher = bot.puncher;
  //Punch
  puncher.shoot();

  if(giveInDelay) {
    Elliot::giveDirect();
    pros::delay(punchTime);
    Elliot::takeCoast();
  }

  //Toggle
  puncher.toggleTarget();

  if(giveInDelay) {
    Elliot::giveDirect();
    pros::delay(toggleTime);
    Elliot::takeCoast();
  }

  //Punch
  puncher.shoot();

  if(giveInDelay) {
    Elliot::giveDirect();
    pros::delay(punchTime);
    Elliot::takeCoast();
  }

  //Toggle
  puncher.toggleTarget();

  if(giveInDelay) {
    Elliot::giveDirect();
    pros::delay(toggleTime);
    Elliot::takeCoast();
  }
}

void autoshoot(int toggleTime, int punchTime) {
  autoshootReal(false, toggleTime, punchTime);
}

void autoshootTask(void*) {
  auto &bot = getRobot();
  auto &ctrl = bot.controller;
  while(true) {
    Elliot::takeCoast();
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
    Elliot::giveDirect();
    pros::delay(5);
  }
}