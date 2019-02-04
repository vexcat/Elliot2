#include "main.h"
#include "okapi/api.hpp"
#include "gps.hpp"
#include "display.hpp"
#include "elliot.hpp"
#include <deque>
using namespace std;
using namespace okapi;

double pos0(double a) {
	if(a == 0) return 0;
	return a;
}

double dz(double a, double zone) {
	if(abs(a) < zone) return 0;
	return a;
}

void Elliot::drive(pros::Controller& m) {
	//Base drive
	int y = m.get_analog(ANALOG_LEFT_Y) * (speedMultiplier / 127.0);
	int x = m.get_analog(ANALOG_LEFT_X) * (speedMultiplier / 127.0);

	box->base.arcade(multiplier * y, x, 32);

	//Scorer drive
	int scoreVel = 150 * (!!m.get_digital(DIGITAL_L2) - !!m.get_digital(DIGITAL_R2));
	score.moveVelocity(scoreVel);

	//Catapult drive
	if(m.get_digital_new_press(DIGITAL_L1)) {
		catapult.goToSwitch();
	} else if(!lastR1 && m.get_digital(DIGITAL_R1)){
		catapult.setVelocity(100);
		lastR1 = true;
	} else if(lastR1 && !m.get_digital(DIGITAL_R1)) {
		catapult.setVelocity(0);
		lastR1 = false;
	}

	//Intake drive
	int intakeVel = -m.get_analog(ANALOG_RIGHT_Y) * (200.0 / 127.0);
	intake.moveVelocity(intakeVel);

	//Reverse button
	if(m.get_digital_new_press(DIGITAL_A)) {
		multiplier *= -1;
	}

	//Slow button
	if(m.get_digital_new_press(DIGITAL_DOWN)) {
		if(speedMultiplier == 200) {
			speedMultiplier = 150;
		} else {
			speedMultiplier = 200;
		}
	}

	//Brake buttons
	if(m.get_digital_new_press(DIGITAL_X)) {
		left .setBrakeMode(AbstractMotor::brakeMode::hold);
		right.setBrakeMode(AbstractMotor::brakeMode::hold);
	}
	if(m.get_digital_new_press(DIGITAL_Y)) {
		left .setBrakeMode(AbstractMotor::brakeMode::coast);
		right.setBrakeMode(AbstractMotor::brakeMode::coast);
	}

	//Allow for controller menus to safely take over the robot.
	//See display.cpp
}

void opcontrol() {
	pros::Controller m(pros::E_CONTROLLER_MASTER);
	auto &bot = getRobot();
  	bot.give();
	while (true) {
		//takeCoast and giveDirect allow for controller
		//menus to safely take over the robot.
		bot.takeCoast();
		bot.drive(m);
		bot.giveDirect();
		pros::delay(5);
	}
}
