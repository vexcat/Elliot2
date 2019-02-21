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
	double y = dz(m.get_analog(ANALOG_LEFT_Y) * (speedMultiplier / 127.0), 32 / 200.0);
	double x = dz(m.get_analog(ANALOG_LEFT_X) * (speedMultiplier / 127.0), 32 / 200.0);

	if(y != 0 || x != 0) {
		box->base.arcade(multiplier * y, x);
	} else {
		left.moveVelocity(0);
		right.moveVelocity(0);
	}

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
		if(speedMultiplier == 1.0) {
			speedMultiplier = 0.75;
		} else {
			speedMultiplier = 1.0;
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

	//30sec warning
	if(opctrlBegin != -1L && opctrlBegin < pros::millis() - 75000)  {
		opctrlBegin = -1L;
		m.rumble(".. ..");
	}
}

void opcontrol() {
	pros::Controller m(pros::E_CONTROLLER_MASTER);
	auto &bot = getRobot();
  	bot.give();
	bot.opctrlBegin = pros::millis();
	while (true) {
		//takeCoast and giveDirect allow for controller
		//menus to safely take over the robot.
		bot.takeCoast();
		bot.drive(m);
		bot.giveDirect();
		pros::delay(5);
	}
}
