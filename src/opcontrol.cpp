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
	double y = dz(m.get_analog(ANALOG_LEFT_Y) * (1 / 127.0), 0.16);
	double x = dz(m.get_analog(ANALOG_LEFT_X) * (1 / 127.0), 0.16);

	if(y != 0 || x != 0) {
		base->arcade(multiplier * y, x);
	} else {
		left.moveVelocity(0);
		right.moveVelocity(0);
	}

	//Scorer drive
	int scorerVel = (!!m.get_digital(DIGITAL_L2) - !!m.get_digital(DIGITAL_R2));
	scorer.moveVelocity(100 * scorerVel);

	//Puncher drive
	if(m.get_digital_new_press(DIGITAL_R1)) {
		puncher.shoot();
	}
	
	//Angler drive
	if(m.get_digital_new_press(DIGITAL_L1)) {
		puncher.toggleTarget();
	}

	//Intake drive
	int intakeVel = -m.get_analog(ANALOG_RIGHT_Y) * (600.0 / 127.0);
	intake.moveVelocity(intakeVel);

	//Reverse button
	if(m.get_digital_new_press(DIGITAL_A)) {
		multiplier *= -1;
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
	bot.scorer.setBrakeMode(AbstractMotor::brakeMode::brake);
	while (true) {
		//takeCoast and giveDirect allow for controller
		//menus to safely take over the robot.
		bot.takeCoast();
		bot.drive(m);
		bot.giveDirect();
		pros::delay(5);
	}
}
