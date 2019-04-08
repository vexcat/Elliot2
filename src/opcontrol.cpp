#include "main.h"
#include "okapi/api.hpp"
#include "gps.hpp"
#include "display.hpp"
#include "elliot.hpp"
#include "autoshoot.hpp"
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
		base->arcade(multiplier * y * (autoshootActive ? 0.75 : 1.0), x * (autoshootActive ? 0.6 : 1.0));
	} else {
		left.moveVelocity(0);
		right.moveVelocity(0);
	}

	//Scorer drive
	int scorerVel = (!!m.get_digital(DIGITAL_L2) - !!m.get_digital(DIGITAL_R2));
	scorer.moveVelocity(100 * scorerVel);

	if(!autoshootActive) {
		//Puncher drive
		if(m.get_digital_new_press(DIGITAL_R1)) {
			puncher.shoot();
		}
		
		//Angler drive
		if(m.get_digital_new_press(DIGITAL_L1)) {
			puncher.toggleTarget();
		}
	}

	//Intake drive
	int intakeVel = -m.get_analog(ANALOG_RIGHT_Y) * (600.0 / 127.0);
	intake.moveVelocity(intakeVel);

	//Reverse button
	if(m.get_digital_new_press(DIGITAL_B)) {
		multiplier *= -1;
	}

	//Brake button
	if(m.get_digital_new_press(DIGITAL_X)) {
		auto old = left.getBrakeMode();
		if(old == AbstractMotor::brakeMode::hold) {
			base->setBrakeMode(AbstractMotor::brakeMode::coast);
		} else {
			base->setBrakeMode(AbstractMotor::brakeMode::hold);
		}
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
  Elliot::give();
	bot.opctrlBegin = pros::millis();
	bot.scorer.setBrakeMode(AbstractMotor::brakeMode::brake);
	while (true) {
		//takeCoast and giveDirect allow for controller
		//menus to safely take over the robot.
		Elliot::takeCoast();
		bot.drive(m);
		Elliot::giveDirect();
		pros::delay(5);
	}
}
