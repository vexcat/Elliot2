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

void opcontrol() {
	int multiplier = 1;
	double speedMultiplier = 200;
	int lastR1 = 0;
	pros::Controller m(pros::E_CONTROLLER_MASTER);
	auto &bot = getRobot();
	while (true) {
		//Motor usage guard
		bot.takeCoast();

		//Base drive
		int y = m.get_analog(ANALOG_LEFT_Y) * (speedMultiplier / 127.0);
		int x = m.get_analog(ANALOG_LEFT_X) * (speedMultiplier / 127.0);
		bot.left.moveVelocity(multiplier * y + x);
		bot.right.moveVelocity(multiplier * y - x);

		//Scorer drive
		int score = 150 * (!!m.get_digital(DIGITAL_L2) - !!m.get_digital(DIGITAL_R2));
		bot.score.moveVelocity(score);

		//Catapult drive
		if(m.get_digital_new_press(DIGITAL_L1)) {
			bot.catapult.goToSwitch();
		} else if(!lastR1 && m.get_digital(DIGITAL_R1)){
			bot.catapult.setVelocity(100);
			lastR1 = true;
		} else if(lastR1 && !m.get_digital(DIGITAL_R1)) {
			bot.catapult.setVelocity(0);
			lastR1 = false;
		}

		//Intake drive
		int intake = -m.get_analog(ANALOG_RIGHT_Y) * (200.0 / 127.0);
		bot.intake.moveVelocity(intake);

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

		//Debugging buttons
		if(m.get_digital(DIGITAL_B)) {
			RoboPosition yeet = bot.gps.getPosition();
			printf("I think I'm at x: %f y: %f, with an orientation of %f radians.\n", yeet.x, yeet.y, yeet.o);
		}
		if(m.get_digital(DIGITAL_X)) {
			auto &gps = bot.gps;
			printf("CPR: %f, CPI: %f\n", gps.radiansToCounts(1), gps.inchToCounts(1));
		}

		//Allow for controller menus to safely take over the robot.
		//See display.cpp
		bot.giveDirect();
		pros::delay(5);
	}
}
