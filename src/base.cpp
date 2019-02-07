#include "main.h"
#include "elliot.hpp"
#include "base.hpp"
#include "debugging.hpp"

void BaseSettings::loadState() {
    if(base) delete base;
    auto dist = loadGains("dist");
    auto angle = loadGains("angle");
    auto turn = loadGains("turn");
    //Negate D gains to workaround OkapiLib/OkapiLib#332
    dist .kD *= -1;
    angle.kD *= -1;
    turn .kD *= -1;
    base = new BaseBox(
        gps.left, gps.right, 
        dist, angle, turn,
        AbstractMotor::gearset::green, {
            (gps.countsToInch(360) / PI) * okapi::inch, gps.countsToInch(gps.radiansToCounts(2)) * okapi::inch
        }
    );
    base->base.startThread();
}