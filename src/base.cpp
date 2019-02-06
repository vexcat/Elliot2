#include "main.h"
#include "elliot.hpp"
#include "base.hpp"
#include "debugging.hpp"

void BaseSettings::loadState() {
    if(base) delete base;
    auto dist = loadGains("dist");
    auto angle = loadGains("angle");
    auto turn = loadGains("turn");
    base = new BaseBox(
        gps.left, gps.right, 
        dist, angle, turn,
        AbstractMotor::gearset::green, {
            gps.countsToInch(360) / PI, gps.radiansToCounts(2)
        }
    );
    base->base.startThread();
}