#include "main.h"
#include "elliot.hpp"
#include "base.hpp"
#include "debugging.hpp"

void BaseSettings::loadState() {
    if(base) delete base;
    debug(std::to_string(gps.left.getPosition()) + "\n");
    debug(std::to_string(gps.right.getPosition()) + "\n");
    debug(data.dump() + "\n");
    debug(std::to_string(gps.countsToInch(360 / PI)) + "\n");
    debug(std::to_string(gps.radiansToCounts(2)) + "\n");
    base = new BaseBox {
        okapi::ChassisControllerFactory::create(gps.left, gps.right, 
        loadGains("dist"), loadGains("angle"), loadGains("turn"),
        AbstractMotor::gearset::green, {
            gps.countsToInch(360) / PI, gps.radiansToCounts(2)
        })
    };
    base->base.startThread();
}