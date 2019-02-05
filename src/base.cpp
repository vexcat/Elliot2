#include "main.h"
#include "elliot.hpp"
#include "base.hpp"
#include "debugging.hpp"

void BaseSettings::loadState() {
    if(base) delete base;
    base = new BaseBox {
        okapi::ChassisControllerFactory::create(gps.left, gps.right, 
        loadGains("dist"), loadGains("angle"), loadGains("turn"),
        AbstractMotor::gearset::green, {
            gps.countsToInch(360) / PI, gps.radiansToCounts(2)
        })
    };
    base->base.startThread();
}