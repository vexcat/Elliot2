#include "main.h"
#include "elliot.hpp"
#include "base.hpp"

void BaseSettings::loadState() {
    auto &bot = getRobot();
    if(base) delete base;
    base = new BaseBox {
        okapi::ChassisControllerFactory::create(bot.left, bot.right, 
        loadGains("dist"), loadGains("angle"), loadGains("turn"),
        AbstractMotor::gearset::green, {
            bot.gps.countsToInch(360) / PI, bot.gps.radiansToCounts(2)
        })
    };
    base->base.startThread();
}