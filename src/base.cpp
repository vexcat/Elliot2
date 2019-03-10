#include "main.h"
#include "elliot.hpp"
#include "base.hpp"
#include "debugging.hpp"
using namespace okapi;

void BaseSettings::loadState() {
    if(base) delete base;
    auto dist = loadGains("dist");
    auto angle = loadGains("angle");
    auto turn = loadGains("turn");
    base = new BaseBox(
        TimeUtil(
            Supplier<std::unique_ptr<AbstractTimer>>([]() { return std::make_unique<Timer>(); }),
            Supplier<std::unique_ptr<AbstractRate >>([]() { return std::make_unique<Rate >(); }),
            Supplier<std::unique_ptr<SettledUtil  >>([]() { return std::make_unique<SettledUtil>(std::make_unique<Timer>(), 0.0, 10.0, 325_ms); })
        ),
        std::make_shared<SkidSteerModel>(std::make_shared<MotorGroup>(std::initializer_list<Motor>{3, 4}), std::make_shared<MotorGroup>(std::initializer_list<Motor>{-2, -1}), gps.left.getEncoder(), gps.right.getEncoder(), 200, 12000),
        std::make_unique<IterativePosPIDController>(dist, TimeUtilFactory::create()),
        std::make_unique<IterativePosPIDController>(angle, TimeUtilFactory::create()),
        std::make_unique<IterativePosPIDController>(turn, TimeUtilFactory::create()),
        AbstractMotor::gearset::green, {
            (gps.countsToInch(360) / PI) * okapi::inch, gps.countsToInch(gps.radiansToCounts(2)) * okapi::inch
        }
    );
    base->base.startThread();
}