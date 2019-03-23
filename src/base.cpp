#include "main.h"
#include "elliot.hpp"
#include "base.hpp"
#include "debugging.hpp"
using namespace okapi;

void BaseSettings::loadState() {
    auto dist = loadGains("dist");
    auto angle = loadGains("angle");
    auto turn = loadGains("turn");
    base = std::unique_ptr<ChassisControllerPID>( new ChassisControllerPID(
        TimeUtil(
            Supplier<std::unique_ptr<AbstractTimer>>([]() { return std::make_unique<Timer>(); }),
            Supplier<std::unique_ptr<AbstractRate >>([]() { return std::make_unique<Rate >(); }),
            Supplier<std::unique_ptr<SettledUtil  >>([]() { return std::make_unique<SettledUtil>(std::make_unique<Timer>(), 0.0, 10.0, 325_ms); })
        ),
        std::make_shared<SkidSteerModel>(std::make_shared<MotorGroup>(left), std::make_shared<MotorGroup>(right), left.getEncoder(), right.getEncoder(), 200, 12000),
        std::make_unique<IterativePosPIDController>(dist , TimeUtilFactory::create()),
        std::make_unique<IterativePosPIDController>(angle, TimeUtilFactory::create()),
        std::make_unique<IterativePosPIDController>(turn , TimeUtilFactory::create()),
        AbstractMotor::gearset::green, {
            (360 / (PI * cpiGetter())) * okapi::inch, ((cprGetter() * 2) / cpiGetter()) * okapi::inch
        }
    ));
    base->startThread();
}
