#pragma once
#include "main.h"
#include "json.hpp"
#include "state.hpp"
#include "okapi/api.hpp"
#include "elliot.hpp"
#include "debugging.hpp"
#include "ccpid_mod.hpp"
#include <utility>
#include <functional>
using json = nlohmann::json;

class BaseSettings {
    json &data;
    std::unique_ptr<Elliot2CCPID> &base;
    MotorGroup& left;
    MotorGroup& right;
    std::function<double()> cpiGetter;
    std::function<double()> cprGetter;
    okapi::IterativePosPIDController::Gains loadGains(const char* name) {
        return {
            data[name]["kP"].get<double>(),
            data[name]["kI"].get<double>(),
            data[name]["kD"].get<double>(),
            0
        };
    }
    public:
    void loadState() {
        auto dist = loadGains("dist");
        auto angle = loadGains("angle");
        auto turn = loadGains("turn");
        base = std::unique_ptr<Elliot2CCPID>( new Elliot2CCPID(
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

    void modGains(const char* name, const okapi::IterativePosPIDController::Gains& newGains) {
        data[name]["kP"] = newGains.kP;
        data[name]["kI"] = newGains.kI;
        data[name]["kD"] = newGains.kD;
        loadState();
        saveState();
    }
    okapi::IterativePosPIDController::Gains getDistGains() {
        return loadGains("dist");
    }
    okapi::IterativePosPIDController::Gains getAngleGains() {
        return loadGains("angle");
    }
    okapi::IterativePosPIDController::Gains getTurnGains() {
        return loadGains("turn");
    }
    void setDistGains(okapi::IterativePosPIDController::Gains newGains) {
        modGains("dist", newGains);
    }
    void setAngleGains(okapi::IterativePosPIDController::Gains newGains) {
        modGains("angle", newGains);
    }
    void setTurnGains(okapi::IterativePosPIDController::Gains newGains) {
        modGains("turn", newGains);
    }
    BaseSettings(MotorGroup& ileft, MotorGroup& iright, 
    std::function<double()> iCPIGetter,
    std::function<double()> iCPRGetter,
    std::unique_ptr<Elliot2CCPID> & ibase, json& settingsLocation):
    base(ibase), left(ileft), right(iright), cpiGetter(iCPIGetter), cprGetter(iCPRGetter), data(settingsLocation) {
        loadState();
    }
};
