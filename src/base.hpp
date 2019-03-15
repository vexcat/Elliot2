#pragma once
#include "main.h"
#include "json.hpp"
#include "state.hpp"
#include "okapi/api.hpp"
#include "elliot.hpp"
#include "debugging.hpp"
#include <utility>
#include <functional>
using json = nlohmann::json;
struct BaseBox;
class BaseSettings {
    json &data;
    BaseBox* &base;
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
    void loadState();

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
    BaseBox* & ibase, json& settingsLocation): base(ibase), left(ileft), right(iright), cpiGetter(iCPIGetter), cprGetter(iCPRGetter), data(settingsLocation) {
        loadState();
    }
};
