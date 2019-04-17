/**
 * @file base.hpp
 * 
 * This file manages base configuration.
 */
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

/**
 * BaseSettings is responsible for loading/saving configuration for the
 * Elliot2CCPID that controls the base. It loads/saves PID values for
 * turning & going forward, and for angle correction while going forward.
 * Because Elliot2CCPID can't be reconfigured, it is reconstructed whenever
 * loadState() is called. This is why it's stored as a unique_ptr in Elliot.
 * 
 * @see Elliot2CCPID
 * @see Elliot
 */
class BaseSettings {
    ///Configuration data location
    json &data;
    ///Where to store new instances of Elliot2CCPID
    std::unique_ptr<Elliot2CCPID> &base;
    ///Left MotorGroup to construct Elliot2CCPID with
    MotorGroup& left;
    ///Right MotorGroup to construct Elliot2CCPID with
    MotorGroup& right;
    ///Function to get CPI value to construct Elliot2CCPID with
    std::function<double()> cpiGetter;
    ///Function to get CPR value to construct Elliot2CCPID with
    std::function<double()> cprGetter;

    /**
     * Load a set of PID gains from the SD card, given the name of the set.
     * Loads from getState()["base"]["<name>"].
     * 
     * @param name Name of PID gain set, "dist", "angle", or "turn"
     * @return Gains associated with name given
     */
    okapi::IterativePosPIDController::Gains loadGains(const char* name) {
        return {
            data[name]["kP"].get<double>(),
            data[name]["kI"].get<double>(),
            data[name]["kD"].get<double>(),
            0
        };
    }

    public:
    /**
     * Constructs a new Elliot2CCPID object from
     * current SD card data.
     */
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
            },
            {{0, 0}, {1, 1}},
            data["voltage"].get<bool>()
        ));
        base->startThread();
    }

    /**
     * Modifies a single set of PID gains by name.
     * This will create a new Elliot2CCPID instance and change SD card data.
     * @param name     Name of PID gain set, one of "dist", "angle", or "turn"
     * @param newGains New PID values
     */
    void modGains(const char* name, const okapi::IterativePosPIDController::Gains& newGains) {
        data[name]["kP"] = newGains.kP;
        data[name]["kI"] = newGains.kI;
        data[name]["kD"] = newGains.kD;
        loadState(); // Load new values
        saveState(); // Save to SD card
    }

    /**
     * Wrapper function for loadGains("dist").
     * This will load PID gain data from getState()["base"]["dist"].
     * 
     * @return PID gains for driving straight
     */
    okapi::IterativePosPIDController::Gains getDistGains() {
        return loadGains("dist");
    }

    /**
     * Wrapper function for loadGains("angle").
     * This will load PID gain data from getState()["base"]["angle"].
     * 
     * @return PID gains for angle correction while driving straight
     */
    okapi::IterativePosPIDController::Gains getAngleGains() {
        return loadGains("angle");
    }

    /**
     * Wrapper function for loadGains("turn").
     * This will load PID gain data from getState()["base"]["turn"].
     * 
     * @return PID gains for turning in-place
     */
    okapi::IterativePosPIDController::Gains getTurnGains() {
        return loadGains("turn");
    }

    /**
     * Wrapper function for modGains("dist", ...).
     * This will modify PID gain data at getState()["base"]["dist"].
     * 
     * @param newGains New PID gains for driving straight
     */
    void setDistGains(okapi::IterativePosPIDController::Gains newGains) {
        modGains("dist", newGains);
    }

    /**
     * Wrapper function for modGains("angle", ...).
     * This will modify PID gain data at getState()["base"]["angle"].
     * 
     * @param newGains New PID gains for correcting angle while driving straight
     */
    void setAngleGains(okapi::IterativePosPIDController::Gains newGains) {
        modGains("angle", newGains);
    }

    /**
     * Wrapper function for modGains("turn", ...).
     * This will modify PID gain data at getState()["base"]["turn"].
     * 
     * @param newGains New PID gains for turning in-place
     */
    void setTurnGains(okapi::IterativePosPIDController::Gains newGains) {
        modGains("turn", newGains);
    }

    /**
     * Sets whether or not the base should use voltage for PID.
     * This will modify data at getState["base"]["voltage"].
     * 
     * @param useVoltage Whether to use voltage for PID. If false, velocity will be used.
     */
    void setVoltagePIDUsage(bool useVoltage) {
        data["voltage"] = useVoltage;
        saveState();
        loadState();
    }

    bool getVoltagePIDUsage() {
        return data["voltage"].get<bool>();
    }

    void setTrueSpeedData(const std::vector<TrueSpeedPoint>& points) {
        auto& truespeed = data["truespeed"] = json::array({});
        for(auto& pt: points) {
            truespeed.push_back(json::array({pt.x, pt.y}));
        }
        saveState();
        loadState();
    }

    std::vector<TrueSpeedPoint> getTrueSpeedData() {
        std::vector<TrueSpeedPoint> ret;
        if(data.find("truespeed") == data.end()) {
            ret = {{0, 0}, {1, 1}};
        } else {
            for(auto &point: data["truespeed"]) {
                ret.push_back({point[0], point[1]});
            }
        }
        return ret;
    }

    void deleteTrueSpeedData() {
        if(data.find("truespeed") != data.end())
            data.erase("truespeed");
        saveState();
        loadState();
    }

    /**
     * Constructs BaseSettings from base parameters, a location to write
     * new Elliot2CCPID instances to, and a JSON settings location.
     * 
     * @param ileft      Left MotorGroup of base
     * @param iright     Right MotorGroup of base
     * @param iCPIGetter Function returning the current CPI ([encoder] counts per inch) value
     * @param iCPRGetter Function returning the current CPR ([encoder] counts per radian) value
     * @param ibase      unique_ptr& where new Elliot2CCPID instances should be written to
     * @param data       JSON object where settings should be stored
     */
    BaseSettings(MotorGroup& ileft, MotorGroup& iright, 
    std::function<double()> iCPIGetter,
    std::function<double()> iCPRGetter,
    std::unique_ptr<Elliot2CCPID> & ibase, json& settingsLocation):
    base(ibase), left(ileft), right(iright), cpiGetter(iCPIGetter), cprGetter(iCPRGetter), data(settingsLocation) {
        loadState();
    }
};
