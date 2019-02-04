#include "main.h"
#include "json.hpp"
#include "state.hpp"
using json = nlohmann::json;

class CameraSettings {
    json &data;
    pros::Vision &vision;
    public:
    void loadState() {
        vision.set_auto_white_balance(data["auto"].get<bool>());
        vision.set_white_balance(data["white"].get<int>());
        vision.set_exposure(data["exposure"].get<int>());
    }
    bool getAutoWhiteBalance() {
        return data["auto"].get<bool>();
    }
    int getWhiteBalance() {
        return vision.get_white_balance();
    }
    int getExposure() {
        return vision.get_exposure();
    }
    void setAutoWhiteBalance(bool autoWB) {
        vision.set_auto_white_balance(autoWB);
        data["auto"] = autoWB;
        saveState();
    }
    void setWhiteBalance(int newBalance) {
        vision.set_white_balance(newBalance);
        data["white"] = newBalance;
        saveState();
    }
    void setExposure(int vexposure) {
        vision.set_exposure(vexposure);
        data["exposure"] = vexposure;
        saveState();
    }
    CameraSettings(pros::Vision& sensor, json& settingsLocation): vision(sensor), data(settingsLocation) {
        loadState();
    }
};