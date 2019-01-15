#pragma once
#include "json.hpp"
using json = nlohmann::json;
void saveState();
json& getState();