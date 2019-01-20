#include "main.h"
#include "display/lvgl.h"
#include "gps.hpp"
#include "pros/apix.h"
#include "okapi/api.hpp"
#include "display.hpp"
#include "elliot.hpp"
#include "autonomous.hpp"
#include "state.hpp"
#include "editors.hpp"
#include "catOS.hpp"
#include <functional>
#include "debugging.hpp"
using namespace okapi;

//Display code! This file contains the code for:
//  - The auton selector
//  - The GPS viewer
//  - The GPS calibrator
//  - The auton planner
//  - And a bunch of random stuff that'll probably get refactored out.

//------------------------------------------------------------------------------------
//  V5 Brain Display using LittleVGL
//  --------------------------------
//  The V5 screen shows an auton selector, a gps viewer, or just our logo, depending
//  on the competition mode. The code for the screen integrates with gps.cpp for the
//  GPS, autonomous.cpp for changing the color mode, and the auton planner for newly
//  created autonomi.
//------------------------------------------------------------------------------------

//---------------------------------------
//  GPS Debug Viewer
//---------------------------------------
lv_point_t origPoints[] = { {0, 0}, {30, 0}, {24, -6}, {30, 0}, {24, 6}, {30, 0} };
lv_point_t newPoints[6];
lv_obj_t* arrow;
lv_obj_t* orientationLabel;

//Rotates the robot position arrow according to the robot's rotation value.
void rotateIt(lv_point_t* points, lv_point_t* dPoints, int count, double rotation) {
  for(int i = 0; i < count; i++) {
    dPoints->x = points->x * cos(rotation) - points->y * sin(rotation);
    dPoints->y = points->y * cos(rotation) + points->x * sin(rotation);
    points++;
    dPoints++;
  }
}                    

//Flips points upside down and makes all points positive.
void fix(lv_point_t* points, int count) {
  for(int i = 0; i < count; i++) {
    points->y = 80-(points->y);
    points->x = (points->x) + 80;
    points++;
  }
}

//Updates robot arrow on screen.
void updateRobot() {
  auto &gps = getRobot().gps;
  RoboPosition yeet = gps.getPosition();
  lv_obj_set_pos(arrow, (gps.countsToInch(yeet.x) * 180.0 / 144.0)-80, 180-(gps.countsToInch(yeet.y) * 180.0 / 144.0)-80);
  rotateIt(origPoints, newPoints, 6, yeet.o);
  fix(newPoints, 6);
  lv_line_set_points(arrow, newPoints, 6);
  lv_obj_invalidate(arrow);
  static char orientationBuffer[1024];
  snprintf(orientationBuffer, sizeof(orientationBuffer), "x: %f\ny: %f\no: %f", gps.countsToInch(yeet.x), gps.countsToInch(yeet.y), yeet.o);
  lv_label_set_text(orientationLabel, orientationBuffer);
}

//Initializes the on-screen field.
lv_obj_t* debugField;
void oyes() {
  debugField = lv_obj_create(lv_scr_act(), NULL);
  lv_obj_set_pos(debugField, 20, 20);
  lv_obj_set_size(debugField, 180, 180);
  static lv_style_t gray;
  static lv_style_t red;
  static lv_style_t blue;
  lv_style_copy(&gray, &lv_style_plain);
  gray.body.main_color = LV_COLOR_HEX(0x999999);
  gray.body.grad_color = LV_COLOR_HEX(0x999999);
  lv_style_copy(&red, &gray);
  red .body.main_color = LV_COLOR_HEX(0xFF0000);
  red .body.grad_color = LV_COLOR_HEX(0xFF0000);
  lv_style_copy(&blue, &gray);
  blue.body.main_color = LV_COLOR_HEX(0x0000FF);
  blue.body.grad_color = LV_COLOR_HEX(0x0000FF);
  static lv_style_t* field[] = {
    &gray, &gray, &gray, &gray, &gray, &gray, 
    &gray, &gray, &gray, &gray, &gray, &gray, 
    &red , &gray, &gray, &gray, &gray, &blue, 
    &gray, &gray, &gray, &gray, &gray, &gray, 
    &red , &gray, &gray, &gray, &gray, &blue, 
    &gray, &gray, &gray, &gray, &gray, &gray, 
  };
  for(int y = 0; y < 6; y++) {
    for(int x = 0; x < 6; x++) {
      lv_obj_t* tile = lv_obj_create(debugField, NULL);
      lv_obj_set_pos(tile, x * 30, y * 30);
      lv_obj_set_size(tile, 30, 30);
      lv_obj_set_style(tile, field[y*6 + x]);
    }
  }
  orientationLabel = lv_label_create(debugField, NULL);
  lv_label_set_text(orientationLabel, "Loading...");
  arrow = lv_line_create(debugField, NULL);
  lv_obj_set_pos(arrow, 12, 108);
  lv_obj_set_style(arrow, &lv_style_plain);
  updateRobot();
  lv_obj_set_hidden(debugField, true);
}

//---------------------------------------
//  Team Logo
//---------------------------------------
extern "C" const lv_img_t cougarImage;
lv_obj_t* image;
void putImage() {
  image = lv_img_create(lv_scr_act(), NULL);
  lv_img_set_src(image, &cougarImage);
}

//---------------------------------------
//  Auton Selector
//---------------------------------------
lv_obj_t* autoSelectorObj;
lv_obj_t* buttonTemplate;
std::string currentlySelected = "#No Auton";
std::string getSelectedAuton() {
  return currentlySelected;
}
std::vector<std::string> autonNames;
std::vector<std::pair<lv_obj_t*, lv_obj_t*>> selectors;
//Adds new autons to the screen
void addAuton(std::string byName) {
  if(std::find(autonNames.begin(), autonNames.end(), byName) != autonNames.end()) {
    throw "ur data is bad and u should feel bad";
  } else {
    autonNames.push_back(byName);
    if(selectors.size() < 8) {
      int i = selectors.size();
      lv_obj_t* button = lv_btn_create(autoSelectorObj, buttonTemplate);
      lv_obj_set_hidden(button, false);
      int bx = i%4;
      int by = i/4;
      lv_obj_set_pos(button, bx * 95 + 90, by * 115 + 10);
      lv_obj_t* label = lv_label_create(button, NULL);
      lv_label_set_text(label, byName.c_str());
      lv_style_t* stylish = lv_obj_get_style(label);
      stylish->text.color = LV_COLOR_HEX(0x000000);
      lv_btn_set_action(button, LV_BTN_ACTION_CLICK, [](lv_obj_t* obj) -> lv_res_t {
        int j = 0;
        for(auto &pair: selectors) {
          if(pair.first == obj) {
            lv_btn_set_state(pair.first, LV_BTN_STATE_TGL_REL);
            currentlySelected = autonNames[j];
          } else {
            lv_btn_set_state(pair.first, LV_BTN_STATE_REL);
          }
          j++;
        }
      });
      selectors.push_back({button, label});
    }
  }
}
//Removes autons from the screen
void removeAuton(std::string byName) {
  auto loc = std::find(autonNames.begin(), autonNames.end(), byName);
  if(loc == autonNames.end()) {
    throw "ur data is bad and u should feel bad";
  } else {
    auto loc2 = selectors.begin() + (loc - autonNames.begin());
    lv_obj_del(loc2->first);
    selectors.erase(loc2);
    autonNames.erase(loc);
    for(int i = 0; i < selectors.size(); i++) {
      int bx = i%4;
      int by = i/4;
      lv_obj_set_pos(selectors[i].first, bx * 95 + 90, by * 115 + 10);
    }
  }
}

//Initializes the auton selector
void autoSelector() {
  setBlue(false);
  autoSelectorObj = lv_obj_create(lv_scr_act(), NULL);
  lv_obj_set_size(autoSelectorObj, 480, 240);
  lv_obj_set_style(autoSelectorObj, &lv_style_plain);
  buttonTemplate = lv_btn_create(autoSelectorObj, NULL);
  lv_obj_set_size(buttonTemplate, 85, 105);
  lv_obj_set_hidden(buttonTemplate, true);
  static lv_obj_t* colorButton = lv_btn_create(autoSelectorObj, NULL);
  lv_btn_set_toggle(colorButton, true);
  static lv_style_t redStyle;
  static lv_style_t blueStyle;
  static lv_style_t redDullStyle;
  static lv_style_t blueDullStyle;
  lv_style_copy(& redStyle, &lv_style_btn_rel);
  lv_style_copy(&blueStyle, &lv_style_btn_tgl_rel);
  lv_style_copy(& redDullStyle, &lv_style_btn_pr);
  lv_style_copy(&blueDullStyle, &lv_style_btn_tgl_pr);
    redStyle.body.grad_color = LV_COLOR_HEX(0xFF0000);
    redStyle.body.main_color = LV_COLOR_HEX(0xFF0000);
  blueStyle.body.grad_color = LV_COLOR_HEX(0x0000FF);
  blueStyle.body.main_color = LV_COLOR_HEX(0x0000FF);
    redDullStyle.body.grad_color = LV_COLOR_HEX(0x7F0000);
    redDullStyle.body.main_color = LV_COLOR_HEX(0x7F0000);
  blueDullStyle.body.grad_color = LV_COLOR_HEX(0x00007F);
  blueDullStyle.body.main_color = LV_COLOR_HEX(0x00007F);
  lv_btn_set_style(colorButton, LV_BTN_STYLE_REL, &redStyle);
  lv_btn_set_style(colorButton, LV_BTN_STYLE_PR, &redDullStyle);
  lv_btn_set_style(colorButton, LV_BTN_STYLE_TGL_REL, &blueStyle);
  lv_btn_set_style(colorButton, LV_BTN_STYLE_TGL_PR, &blueDullStyle);
  lv_obj_set_pos(colorButton, 10, 10);
  lv_obj_set_size(colorButton, 70, 220);
  static lv_obj_t* colorLabel = NULL;
  colorLabel = lv_label_create(colorButton, NULL);
  lv_label_set_text(colorLabel, "RED");
  lv_btn_set_action(colorButton, LV_BTN_ACTION_CLICK, [](lv_obj_t* obj) -> lv_res_t {
    if(lv_btn_get_state(colorButton) == LV_BTN_STATE_REL || lv_btn_get_state(colorButton) == LV_BTN_STATE_PR) {
      setBlue(false);
      lv_label_set_text(colorLabel, "RED");
    } else {
      setBlue(true);
      lv_label_set_text(colorLabel, "BLUE");
    }
  });
  addAuton("#No Auton");
  //Load existing autons
  auto &stateAutons = getState()["autons"];
  for(auto &elem: stateAutons.items()) {
    addAuton(elem.key());
  }
  lv_btn_set_state(selectors[0].first, LV_BTN_STATE_TGL_REL);
}

//-------------------------------------
//  UI Executor
//-------------------------------------
int lastCompStatus = -1;
bool lastMenuWasEntered = false;
bool menuWasEntered = false;
//Responsible for running all V5 Brain tasks.
void uiExecutor(void*) {
  uint32_t startTime = pros::millis();
  putImage();
  autoSelector();
  oyes();
  while(true) {
    pros::c::task_delay_until(&startTime, 50);
    updateRobot();
    int currentStatus = pros::competition::get_status();
    int curMenuEnter = menuWasEntered;
    if(currentStatus != lastCompStatus || curMenuEnter != lastMenuWasEntered) {
      lastCompStatus = currentStatus;
      lastMenuWasEntered = curMenuEnter;
      if(currentStatus & COMPETITION_CONNECTED) {
        if(currentStatus & COMPETITION_DISABLED) {
          lv_obj_set_hidden(autoSelectorObj, false);
          lv_obj_set_hidden(debugField, true);
        } else {
          lv_obj_set_hidden(autoSelectorObj, true);
          lv_obj_set_hidden(debugField, true);
        }
      } else {
        if(currentStatus & COMPETITION_AUTONOMOUS || curMenuEnter) {
          lv_obj_set_hidden(autoSelectorObj, true);
          lv_obj_set_hidden(debugField, false);
        } else {
          lv_obj_set_hidden(autoSelectorObj, false);
          lv_obj_set_hidden(debugField, true);
        }
      }
    }
  }
}

//-----------------------------------------------------------------------------------
//  V5 Controller Menus
//  -------------------
//  The V5 Controller is responsible for the display of many debug menus. It runs
//  catOS, a collection of classes that aid in the display of debug information.
//  The controller has an autonomous planner, a very large feature that allows us to
//  create and edit autonomouses from within the controller, and save them to the 
//  microSD card. This system removes a lot of overhead in the process for developing
//  auton, leaving our team with more time to practice and build. For more details on
//  what JSON-based autons can do, take a look at autonomous.cpp and gps.cpp.
//-----------------------------------------------------------------------------------

//Allows you to drive before selecting a menu option.
int checkTemporaryExit() {
  auto &ctrl = getRobot().controller;
  if(ctrl.get_digital_new_press(DIGITAL_Y)) {
    line_set(0, "Now driving,");
    line_set(1, "Press B");
    line_set(2, "to exit.");
    auto &bot = getRobot();
    while(true) {
      bot.drive(ctrl);
      if(ctrl.get_digital(DIGITAL_B)) {
        while(ctrl.get_digital(DIGITAL_B)) pros::delay(25);
        break;
      }
      pros::delay(5);
    }
    return true;
  }
  return false;
}

//Can edit a motion object, given its keys.
class MotionEditor: public ControllerMenu {
  std::vector<std::pair<std::string, std::string>> options;
  public:
  MotionEditor(
    json& auton,
    int idx, 
    std::initializer_list<std::tuple<std::string, int, std::string>> options,
    std::initializer_list<MenuEntry> conveniences = {}
  ) {
    auto &object = auton[idx];
    for(auto &[key, fix, option]: options) {
      list.push_back({option, [&object, key, fix](){
        object[key] = editNumber(object[key].get<double>(), fix);
      }});
    }
    list.insert(list.end(), conveniences);
    list.insert(list.end(), {
      {"Run this", [&]() {
        auto &bot = getRobot();
        auto old = bot.left.getBrakeMode();
        bot. left.setBrakeMode(AbstractMotor::brakeMode::hold);
        bot.right.setBrakeMode(AbstractMotor::brakeMode::hold);
        //Track with just current position
        auto tracking = bot.gps.getPosition();
        runMotion(object, tracking, getBlue());
        bot. left.setBrakeMode(old);
        bot.right.setBrakeMode(old);
      }},
      {"Run to here", [&auton, idx]() {
        auto &bot = getRobot();
        bot.left .setBrakeMode(AbstractMotor::brakeMode::hold);
        bot.right.setBrakeMode(AbstractMotor::brakeMode::hold);
        auto loc = auton.begin();
        //Process the first entry, an Origin.
        RoboPosition tracking = {
          bot.gps.inchToCounts((*loc)["x"].get<double>()),
          bot.gps.inchToCounts((*loc)["y"].get<double>()),
          (*loc)["o"].get<double>()
        };
        tracking.x = getBlue() ? (bot.gps.countsToInch(144) - tracking.x) : tracking.x;
        tracking.o = getBlue() ? PI - tracking.o : tracking.o;
        bot.gps.setPosition(tracking);
        loc++;
        for(; loc != auton.begin() + idx + 1; loc++) {
          puts(((*loc)["type"].get<std::string>()).c_str());
          try {
            runMotion(*loc, tracking, getBlue());
          } catch(...) {
            debug("An exception was thrown.\n");
          }
        }
      }}
    });
  }
};

//Can edit an autonomous, given its motion list.
class MotionList: public CRUDMenu {
  json &motionData;
  public:
  MotionList(std::string autonName): CRUDMenu(), motionData(getState()["autons"][autonName]) {
    //Add inserters here
    auto &bot = getRobot();
    addInserter("Position", [&](int index) -> std::string {
      RoboPosition pos = bot.gps.getPosition();
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "position"}, {"x", 0}, {"y", 0}, {"t", 0.2}, {"v", 1.0}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Rotation", [&](int index) -> std::string {
      RoboPosition pos = bot.gps.getPosition();
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "rotateTo"}, {"o", 0}, {"t", 0.2}, {"v", 1.0}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("SLine", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "sline"}, {"d", 12.0}, {"t", 0.2}, {"v", 1.0}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Scorer", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "scorer"}, {"v", 1.0}, {"t", 0.2}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Catapult", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "catapult"}, {"v", 1.0}, {"t", 0.2}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Intake", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "intake"}, {"v", 1.0}, {"t", 0.2}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Shoot", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "shoot"}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Delay", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "delay"}, {"t", 0.2}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("BHold", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({ {"type", "hold"} }));
      return nameFor(motionData[index]);
    });
    addInserter("BCoast", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({ {"type", "coast"} }));
      return nameFor(motionData[index]);
    });
    addInserter("BShort", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({ {"type", "short"} }));
      return nameFor(motionData[index]);
    });
    //Add existing items
    for(auto &motion: motionData) {
      addItem(nameFor(motion));
    }
  }
  virtual void attemptDelete(int idx, const std::string& oldName) {
    motionData.erase(motionData.begin() + idx);
  }
  virtual void attemptMove(int idx, int newIdx, const std::string& oldName) {
    std::swap(motionData[idx], motionData[newIdx]);
  }
  virtual void attemptRename(int idx, std::string newName, const std::string& oldName) {
    motionData[idx]["name"] = newName;
  }
  virtual std::string attemptDuplicate(int idx, int newIdx, const std::string& oldName) {
    motionData.insert(motionData.begin() + newIdx, motionData[idx]);
    return nameFor(motionData[newIdx]);
  }
  void handleSelect(int idx, const std::string& name) override {
    std::string type = motionData[idx]["type"];
    auto &motionSelected = motionData[idx];
    if(type == "origin") {
      MotionEditor(motionData, idx, {
        {"x", 2, "Set X"},
        {"y", 2, "Set Y"}
      }, {
        {"Move Here", [&motionSelected](){
          json copy = motionSelected;
          moveToSetpoint({
            getRobot().gps.inchToCounts(copy["x"].get<double>()),
            getRobot().gps.inchToCounts(copy["y"].get<double>()),
            0
          }, getRobot().gps, 1.0, false);
          pros::delay(500);
          copy["type"] = "rotateTo";
          copy["o"] = copy["o"].get<double>();
          RoboPosition tracking;
          runMotion(copy, tracking, getBlue());
        }},
        //Shows up as "*Set Orientatio" due to character limit
        {"Set Orientation", [&motionSelected]() {
          motionSelected["o"] = (PI / 180.0) * editNumber((180.0 / PI) * motionSelected["o"].get<double>(), 2);
        }}
      })();
    } else if(type == "position") {
      MotionEditor(motionData, idx, {
        {"x", 2, "Set X"},
        {"y", 2, "Set Y"},
        {"v", 2, "Set velocity"},
        {"t", 3, "Set timing"}
      })();
    } else if(type == "rotateTo") {
      //Use a manual convenience for the radian/degree conversion
      MotionEditor(motionData, idx, {
        {"v", 2, "Set velocity"}
      }, {
        {"Set Orientation", [&motionSelected]() {
          motionSelected["o"] = (PI / 180.0) * editNumber((180.0 / PI) * motionSelected["o"].get<double>(), 2);
        }}
      })();
    } else if(type == "sline") {
      MotionEditor(motionData, idx, {
        {"d", 2, "Set distance"},
        {"t", 3, "Set timing"},
        {"v", 2, "Set velocity"}
      })();
    } else if(type == "scorer" || type == "catapult" || type == "intake") {
      MotionEditor(motionData, idx, {
        {"v", 2, "Set velocity"},
        {"t", 3, "Set timing"}
      })();
    } else if(type == "delay") {
      motionData[idx]["t"] = editNumber(motionData[idx]["t"].get<double>(), 3);
    } else if(type == "shoot") {
      MotionEditor(motionData, idx, {})();
    } else if(type == "rotation") {
      motionData[idx]["type"] = "rotateTo";
    }
    updateItem(idx, nameFor(motionData[idx]));
    finalizeData();
  }
  void finalizeData() override {
    saveState();
  }
  private:
  std::string nameFor(json& motion) {
    auto loc = motion.find("name");
    if(loc != motion.end()) return loc->get<std::string>();
    auto type = motion["type"];
    if(type == "delay") {
      char buf[16];
      snprintf(buf, 16, "%5.3fs delay", motion["t"].get<double>());
      return buf;
    }
    return motion["type"].get<std::string>();
  }
};

//Can create/remove autons
class AutonList: public CRUDMenu {
  json &autonData;
  public:
  AutonList(): CRUDMenu(), autonData(getState()["autons"]) {
    addInserter("auton", [&](int index) -> std::string {
      if(autonData.find("unnamed") != autonData.end()) {
        throw "ur data is bad and u should feel bad";
      }
      autonData["unnamed"] = json::array({
        {
          {"type", "origin"},
          {"name", "ORIGIN"},
          {"x", 0.0},
          {"y", 0.0},
          {"o", 0.0}
        }
      });
      addAuton("unnamed");
      return "unnamed";
    });
    //Add existing items
    for(auto &[k, v]: autonData.items()) {
      addItem(k);
    }
  }

  void attemptDelete(int idx, const std::string& oldName) override {
    auto location = autonData.find(oldName);
    if(location == autonData.end()) {
      throw "ur data is bad and u should feel bad";
    } else {
      autonData.erase(location);
      removeAuton(oldName);
    }
  }

  //attemptMove is skipped because it's not useful on an object where string keys matter.

  void attemptRename(int idx, std::string newName, const std::string& oldName) override {
    auto newLocation = autonData.find(newName);
    auto oldLocation = autonData.find(oldName);
    if(newLocation != autonData.end()) {
      throw "ur data is bad and u should feel bad";
    } else {
      autonData[newName] = autonData[oldName];
      autonData.erase(oldLocation);
      removeAuton(oldName);
      addAuton(newName);
    }
  }

  std::string attemptDuplicate(int idx, int newIdx, const std::string& oldName) override {
    auto location = autonData.find("unnamed");
    if(location != autonData.end()) {
      throw "ur data is bad and u should feel bad";
    } else {
      autonData["unnamed"] = autonData[oldName];
      addAuton("unnamed");
    }
    return "unnamed";
  }

  void handleSelect(int idx, const std::string& name) override {
    MotionList auton(name);
    auton();
  };

  void finalizeData() override {
    saveState();
  }
};

//Can test a single motor
class MotorTest: public ControllerTask {
  pros::Motor m;
  int pn;
  public:
  MotorTest(int n): m(n), pn(n) {};
  void render() override {
    line_set(0, "Test motor# " + std::to_string(pn));
    line_set(1, "Press <-/->");
    line_set(2, "Or use left joy");
  }
  int checkController() override {
    auto &ctrl = getRobot().controller;
    if(ctrl.get_digital(DIGITAL_LEFT)) {
      m.move_velocity(-200);
    } else if(ctrl.get_digital(DIGITAL_RIGHT)) {
      m.move_velocity(200);
    } else {
      m.move_velocity(ctrl.get_analog(ANALOG_LEFT_Y) * 200 / 127.0);
    }
    if(ctrl.get_digital(DIGITAL_B)) {
      return GO_UP;
    } else {
      return NO_CHANGE;
    }
  };
};

//Can list all motors you can test
class MotorList: public ControllerMenu {
  public:
  MotorList() {
    for(int i = 1; i <= 21; i++) {
      list.push_back({std::to_string(i), [=]() {
        taskOption<MotorTest>(i);
      }});
    }
  }
};

class GPSCalibrator: public ControllerTask {
  double initial_left;
  double initial_right;
  Elliot& robot;
  int state = 0;
  public:
  GPSCalibrator(): robot(getRobot()) {
    initial_left  = robot.left .getPosition();
    initial_right = robot.right.getPosition();
  }

  void render() override {
    if(state == 0) {
      line_set(0, "Move robot 96in");
      line_set(1, "<-  ->/LeftJoyY");
      line_set(2, "then press A.");
    } else if(state == 1) {
      line_set(0, "Reset position");
      line_set(1, "of robot, then");
      line_set(2, "press A.");
    } else if(state == 2) {
      line_set(0, "Turn 32x CCW");
      line_set(1, "<- ->/LeftJoyX");
      line_set(2, "then press A.");
    } else if(state == 3) {
      line_set(0, "All done!");
      line_set(1, "A to exit.");
      line_set(2, "");
    }
  }

  int checkController() override {
    auto &ctrl = getRobot().controller;
    if(ctrl.get_digital_new_press(DIGITAL_A)) {
      if(state == 0) {
        double lastAvg = (initial_left + initial_right) / 2;
        double curAvg = (robot.left .getPosition() + robot.right.getPosition()) / 2;
        double measuredTPI = (curAvg - lastAvg) / 96.0;
        robot.gps.setCPI(measuredTPI);
      } else if(state == 1) {
        initial_left  = robot.left .getPosition();
        initial_right = robot.right.getPosition();
      } else if(state == 2) {
        double deltaL = robot.left .getPosition() - initial_left;
        double deltaR = robot.right.getPosition() - initial_right;
        double measuredCPR = (deltaR - deltaL) / (128 * PI);
        robot.gps.setCPR(measuredCPR);
      }
      state++;
      if(state == 4) return GO_UP;
      render();
    }
    if(state == 0) {
      double y = (!!ctrl.get_digital(DIGITAL_RIGHT) - !!ctrl.get_digital(DIGITAL_LEFT));
      if(!y) y = ctrl.get_analog(ANALOG_LEFT_Y) / 127.0;
      robot. left.moveVelocity(y * (int)robot. left.getGearing());
      robot.right.moveVelocity(y * (int)robot.right.getGearing());
    } else if(state == 2) {
      double x = (!!ctrl.get_digital(DIGITAL_RIGHT) - !!ctrl.get_digital(DIGITAL_LEFT));
      if(!x) x = ctrl.get_analog(ANALOG_LEFT_X) / 127.0;
      robot. left.moveVelocity( x * (int)robot. left.getGearing());
      robot.right.moveVelocity(-x * (int)robot.right.getGearing());
    }
    if(ctrl.get_digital_new_press(DIGITAL_B)) {
      return GO_UP;
    } else {
      return NO_CHANGE;
    }
  }
};

class GPSList: public ControllerMenu {
  public:
  GPSList() {
    auto &gps = getRobot().gps;
    list.insert(list.end(), {
      {"Calibrate GPS", taskOption<GPSCalibrator>},
      {"Set X", [&]() {
        RoboPosition pos = gps.getPosition();
        pos.x = editNumber(pos.x, 3);
        gps.setPosition(pos);
      }},
      {"Set Y", [&]() {
        RoboPosition pos = gps.getPosition();
        pos.y = editNumber(pos.y, 3);
        gps.setPosition(pos);
      }},
      {"Set O", [&]() {
        RoboPosition pos = gps.getPosition();
        pos.o = editNumber(pos.o, 3);
        gps.setPosition(pos);
      }},
      {"Set CPR", [&]() {
        gps.setCPR(editNumber(gps.radiansToCounts(1), 4));
      }},
      {"Set CPI", [&]() {
        gps.setCPI(editNumber(gps.inchToCounts(1), 4));
      }}
    });
  }
};

class RootList: public ControllerMenu {
  public:
  RootList() {
    list.insert(list.end(), {
      {"Autonomous"  , taskOption<AutonList>},
      {"Motors"      , taskOption<MotorList>},
      {"GPS Settings", taskOption<  GPSList>}
    });
  }
};

void drawCatOSScreen() {
  line_set(0, "catOS v1.2");
  line_set(1, "press X+Y to");
  line_set(2, "activate menu");
}

void catOS(void*) {
  pros::Controller ctrl(pros::E_CONTROLLER_MASTER);
  drawCatOSScreen();
  while(true) {
    if(ctrl.get_digital(DIGITAL_X) && ctrl.get_digital(DIGITAL_Y)) {
      getRobot().takeStopped();
      menuWasEntered = true;
      RootList()();
      drawCatOSScreen();
      getRobot().give();
      menuWasEntered = false;
    }
    pros::delay(5);
  }
}

void disabled() { }

void startupDisplay() {
  pros::Task uiThread(uiExecutor, NULL, LVGL_PRIORITY);
  pros::Task controllerUI(catOS);
}

void competition_initialize() {}
