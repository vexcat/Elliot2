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
using namespace okapi;

//Display code! This file contains the code for:
//  - The auton selector
//  - The GPS viewer
//  - The GPS calibrator
//  - The auton planner
//  - And a bunch of random stuff that'll probably get refactored out.

//Debug Logger - This function should not be used in production code.
void debug(std::string text) {
  printf(text.c_str());
  pros::delay(500);
}

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
      list.push_back({option, [&object, key, fix](auto& ctrl){
        object[key] = editNumber(ctrl, object[key].get<double>(), fix);
      }});
    }
    list.insert(list.end(), conveniences);
    list.push_back({"Run this", [&](auto&) {
      runMotion(object, getBlue());
    }});
    list.push_back({"Run to here", [&](auto&) {
      auto loc = auton.begin();
      //Process the first entry, an Origin.
      RoboPosition origin = {
        (*loc)["x"].get<double>(),
        (*loc)["y"].get<double>(),
        (*loc)["o"].get<double>()
      };
      auto &bot = getRobot();
      bot.gps.setPosition(origin);
      loc++;
      for(; loc != auton.begin() + idx + 1; loc++) {
        runMotion(*loc, getBlue());
      }
    }});
  }
};

class MotionList: public CRUDMenu {
  json &motionData;
  public:
  MotionList(std::string autonName): CRUDMenu(), motionData(getState()["autons"][autonName]) {}
  void initialize(pros::Controller& ctrl) override {
    //Add inserters here
    auto &bot = getRobot();
    addInserter("Position", [&](int index) -> std::string {
      RoboPosition pos = bot.gps.getPosition();
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "position"}, {"x", pos.x}, {"y", pos.y}, {"t", 200.0}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Direct", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "direct"}, {"l", 1.0}, {"r", 1.0}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Rotation", [&](int index) -> std::string {
      RoboPosition pos = bot.gps.getPosition();
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "rotation"}, {"o", pos.o}, {"t", 200.0}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("SLine", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "sline"}, {"d", 12.0}, {"t", 200.0}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Scorer", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "scorer"}, {"v", 1.0}, {"t", 200.0}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Catapult", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "catapult"}, {"v", 1.0}, {"t", 200.0}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Intake", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "intake"}, {"v", 1.0}, {"t", 200.0}
      }));
      return nameFor(motionData[index]);
    });
    //Fun fact: Shooting doesn't get an editor. It has no parameters.
    addInserter("Shoot", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "shoot"}
      }));
      return nameFor(motionData[index]);
    });
    addInserter("Delay", [&](int index) -> std::string {
      motionData.insert(motionData.begin() + index, json::object({
        {"type", "delay"}, {"t", 200.0}
      }));
      return nameFor(motionData[index]);
    });
    //Add existing items
    for(auto &motion: motionData) {
      addItem(nameFor(motion));
    }
    //First render
    CRUDMenu::initialize(ctrl);
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
  void handleSelect(pros::Controller& ctrl, int idx, const std::string& name) override {
    std::string type = motionData[idx]["type"];
    if(type == "origin") {
      json &origin = motionData[idx];
      MotionEditor(motionData, idx, {
        {"x", 2, "Set X"},
        {"y", 2, "Set Y"},
        {"o", 2, "Set Orientation"}
      }, {
        {"Move Here", [&origin](auto&){
          json copy = origin;
          copy["type"] = "position";
          runMotion(copy, getBlue());
          copy["type"] = "rotation";
          runMotion(copy, getBlue());
          //This line *shouldn't* have an effect, but here for safety.
          copy["type"] = "origin";
        }}
      })(ctrl);
    } else if(type == "position") {
      MotionEditor(motionData, idx, {
        {"x", 2, "Set X"},
        {"y", 2, "Set Y"}
      })(ctrl);
    } else if(type == "direct") {
      MotionEditor(motionData, idx, {
        {"l", 2, "Set left vel"},
        {"r", 2, "Set right vel"}
      })(ctrl);
    } else if(type == "rotation") {
      MotionEditor(motionData, idx, {
        {"o", 2, "Set Orientation"}
      })(ctrl);
    } else if(type == "sline") {
      MotionEditor(motionData, idx, {
        {"d", 2, "Set distance"},
        {"t", 3, "Set timing"}
      })(ctrl);
    } else if(type == "scorer" || type == "catapult" || type == "intake") {
      MotionEditor(motionData, idx, {
        {"v", 2, "Set velocity"},
        {"t", 3, "Set timing"}
      })(ctrl);
    } else if(type == "delay") {
      motionData[idx]["t"] = editNumber(ctrl, motionData[idx]["t"].get<double>(), 3);
    }
    updateItem(idx, nameFor(motionData[idx]));
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
      snprintf(buf, 16, "%6.4fs delay");
      return buf;
    }
    return motion["type"].get<std::string>();
  }
};

class AutonList: public CRUDMenu {
  json &autonData;
  public:
  AutonList(): CRUDMenu(), autonData(getState()["autons"]) {}
  void initialize(pros::Controller& ctrl) override {
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
    //Render for the first time
    CRUDMenu::initialize(ctrl);
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
      autonData.erase(location);
      addAuton("unnamed");
    }
    return "unnamed";
  }

  void handleSelect(pros::Controller& ctrl, int idx, const std::string& name) override {
    taskOption<MotionList>(ctrl, name);
  };
  void finalizeData() override {
    saveState();
  }
};

class MotorTest: public ControllerTask {
  pros::Motor m;
  int pn;
  public:
  MotorTest(int n): m(n), pn(n) {};
  void initialize(pros::Controller& ctrl) override {
    line_set(ctrl, 0, "Test motor# " + std::to_string(pn));
    line_set(ctrl, 1, "Press <-/->");
    line_set(ctrl, 2, "Or use left joy");
  }
  int checkController(pros::Controller& ctrl) override {
    if(ctrl.get_digital(DIGITAL_LEFT)) {
      m.move_velocity(-200);
    } else if(ctrl.get_digital(DIGITAL_RIGHT)) {
      m.move_velocity(200);
    } else {
      m.move_velocity(ctrl.get_analog(ANALOG_LEFT_Y) * 200.0 / 127.0);
    }
    if(ctrl.get_digital(DIGITAL_B)) {
      return GO_UP;
    } else {
      return NO_CHANGE;
    }
  };
};

class MotorList: public ControllerMenu {
  public:
  MotorList() {}
  void initialize(pros::Controller& ctrl) override {
    for(int i = 1; i <= 8; i++) {
      list.push_back({std::to_string(i), [=](auto& ctrl) {
        taskOption<MotorTest>(ctrl, i);
      }});
    }
    ControllerMenu::initialize(ctrl);
  }
};

class GPSCalibrator: public ControllerTask {
  double initial_left;
  double initial_right;
  Elliot& robot;
  int state = 0;
  public:
  GPSCalibrator(): robot(getRobot()) {}
  void initialize(pros::Controller& ctrl) override {
    initial_left  = robot.left .getPosition();
    initial_right = robot.right.getPosition();
    line_set(ctrl, 0, "Move robot 96in");
    line_set(ctrl, 1, "<-  ->/LeftJoyY");
    line_set(ctrl, 2, "then press A.");
  }
  int checkController(pros::Controller& ctrl) override {
    if(ctrl.get_digital_new_press(DIGITAL_A)) {
      if(state == 0) {
        double lastAvg = (initial_left + initial_right) / 2;
        double curAvg = (robot.left .getPosition() + robot.right.getPosition()) / 2;
        double measuredTPI = curAvg - lastAvg;
        robot.gps.setCPI(measuredTPI);
        line_set(ctrl, 0, "Reset position");
        line_set(ctrl, 1, "of robot, then");
        line_set(ctrl, 2, "press A.");
      } else if(state == 1) {
        initial_left  = robot.left .getPosition();
        initial_right = robot.right.getPosition();
        line_set(ctrl, 0, "Turn 32x CCW");
        line_set(ctrl, 1, "<- ->/LeftJoyX");
        line_set(ctrl, 2, "then press A.");
      } else if(state == 2) {
        double deltaL = robot.left .getPosition() - initial_left;
        double deltaR = robot.right.getPosition() - initial_right;
        double measuredCPR = (deltaR - deltaL) / 32 * PI;
        robot.gps.setCPR(measuredCPR);
        line_set(ctrl, 0, "All done!");
        line_set(ctrl, 1, "A to exit.");
        line_set(ctrl, 2, "");
      } else if(state == 3) return GO_UP;
      state++;
    }
    if(state == 0) {
      int y = (!!ctrl.get_digital(DIGITAL_RIGHT) - !!ctrl.get_digital(DIGITAL_LEFT));
      if(!y) y = ctrl.get_analog(ANALOG_LEFT_Y) / 127.0;
      robot.left.controllerSet(y);
      robot.right.controllerSet(y);
    } else if(state == 2) {
      int x = (!!ctrl.get_digital(DIGITAL_RIGHT) - !!ctrl.get_digital(DIGITAL_LEFT));
      if(!x) x = ctrl.get_analog(ANALOG_LEFT_X) / 127.0;
      robot.left.controllerSet(x);
      robot.right.controllerSet(-x);
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
  GPSList() {}
  void initialize(pros::Controller& ctrl) override {
    list.push_back({"Calibrate GPS", taskOption<GPSCalibrator>});
    auto &gps = getRobot().gps;
    list.push_back({"Set X", [&](auto& ctrl) {
      RoboPosition pos = gps.getPosition();
      pos.x = editNumber(ctrl, pos.x, 3);
      gps.setPosition(pos);
    }});
    list.push_back({"Set Y", [&](auto& ctrl) {
      RoboPosition pos = gps.getPosition();
      pos.x = editNumber(ctrl, pos.x, 3);
      gps.setPosition(pos);
    }});
    list.push_back({"Set Z", [&](auto& ctrl) {
      RoboPosition pos = gps.getPosition();
      pos.x = editNumber(ctrl, pos.x, 3);
      gps.setPosition(pos);
    }});
    list.push_back({"Set CPR", [&](auto& ctrl) {
      gps.setCPR(editNumber(ctrl, gps.radiansToCounts(1), 4));
    }});
    list.push_back({"Set CPI", [&](auto& ctrl) {
      gps.setCPI(editNumber(ctrl, gps.inchToCounts(1), 4));
    }});
    ControllerMenu::initialize(ctrl);
  }
};

class RootList: public ControllerMenu {
  public:
  RootList() {}
  void initialize(pros::Controller& ctrl) override {
    list.push_back({"Autonomous"  , taskOption<AutonList>});
    list.push_back({"Motors"      , taskOption<MotorList>});
    list.push_back({"GPS Settings", taskOption<  GPSList>});
    ControllerMenu::initialize(ctrl);
  }
};

class CatOSScreen: public ControllerTask {
  std::unique_ptr<ControllerMenu> root = std::make_unique<RootList>();
  void initialize(pros::Controller& ctrl) override {
    line_set(ctrl, 0, "catOS v1.2");
    line_set(ctrl, 1, "press X+Y to");
    line_set(ctrl, 2, "activate menu");
  }
  int checkController(pros::Controller& ctrl) override {
    if(ctrl.get_digital(DIGITAL_X) && ctrl.get_digital(DIGITAL_Y)) {
      getRobot().takeStopped();
      menuWasEntered = true;
      (*root)(ctrl);
      initialize(ctrl);
      getRobot().give();
      menuWasEntered = false;
    }
    return NO_CHANGE;
  }
};

void catOS(void*) {
  std::unique_ptr<CatOSScreen> screen = std::make_unique<CatOSScreen>();
  pros::Controller master(pros::E_CONTROLLER_MASTER);
  (*screen)(master);
}

void disabled() { }

void startupDisplay() {
  pros::Task uiThread(uiExecutor, NULL, LVGL_PRIORITY);
  pros::Task controllerUI(catOS);
}

void competition_initialize() {}
