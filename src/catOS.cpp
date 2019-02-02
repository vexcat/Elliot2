#include "catOS.hpp"
#include "elliot.hpp"

void ControllerTask::operator()() {
  int result;
  render();
  do {
    result = checkController();
    pros::delay(5);
    if(checkTemporaryExit()) {
      render();
    }
  } while(result != ControllerTask::CheckResult::GO_UP);
}

void line_set(int line, std::string str) {
  if(15 >= str.size())
    str.insert(str.end(), 15 - str.size(), ' ');
  else
    str.erase(str.begin() + 15, str.end());
  getRobot().controller.set_text(line, 0, str.c_str());
  pros::delay(52);
}

void ControllerMenu::render() {
  for(int i = index; i < index + 3; i++) {
    if(i < list.size()) {
      line_set(i - index, (i == index ? "*": " ") + list[i].first);
    } else {
      line_set(i - index, "");
    }
  }
}

int getVerticalDirection(int flip) {
  auto &ctrl = getRobot().controller;
  int dir = (!!ctrl.get_digital_new_press(DIGITAL_UP) - !!ctrl.get_digital_new_press(DIGITAL_DOWN));
  if(!dir) {
    dir = 4 * ctrl.get_analog(ANALOG_LEFT_Y)/127.0;
    dir = std::clamp(dir, -3, 3);
  }
  return dir * flip;
}

int getHorizontalDirection(int flip) {
  auto &ctrl = getRobot().controller;
  int dir = (!!ctrl.get_digital_new_press(DIGITAL_RIGHT) - !!ctrl.get_digital_new_press(DIGITAL_LEFT));
  if(!dir) {
    dir = 4 * -ctrl.get_analog(ANALOG_LEFT_X)/127.0;
    dir = std::clamp(dir, -3, 3);
  }
  return dir * flip;
}

int ControllerMenu::checkController() {
  auto &ctrl = getRobot().controller;
  if(ctrl.get_digital_new_press(DIGITAL_B)) return GO_UP;
  if(ctrl.get_digital_new_press(DIGITAL_A)) {
    list[index].second();
    render();
  }
  int dir = getVerticalDirection(-1);
  if(dir) {
    index += dir;
    bound(index, list.size());
    render();
  }
  return NO_CHANGE;
}

void bound(int& index, int size) {
  if(index < 0) index = size - 1;
  if(index > size - 1) index = 0;
}

CRUDMenu::CRUDMenu() {
  crudOptions.insert(crudOptions.begin(), {
    {"delet this", [this](){
      try {
        if(idx < items.size()) {
          attemptDelete(idx, items[idx]);
          items.erase(items.begin() + idx);
          bound(idx, items.size());
          if(idx && idx >= items.size()) {
            idx--;
          }
        }
      } catch(...) {}
    }},
    {"move ^", [this](){
      try {
        if(idx != 0) {
          attemptMove(idx, idx - 1, items[idx]);
          std::swap(items[idx], items[idx - 1]);
          idx--;
        }
      } catch (...) {}
    }},
    {"move v", [this](){
      try {
        if(idx != items.size() - 1) {
          attemptMove(idx, idx + 1, items[idx]);
          std::swap(items[idx], items[idx + 1]);
          idx++;
        }
      } catch(...) {}
    }},
    {"duplicate", [this](){
      try {
        auto dupName = attemptDuplicate(idx, idx + 1, items[idx]);
        items.insert(items.begin() + idx + 1, dupName);
      } catch(...) {}
    }},
    {"rename", [this](){
      try {
        std::string newName = editString(items[idx]);
        selectedVector = ITEM_NAME_LIST;
        render();
        attemptRename(idx, newName, items[idx]);
        items[idx] = newName;
        render();
      } catch(...) {}
    }}
  });
}

void CRUDMenu::renderCRUD() {
  for(int i = crudIndex; i < crudIndex + 3; i++) {
    if(i < crudOptions.size()) {
      line_set(i - crudIndex, (i == crudIndex ? "*": " ") + crudOptions[i].first);
    } else {
      line_set(i - crudIndex, "");
    }
  }
}

void CRUDMenu::renderItems() {
  for(int i = idx; i < idx + 3; i++) {
    if(i < items.size()) {
      line_set(i - idx, (i == idx ? "*": " ") + items[i]);
    } else {
      line_set(i - idx, "");
    }
  }
}

void CRUDMenu::render() {
  if(selectedVector == CRUD_OPTION_LIST) {
    renderCRUD();
  } else {
    renderItems();
  }
}

int CRUDMenu::checkController() {
  auto &ctrl = getRobot().controller;
  if(ctrl.get_digital_new_press(DIGITAL_B)) {
    if(selectedVector == ITEM_NAME_LIST) {
      finalizeData();
      return GO_UP;
    } else {
      selectedVector = ITEM_NAME_LIST;
      render();
    }
  };
  if(ctrl.get_digital_new_press(DIGITAL_X)) {
    selectedVector = CRUD_OPTION_LIST;
    render();
  }
  if(ctrl.get_digital_new_press(DIGITAL_A)) {
    if(selectedVector == CRUD_OPTION_LIST) {
      crudOptions[crudIndex].second();
    } else {
      handleSelect(idx, items[idx]);
    }
    render();
  }
  int dir = getVerticalDirection(-1);
  if(dir) {
    if(selectedVector != ITEM_NAME_LIST) {
      crudIndex += dir;
      bound(crudIndex, crudOptions.size());
    } else {
      idx += dir;
      bound(idx, items.size());
    }
    render();
  }
  return NO_CHANGE;
}

void CRUDMenu::addInserter(const std::string name, std::function<const std::string(int)> attemptAdd) {
  crudOptions.push_back({"+ " + name, [attemptAdd, this](){
    try {
      if(items.size()) {
        items.insert(items.begin() + idx + 1, attemptAdd(idx + 1));
      } else {
        items.push_back(attemptAdd(0));
      }
    } catch(...) {}
  }});
}

void CRUDMenu::addConvenience(const std::string name, std::function<void(int, std::string)> convenience) {
  crudOptions.push_back({name, [convenience, this]() {
    try {
      convenience(idx, items[idx]);
    } catch(...) {}
  }});
}

void CRUDMenu::addItem(std::string item) {
  items.insert(items.end(), item);
}

void CRUDMenu::updateItem(int idx, std::string newName) {
  items[idx] = newName;
}