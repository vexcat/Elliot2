#include "catOS.hpp"

void ControllerTask::operator()(pros::Controller& ctrl) {
  int result;
  initialize(ctrl);
  do {
    result = checkController(ctrl);
    pros::delay(5);
    checkTemporaryExit(ctrl);
  } while(result != ControllerTask::CheckResult::GO_UP);
}

void line_set(pros::Controller& c, int line, std::string str) {
  if(15 >= str.size())
    str.insert(str.end(), 15 - str.size(), ' ');
  else
    str.erase(str.begin() + 15, str.end());
  c.set_text(line, 0, str.c_str());
  pros::delay(52);
}

void ControllerMenu::render(pros::Controller& ctrl) {
  for(int i = index; i < index + 3; i++) {
    if(i < list.size()) {
      line_set(ctrl, i - index, (i == index ? "*": " ") + list[i].first);
    } else {
      line_set(ctrl, i - index, "");
    }
  }
}

void ControllerMenu::initialize(pros::Controller& ctrl) {
  render(ctrl);
}

int ControllerMenu::checkController(pros::Controller& ctrl) {
  if(ctrl.get_digital_new_press(DIGITAL_B)) return GO_UP;
  if(ctrl.get_digital_new_press(DIGITAL_A)) {
    list[index].second(ctrl);
    render(ctrl);
  }
  if(ctrl.get_digital_new_press(DIGITAL_DOWN)) {
    index++;
    if(index >= list.size()) index = 0;
    render(ctrl);
  }
  if(ctrl.get_digital_new_press(DIGITAL_UP)) {
    index--;
    if(index < 0) index = list.size() - 1;
    render(ctrl);
  }
}

void bound(int& index, int size) {
  if(index < 0) index = size - 1;
  if(index > size - 1) index = 0;
}

CRUDMenu::CRUDMenu() {
  crudOptions.insert(crudOptions.begin(), {
    {"delet this", [this](pros::Controller& ctrl){
      try {
        attemptDelete(idx, items[idx]);
        items.erase(items.begin() + idx);
      } catch(...) {}
    }},
    {"move ^", [this](pros::Controller& ctrl){
      try {
        if(idx != 0) {
          attemptMove(idx, idx - 1, items[idx]);
          std::swap(items[idx], items[idx - 1]);
        }
      } catch (...) {}
    }},
    {"move v", [this](pros::Controller& ctrl){
      try {
        if(idx != items.size() - 1) {
          attemptMove(idx, idx + 1, items[idx]);
          std::swap(items[idx], items[idx + 1]);
        }
      } catch(...) {}
    }},
    {"duplicate", [this](pros::Controller& ctrl){
      try {
        attemptDuplicate(idx, idx + 1, items[idx]);
        items.insert(items.begin() + idx + 1, items[idx]);
      } catch(...) {}
    }},
    {"rename", [this](pros::Controller& ctrl){
      try {
        std::string newName = editString(ctrl, items[idx]);
        selectedVector = ITEM_NAME_LIST;
        render(ctrl);
        attemptRename(idx, newName, items[idx]);
        items[idx] = newName;
        render(ctrl);
      } catch(...) {}
    }}
  });
}

void CRUDMenu::renderCRUD(pros::Controller& ctrl) {
  for(int i = crudIndex; i < crudIndex + 3; i++) {
    if(i < crudOptions.size()) {
      line_set(ctrl, i - crudIndex, (i == crudIndex ? "*": " ") + crudOptions[i].first);
    } else {
      line_set(ctrl, i - crudIndex, "");
    }
  }
}

void CRUDMenu::renderItems(pros::Controller& ctrl) {
  for(int i = idx; i < idx + 3; i++) {
    if(i < items.size()) {
      line_set(ctrl, i - idx, (i == idx ? "*": " ") + items[i]);
    } else {
      line_set(ctrl, i - idx, "");
    }
  }
}

void CRUDMenu::render(pros::Controller& ctrl) {
  if(selectedVector == CRUD_OPTION_LIST) {
    renderCRUD(ctrl);
  } else {
    renderItems(ctrl);
  }
}

void CRUDMenu::initialize(pros::Controller& ctrl) {
  crudIndex = 0;
  idx = 0;
  render(ctrl);
}

int CRUDMenu::checkController(pros::Controller& ctrl) {
  if(ctrl.get_digital_new_press(DIGITAL_B)) {
    if(selectedVector == ITEM_NAME_LIST) {
      finalizeData();
      return GO_UP;
    } else {
      selectedVector = ITEM_NAME_LIST;
      render(ctrl);
    }
  };
  if(ctrl.get_digital_new_press(DIGITAL_X)) {
    selectedVector = CRUD_OPTION_LIST;
    render(ctrl);
  }
  if(ctrl.get_digital_new_press(DIGITAL_A)) {
    if(selectedVector == CRUD_OPTION_LIST) {
      crudOptions[crudIndex].second(ctrl);
    } else {
      handleSelect(ctrl, idx, items[idx]);
    }
    crudIndex = 0;
    idx = 0;
    render(ctrl);
  }
  if(ctrl.get_digital_new_press(DIGITAL_DOWN)) {
    if(selectedVector != ITEM_NAME_LIST) {
      crudIndex++;
      bound(crudIndex, crudOptions.size());
    } else {
      idx++;
      bound(idx, items.size());
    }
    render(ctrl);
  }
  if(ctrl.get_digital_new_press(DIGITAL_UP)) {
    if(selectedVector != ITEM_NAME_LIST) {
      crudIndex--;
      bound(crudIndex, crudOptions.size());
    } else {
      idx--;
      bound(idx, items.size());
    }
    render(ctrl);
  }
}

void CRUDMenu::addInserter(const std::string name, std::function<const std::string(int)> attemptAdd) {
  crudOptions.push_back({"+ " + name, [attemptAdd, this](pros::Controller& ctrl){
    try {
      if(items.size()) {
        items.insert(items.begin() + idx + 1, attemptAdd(idx + 1));
      } else {
        items.push_back(attemptAdd(0));
      }
    } catch(...) {}
  }});
}

void CRUDMenu::addItem(std::string item) {
  items.insert(items.end(), item);
}

void CRUDMenu::updateItem(int idx, std::string newName) {
  items[idx] = newName;
}