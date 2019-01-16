#pragma once
#include "main.h"
#include "display.hpp"
#include "editors.hpp"
#include <memory>
#include <functional>

class ControllerTask {
  public:
  enum CheckResult { NO_CHANGE, GO_UP };
  virtual void initialize(pros::Controller&) = 0;
  virtual int checkController(pros::Controller&) = 0;
  void operator()(pros::Controller& ctrl);
  virtual ~ControllerTask() {};
};

using MenuAction = std::function<void(pros::Controller&)>;
using MenuEntry = std::pair<std::string, MenuAction>;

//C++ is really, really cool sometimes.
//This function is passed as a menu option. When called,
//it will create a T using the provided args, then call it
//on the controller. 
//The strange syntax is to be able to work on any class T,
//and to perfectly forward the constructor arguments.
template <typename T, typename ...Args>
MenuAction taskOption(pros::Controller& ctrl, Args&&... args) {
  auto ptr = std::make_unique<T>(std::forward<Args>(args)...);
  (*ptr)(ctrl);
}

void line_set(pros::Controller& c, int line, std::string str);

class ControllerMenu: public ControllerTask {
  protected:
  ControllerMenu() {};
  std::vector<MenuEntry> list;
  int index = 0;
  void render(pros::Controller& ctrl);
  public:
  void initialize(pros::Controller& ctrl) override;
  int checkController(pros::Controller& ctrl) override;
};

void bound(int& index, int size);

using MenuPair = std::pair<int, std::vector<MenuEntry>>;
class CRUDMenu: public ControllerTask {
  std::vector<MenuEntry> crudOptions;
  int crudIndex = 0;
  void renderCRUD(pros::Controller& ctrl);

  std::vector<std::string> items;
  int idx = 0;
  void renderItems(pros::Controller& ctrl);


  int selectedVector = ITEM_NAME_LIST;
  void render(pros::Controller& ctrl);

  enum {
    CRUD_OPTION_LIST = 0,
    ITEM_NAME_LIST = 1
  };

  protected:
  CRUDMenu();

  void addInserter(const std::string name, std::function<const std::string(int)> attemptAdd);
  void addItem(std::string item);

  void updateItem(int idx, std::string newName);

  //Passing no-ops means the only state held will be in this class. Generally not that useful.
  virtual void attemptDelete(int idx, const std::string& oldName) {}
  virtual void attemptMove(int idx, int newIdx, const std::string& oldName) {}
  virtual void attemptRename(int idx, std::string newName, const std::string& oldName) {}
  virtual std::string attemptDuplicate(int idx, int newIdx, const std::string& oldName) {}
  virtual void handleSelect(pros::Controller& ctrl, int idx, const std::string& name) = 0;
  virtual void finalizeData() = 0;

  public:
  void initialize(pros::Controller& ctrl) override;
  int checkController(pros::Controller& ctrl) override;
};
