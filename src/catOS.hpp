#pragma once
#include "main.h"
#include "display.hpp"
#include "editors.hpp"
#include <memory>
#include <functional>

class ControllerTask {
  public:
  enum CheckResult { NO_CHANGE, GO_UP };
  virtual void render() = 0;
  virtual int checkController() = 0;
  void operator()();
  virtual ~ControllerTask() {};
};

using MenuAction = std::function<void(void)>;
using MenuEntry = std::pair<std::string, MenuAction>;

//C++ is really, really cool sometimes.
//This function is passed as a menu option. When called,
//it will create a T using the provided args, then call it
//on the controller. 
//The strange syntax is to be able to work on any class T,
//and to perfectly forward the constructor arguments.
template <typename T, typename ...Args>
void taskOption(Args&&... args) {
  auto ptr = std::make_unique<T>(std::forward<Args>(args)...);
  (*ptr)();
}

class ControllerMenu: public ControllerTask {
  protected:
  ControllerMenu() {}
  std::vector<MenuEntry> list;
  int index = 0;
  void render() override;
  public:
  int checkController() override;
};

void bound(int& index, int size);

class CRUDMenu: public ControllerTask {
  std::vector<MenuEntry> crudOptions;
  int crudIndex = 0;
  void renderCRUD();

  std::vector<std::string> items;
  int idx = 0;
  void renderItems();


  int selectedVector = ITEM_NAME_LIST;

  enum {
    CRUD_OPTION_LIST = 0,
    ITEM_NAME_LIST = 1
  };

  protected:
  void render() override;
  CRUDMenu();

  void addInserter(const std::string name, std::function<const std::string(int)> attemptAdd);
  void addConvenience(const std::string name, std::function<void(int, std::string)> convenience);
  void addItem(std::string item);

  void updateItem(int idx, std::string newName);

  //Passing no-ops means the only state held will be in this class. Generally not that useful.
  virtual void attemptDelete(int idx, const std::string& oldName) { throw 0; }
  virtual void attemptMove(int idx, int newIdx, const std::string& oldName) { throw 0; }
  virtual void attemptRename(int idx, std::string newName, const std::string& oldName) { throw 0; }
  virtual std::string attemptDuplicate(int idx, int newIdx, const std::string& oldName) { throw 0; }
  virtual void handleSelect(int idx, const std::string& name) {};
  virtual void finalizeData() {};

  public:
  int checkController() override;
};

int checkTemporaryExit();
int getVerticalDirection(int flip = 1);
int getHorizontalDirection(int flip = 1);
void line_set(int line, std::string str);
