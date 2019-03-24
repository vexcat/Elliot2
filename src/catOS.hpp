/**
 * @file catOS.hpp
 * 
 * This file declares classes & functions that are part of catOS,
 * a controller UI framework.
 */

#pragma once
#include "main.h"
#include "display.hpp"
#include <memory>
#include <functional>

/**
 * This class is the parent class of all controller menus,
 * providing a standard way to re-render menus and check
 * input in a loop.
 */
class ControllerTask {
  public:
  ///Action to take from menu, as determined by checkController().
  enum CheckResult {
    NO_CHANGE, ///< Take no action
    GO_UP,     ///< Exit this menu
  };

  /**
   * @brief Abstract function that updates all three controller lines.
   * 
   * This function should <i>never</i> block.
   */
  virtual void render() = 0;
  
  /**
   * @brief Abstract function to check controller and return a CheckResult.
   * 
   * This function may choose to block, but doing so will prevent temporary
   * exits by checkTemporaryExit() from occurring, blocking battery warnings
   * and global key-combos.
   * 
   * @return A CheckResult saying whether to go up from menu.
   */
  virtual int checkController() = 0;

  /**
   * @brief Displays this ControllerTask
   * 
   * Runs checkController() in a loop, re-rendering when
   * checkTemporaryExit() returns true, and exiting when
   * checkController() returns GO_UP.
   */
  void operator()();

  ///Overridable destructor
  virtual ~ControllerTask() {};
};

///For functions to run on option selection.
using MenuAction = std::function<void(void)>;
///For individual entries in a ControllerMenu.
using MenuEntry = std::pair<std::string, MenuAction>;

/**
 * This function gets passed as a menu option. When called,
 * it will create a T using the provided args, then call
 * its operator()(). This works for any class T, perfectly
 * forwarding constructor parameters.
 */
template <typename T, typename ...Args>
void taskOption(Args&&... args) {
  auto ptr = std::make_unique<T>(std::forward<Args>(args)...);
  (*ptr)();
}

/**
 * A ControllerTask which shows a list of selectable
 * MenuEntry entries.
 */
class ControllerMenu: public ControllerTask {
  protected:
  ///Creates a ControllerMenu with an empty list.
  ControllerMenu() {}
  ///List of entries. Subclass constructors should add MenuEntry
  ///entries to this vector.
  std::vector<MenuEntry> list;
  ///Position in list.
  int index = 0;
  /**
   * @brief Render list on-screen.
   * 
   * Rendering will place all item names one column to the
   * right, and put a star next to the currently selected item.
   */
  void render() override;
  public:
  /**
   * @brief Check controller for list controlling inputs.
   * 
   * This will move the index up/down when either the up/down
   * keys are pressed or the left joystick is used. It will
   * run the selected MenuEntry's handler if the A key is
   * pressed, and will return GO_UP is B is pressed.
   * 
   * @return GO_UP if B is pressed, NO_CHANGE otherwise.
   */
  int checkController() override;
};

/**
 * Bound an index to a range [0, size). This will add/substract
 * by size in a loop to bring the index to its range. In the
 * special case that size == 0, index will be set to 0.
 * 
 * @param index Reference to index, new index will be written back here.
 * @param size  Number that index must be less than, the size of the
 * range of allowed values.
 */
void bound(int& index, int size);

/**
 * This abstract class allows the user to edit a list, with the
 * subclass determining how items should be created & edited and
 * finalized to a backing store, usually the global JSON state.
 * 
 * This class behaves similarly to a ControllerMenu, but in fact
 * contains two menus inside. These can be switched using the X
 * and B keys. The crudOptions menu allows for menu manipulation
 * (deletion, movement, renaming, creation), while the \ref items
 * menu allows for selection.
 */
class CRUDMenu: public ControllerTask {
  ///List of entries used to manipulate the \ref items list
  std::vector<MenuEntry> crudOptions;
  ///Index user is at in \ref crudOptions
  int crudIndex = 0;
  ///Renders the \ref crudOptions list
  void renderCRUD();

  ///List of entries in this editable list
  std::vector<std::string> items;
  ///Index user is at in \ref items
  int idx = 0;
  ///Renders the \ref items list
  void renderItems();

  ///@brief Whether to use \ref crudOptions or \ref items
  ///Determines whether render() and list controls on the
  ///controller will use the \ref crudOptions list or the
  ///\ref items list.
  enum {
    CRUD_OPTION_LIST = 0, ///< Render/use \ref crudOptions list
    ITEM_NAME_LIST = 1    ///< Render/use \ref items list
  } selectedVector = ITEM_NAME_LIST;

  protected:
  ///Will call either renderItems() or renderCrud() depending on \ref selectedVector
  void render() override;

  /**
   * @brief Constructs an empty CRUDMenu.
   * 
   * This menu will have no items, and no options to insert any new
   * items. Subclasses should, in their own constructor, add any
   * existing items from their backing store to the \ref items list.
   */
  CRUDMenu();

  /**
   * This adds an option to the \ref crudOptions list to create a new
   * item in the \ref items list. The option will appear as "+ <name>",
   * and on select, will call the provided attemptAdd function. The
   * attemptAdd function will receive the index the new item should go
   * to. If attemptAdd can add an item at that index, it will return
   * a string which will both refer to the item and be displayed to the
   * user. It should also store the change in some backing store, to
   * persist once this CRUDMenu is closed. If addition of an item at the
   * given position is not possible, attemptAdd should throw, and addition
   * of the item will be canceled.
   * 
   * @param name       Name of type of item this inserter will create
   * @param attemptAdd Function that will either create a new item at
   * a given position, returning its name, or throw if it was not
   * possible.
   */
  void addInserter(const std::string name, std::function<const std::string(int)> attemptAdd);

  /**
   * This adds an option to the \ref crudOptions list to perform some
   * action on an \ref items list entry. On selection of the item, the
   * provided convenience function will be called with the index of the
   * item in \ref items selected, and the item's name. The function is
   * not expected to return or throw anything.
   * 
   * @param name        Name of option to be added to \ref crudOptions
   * @param convenience Function to be called when this new item is selected
   */
  void addConvenience(const std::string name, std::function<void(int, std::string)> convenience);

  /**
   * @brief This adds an item to the end of the \ref items list.
   * 
   * This should be called during construction by subclasses, to
   * load in existing data from a backing store.
   * 
   * @param item Name of item to be appended
   */
  void addItem(std::string item);

  /**
   * @brief Renames an item.
   * 
   * This will change the item at index `idx` of \ref items to
   * have a name of `newName`.
   * 
   * @param idx     Index of entry to change the name of
   * @param newName New name for entry
   */
  void updateItem(int idx, std::string newName);

  /**
   * @brief Removes an item from subclass's backing store.
   * 
   * This function is called when the user attempts to delete an entry.
   * Subclasses should override this function to delete corresponding
   * data in a backing store. If an exception is thrown, deletion will
   * be canceled. By default, this function simply does `throw 0`.
   * 
   * @param idx     Index of item to be deleted
   * @param oldName Name of item to be deleted
   */
  virtual void attemptDelete(int idx, const std::string& oldName) { throw 0; }

  /**
   * @brief Moves an item in subclass's backing store.
   * 
   * This function is called when the user attempts to move an entry from
   * one index to another. Subclasses should override this function to
   * swap corresponding data in a backing store. If an exception is thrown,
   * movement will be canceled. By default, this function simply does `throw 0`.
   * 
   * @param idx     Current index of item to be moved
   * @param newIdx  Index item is trying to be moved to
   * @param oldName Name of item to be moved
   */
  virtual void attemptMove(int idx, int newIdx, const std::string& oldName) { throw 0; }

  /**
   * @brief Renames an item in subclass's backing store.
   * 
   * This function is called when the user attempts to rename an entry.
   * Subclasses should override this function to rename corresponding data
   * in a backing store. If an exception is thrown, renaming will be canceled.
   * By default, this function simply does `throw 0`.
   * 
   * @param idx     Index of item to be renamed
   * @param newName New name of item
   * @param oldName Old/current name of item to be renamed
   */
  virtual void attemptRename(int idx, std::string newName, const std::string& oldName) { throw 0; }

  /**
   * @brief Duplicates an item in subclass's backing store.
   * 
   * This function is called when the user attempts to duplicate an
   * entry. Subclasses should override this function to duplicate
   * corresponding data with a new name in a backing store. The name
   * of the duplicated item is determined by the subclass, and should
   * be returned. If an exception is thrown, duplication will be canceled.
   * By default, this function simply does `throw 0`.
   * 
   * @param idx     Index of item to be duplicated
   * @param newIdx  Index for duplicate to go to
   * @param oldName Name of item to be duplicated
   * @return Name of item once duplicated
   */
  virtual std::string attemptDuplicate(int idx, int newIdx, const std::string& oldName) { throw 0; }

  /**
   * @brief Handles selection of an item.
   * 
   * This function is called when the user selects an entry from
   * \ref items, with the index & name of the item. 
   * 
   * @param idx  Index of item selected
   * @param name Name of item selected
   */
  virtual void handleSelect(int idx, const std::string& name) {};

  /**
   * @brief Called on menu exit, to save any unsaved changes.
   * 
   * Some implementations may choose to delay saving of data until
   * the user is done editing. To do this, attempt* functions will
   * modify some in-memory data store, and this function, finalizeData()
   * will persist the data to a non-volatile destination.
   */
  virtual void finalizeData() {};

  public:
  /**
   * @brief Check controller for list controlling inputs.
   * 
   * This will check the left joystick & up/down keys to move the
   * selected list, the X key to switch to the \ref crudOptions
   * list, the A key to select an option, and the B key to exit.
   * 
   * @return GO_UP if B key pressed, NO_CHANGE otherwise.
   */
  int checkController() override;
};

/**
 * @brief Handles global key-combos and controller warnings.
 * 
 * This function is called by ControllerTask's operator()(), and
 * is defined outside of catOS.
 * 
 * In display.cpp, this function is defined to handle opening
 * the PID test menu (-> and Y), temporary driving (just Y), and
 * will stop and warn the user if the battery is low.
 */
int checkTemporaryExit();

/**
 * @brief Gets vertical direction from the controller.
 * 
 * This uses both the up/down keys and the left joystick to report
 * vertical direction. If up is pressed, this will return 1, and if
 * down is pressed, this will return -1. If neither key is pressed,
 * this will return the vertical joystick position in the integer
 * range [-3, 3].
 * 
 * @param flip Multiplies output before returning
 * @return The vertical direction from the controller
 */
int getVerticalDirection(int flip = 1);

/**
 * @brief Gets horizontal direction from the controller.
 * 
 * This uses both the up/down keys and the left joystick to report
 * horizontal direction. If up is pressed, this will return 1, and if
 * down is pressed, this will return -1. If neither key is pressed,
 * this will return the horizontal joystick position in the integer
 * range [-3, 3].
 * 
 * @param flip Multiplies output before returning
 * @return The horizontal direction from the controller
 */
int getHorizontalDirection(int flip = 1);

/**
 * @brief Sets a line on the controller.
 * 
 * This will call set_text on the controller, with trailing spaces
 * to clear previous text. It will also wait 52ms to make sure the
 * change occurred, as controller updates cannot occur more often
 * than every 50ms. 
 * 
 * @param line Line number, from 0 to 2.
 * @param str  Text to display
 */
void line_set(int line, std::string str);
