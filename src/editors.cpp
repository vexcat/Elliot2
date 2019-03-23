/**
 * @file editors.cpp
 * 
 * This file defines the functions for editing numbers & text on the V5 Controller's
 * screen. It is used throughout display.cpp to edit robot configuration & auton.
 */

#include "main.h"
#include "display.hpp"
#include "elliot.hpp"
#include "catOS.hpp"

/**
 * Render cursor arrows to display place being edited.
 * 
 * @param cursor Index of place to display arrows from [0, 15)
 */
void renderEditorArrows(int cursor) {
  std::string arrowText(15, ' ');
  arrowText[cursor] = '^';
  line_set(0, arrowText);
  arrowText[cursor] = 'v';
  line_set(2, arrowText);
}

/**
 * Renders the number editor, called internally by editNumber().
 * 
 * @param number Number to display on-screen
 * @param fix    Decimal places of precision to display number with
 * @param cursor Index of place being edited
 * @see editNumber()
 */
void renderNumberEditor(double number, int fix, int cursor) {
  char textData[16];
  snprintf(textData, 16, "%+014.*f\n", fix, number);
  line_set(1, std::string(textData));
  renderEditorArrows(cursor);
}

/**
 * Renders the string editor, called internally by editString().
 * 
 * @param text   Text to display on-screen
 * @param cursor Index in text being edited
 * @see editString()
 */
void renderStringEditor(std::string& text, int cursor) {
  line_set(1, text);
  renderEditorArrows(cursor);
}

//Edit number using controller LCD & input
double editNumber(double number, int fix) {
  auto &ctrl = getRobot().controller;
  int cursor = 14 - fix - (fix > 0);
  renderNumberEditor(number, fix, cursor);
  while(!ctrl.get_digital_new_press(DIGITAL_A) && !ctrl.get_digital_new_press(DIGITAL_B)) {
    //Cursor Movement
    int hdir = getHorizontalDirection();
    cursor += hdir;
    bound(cursor, 15);
    //(Skip the decimal place)
    if(14 - cursor == fix && fix != 0) {
      if(hdir < 0) cursor --;
      if(hdir > 0) cursor ++;
    }
    if(hdir) renderNumberEditor(number, fix, cursor);

    //Place-value Addition/Subtraction
    int vdir = getVerticalDirection();
    if(vdir && cursor == 0) {
      number *= -1;
      renderNumberEditor(number, fix, cursor);
    } else if(vdir) {
      int place;
      if(fix == 0) {
        place = 14 - cursor;
      } else {
        if(14 - cursor < fix) {
          place = 14 - cursor - fix;
        } else if(14 - cursor == fix) {
          place = 0;
        } else {
          place = 13 - cursor - fix;
        }
      }
      number += vdir * std::pow(10, place);
      renderNumberEditor(number, fix, cursor);
    }

    //Floor button
    if(ctrl.get_digital_new_press(DIGITAL_X)) {
      number = std::floor(number);
      renderNumberEditor(number, fix, cursor);
    }

    //Loop delay
    pros::delay(5);
  }
  return number;
}

/**
 * Removes trailing spaces in a string.
 * 
 * @param str Text to be trimmed
 */
void unrightPad(std::string& str) {
  //Count the spaces on the right of the string.
  int count = 0;
  while(count < str.size() && str[str.size() - 1 - count] == ' ') count++;
  //Now, delete this amount.
  str.erase(str.end() - count, str.end());
}

//Edit text using controller LCD & input
std::string editString(std::string text) {
  auto &ctrl = getRobot().controller;
  int cursor = 0;
  text.insert(text.size(), 15 - text.size(), ' ');
  renderStringEditor(text, cursor);
  while(!ctrl.get_digital_new_press(DIGITAL_A) && !ctrl.get_digital_new_press(DIGITAL_B)) {
    //Cursor Movement
    int hdir = getHorizontalDirection();
    cursor += hdir;
    bound(cursor, 15);
    if(hdir) renderStringEditor(text, cursor);

    //Symbol adjustment
    int vdir = getVerticalDirection();
    for(int i = 0; i < std::abs(vdir); i++) {
      auto &sym = text[cursor] += vdir > 0 ? 1 : -1;

      //Forward adjustment
      if(vdir > 0) {
        if(sym == ' ' + 1) {
          sym = 'a';
        }
        if(sym == 'z' + 1) {
          sym = 'A';
        }
        if(sym == 'Z' + 1) {
          sym = '0';
        }
        if(sym == '9' + 1) {
          sym = ' ';
        }
      }

      //Reverse adjustment
      if(vdir < 0) {
        if(sym == ' ' - 1) {
          sym = '9';
        }
        if(sym == '0' - 1) {
          sym = 'Z';
        }
        if(sym == 'A' - 1) {
          sym = 'z';
        }
        if(sym == 'a' - 1) {
          sym = ' ';
        }
      }
    }

    //Just in case, replace any unwanted symbols with spaces
    auto &sym = text[cursor];
    if(!(('a' <= sym && sym <= 'z') || ('A' <= sym && sym <= 'Z') || ('0' <= sym && sym <= '9') || sym == ' ')) {
      sym = ' ';
    }

    //Re-render if symbol was changed
    if(vdir) renderStringEditor(text, cursor);

    //Loop delay
    pros::delay(5);
  }
  unrightPad(text);
  return text;
}

//Select option from list using controller LCD & input
int selectOption(std::initializer_list<std::string> list, int idx) {
  auto &ctrl = getRobot().controller;
  renderEditorArrows(0);
  line_set(1, list.begin()[idx]);
  while(true) {
    //Option Changing
    int vdir = getVerticalDirection();
    idx += vdir;
    bound(idx, list.size());
    if(vdir) line_set(1, list.begin()[idx]);

    //Option Selection
    if(ctrl.get_digital_new_press(DIGITAL_B) || ctrl.get_digital_new_press(DIGITAL_A)) {
      break;
    }

    //Loop delay
    pros::delay(5);
  }
  return idx;
}
