#include "main.h"
#include "display.hpp"
#include "elliot.hpp"

//------------------------------------------------------------------------------------
//  Controller Editors
//  ------------------
//  This file contains the functions for editing numbers & text on the V5 Controller's
//  screen. It is used primarily in MotionEditor of display.cpp.
//------------------------------------------------------------------------------------

//Render the cursor arrows that show you what place you're editing
void renderEditorArrows(int cursor) {
  std::string arrowText(15, ' ');
  arrowText[cursor] = '^';
  line_set(0, arrowText);
  arrowText[cursor] = 'v';
  line_set(2, arrowText);
}

//Render a number to a certain amount of decimals, and the cursor
void renderNumberEditor(double number, int fix, int cursor) {
  char textData[16];
  snprintf(textData, 16, "%+015.*f\n", fix, number);
  line_set(1, std::string(textData));
  renderEditorArrows(cursor);
}

//Render text and the cursor
void renderStringEditor(std::string& text, int cursor) {
  line_set(1, text);
  renderEditorArrows(cursor);
}

//Edit an existing number and return the result
double editNumber(double number, int fix) {
  auto &ctrl = getRobot().controller;
  int cursor = 14 - fix - (fix > 0);
  renderNumberEditor(number, fix, cursor);
  while(!ctrl.get_digital_new_press(DIGITAL_A) && !ctrl.get_digital_new_press(DIGITAL_B)) {
    if(ctrl.get_digital_new_press(DIGITAL_LEFT)) {
      cursor--;
      if(14 - cursor == fix) cursor--;
      if(cursor < 0) cursor = 0;
      renderNumberEditor(number, fix, cursor);
    }
    if(ctrl.get_digital_new_press(DIGITAL_RIGHT)) {
      cursor++;
      if(14 - cursor == fix) cursor++;
      if(cursor > 14) cursor = 14;
      renderNumberEditor(number, fix, cursor);
    }

    int dir = (!!ctrl.get_digital_new_press(DIGITAL_UP) - !!ctrl.get_digital_new_press(DIGITAL_DOWN));
    if(dir && cursor == 0) {
      number *= -1;
    } else if(dir) {
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
      number += dir * std::pow(10, place);
      renderNumberEditor(number, fix, cursor);

      //Floor button
      if(ctrl.get_digital_new_press(DIGITAL_X)) {
        number = std::floor(number);
        renderNumberEditor(number, fix, cursor);
      }
    }
    pros::delay(5);
  }
  return number;
}

//Remove padding to the right of a string
void unrightPad(std::string& str) {
  //Count the spaces on the right of the string.
  int count = 0;
  while(count < str.size() && str[str.size() - 1 - count] == ' ') count++;
  //Now, delete this amount.
  str.erase(str.end() - count, str.end());
}

//Edit an existing string and return the result
std::string editString(std::string text) {
  auto &ctrl = getRobot().controller;
  int cursor = 0;
  text.insert(text.size(), 15 - text.size(), ' ');
  renderStringEditor(text, cursor);
  while(!ctrl.get_digital_new_press(DIGITAL_A) && !ctrl.get_digital_new_press(DIGITAL_B)) {
    if(ctrl.get_digital_new_press(DIGITAL_LEFT)) {
      cursor--;
      if(cursor < 0) cursor = 0;
      renderStringEditor(text, cursor);
    }
    if(ctrl.get_digital_new_press(DIGITAL_RIGHT)) {
      cursor++;
      if(cursor > 14) cursor = 14;
      renderStringEditor(text, cursor);
    }

    int dir = (!!ctrl.get_digital_new_press(DIGITAL_UP) - !!ctrl.get_digital_new_press(DIGITAL_DOWN));
    auto &sym = text[cursor] += dir;

    if(dir > 0) {
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

    if(dir < 0) {
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

    if(dir)
      renderStringEditor(text, cursor);

    pros::delay(5);
  }
  unrightPad(text);
  return text;
}