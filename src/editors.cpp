#include "main.h"
#include "display.hpp"

void renderEditorArrows(pros::Controller& ctrl, int cursor) {
  std::string arrowText(15, ' ');
  arrowText[cursor] = '^';
  line_set(ctrl, 0, arrowText);
  arrowText[cursor] = 'v';
  line_set(ctrl, 2, arrowText);
}

void renderNumberEditor(pros::Controller& ctrl, double number, int fix, int cursor) {
  char textData[16];
  snprintf(textData, 16, "%+015.*f\n", fix, number);
  line_set(ctrl, 1, std::string(textData));
  renderEditorArrows(ctrl, cursor);
}

void renderStringEditor(pros::Controller& ctrl, std::string& text, int cursor) {
  line_set(ctrl, 1, text);
  renderEditorArrows(ctrl, cursor);
}

double editNumber(pros::Controller& ctrl, double number, int fix) {
  int cursor = 14 - fix - (fix > 0);
  renderNumberEditor(ctrl, number, fix, cursor);
  while(!ctrl.get_digital_new_press(DIGITAL_A) && !ctrl.get_digital_new_press(DIGITAL_B)) {
    if(ctrl.get_digital_new_press(DIGITAL_LEFT)) {
      cursor--;
      if(cursor < 0) cursor = 0;
      renderNumberEditor(ctrl, number, fix, cursor);
    }
    if(ctrl.get_digital_new_press(DIGITAL_RIGHT)) {
      cursor++;
      if(cursor > 14) cursor = 14;
      renderNumberEditor(ctrl, number, fix, cursor);
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
      renderNumberEditor(ctrl, number, fix, cursor);
    }
    pros::delay(5);
  }
  return number;
}

void unrightPad(std::string& str) {
  //Count the spaces on the right of the string.
  int count = 0;
  while(count < str.size() && str[str.size() - 1 - count] == ' ') count++;
  //Now, delete this amount.
  str.erase(str.end() - count, str.end());
}

std::string editString(pros::Controller& ctrl, std::string text) {
  int cursor = 0;
  text.insert(text.size(), 15 - text.size(), ' ');
  renderStringEditor(ctrl, text, cursor);
  while(!ctrl.get_digital_new_press(DIGITAL_A) && !ctrl.get_digital_new_press(DIGITAL_B)) {
    if(ctrl.get_digital_new_press(DIGITAL_LEFT)) {
      cursor--;
      if(cursor < 0) cursor = 0;
      renderStringEditor(ctrl, text, cursor);
    }
    if(ctrl.get_digital_new_press(DIGITAL_RIGHT)) {
      cursor++;
      if(cursor > 14) cursor = 14;
      renderStringEditor(ctrl, text, cursor);
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
      renderStringEditor(ctrl, text, cursor);

    pros::delay(5);
  }
  unrightPad(text);
  return text;
}