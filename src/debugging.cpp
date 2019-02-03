#include "main.h"
#include <string>
//Debug Logger - This function should not be used in production code.
void debug(std::string text) {
  printf(text.c_str());
  pros::delay(500);
}