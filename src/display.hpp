#pragma once
#include "main.h"
std::string getSelectedAuton();
void startupDisplay();
void line_set(int line, std::string str);
int selectOption(std::initializer_list<std::string> list, int idx);
