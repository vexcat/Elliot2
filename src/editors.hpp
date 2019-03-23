/**
 * @file editors.hpp
 * 
 * This file declares the functions for editing numbers & text on the V5 Controller's
 * screen. It is used throughout display.cpp to edit robot configuration & auton.
 */

#pragma once

/**
 * Edits a number using the controller LCD.
 * 
 * @param number Initial number to show in editor
 * @param fix    Number of editable places after the decimal point
 * @return The number, after being edited
 */
double editNumber(double number, int fix);

/**
 * Edits text using the controller LCD.
 * 
 * @param text Initial text to show in editor
 * @return The text, after being edited
 */
std::string editString(std::string text);

/**
 * Selects an option out of a list of options on the controller LCD.
 * 
 * @param list List of all selectable options
 * @param idx  Index of first shown option
 * @return Index selected by user
 */
int selectOption(std::initializer_list<std::string> list, int idx);
