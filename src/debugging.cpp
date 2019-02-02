#include "main.h"
#include <string>
//Debug Logger - This function should not be used in production code.
void debug(std::string text) {
  printf(text.c_str());
  pros::delay(500);
}



//Diagnostic Logger - Will log data to a file.
void log(const std::string& text) {
  FILE* diagnostics;
  static FILE* diagnostics;
  static bool diagnosticsFileOpenFailure;
  if(!diagnostics) {
    diagnostics = fopen("/usd/diagonal-stick", "w");
  }
}