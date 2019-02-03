#include "elliot.hpp"
#include "display.hpp"

void initialize() {
  try {
    createRobot();
    startupDisplay();
    getRobot().beginTasks();
  } catch(const std::exception& e) {
    std::cout << "Fatal initialization error: " + std::string(e.what()) << std::endl;
  } catch(...) {
    std::cout << "Unknown fatal initialization error." << std::endl;
  }
}