#include "elliot.hpp"
#include "display.hpp"

void initialize() {
  try {
    createRobot();
    startupDisplay();
    getRobot().beginTasks();
    Logger::initialize(
      TimeUtilFactory::create().getTimer(),
      "/ser/sout", // output to the PROS terminal over standard out
      Logger::LogLevel::debug /* // most verbose log level is debug*/
    );
  } catch(const std::exception& e) {
    std::cout << "Fatal initialization error: " + std::string(e.what()) << std::endl;
    while(true) {
      pros::delay(1000);
    }
  } catch(...) {
    std::cout << "Unknown fatal initialization error." << std::endl;
    while(true) {
      pros::delay(1000);
    }
  }
}