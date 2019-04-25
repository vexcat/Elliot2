/**
 * @file elliot.hpp
 * 
 * This file declares all devices present on Elliot.
 */
#pragma once
#include "main.h"
#include "okapi/api.hpp"
#include "gps.hpp"
#include "display.hpp"
#include "base.hpp"
#include "debugging.hpp"
#include "ccpid_mod.hpp"
#include <deque>
using namespace okapi;

/**
 * Class responsible for running the puncher and its angle
 * changer. It uses PID with two different targets to drive
 * the angler, and integrated-PID to drive the puncher.
 */
class Puncher {
    ///Target potentiometer value for the angler to be aimed to the middle flag.
    int lowTargetPosition;
    ///Target potentiometer value for the angler to be aimed to the high flag.
    int highTargetPosition;
    ///Target position of the puncher motor in degrees since power-on.
    int puncherTarget;
    ///Motor driving puncher
    okapi::MotorGroup &puncher;
    ///Motor driving puncher angle-changer.
    okapi::MotorGroup &angler;
    ///Potentiometer sensor attached to angle-changer.
    okapi::Potentiometer &angleSense;
    ///Puncher JSON settings location
    json& puncherData;
    ///Angler PID controller
    std::shared_ptr<okapi::AsyncPosPIDController> controllerPtr;
    ///Reload puncher settings from SD card, called automatically from all setter functions.
    void loadState();
    ///Background task for puncher, which will stop the integrated PID controller when it is
    ///motionless and close to the target.
    void puncherTask();
    public:
    ///Puncher constructor, taking the puncher motor, the angler motor, the potentiometer,
    ///and the JSON data save location.
    Puncher(okapi::MotorGroup& puncher, okapi::MotorGroup& angler, okapi::Potentiometer& angleSense, json& puncherData);
    ///Runs puncherTask() as a pros::Task
    void beginTask() {
        pros::Task([](void* me) {((Puncher*)me)->puncherTask();}, (void*)this);
    }
    ///Sets the target of \ref controllerPtr to \ref lowTargetPosition.
    void lowTarget();
    ///Sets the target of \ref controllerPtr to \ref highTargetPosition.
    void highTarget();
    ///Stops \ref controllerPtr.
    void stopAutoControl();
    ///Advances puncher motor's target by 1 revolution.
    void shoot();
    ///Manually controls puncher, implicitly calls stopAutoControl().
    void setVelocity(double vel);
    ///Angler's error to its target.
    int targetError();
    ///Toggle angler between high & low target.
    void toggleTarget();
    ///Gets the PID gains used to control the angler.
    okapi::IterativePosPIDController::Gains getGains();
    ///Sets and saves the PID gains used to control the angler.
    void setGains(okapi::IterativePosPIDController::Gains gains);
    ///Sets and saves the low target value for the angler potentiometer.
    void setLowTarget(int target);
    ///Sets and saves the high target value for the angler potentiometer.
    void setHighTarget(int target);
    ///Gets the low target value for the angler potentiometer.
    int getLowTarget() { return lowTargetPosition; }
    ///Gets the high target value for the angler potentiometer.
    int getHighTarget() { return highTargetPosition; }
};

enum DriveStyle {
    DADDY_DRIVING  = 0,
    UNGATO_DRIVING = 1337
};

class Elliot {
    ///Set to 1 to drive forward, -1 to drive in reverse
	int multiplier = 1;
    ///Guards against two tasks using an Elliot object at the same time.
    static pros::Mutex usageGuard;
    public:
    ///Style of driving to use in opcontrol
    DriveStyle driveStyle = DADDY_DRIVING;
    ///Time at beginning of opcontrol, used to know when to activate
    ///30 second warning.
    long opctrlBegin;
    ///Master controller connected to robot.
    pros::Controller controller;
    ///Secondary controller connected to robot.
    pros::Controller partner;
    ///Motors driving left side of robot.
	MotorGroup left;
    ///Motors driving right side of robot.
	MotorGroup right;
    ///Motor driving the puncher.
	MotorGroup puncherMtr;
    ///Motor driving the puncher's angler.
	MotorGroup angler;
    ///Motor driving the ball intake.
	MotorGroup intake;
    ///Motor driving the cap scorer.
    MotorGroup scorer;
    ///Potentiometer sensing what position the angler is at.
    okapi::Potentiometer angleSense;
    ///Puncher object managing the puncher and its angler.
    Puncher puncher;
    ///@brief GPS object keeping track of robot position & orientation.
    ///@see GPS
    GPS gps;
    ///Pointer to an Elliot2CCPID with current base settings, can
    ///be reloaded with new settings with the \ref baseSettings object.
    std::unique_ptr<Elliot2CCPID> base;
    ///Object responsible for managing the \ref base pointer. Loads
    ///and saves JSON data for the Elliot2CCPID base.
    BaseSettings baseSettings;
    ///Constructs an Elliot object
    Elliot();
    ///Elliot objects cannot be copy constructed.
    Elliot(const Elliot&) = delete;
    ///Elliot objectes cannot be copy assigned.
    Elliot& operator=(const Elliot&) = delete;
    ///Stops all motors on the robot
    void stop();
    ///Takes the \ref usageGuard mutex without stopping the robot.
    static void takeCoast();
    ///Takes the \ref usageGuard mutex and stops the robot.
    static void takeStopped();
    ///Gives the \ref usageGuard mutex and stops the robot.
    static void give();
    ///Gives the \ref usageGuard mutex without stopping the robot.
    static void giveDirect();
    ///Does one tick of robot driving, with control from \ref controller.
    void drive(pros::Controller&, pros::Controller&, DriveStyle style);
    ///Begins the background tasks for the GPS \ref gps & the \ref base.
    void beginTasks();
};
/**
 * Creates the global elliot object, accessible by getRobot().
 * It is not safe to call this function during program execution (yet).
 * An exception thrown from this function is fatal.
 * 
 * @see getRobot()
 */
void createRobot();
extern Elliot* elliot;
/**
 * Retrieves a reference to the global Elliot object.
 * Do not expect this function to work while the global Elliot
 * object is being constructed, because the object is unfinished.
 * 
 * @see createRobot()
 */
inline Elliot& getRobot() { return *elliot; }
