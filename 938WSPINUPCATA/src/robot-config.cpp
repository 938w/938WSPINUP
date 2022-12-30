#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Catapult = motor(PORT1, ratio36_1, true);
motor Intake = motor(PORT2, ratio6_1, true);
controller Controller1 = controller(primary);
rotation Rotation3 = rotation(PORT3, false);
motor RightM = motor(PORT4, ratio6_1, true);
motor LeftM = motor(PORT6, ratio6_1, false);
motor RightB = motor(PORT8, ratio6_1, false);
motor LeftB = motor(PORT7, ratio6_1, true);
motor LeftF = motor(PORT9, ratio6_1, true);
motor RightF = motor(PORT10, ratio6_1, false);
motor_group Leftside = motor_group(LeftM, LeftB, LeftF);
motor_group Rightside = motor_group(RightM, RightB, RightF);
inertial Inertial = inertial (PORT21); 
smartdrive Drivetrain = smartdrive(Leftside, Rightside, Inertial, 320, 320, 130, distanceUnits::mm, 1);
// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  Inertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (Inertial.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.clearScreen();
}