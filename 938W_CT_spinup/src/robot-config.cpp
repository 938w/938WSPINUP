#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FrontLeft = motor(PORT1, ratio18_1, true);
motor LeftBack = motor(PORT2, ratio18_1, true);
motor RightFront = motor(PORT3, ratio18_1, false);
motor RightBack = motor(PORT20, ratio18_1, false);
motor_group left1 = motor_group(FrontLeft, LeftBack);
motor_group right1 = motor_group(RightFront, RightBack);
inertial Inertial = inertial(PORT11);
smartdrive maindrive = smartdrive(left1, right1, Inertial, 320, 320, 130, distanceUnits::mm, 1.4);
motor Flywheel1 = motor(PORT7, ratio18_1, false);
motor FlywheelReversed = motor(PORT6, ratio18_1, true);
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
motor INtakeMotorA = motor(PORT16, ratio18_1, false);
motor INtakeMotorB = motor(PORT17, ratio18_1, false);
motor_group INtake = motor_group(INtakeMotorA, INtakeMotorB);
digital_out Piston = digital_out(Brain.ThreeWirePort.A);

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