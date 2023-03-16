#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor Catapult = motor(PORT1, ratio36_1, true);
limit CataLimit = limit(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);
motor Intake = motor(PORT8, ratio6_1, false);
motor BR = motor(PORT2, ratio6_1, false);
motor OR = motor(PORT3, ratio6_1, false);
motor TR = motor(PORT4, ratio6_1, true);
motor OL = motor(PORT5, ratio6_1, true);
motor BL = motor(PORT6, ratio6_1, true);
motor TL = motor(PORT7, ratio6_1, false);
motor_group Leftside = motor_group(OL, BL, TL);
motor_group Rightside = motor_group(BR, OR, TR);
digital_out Endgame = digital_out(Brain.ThreeWirePort.C);
digital_out IntakeP = digital_out(Brain.ThreeWirePort.B);
rotation Rotation19 = rotation(PORT19, false);
rotation Rotation20 = rotation(PORT20, true);
inertial Inertial9 = inertial(PORT9);
distance Distance21 = distance(PORT21);
smartdrive Drivetrain = smartdrive(Leftside, Rightside, Inertial9, 300, 320, 130, distanceUnits::mm, 0.6);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  Inertial9.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (Inertial9.isCalibrating()) {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
 
}