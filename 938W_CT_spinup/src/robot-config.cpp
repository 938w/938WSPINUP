#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor FrontLeft = motor(PORT1, ratio18_1, false);
motor LeftBack = motor(PORT2, ratio18_1, false);
motor RightFront = motor(PORT3, ratio18_1, false);
motor RightBack = motor(PORT20, ratio18_1, false);
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
  // nothing to initialize
}