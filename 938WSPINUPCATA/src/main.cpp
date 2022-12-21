/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Catapult             motor         1               
// Intake               motor         2               
// Controller1          controller                    
// Rotation3            rotation      3               
// RightM               motor         4               
// LeftM                motor         6               
// RightB               motor         8               
// LeftB                motor         7               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Intake.setVelocity(100, percent);
  Catapult.setVelocity(100, percent);
  LeftM.setVelocity((100), percent);
  RightM.setVelocity((100), percent);LeftM.setVelocity((100), percent);
  Rotation3.resetPosition();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void autonomous(void) {

}

//Driver Control ZONE!!!!!!!!!!! 
//______________________________________________________________________________________________________________________________________
//Catapult Threads and Functions
void setCata () {
  Rotation3.resetPosition();
  while (Rotation3.position(degrees) < 62) {
    vex::this_thread::sleep_for(1);
    Catapult.spin(forward); 
  }
  while (Rotation3.position(degrees) < 74) {
 
    Catapult.spin(forward, 50, pct);
  }
  Catapult.stop();
}
void launchCata () {
  Catapult.spinFor(forward, 90, degrees);
  vex::this_thread::sleep_for(900);
  setCata();
}
int cataThread () {
  Controller1.ButtonDown.pressed(setCata);
  Controller1.ButtonR1.pressed(launchCata);
  return (0);
}
void usercontrol(void) {
  // User control code here, inside the loop
  Intake.setVelocity(100, percent);
  Catapult.setVelocity(100, percent);
  LeftM.setVelocity((100), percent);
  RightM.setVelocity((100), percent);LeftM.setVelocity((100), percent);
  Rotation3.resetPosition();
  while (1) {
    //Rotation Sensor Display(should be zero when the cata is up)
      Controller1.Screen.newLine();
      Controller1.Screen.print(Rotation3.position(degrees));
    //Intake Code
    if(Controller1.ButtonL2.pressing()) {
      Intake.spin(forward);
    } else if (Controller1.ButtonL1.pressing()) {
      Intake.spin(reverse);
    } else {
      Intake.stop();
    }
    //Drive Code
      RightM.spin(forward, Controller1.Axis2.position(), percent);
      LeftM.spin(forward, Controller1.Axis3.position(), percent);
      RightB.spin(forward, Controller1.Axis2.position(), percent);
      LeftB.spin(forward, Controller1.Axis3.position(), percent);
    vex::thread t(cataThread);
    wait(10, msec);
  }
}
// END OF DRIVE CONTROL ZONE!!!!!
//_______________________________________________________________________________________________________________________________________
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
