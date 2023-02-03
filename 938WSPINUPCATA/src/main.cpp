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
// LeftF                motor         9
// RightF               motor         10
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "button.h"
#include "pid.h"
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
  RightM.setVelocity((100), percent);
  LeftB.setVelocity((100), percent);
  RightF.setVelocity((100), percent);
  LeftF.setVelocity((100), percent);
  RightB.setVelocity((100), percent);
  Rotation3.resetPosition();
}
// ----- Definition of Variables and Catapult Functions -----
bool iscatagoingdown = false;
int autonToRun = 0;
void setCata() {
  iscatagoingdown = true;
  Rotation3.resetPosition();
  while (!LimitSwitchC.pressing()) {
    Catapult.spin(forward, 90, rpm);
    vex::this_thread::sleep_for(1);
  }
  Catapult.stop();
  iscatagoingdown = false;
}
void launchCata() {
  Catapult.setStopping(hold);
  Catapult.spinFor(forward, 80, degrees);
  wait(0.1, sec);
  setCata();
}
// ----- End of Definition of Variables and Catapult Functions -----
void autonomous(void) {
  timer t_auton;
  vex::thread o(odomthread);
  Rightside.resetPosition();
  Leftside.resetPosition();
  boost.set(false);
  endgame.set(false);
  // we do stuff here
  Drivetrain.setStopping(hold);
  pid drive;
  if (autonToRun == 0 || autonToRun == 2) {
    // ----- Left Side Win Point -----
    //  We get two Rollers and shoot two sets of discs
    //  X axis <->
    //  Y axis up and down
    // if angle is 69, it doesnt turn to the angle at the end.
    // if robot start ocillating then increase last val, if robot is slow then decrease last val
    // -----  Left Side WP Code  -----
    // Spinning the roller to the right color
    //
    Drivetrain.driveFor(forward, 1, inches, 600, rpm);
    Intake.spin(reverse);
    wait(0.2, sec);
    Intake.stop(hold);

    printf("[roller1] current time: %f seconds.\n", t_auton.time(sec));

    Drivetrain.driveFor(reverse, 5, inches, 600, rpm);
    drive.driveturn(-85, 0.7, 0.48);
    pursuit(-29, -13, 69, 2.5, 15, 5, true);
    pursuit(-44, -52, -44, 1.68, 10, 3.7, false);
    wait(0.1, sec);
    vex::thread w(launchCata);
    wait(0.2, sec);

    printf("[shot1] current time: %f seconds.\n", t_auton.time(sec));

    drive.driveturn(-135, 0.7, 0.48);
    Intake.spin(forward);
    pursuit(-74, -80, -65, 1.68, 20, 4, false);
    Intake.stop();
    wait(0.1, sec);
    vex::thread m(launchCata);
    wait(0.2, sec);

    printf("[shot2] current time: %f seconds.\n", t_auton.time(sec));

    drive.driveturn(-145, 0.63, 0.28);
    pursuit(-100, -105, -90.5, 1.68, 20, 4.2, false);
    Drivetrain.drive(forward);
    wait(0.3, sec);
    Drivetrain.stop(hold);
    Intake.spin(reverse);
    wait(0.25, sec);
    Intake.stop(hold);
    printf("[roller2] current time: %f seconds.\n", t_auton.time(sec));

    t_auton.clear();
  }
  if (autonToRun == 1 || autonToRun == 3) {
    
    Drivetrain.driveFor(1, inches, 500, rpm);
    Intake.spin(reverse);
    wait(0.2, sec);
    Intake.stop(hold);

    printf("[roller1] current time: %f seconds.\n", t_auton.time(sec));

    Drivetrain.driveFor(reverse, 5, inches, 600, rpm);
  }
  
  if (autonToRun == 4) {
    colour = vex::red;
    
    
    Drivetrain.drive(forward);
    wait(0.2, sec);
    Drivetrain.stop(hold);
    spinroller();
    drive.driveturn(135, 0.64, 0.26);
       Intake.spin(forward);
    pursuit(20, -20, 90, 1.6, 15, 4.2, false);
    Drivetrain.drive(forward);
    wait(0.3, sec);
    Drivetrain.stop(hold);
    spinroller();
 
    pursuit(10, -20, 0, 1.6, 15, 4, false, true);
    pursuit(15, -70, 69, 1.6, 15, 4.2, false, true);
    thread c(launchCata);  
    wait (0.2, sec);
    pursuit(15, 36, 69, 1.6, 15, 4.2, false);  
  }
}
  


// Driver Control ZONE!!!!!!!!!!!
//______________________________________________________________________________________________________________________________________
// Catapult Threads and Functions
void rollerthread() { vex::thread l(spinroller); }
int cataThread() {
  Controller1.ButtonDown.pressed(setCata);
  Controller1.ButtonL1.pressed(launchCata);
  vex::this_thread::sleep_for(5);
  return (0);
}
//_______________________________________________________________________________________________________________________________________
// User Control
void usercontrol(void) {
  boost.set(true);
  vex::thread o(odomthread);
  // User control code here, inside the loop
  Intake.setVelocity(100, percent);
  Catapult.setVelocity(100, percent);
  LeftM.setVelocity((100), percent);
  RightM.setVelocity((100), percent);
  Rotation3.resetPosition();
  Drivetrain.setStopping(coast);
  if (autonToRun == 0 || autonToRun == 1) {
    colour = red;
  }
  if (autonToRun == 2 || autonToRun == 3) {
    colour = blue;
  }
  while (1) {
    // Rotation Sensor Display(should be zero when the cata is up)
    if (!iscatagoingdown) {
      if (Controller1.ButtonR2.pressing()) {
        Intake.spin(reverse, 400, rpm);
      } else if (Controller1.ButtonL2.pressing()) {
        Intake.spin(forward);
      } else {
        Intake.stop();
      }
    } else {
      Intake.stop();
    }
    if (Controller1.ButtonB.pressing() && Controller1.ButtonLeft.pressing()) {
      endgame.set(true);
    } else {
      endgame.set(false);
    }
    Controller1.ButtonR1.pressed(rollerthread);
    // Drive Code
    RightM.spin(forward, Controller1.Axis2.position(), percent);
    LeftM.spin(forward, Controller1.Axis3.position(), percent);
    RightB.spin(forward, Controller1.Axis2.position(), percent);
    LeftB.spin(forward, Controller1.Axis3.position(), percent);
    RightF.spin(forward, Controller1.Axis2.position(), percent);
    LeftF.spin(forward, Controller1.Axis3.position(), percent);
    vex::thread t(cataThread);
    wait(5, msec);
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
  while (true) {
    wait(100, msec);
  }
}
