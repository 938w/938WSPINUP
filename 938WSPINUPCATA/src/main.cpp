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

#include "vex.h"
#include "pid.h"
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
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

bool iscatagoingdown = false;
void setCata() {
  iscatagoingdown = true;
  Rotation3.resetPosition();
  while (Rotation3.position(degrees) < 62) {
    vex::this_thread::sleep_for(1);
    Catapult.spin(forward);
  }

  while (Rotation3.position(degrees) < 71.5) {
    Catapult.spin(forward, 5 + (71.5 - Rotation3.position(degrees)) * 1, pct);
  }
  Catapult.stop();
  iscatagoingdown = false;
}
void launchCata() {
  Catapult.spinFor(forward, 80, degrees);
  wait(0.1, sec);
  setCata();
}

// at+uton staorwjeiorjoiitinnsi here
void autonomous(void) {
  // we do stuff here
  Drivetrain.setStopping(hold);
  pid drive;
  // when the actual code starts
  Intake.spin(reverse, 550, rpm);
  Drivetrain.driveFor(forward, 1, inches, 600, rpm);
  wait(0.25, sec);
  drive.drivepid(-5, 7, 1, 0, 0);
  //turn towards first disk
  drive.driveturn(135, 0.7, 0.6);
  Intake.spin(forward, 600, rpm);
  drive.drivepid(24, 4.5, 1, 0.2, Inertial.yaw(), true);
  //autofix intake
  vex::thread t(intakeoutake);
  drive.driveturn(90, 0.76, 0.2);
  //drive into first roller
  drive.drivepid(7.5, 7, 1, 0);
  wait(0.26, sec);
  drive.drivepid(-5, 7.1, 1, 0.1, true);
  //turn towards high goal
  drive.driveturn(0, 0.8, 0.5);
  //stop thread
  vex::thread interrupt(t);
  Intake.stop();
  //wait for accuracy
  wait(50, msec);
  //drive to high goal and shoot disks
  drive.drivepid(-58, 6, 1, 0.1, 0, true);
  drive.driveturn(8, 1, 0.1);
  vex::thread r(launchCata);
  wait(1, sec);
  //turn and drive to next set
  drive.driveturn(-2, 1, 0.1);
  drive.drivepid(47, 7, 1, 0.1, -2, true);
  //intake diagonal row of discs
  Intake.spin(forward, 100, pct);
  drive.driveturn(-135, 0.8, 0.4);
  wait(0.1, sec);
  drive.drivepid(75, 7, 1, 0.1, -135, true);
  Intake.spin(reverse);
  //turn and drive towards goal
  drive.driveturn(0, 1, 0.5);
  drive.drivepid(-24, 6, 1, 0.5, 0);
  drive.driveturn(-90, 0.8, 0.5);
  Intake.stop();
  //shoot second set of disks
  vex::thread c(launchCata);

}

// Driver Control ZONE!!!!!!!!!!!
//______________________________________________________________________________________________________________________________________
// Catapult Threads and Functions

int cataThread() {
  vex::this_thread::sleep_for(5);
  Controller1.ButtonDown.pressed(setCata);
  Controller1.ButtonL1.pressed(launchCata);
  return (0);
}
void usercontrol(void) {
  // User control code here, inside the loop
  Intake.setVelocity(100, percent);
  Catapult.setVelocity(100, percent);
  LeftM.setVelocity((100), percent);
  RightM.setVelocity((100), percent);
  Rotation3.resetPosition();
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

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
