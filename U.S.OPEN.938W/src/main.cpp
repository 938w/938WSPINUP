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
// CataLimit            limit         A
// Controller1          controller
// Intake               motor         8
// BR                   motor         2
// OR                   motor         3
// TR                   motor         4
// OL                   motor         5
// BL                   motor         6
// TL                   motor         7
// Endgame              digital_out   C
// IntakeP              digital_out   B
// Rotation19           rotation      19
// Rotation20           rotation      20
// Inertial9            inertial      9
// Distance21           distance      21
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "pid_pursuit.h"
#include "robot.h"
#include "vex.h"
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!

  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
pid drive;
void autonomous(void) {
  Rotation19.resetPosition();
  Rotation20.resetPosition();
  thread o(odomthread);
  odometry.reset();
  Rightside.spin(forward, 150, rpm);
  wait(0.2, sec);
  Drivetrain.stop(hold);
  Intake.spin(reverse, 600, rpm);
  wait(0.25, sec);
  Intake.stop(hold);
  Drivetrain.driveFor(reverse, 2, inches, 400, rpm);
  pursuit2(true, -36.2, -12.5, -10.5, 7, 20, 90, 90);
  wait(0.1, sec);
  thread b(launchCata);
  wait(0.2, sec);
  IntakeP.set(true);
  pursuit2(false, -48, -21, 69, 10, 7, 90, 90);
  IntakeP.set(false);
  Intake.spin(forward, 500, rpm);
  wait(1.3, sec);
  pursuit2(false, -68, -48, -39, 20, 8, 75, 90);
  thread m(launchCata);
  wait(0.1, sec);
  Intake.spin(forward, 600, rpm);
  // leftside ends
  pursuit2(false, -108, -80, -74, 20, 8, 75, 90);
  Drivetrain.drive(reverse, 600, rpm);
  wait(0.2, sec);
  Drivetrain.stop(coast);
  thread n(launchCata);
  wait(0.1, sec);
  pursuit2(false, -124, -94, -90, 20, 8, 90, 90);
  Drivetrain.drive(forward, 600, rpm);
  wait(0.2, sec);
  Drivetrain.stop(hold);
  Intake.spin(reverse);
  wait(0.25, sec);
  Intake.stop(hold);

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  o.detach();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
int catatask() {
  Controller1.ButtonL1.pressed(launchCata);
  return 0;
}
int rever = 1;
int revel = 1;
void usercontrol(void) {
  // User control code here, inside the loop
  IntakeP.set(false);
  Endgame.set(false);
  thread o(odomthread);
  while (1) {
    thread m(catatask);
    if (Controller1.ButtonR2.pressing()) {
      Intake.spin(reverse, 400, rpm);
    } else if (Controller1.ButtonL2.pressing()) {
      Intake.spin(forward, 100, pct);
    } else {
      Intake.stop();
    }
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    double leftVelocity = pow(Controller1.Axis3.position() / 18, 3);
    double rightVelocity = pow(Controller1.Axis2.position() / 18, 3);
    BL.spin(forward, leftVelocity, pct);
    TL.spin(forward, leftVelocity, pct);
    OL.spin(forward, leftVelocity, pct);
    BR.spin(forward, rightVelocity, pct);
    TR.spin(forward, rightVelocity, pct);
    OR.spin(forward, rightVelocity, pct);
    wait(5, msec); // Sleep the task for a short amount of time to
                   // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
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
