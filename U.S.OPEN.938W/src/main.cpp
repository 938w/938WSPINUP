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
int autontorun = 2;
// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after t  he V5 has been powered on and */
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
  G.set(false);
  IntakeP.set(false);
  Endgame.set(false);
  Rotation19.resetPosition();
  Rotation20.resetPosition();
  thread o(odomthread);
  odometry.reset();
  if (autontorun == 0) {
    G.set(false);
    Rightside.spin(forward, 150, rpm);
    wait(0.2, sec);
    Drivetrain.stop(hold);
    Intake.spin(reverse, 600, rpm);
    wait(0.25, sec);
    Intake.stop(hold);
    Drivetrain.driveFor(reverse, 3, inches, 400, rpm);
    wait(0.2, sec);
    Intake.spin(forward, 600, rpm);
    Leftside.spin(reverse, 600, rpm);
    Rightside.spin(reverse, 200, rpm);
    wait(0.3, sec);
    thread g(launchCata);
    wait(0.1, sec);
    Drivetrain.stop(coast);
    wait(0.05, sec);
    Drivetrain.drive(forward, 400, rpm);
    wait(0.175, sec);
    pursuit2(false, -60, -44, -30, 15, 6, 90, 50);
    wait(0.1, sec);
    Intake.spin(forward, 100, rpm);
    Drivetrain.drive(reverse, 600, rpm);
    wait(0.2, sec);
    thread m(launchCata);
    thread l(launchCata);
    wait(0.1, sec);
    Drivetrain.stop(coast);
    wait(0.05, sec);
    Drivetrain.drive(forward, 500, rpm);
    wait(0.2, sec);
    // leftside ends
    Intake.spin(forward, 600, rpm);
    pursuit2(false, -102, -88, 69, 20, 8, 90, 50);
    drive.drivepid(-8, 2.7, 0.2, 0.4);
    drive.driveturn(-56, 0.45, 0.1);
    Drivetrain.drive(reverse, 600, rpm);
    wait(0.15, sec);
    m.detach();
    thread n(launchCata);
    wait(0.15, sec);
    
    Drivetrain.stop(coast);
    Intake.stop(coast);
    wait(0.15, sec);
    Drivetrain.drive(forward);
    wait(0.3, sec);
    Drivetrain.stop();
    pursuit2(false, -133, -120, -91, 15, 10, 80, 90);
    Drivetrain.drive(forward, 400, rpm);
    wait(0.3, sec);
    Drivetrain.stop(hold);
    wait(0.1, sec);
    Intake.spin(reverse, 600, rpm);
    wait(0.3, sec);
    Intake.stop(hold);
    Drivetrain.driveFor(reverse, 3, inches, 300, rpm);
  }
  if (autontorun == 1) {
    G.set(false);
    odometry.setStarting(0, 0);
    Intake.spin(forward, 600, rpm);
    pursuit2(false, 0, 25, 69, 15, 7, 80, 50);
    drive.driveturn(-159, 0.42, 0.4);
    Drivetrain.drive(reverse, 600, rpm);
    wait(0.15, sec);
    thread n(launchCata);
    wait(0.1, sec);
    Drivetrain.stop(coast);
    wait(0.05, sec);
    Drivetrain.drive(forward);
    wait(0.3, sec);
    Drivetrain.stop(coast);
    pursuit2(true, -23, 4, -10, 8, 20, 80, 80);
    Intake.spin(forward, 600, rpm);
    drive.drivepid(49, 2.9, 0.05, 0.1, 0, 25, 2500);
    Intake.spin(reverse, 300, rpm);
    Drivetrain.driveFor(forward, 6, inches);
    Intake.stop();
    drive.driveturn(-137, 0.42, 0.2);
    Intake.spin(forward);
    Drivetrain.drive(reverse, 600, rpm);
    wait(0.3, sec);
    thread q(launchCata);
    wait(0.05, sec);
    Drivetrain.stop(coast);
    wait(0.1, sec);
    Drivetrain.drive(forward);
    wait(0.3, sec);
    Drivetrain.stop(coast);
    Intake.spin(forward, 600, rpm);
    wait(0.2, sec);
    pursuit2(false, 19, 13, 180, 20, 8, 80, 90, true);
    Intake.stop();
     Drivetrain.drive(forward, 300, rpm);
    wait(0.3, sec);
    Drivetrain.stop(hold);
    wait(0.1, sec);
    Intake.spin(reverse, 600, rpm);
    wait(0.4, sec);
    Intake.stop(hold);
    Drivetrain.driveFor(reverse, 3, inches, 300, rpm);
  }
  if (autontorun == 2) {
    G.set(false);
    Rightside.spin(forward, 150, rpm);
    wait(0.2, sec);
    Drivetrain.stop(hold);
    Intake.spin(reverse, 600, rpm);
    wait(0.25, sec);
    Intake.stop(hold);
    Drivetrain.driveFor(reverse, 3, inches, 400, rpm);
    wait(0.2, sec);
    Intake.spin(forward, 600, rpm);
    Leftside.spin(reverse, 600, rpm);
    Rightside.spin(reverse, 200, rpm);
    wait(0.3, sec);
    thread g(launchCata);
    wait(0.1, sec);
    Drivetrain.stop(coast);
    wait(0.05, sec);
    Drivetrain.drive(forward, 400, rpm);
    wait(0.175, sec);
    pursuit2(false, -60, -44, -32, 15, 6, 90, 50);
    wait(0.1, sec);
    Intake.spin(forward, 100, rpm);
    Drivetrain.drive(reverse, 600, rpm);
    wait(0.2, sec);
    thread m(launchCata);
    thread l(launchCata);
    wait(0.1, sec);
    Drivetrain.stop(coast);
    wait(0.05, sec);
    Drivetrain.drive(forward, 500, rpm);
    wait(0.2, sec);
    pursuit2(false, -84, -14, 190, 16, 8, 90, 90);
    // leftside ends
    drive.drivepid(49, 2.9, 0.05, 0.1, 180, 25, 2500);
    Intake.spin(reverse, 300, rpm);
    Drivetrain.driveFor(forward, 7, inches);
    Intake.stop();
    drive.driveturn(-41, 0.42, 0.2);
    Drivetrain.drive(reverse, 600, rpm);
    wait(0.3, sec);
    thread q(launchCata);
    wait(0.05, sec);
    Drivetrain.stop(coast);
    wait(0.1, sec);
    Drivetrain.drive(forward);
    wait(0.3, sec);
    Drivetrain.stop(coast);
  }
  if (autontorun == 4) {
    Rightside.spin(forward, 150, rpm);
    wait(0.2, sec);
    Drivetrain.stop(hold);
    Intake.spin(reverse, 600, rpm);
    wait(0.5, sec);
    Intake.stop(hold);
    Drivetrain.driveFor(reverse, 5, inches, 300, rpm);
    Intake.spin(forward, 600, rpm);
    pursuit2(false, -8, -14, 90, 20, 7, 90, 70);
    wait(0.1, sec);
    Intake.stop(coast);
    Drivetrain.drive(forward, 300, rpm);
    wait(0.3, sec);
    Drivetrain.stop(hold);
    Intake.spin(reverse, 600, rpm);
    wait(0.5, sec);
    Intake.stop();
    Drivetrain.driveFor(reverse, 5, inches, 300, rpm);
    Intake.spin(forward, 200, rpm);
    pursuit2(true, -78, -6, 85, 7, 20, 90, 90);
    thread t(launchCata);
    wait(0.2, sec);
    pursuit2(false, -75, -4, 0, 7, 20, 90, 90);
    Drivetrain.drive(forward, 80, rpm);
    wait(0.6, sec);
    Drivetrain.stop();
    Intake.spin(reverse, 200, rpm);
    wait(1.5, sec);
    Intake.spin(forward, 600, rpm);
    Drivetrain.driveFor(reverse, 6, inches);
    pursuit2(true, -78, -8, 90, 7, 20, 90, 90);
    thread r(launchCata);
    wait(0.2, sec);
    pursuit2(false, -74, -4, 0, 7, 20, 90, 90);
    Drivetrain.drive(forward, 80, rpm);
    wait(0.6, sec);
    Drivetrain.stop();
    Intake.spin(reverse, 200, rpm);
    wait(1.5, sec);
    Intake.spin(forward, 600, rpm);
    Drivetrain.driveFor(reverse, 6, inches);
    pursuit2(true, -78, -8, 90, 8, 20, 80, 90);
    thread p(launchCata);
    wait(0.2, sec);
    Intake.spin(forward, 600, rpm);
    pursuit2(false, -58, -28, 69, 7, 20, 110, 35);
    pursuit2(true, -82, -7, 90, 20, 7, 90, 90);
    thread i(launchCata);
    wait(0.2, sec);
    drive.driveturn(-155, 0.4, 0.2);
    Drivetrain.drive(forward, 150, rpm);
    wait(0.3, sec);
    Drivetrain.stop(coast);
    Intake.spin(forward, 600, rpm);
    pursuit2(false, -93, -47, -220, 20, 7, 90, 25);
    pursuit2(false, -92.5, -49, -220, 20, 7, 90, 25);
    Drivetrain.drive(reverse, 300, rpm);
    wait(0.2, sec);
    Drivetrain.stop(coast);
    thread n(launchCata);
    wait(0.2, sec);
    Drivetrain.driveFor(forward, 8, inches);
    drive.driveturn(-69, 0.4, 0.2);
    Drivetrain.drive(forward, 150, rpm);
    wait(0.3, sec);
    Drivetrain.stop(coast);
    pursuit2(false, -135, -52, 182, 20, 7, 90, 25);
    thread w(launchCata);
    wait(0.2, sec);
    pursuit2(false, -139, -97, 270, 20, 7, 90, 90, true);
    Intake.stop();
    Drivetrain.drive(forward, 150, rpm);
    wait(0.35, sec);
    Drivetrain.stop(hold);
    Intake.spin(reverse, 600, rpm);
    wait(0.5, sec);
    Intake.stop();
  }
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
  Intake.setVelocity(100, pct);
  thread o(odomthread);
  odometry.reset();
  G.set(false);
  // User control code here, inside the loop
  IntakeP.set(false);
  Endgame.set(false);
  Drivetrain.stop(coast);
  while (1) {
    thread m(catatask);
    if (!CataLimit.pressing()) {
      Intake.stop();
    } else {
      if (Controller1.ButtonR2.pressing()) {
        Intake.spin(reverse, 400, rpm);
      } else if (Controller1.ButtonL2.pressing()) {
        Intake.spin(forward);
      } else {
        Intake.stop();
      }
    }
    if (Controller1.ButtonB.pressing() && Controller1.ButtonLeft.pressing()) {
      Endgame.set(true);
    } else {
      Endgame.set(false);
    }
    if (Controller1.ButtonUp.pressing()) {
      G.set(true);
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
