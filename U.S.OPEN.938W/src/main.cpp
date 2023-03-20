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
int autontorun = 4;
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
    Drivetrain.driveFor(reverse, 2, inches, 400, rpm);
    pursuit2(true, -36.2, -15.5, -10.5, 7, 25, 90, 90);
    Drivetrain.drive(reverse, 600, rpm);
    thread b(launchCata);
    wait(0.1, sec);
    Drivetrain.stop(hold);
    wait(0.3, sec);
    Drivetrain.drive(forward, 500, rpm);
    wait(0.1, sec);
    Intake.spin(forward, 600, rpm);
    pursuit2(false, -69, -42, -30, 15, 6, 75, 50);
    b.detach();
    wait(0.1, sec);
    Drivetrain.drive(reverse, 550, rpm);
    wait(0.3, sec);
    Drivetrain.stop(coast);
    thread m(launchCata);
    Drivetrain.stop(coast);
    Drivetrain.drive(forward, 500, rpm);
    wait(0.2, sec);
    // leftside ends
    Intake.spin(forward, 600, rpm);
    pursuit2(false, -112, -88, -62, 20, 8, 70, 40);
    Drivetrain.drive(reverse, 500, rpm);
    wait(0.1, sec);
    m.detach();
    thread n(launchCata);
    wait(0.1, sec);
    Drivetrain.stop(coast);
    pursuit2(false, -146, -102, -90, 20, 8, 90, 80);
    Intake.stop(coast);
    Drivetrain.drive(forward, 600, rpm);
    wait(0.2, sec);
    Drivetrain.stop(hold);
    Intake.spin(reverse, 600, rpm);
    wait(0.2, sec);
    Intake.stop(hold);
  }
  if (autontorun == 1) {
    G.set(false);
    odometry.setStarting(0, 0);
    Intake.spin(forward, 600, rpm);
    pursuit2(false, 0, 24, 69, 15, 7, 80, 50);
    drive.driveturn(-152, 0.46, 0.4);
    Drivetrain.drive(reverse, 500, rpm);
    wait(0.2, sec);
    thread n(launchCata);
    wait(0.05, sec);
    Drivetrain.stop(brake);
    wait(0.2, sec);
    Drivetrain.drive(forward);
    wait(0.3, sec);
    Drivetrain.stop(coast);
    pursuit2(false, 24, 2, 180, 15, 7, 80, 50);
    Drivetrain.drive(forward, 600, rpm);
    wait(0.2, sec);
    Drivetrain.stop(hold);
    Intake.stop();
    wait(0.1, sec);
    Intake.spin(reverse, 600, rpm);
    wait(0.2, sec);
    Intake.stop(hold);
  }
  if (autontorun == 2) {
  }
  if (autontorun == 4) {
    G.set(true);
    Rightside.spin(forward, 150, rpm);
    wait(0.2, sec);
    Drivetrain.stop(hold);
    Intake.spin(reverse, 600, rpm);
    wait(0.5, sec);
    Intake.stop(hold);
    Drivetrain.driveFor(reverse, 3, inches, 400, rpm);
    Intake.spin(forward, 600, rpm);
    pursuit2(false, -9, -12, 90, 20, 7, 80, 70);
    wait(0.1, sec);
    Intake.stop(coast);
    Drivetrain.drive(forward, 400, rpm);
    wait(0.3, sec);
    Drivetrain.stop(hold);
    Intake.spin(reverse, 600, rpm);
    wait(0.5, sec);
    Intake.stop(hold);
    Drivetrain.driveFor(reverse, 5, inches, 400, rpm);
    wait(0.1, sec);
    Drivetrain.stop(coast);
    Intake.spin(forward, 300, rpm);
    pursuit2(true, -6, -75, 8, 7, 20, 80, 80);
    thread l(launchCata);
    pursuit2(false, -8, -60, 69, 20, 7, 80, 80);
    Intake.spin(forward, 600, rpm);
    pursuit2(false, -60, -95, -52, 20, 7, 80, 50);
    Drivetrain.drive(reverse, 500, rpm);
    wait(0.24, sec);
    thread m(launchCata);
    wait(0.05, sec);
    Drivetrain.stop(coast);
    wait(0.4, sec);
    Drivetrain.drive(forward, 500, rpm);
    wait(0.2, sec);
    Drivetrain.stop(coast);
    wait(0.1, sec);
    Intake.spin(forward, 600, rpm);
    pursuit2(false, -108, -120, 69, 16, 7, 80, 50);
    pursuit2(true, -65, -130, -96, 7, 16, 80, 60);
    Intake.spin(forward, 600, rpm);
    wait(0.1, sec);
    thread p(launchCata);
    pursuit2(false, -108, -134, 180, 8, 8, 50, 50);
    Drivetrain.drive(forward, 400, rpm);
    wait(0.5, sec);
    Drivetrain.stop(hold);
    Intake.spin(reverse, 600, rpm);
    wait(0.5, sec);
    Intake.stop(hold);
    Drivetrain.driveFor(reverse, 5, inches, 400, rpm);
    pursuit2(false, -135, -134, -135, 8, 8, 60, 60);

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
  thread o(odomthread);
  odometry.reset();
  G.set(true);
  // User control code here, inside the loop
  IntakeP.set(false);
  Endgame.set(false);
  Drivetrain.stop(coast);
  while (1) {
    thread m(catatask);
    if (Controller1.ButtonR2.pressing()) {
      Intake.spin(reverse, 400, rpm);
    } else if (Controller1.ButtonL2.pressing()) {
      Intake.spin(forward, 100, pct);
    } else {
      Intake.stop();
    }
    if (Controller1.ButtonB.pressing() && Controller1.ButtonLeft.pressing()) {
      Endgame.set(true);
    } else {
      Endgame.set(false);
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
