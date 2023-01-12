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
int addspeed = 0;
int spinroller () {
  while (Optical1.hue() < 250) {
    Intake.spin(reverse);
    vex::this_thread::sleep_for(1);
    if (Controller1.ButtonX.pressing()){
      break;
    }
  }
  while (Optical1.hue() < 250) {
    Intake.spin(forward);
    vex::this_thread::sleep_for(1);
    if (Controller1.ButtonX.pressing()){
      break;
    }
  }
  Intake.spin(forward);
  vex::this_thread::sleep_for(50);
  Intake.stop(hold);
  return 0;
}
bool iscatagoingdown = false;
void setCata() {
  iscatagoingdown = true;
  Rotation3.resetPosition();
  while (Rotation3.position(degrees) < 62) {
    vex::this_thread::sleep_for(1);
    Catapult.spin(forward);
  }
// no shot is that pid control
  while (Rotation3.position(degrees) < 70.8) {
    Catapult.spin(forward, 5 + (71 - Rotation3.position(degrees)) * 0.8, pct);
  }
  Catapult.stop();
  iscatagoingdown = false;
  
}
void launchCata() {
  Catapult.setStopping(hold);
  Catapult.spinFor(forward, 80, degrees);
  wait(0.1, sec);
  setCata();
  Catapult.spinFor(2, degrees);
}

// at+uton staorwjeiorjoiitinnsi here
void autonomous(void) {
  boost.set(true);
  // we do stuff here
  Drivetrain.setStopping(hold);
  pid drive;
  // when the actual code starts
  Drivetrain.driveFor(forward, 1, inches, 600, rpm);
  spinroller();
  drive.drivepid(-5, 7, 1, 0, 0);
  //turn towards first disk
  drive.driveturn(135, 0.7, 0.6);
  Intake.spin(forward, 600, rpm);
  drive.drivepid(23, 4.5, 1, 0.2, Inertial.yaw(), 60);
  //autofix intake
  vex::thread t(intakeoutake);
  drive.driveturn(90, 0.76, 0.2);
  //drive into first roller
  drive.drivepid(6.5, 7, 1, 0);
  spinroller();
  drive.drivepid(-7, 7.1, 1, 0.1, Inertial.yaw(), 60);
  //turn towards high goal
  drive.driveturn(0, 0.8, 0.5);
  //stop thread
  vex::thread interrupt(t);
  Intake.stop();
  //wait for accuracy
  wait(200, msec);
  //drive to high goal and shoot disks
  drive.drivepid(-54, 6, 1.2, 0.1, 0, 90);
  drive.driveturn(6, 0.8, 0.1);
  vex::thread r(launchCata);
  wait(0.9, sec);
  //turn and drive to next set
  drive.driveturn(-2, 1, 0.1);
  drive.drivepid(44, 6.5, 1, 0.1, -2, 60);
  //intake diagonal row of discs
  Intake.spin(forward, 100, pct);
  drive.driveturn(-135, 0.8, 0.4);
  drive.drivepid(70, 7, 0.8, 0.5, -135, 60);
  Intake.spin(reverse);
  //turn and drive towards goal
  drive.driveturn(0, 0.8, 0.2);
  drive.drivepid(-40, 6, 0.8, 0.5, 0, 80);
  drive.driveturn(-90, 0.8, 0.5);
  drive.drivepid(-14.5, 7, 0.8, 0.5, -90);
  Intake.stop();
  //shoot second set of disks
  drive.driveturn(-95, 0.8, 0.5);
  vex::thread c(launchCata);
  wait(0.8, sec);
  drive.drivepid(14, 7, 0.8, 0.5, -90);
  drive.driveturn(-43, 0.8, 0.2);
  //intake next set
  Intake.spin(forward);
  drive.drivepid(40, 7, 0.8, 0.1, -43, 30);
  drive.drivepid(-40, 7, 0.8, 0.1, -43);
  drive.driveturn(-90, 0.8, 0.5);
  drive.drivepid(-15, 7, 0.8, 0.5, -90);
  vex::thread l(launchCata);
  wait(0.8, sec);
  drive.drivepid(70, 7, 0.8, 0.1);
  drive.driveturn(45, 0.8, 0.5);

}

// Driver Control ZONE!!!!!!!!!!!
//______________________________________________________________________________________________________________________________________
// Catapult Threads and Functions
void rollerthread (){
  vex::thread l(spinroller);
}
int cataThread() {
  Controller1.ButtonDown.pressed(setCata);
  Controller1.ButtonL1.pressed(launchCata);
  vex::this_thread::sleep_for(5);
  return (0);
}
void usercontrol(void) {
  boost.set(true);
  // User control code here, inside the loop
  Intake.setVelocity(100, percent);
  Catapult.setVelocity(100, percent);
  LeftM.setVelocity((100), percent);
  RightM.setVelocity((100), percent);
  Rotation3.resetPosition();
  Drivetrain.setStopping(coast);
  while (1) {
    // Rotation Sensor Display(should be zero when the cata is up)
    printf("%f\n", Rotation3.position(degrees));
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
    if (Controller1.ButtonB.pressing()&&Controller1.ButtonLeft.pressing()) {
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

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
