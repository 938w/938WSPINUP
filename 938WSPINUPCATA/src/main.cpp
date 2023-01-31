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
#include "ogeometry.h"
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
  timer t1;
  while (!(Optical1.color() == red)) {
    Intake.spin(reverse, 500, rpm);
    vex::this_thread::sleep_for(1);
    if (Controller1.ButtonX.pressing()){
      break;
    }
    if(t1.time(sec) > 3) {
      break;
    }
  }
  while (!(Optical1.color() == red) ){
    Intake.spin(forward, 500, rpm);
    vex::this_thread::sleep_for(1);
    if (Controller1.ButtonX.pressing()){
      break;
    }
    if(t1.time(sec) > 3) {
      break;
    }
  }
  Intake.stop(hold);
  t1.clear();
  return 0;
}
bool iscatagoingdown = false;
void setCata() {
  iscatagoingdown = true;
  Rotation3.resetPosition();
  while(!LimitSwitchC.pressing()) {
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


// at+uton staorwjeiorjoiitinnsi here
void autonomous(void) {
  vex::thread o(odomthread);
  Rightside.resetPosition();
  Leftside.resetPosition();
  boost.set(false);
  endgame.set(false);
  // we do stuff here
  Drivetrain.setStopping(hold);
  pid drive;  
  Drivetrain.driveFor(forward, 1, inches, 600, rpm);
  Intake.spin(reverse);
  wait(0.2, sec);
  Intake.stop(hold);
  Drivetrain.driveFor(reverse, 5, inches,  600, rpm);
  drive.driveturn(-85, 0.7, 0.48);
  pursuit(-34, -18, 69, 2.5, 15, 5, true);
  pursuit(-44, -52, -44, 1.72, 10, 2.8, false);
  wait(0.2, sec);
  vex::thread w(launchCata);
  wait(1, sec);
  wait(0.1, sec);
  drive.driveturn(-135, 0.7, 0.48);
  Intake.spin(forward);
  pursuit(-74, -78, -64, 1.72, 20, 3.6, false);
  Intake.stop();
  wait(0.2, sec);
  vex::thread m(launchCata);
  wait(0.5, sec);
  drive.driveturn(-150, 0.62, 0.26);
  drive.drivepid(38, 4, 0.35, 1.2, -135, 60);
  drive.driveturn(-90, 0.62, 0.26);
  /*
  // when the actual code starts
  Drivetrain.driveFor(forward, 1, inch70es, 500, rpm);
  spinroller();
  drive.drivepid(-5, 7, 1, 0.1, 0);
  
  //turn towards first disk
  drive.driveturn(135, 0.62, 0.26);
  Intake.spin(forward, 600, rpm);
  drive.drivepid(23, 4.5, 1, 0.2, Inertial.yaw(), 40);
  //autofix intake
  drive.driveturn(89, 0.62, 0.26);
  wait(0.1, sec);
  //drive into first roller
  drive.drivepid(6.5, 7, 1, 0);
  spinroller();
  drive.drivepid(-5.5, 7.1, 1, 0.1, Inertial.yaw(), 60);
  //turn towards high goal
  drive.driveturn(0, 0.62, 0.26);
  //stop thread

  Intake.stop();
  //wait for accuracy
  wait(200, msec);
  //drive to high goal and shoot disks
  vex::thread a(intakeoutake);
  drive.drivepid(-57, 4, 1.2, 1.2, 0, 80);
  drive.driveturn(6, 0.8, 0.1);
  Intake.stop();
  vex::thread r(launchCata);
  wait(0.8, sec);
  //turn and drive to next set
  drive.driveturn(-2, 0.6, 0.3);
  drive.drivepid(45, 3.9, 0.35, 1.2, -2, 70);
  //intake diagonal row of discs
  Intake.spin(forward, 100, pct);
  drive.driveturn(-135, 0.62, 0.5);
  wait(0.1,sec);
  drive.drivepid(71, 3.9, 1, 1.2, -135, 60);
  //turn and drive towards goal
  drive.driveturn(0, 0.62, 0.5);
  wait(0.1,sec);
  drive.drivepid(-38, 4, 0.6, 1.2, 0, 60);
  drive.driveturn(-90, 0.62, 0.26);
  drive.drivepid(-10, 4, 0.4, 1.2, -90);
  Intake.stop();
  //shoot second set of disks
  drive.driveturn(-95, 0.8, 0.5);
  vex::thread c(launchCata);
  wait(0.8, sec);
  drive.drivepid(5, 4, 0.8, 1.2, -90);
  drive.driveturn(-45, 0.62, 0.49);
  //intake next set
  Intake.spin(forward, 500, rpm);
  drive.drivepid(37, 7, 0.8, 0.1, -45, 20);
  drive.drivepid(-36, 4, 0.8, 1.2, -45);
  vex::thread t(intakeoutake);
  drive.driveturn(-90, 0.62, 0.5);
  drive.drivepid(-2, 7, 0.6, 0.5, -90);
  Intake.stop();
  vex::thread l(launchCata);
  wait(0.8, sec);
  drive.drivepid(60, 4, 0.6, 1.2, -90, 70);
  drive.driveturn(-135, 0.62, 0.28);
  drive.drivepid(-9, 7, 0.4, 0.1, -135);
  drive.driveturn(180, 0.8, 0.26);
  wait(0.1, sec);
  Drivetrain.drive(forward);
  wait(0.3, sec);
  Drivetrain.stop(hold);
  spinroller();
  drive.drivepid(-5, 7, 0.4, 0.1, 180);
  drive.driveturn(-45, 0.62, 0.26);
  Intake.spin(forward, 600, rpm);
  drive.drivepid(21, 4.5, 1, 0.2, Inertial.yaw(), 40);
  drive.driveturn(-90, 0.62, 0.26);
  drive.drivepid(6.5, 7, 1, -90);
  spinroller();
  drive.drivepid(-18, 7.1, 1, 0.1, Inertial.yaw(), 60);
  drive.driveturn(-135, 0.62, 0.28);
  drive.drivepid(9, 7, 0.4, 0.1, -135);
 */
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
  vex::thread o(odomthread);
  // User control code here, inside the loop
  Intake.setVelocity(100, percent);
  Catapult.setVelocity(100, percent);
  LeftM.setVelocity((100), percent);
  RightM.setVelocity((100), percent);
  Rotation3.resetPosition();
  Drivetrain.setStopping(coast);
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
    /*
    position odom = odomoutputs();
    Controller1.Screen.newLine();
    Controller1.Screen.print("%f, %f", odom.x, odom.y);*/
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
