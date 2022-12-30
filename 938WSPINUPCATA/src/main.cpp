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
double wheelcircumfrence = 3.25 * M_PI;
double a = wheelcircumfrence * 0.6/360;
bool iscatagoingdown = false;
void setCata () {
  iscatagoingdown = true;
  Rotation3.resetPosition();
  while (Rotation3.position(degrees) < 62) {
    vex::this_thread::sleep_for(1);
    Catapult.spin(forward); 
  }
  double error = 5;
  while (fabs(error) > 0.2) {
    error = 75.5 - Rotation3.position(degrees);
    Catapult.spin(forward, error*6, pct);
    vex::this_thread::sleep_for(1);
  }
  Catapult.spinFor(forward, 1, degrees, 100, rpm);
  Catapult.stop();
  iscatagoingdown = false;
}
void launchCata () {
  Catapult.spinFor(forward, 100, degrees);
  setCata();
}
void drivepid(double target, double p, double pp, double d = 0.05) {
  double derror =  1;
  double aerror = 1;
  double lasterror = 0;
  Leftside.resetPosition();
  Rightside.resetPosition();
  double currentyaw = Inertial.yaw();
  while (fabs(derror) > 0.1 || fabs(aerror) > 1) {
    
    double cdistance = ((Leftside.position(deg)+Rightside.position(deg))/2)*a;
    derror = target - cdistance;
    double derivative = derror - lasterror;
    double speed = derror * p + derivative * d;
    aerror = currentyaw - Inertial.yaw();
    double anglespeed = aerror * pp;
    printf("%f, %f\n", derror, cdistance);
    Leftside.spin(forward, 0.5 + speed + anglespeed, pct);
    Rightside.spin(forward, 0.5 + speed - anglespeed, pct);
    lasterror = derror;
  }
  Drivetrain.stop(hold);
}
void driveturn(double target, double p, double d) {
  double error = 1;
  double lasterror = 0;
  Leftside.resetPosition();
  Rightside.resetPosition();
  Inertial.resetRotation();
  while (fabs(error) > 0.1) {
    error = target - Inertial.yaw();
    double derivative = error - lasterror;
    double speed = error * p + derivative * d;
    printf("%f\n", error);
    Leftside.spin(forward, speed, pct);
    Rightside.spin(reverse, speed, pct);
    lasterror = error;
  }
  Drivetrain.stop(hold);
}
int intakeoutake () {
  Intake.spin(reverse);
  wait(200, msec);
  Intake.spin(forward);
  wait(300, msec);
  Intake.spin(reverse);
  vex::this_thread::sleep_for(5);
  return(0);
}
void autonomous(void) {
  Drivetrain.setStopping(hold);
  Intake.spin(reverse, 550, rpm);
  Drivetrain.driveFor(forward, 1, inches, 600, rpm);
  wait(0.25, sec);
  drivepid(-5, 7, 1, 0);
  driveturn(135, 0.725, 0.2);
  Intake.spin(forward, 600, rpm);
  drivepid(24, 3.95, 1, 0.2);
  vex::thread t(intakeoutake);
  driveturn(90, 0.75, 0.2);
  drivepid(7.9, 7, 1, 0);
  wait(0.26, sec);
  drivepid(-5, 7.1, 1, 0.1);
  driveturn(0, 0.7, 0.06);
  Intake.stop();
  drivepid(-58, 7, 1, 0.2);
  driveturn(5, 1.2, 0);
  launchCata();
  
}

//Driver Control ZONE!!!!!!!!!!! 
//______________________________________________________________________________________________________________________________________
//Catapult Threads and Functions


int cataThread () {
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
    //Rotation Sensor Display(should be zero when the cata is up)
     printf("%f\n", Rotation3.position(degrees));
    //Intake Code
    if (!iscatagoingdown) {
    if(Controller1.ButtonL2.pressing()) {
      Intake.spin(forward);
    } else if (Controller1.ButtonR2.pressing()) {
      Intake.spin(reverse);
    } else {
      Intake.stop();
    }
    } else {
      Intake.stop();
    }
    //Drive Code
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
