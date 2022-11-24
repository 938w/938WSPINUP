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
// FrontLeft            motor         1               
// LeftBack             motor         2               
// RightFront           motor         3               
// RightBack            motor         20              
// Flywheel1            motor         7               
// FlywheelReversed     motor         6               
// Controller1          controller                    
// Controller2          controller                    
// INtake               motor_group   16, 17          
// Piston               digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----

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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

void flywheelpid (int setspeed, double P) {
  double totalerror = 0;
  double currentspeed = (Flywheel1.velocity(percent) + FlywheelReversed.velocity(percent))/2;
  double error = currentspeed-setspeed;
  double addamps = error * P * (totalerror * 1.1);
  totalerror += error;
  Flywheel1.spin(forward, setspeed/7 + addamps, vex::voltageUnits::volt);
  FlywheelReversed.spin(forward, setspeed/7 + addamps, vex::voltageUnits::volt);
  wait(5, msec);

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
bool last = false;
bool state = false;
int SPEED = 75;
bool last1 = false;
bool last2 = false;
int myThread() {
  while (true) {
  if ((Controller1.ButtonL1.pressing()||Controller2.ButtonL1.pressing()) && !last) {
      last = true;
      state = !state;
    } else if (!Controller1.ButtonL1.pressing()||!Controller2.ButtonL1.pressing()) {
      last = false;
    }
    if (state) {
      if (SPEED < 100) {
      flywheelpid(SPEED, 0.7);
      } else {
        Flywheel1.spin(fwd, SPEED, pct);
        FlywheelReversed.spin(fwd, SPEED, pct);
      }
    } else {
      Flywheel1.stop();
      FlywheelReversed.stop();
    }
     if (SPEED < 0) {SPEED = 0;}
    if (SPEED > 110) {SPEED = 110;}
    vex::this_thread::sleep_for(10);
  }
  return (0);
}
void usercontrol(void) {
    FrontLeft.setBrake(coast);
    LeftBack.setBrake(coast);
    RightBack.setBrake(coast);
    RightFront.setBrake(coast);
    FrontLeft.setVelocity(100, pct);
    LeftBack.setVelocity(100, pct);
    RightFront.setVelocity(100, pct);
    RightBack.setVelocity(100, pct);
    INtake.setVelocity(100, pct);
  // User control code here, inside the loop
  vex::thread t(myThread);
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    
    FrontLeft.spin(reverse, Controller1.Axis3.position(), percent);
  RightFront.spin(fwd, Controller1.Axis2.position(), percent);
    RightBack.spin(fwd, Controller1.Axis2.position(), percent);
      LeftBack.spin(reverse, Controller1.Axis3.position(), percent);
      if (Controller1.ButtonL2.pressing()){
        INtake.spin(fwd);
      } 
      if (Controller1.ButtonR2.pressing()){
        INtake.spin(reverse);
      }
      if (!Controller1.ButtonL2.pressing() && !Controller1.ButtonR2.pressing()) {
        INtake.stop();
      }
      if (Controller1.ButtonY.pressing()){
        Piston.set(false);
      } else {
        Piston.set(true);
      }

    if (Controller1.ButtonUp.pressing() && !last) {
      last2 = true;
      SPEED = SPEED + 5;
    } else if (!Controller1.ButtonUp.pressing()) {
      last2 = false;
    }
    if (Controller1.ButtonDown.pressing() && !last) {
      last1 = true;
      SPEED = SPEED - 5;
    } else if (!Controller1.ButtonDown.pressing()) {
      last1 = false;
    }

   /* Controller1.Screen.clearScreen();    Controller1.Screen.newLine();
    Controller1.Screen.print(Flywheel1.current(currentUnits::amp)+(FlywheelReversed.current(amp)));
    Controller1.Screen.print("AS");
    Controller1.Screen.print(Flywheel1.velocity(rpm)/2);
    Controller1.Screen.print("SS");
    Controller1.Screen.print(SPEED);
    Controller1.Screen.print(Flywheel1.temperature());*/
        // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(10, msec); // Sleep the task for a short amount of time to
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