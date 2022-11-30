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
void flywheelpid (int setspeed, double P) {
  double totalerror = 0;
  double currentspeed = (Flywheel1.velocity(percent) + FlywheelReversed.velocity(percent))/2;
  double error = currentspeed-setspeed;
  double addamps = error * P * (totalerror * 1.1);
  totalerror += error;
  Flywheel1.spin(forward, setspeed/7 + addamps, vex::voltageUnits::volt);
  FlywheelReversed.spin(forward, setspeed/7 + addamps, vex::voltageUnits::volt);
}
bool Exit = false;
int flywheelloop () {
  while (1) {
  double totalerror = 0;
  double currentspeed = (Flywheel1.velocity(percent) + FlywheelReversed.velocity(percent))/2;
  double error = currentspeed-75;
  double addamps = error * 0.7 * (totalerror * 1.1);
  totalerror += error;
  Flywheel1.spin(forward, 75/7 + addamps, vex::voltageUnits::volt);
  FlywheelReversed.spin(forward, 75/7 + addamps, vex::voltageUnits::volt);
  vex::this_thread::sleep_for(10);
  if (Exit) {
    break;
  }
  }
  return (0);
}

void autonomous(void) {
  Piston.set(true);
  maindrive.setStopping(hold);
  maindrive.driveFor(forward, 2, inches, 150, rpm);
  INtake.spinFor(reverse, 300, degrees, 200, rpm);
  maindrive.driveFor(reverse, 6, inches, 150, rpm); 
  maindrive.setStopping(brake);  
  maindrive.turnFor(left, 130, degrees, 90, rpm);
  vex::thread t(flywheelloop);
  INtake.spin(forward, 200, rpm);
  maindrive.driveFor(forward, 65, inches, 50, rpm);
  maindrive.turnFor(left, 90, degrees, 70, rpm);
  maindrive.setStopping(coast);
  maindrive.driveFor(forward, 3, inches, 150, rpm);
  INtake.stop();
  Piston.set(false);
  wait(100, msec);
  Piston.set(true);
  wait(900, msec);
  Piston.set(false);
  wait(100, msec);
  Piston.set(true);
  wait(900, msec);
  Piston.set(false);
  wait(100, msec);
  Piston.set(true);
  wait(900, msec);
  Exit = true;




   // .......................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}
/*
double olom (bool yes, bool sey, bool ye) {
  static double rightturns = 0.0;
  static double leftturns = 0.0;
  static double midturns = 0.0;
  static double thetaB = 0;
  static double x = 0;
  static double y = 0;
  double runchenwu = 1.3;
  //encoder turns
  double dleft = Left.position(turns);
  double dright = Right.position(turns);
  double dmid = Middle.position(turns);
  //get number of turns in the current cycle which is equal to the whatever
  dleft -= leftturns;
  dright -= rightturns;
  dmid -= midturns;
  //store current amount of turns
  leftturns +=dleft;
  rightturns +=dright;
  // midturns += dmid
  //useformula to calculate distance or whatever
  double dl = dleft * M_PI * 3.25;
  double dr = dright * M_PI * 3.25;
  //double dm = dmid
  //double athetaB = (dl - dr)/(2*runchenwu);
  double athetaB = (dl - dr - dmid)/(3);
  //calculate distance and whatever
  double dist = (dr + dl)/2;
  double dy = dist * cos(thetaB + athetaB/2);
  double dx = dist * sin(thetaB + athetaB/2);
  //update xy values or whatever
  x += dx;
  y += dy;
  thetaB += athetaB;
  if (thetaB > 2+M_PI) {thetaB -= (2+M_PI);}
  if (thetaB < - (2*M_PI)) {thetaB += (2+M_PI);}
}*/
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
    if (Controller1.ButtonL1.pressing()) {
      state = true;
    } if (Controller1.ButtonR1.pressing()) {
      state = false;
    }
    if (state) {
        flywheelpid(SPEED, 0.7);
    } else {
      Flywheel1.stop();
      FlywheelReversed.stop();
    }
    
    vex::this_thread::sleep_for(10);
  }
  return (0);
}
bool ringlast = false;
bool ringstate = false;
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
    
    FrontLeft.spin(fwd, Controller1.Axis3.position(), percent);
  RightFront.spin(fwd, Controller1.Axis2.position(), percent);
    RightBack.spin(fwd, Controller1.Axis2.position(), percent);
      LeftBack.spin(fwd, Controller1.Axis3.position(), percent);
   if (Controller1.ButtonL2.pressing()) {
     INtake.spin(forward);
   }
   if (Controller1.ButtonR2.pressing()) {
     INtake.spin(reverse);
   }
   if(!Controller1.ButtonL2.pressing() && !Controller1.ButtonR2.pressing()) {INtake.stop();}
      if (Controller1.ButtonY.pressing()){
        Piston.set(false);
      } else {
        Piston.set(true);
      }

    if ((Controller1.ButtonUp.pressing()||Controller2.ButtonRight.pressing()) && !last) {
      last2 = true;
      SPEED = SPEED + 5;
    } else if (!Controller1.ButtonUp.pressing()||!Controller2.ButtonRight.pressing()) {
      last2 = false;
    }
    if ((Controller1.ButtonDown.pressing()||Controller2.ButtonLeft.pressing()) && !last) {
      last1 = true;
      SPEED = SPEED - 5;
    } else if (!Controller1.ButtonDown.pressing()||!Controller2.ButtonLeft.pressing()) {
      last1 = false;
    }
    if (SPEED < 0) {SPEED = 0;}
    if (SPEED > 110) {SPEED = 110;}

  Controller1.Screen.clearScreen();    Controller1.Screen.newLine();/*
    Controller1.Screen.print(Flywheel1.current(currentUnits::amp)+(FlywheelReversed.current(amp)));
    Controller1.Screen.print("AS");
    Controller1.Screen.print(Flywheel1.velocity(rpm)/2);*/
    Controller1.Screen.print(SPEED);/*
    Controller1.Screen.print(Flywheel1.temperature());*/
        // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

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