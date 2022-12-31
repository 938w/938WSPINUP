#include "vex.h" 
#include "pid.h"
double wheelcircumfrence = 3.25 * M_PI;
  void pid::drivepid(double target, double p, double pp, double d,
                double c, bool slow) {
    double derror = 1;
    double aerror = 1;
    double lasterror = 0;
    Leftside.resetPosition();
    Rightside.resetPosition();
    double currentyaw = c;
    while (fabs(derror) > 0.1 || fabs(aerror) > 1) {

      double cdistance =
          ((Leftside.position(deg) + Rightside.position(deg)) / 2) * wheelcircumfrence * 0.6 / 360;;
      derror = target - cdistance;
      double derivative = derror - lasterror;
      double speed = derror * p + derivative * d;
      aerror = currentyaw - Inertial.yaw();
      double anglespeed = aerror * pp;
      printf("%f, %f\n", derror, cdistance);
      if (slow) {
        if (speed > 60) {
          speed = 60;
        }
      }
      Leftside.spin(forward, 0.5 + speed + anglespeed, pct);
      Rightside.spin(forward, 0.5 + speed - anglespeed, pct);
      lasterror = derror;
    }
    Drivetrain.stop(hold);
  }
  void pid::driveturn(double target, double p, double d) {
    double error = 1;
    double lasterror = 0;
    double totalerror = 0;
    Leftside.resetPosition();
    Rightside.resetPosition();
    Inertial.resetRotation();
    while (fabs(error) > 0.2) {
      error = target - Inertial.yaw();
      double derivative = error - lasterror;
      double speed = error * p + derivative * d;
      totalerror += error;
      printf("%f\n", error);
      Leftside.spin(forward, speed, pct);
      Rightside.spin(reverse, speed, pct);
      lasterror = error;
    }
    Drivetrain.stop(hold);
  }
int intakeoutake() {
  Intake.spin(reverse);
  wait(100, msec);
  Intake.spin(forward);
  wait(600, msec);
  Intake.spin(reverse);
  vex::this_thread::sleep_for(5);
  return 0;
}