#include "pid.h"
#include "ogeometry.h"
#include "vex.h"
static double wheelcircumfrence = 3.25 * M_PI;
void pid::drivepid(double target, double p, double pp, double d, double c,
                   double maxvelocity) {
  double derror = 1;
  double aerror = 1;
  double lasterror = 0;
  double base = 0;
  Leftside.resetPosition();
  Rightside.resetPosition();
  double currentyaw = c;

  while (fabs(derror) > 0.1 || fabs(aerror) > 0.5) {
    double nowyaw = Inertial.yaw();
    double cdistance =
        ((Leftside.position(deg) + Rightside.position(deg)) / 2) *
        wheelcircumfrence * 0.6 / 360;
    derror = target - cdistance;
    double derivative = derror - lasterror;
    double speed = derror * p + derivative * d;

    double anglespeed = aerror * pp;
    printf("%f, %f\n", derror, cdistance);
    if (speed > maxvelocity) {
      speed = maxvelocity;
    }
    if (cdistance > target) {
      base = -1;
    }
    if (cdistance < target) {
      base = 1;
    }
    if (c == 180) {
      if (Inertial.yaw() < 0) {
        nowyaw = 180 + (180 + Inertial.yaw());
      }
    }
    aerror = currentyaw - nowyaw;
    Leftside.spin(forward, base + speed + anglespeed, pct);
    Rightside.spin(forward, base + speed - anglespeed, pct);
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
  double base = 0;

  double currentyaw;
  while (fabs(error) > 0.11) {
    currentyaw = Inertial.yaw();
    if (target == 180) {
      if (Inertial.yaw() < 0) {
        currentyaw = 180 + (180 + Inertial.yaw());
      }
    }
    error = target - currentyaw;
    double derivative = error - lasterror;
    double speed = error * p + derivative * d;
    totalerror += error;
    printf("%f\n", error);
    if (Inertial.yaw() > target) {
      base = -1;
    }
    if (Inertial.yaw() < target) {
      base = 1;
    }
    Leftside.spin(forward, base + speed, pct);
    Rightside.spin(reverse, base + speed, pct);
    lasterror = error;
  }
  Drivetrain.stop(hold);
}
int intakeoutake() {
  Intake.spin(reverse);
  vex::this_thread::sleep_for(100);
  Intake.spin(forward);
  vex::this_thread::sleep_for(5);
  return 0;
}
void pursuit(double targetX, double targetY, double targetA) {
  double base = 0;
  double xError = 100;
  double yError = 100;
  while (xError > 1 || yError > 1) {
    position odom = odomoutputs();
    xError = targetX - odom.x;
    yError = targetY - odom.y;
    Rightside.spin(forward, base, pct);
    Leftside.spin(forward, base, pct);
    this_thread::sleep_for(10);
  }
}