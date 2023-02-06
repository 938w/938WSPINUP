#include "pid.h"
#include "ogeometry.h"
#include "vex.h"

// ----- Odometry Thread and Pursuit -----
//  There is the defnition of the odom thread
//  There is the definition of the pursuit
// ----- Odometry Thread and Pursuit Code;
position odom;
int odomthread() {
  Inertial.resetRotation();
  while (true) {
    // printf("%f, %f\n", odom.x, odom.y);
    odom = odometry.odomoutputs();
    vex::this_thread::sleep_for(5);
  }
  return 0;
}

void pursuit(double targetX, double targetY, double targetA, double P,
             double max, double maxd, bool constant) {
  pid ang;
  // error values
  double xError = 100;
  double yError = 100;
  double dError = 100;

  double turnVelocity = 1;
  double fwVelocity = 0;
  double previous = 0;
  double c = Inertial.yaw();
  while (abs(xError) > 0.25 || abs(yError) > 0.25 || turnVelocity > 0.1) {
    xError = targetX - odom.x;
    yError = targetY - odom.y;
    dError = sqrt((pow(xError, 2) + pow(yError, 2)));
    double tAngle = atan2(xError, yError) * (180 / M_PI);
    c = Inertial.yaw();
    turnVelocity = ((tAngle - c) * 0.7);

    fwVelocity = dError * P;
    // printf("%f\n", tAngle);
    double change = fwVelocity - previous;
    if (change > max) {
      fwVelocity -= change - max;
    }
    if (change < -maxd) {
      fwVelocity -= change + maxd;
    }
    if (constant) {
      fwVelocity = 55;
    }
    if (fwVelocity > 100) {
      fwVelocity = 100;
    }
    if (fwVelocity < 2) {
      fwVelocity = 2;
    }
    printf("%f, %f\n", tAngle, dError);
    Rightside.spin(forward, fwVelocity - turnVelocity, pct);
    Leftside.spin(forward, fwVelocity + turnVelocity, pct);
    this_thread::sleep_for(2);
    previous = Drivetrain.velocity(pct);
  }
  if (!(targetA == 69)) {
    ang.driveturn(targetA, 0.58, 0.26);
  }
  Drivetrain.stop(hold);
  printf("%f, %f\n", odom.x, odom.y);
}

// ----- PURSUIT V2 -----
static double wheelcircumfrence = 3.25 * M_PI;
void pursuit2(bool backwards, double targetX, double targetY, double endA,
              double slewUP, double slewDOWN) {
  static double p = 4;
  static double ap = 0.62;
  static double d = 1.2;
  static double ad = 0.26;
  if (!backwards) {
    double xError = targetX - odom.x;
    double yError = targetY - odom.y;
    double tAngle = atan2(xError, yError) * (180 / M_PI);
    double angleError = 100;
    double lasterror = 0;
    double previous = 0;
    while (angleError > 0.2) {
      xError = targetX - odom.x;
      yError = targetY - odom.y;
      tAngle = atan2(xError, yError) * (180 / M_PI);
      double derivative = angleError - lasterror;
      double currentAngle = Inertial.rotation();
      if (currentAngle > 360) {
        currentAngle = currentAngle - 360;
      }
      if (currentAngle < -360) {
        currentAngle = currentAngle + 360;
      }
      angleError = tAngle - currentAngle;
      double turnVelocity = angleError * ap + derivative * ad;
      double change = turnVelocity - previous;
      if (change > 20) {
        turnVelocity -= change - 20;
      }
      if (change < -2) {
        turnVelocity -= change + 2;
      }
      Rightside.spin(forward, turnVelocity, pct);
      Leftside.spin(forward, -turnVelocity, pct);
      lasterror = angleError;
      previous = Drivetrain.velocity(pct);
      wait(1, msec);
    }
    Drivetrain.stop(hold);
    angleError = 100;
    lasterror = 0;
    previous = 0;
    double distanceError = 100;
    double travel = sqrt((pow(xError, 2) + pow(yError, 2)));
    double tldist = Leftside.position(deg);
    double trdist = Rightside.position(deg);
    double crdist = 0;
    double cldist = 0;
    while (distanceError > 0.2) {
      tAngle = atan2(xError, yError) * (180 / M_PI);
      double currentAngle = Inertial.rotation();
      if (currentAngle > 360) {
        currentAngle = currentAngle - 360;
      }
      if (currentAngle < -360) {
        currentAngle = currentAngle + 360;
      }
      angleError = tAngle - currentAngle;
      double turnVelocity = angleError * 0.7;
      crdist = Rightside.position(deg) - trdist;
      cldist = Leftside.position(deg) - tldist;
      distanceError =
          travel - (((crdist + cldist) / 2) * wheelcircumfrence * 0.6 / 360);
      double derivative = distanceError - lasterror;
      double forwardVelocity = distanceError * p + derivative * d;
      double change = forwardVelocity - previous;
      if (change > slewUP) {
        change -= change - 20;
      }
      if (change < -slewDOWN) {
        change -= change + slewDOWN;
      }
      Leftside.spin(forward, forwardVelocity + turnVelocity, pct);
      Rightside.spin(forward, forwardVelocity - turnVelocity, pct);
      lasterror = distanceError;
      wait(1, msec);
      previous = Drivetrain.velocity(pct);
    }
  }
  if (backwards) {
    double xError = targetX - odom.x;
    double yError = targetY - odom.y;
    double tAngle = atan2(xError, yError) * (180 / M_PI);
    double angleError = 100;
    double lasterror = 0;
    double previous = 0;
    double previoust = 0;
    while (angleError > 0.2) {
      xError = targetX - odom.x;
      yError = targetY - odom.y;
      tAngle = atan2(-xError, -yError) * (180 / M_PI);
      if (previoust < 0 && tAngle > 0) {
        tAngle-
      }
      double derivative = angleError - lasterror;
      double currentAngle = Inertial.rotation();
      if (currentAngle > 360) {
        currentAngle = currentAngle - 360;
      }
      if (currentAngle < -360) {
        currentAngle = currentAngle + 360;
      }
      angleError = tAngle - currentAngle;
      double turnVelocity = angleError * ap + derivative * ad;
      double change = turnVelocity - previous;
      if (change > 20) {
        turnVelocity -= change - 20;
      }
      if (change < -2) {
        turnVelocity -= change + 2;
      }
      Rightside.spin(forward, turnVelocity, pct);
      Leftside.spin(forward, -turnVelocity, pct);
      lasterror = angleError;
      previous = turnVelocity;
      previoust = tAngle;
      wait(1, msec);
    }
    Drivetrain.stop(hold);
    angleError = 100;
    lasterror = 0;
    previous = 0;
    double distanceError = 100;
    double travel = sqrt((pow(xError, 2) + pow(yError, 2)));
    double tldist = Leftside.position(deg);
    double trdist = Rightside.position(deg);
    double crdist = 0;
    double cldist = 0;
    while (distanceError > 0.2) {
      tAngle = atan2(-xError, -yError) * (180 / M_PI);
      double currentAngle = Inertial.rotation();
      if (currentAngle > 360) {
        currentAngle = currentAngle - 360;
      }
      if (currentAngle < -360) {
        currentAngle = currentAngle + 360;
      }
      angleError = tAngle - currentAngle;
      double turnVelocity = angleError * 0.7;
      crdist = Rightside.position(deg) - trdist;
      cldist = Leftside.position(deg) - tldist;
      distanceError =
          travel - (((crdist + cldist) / 2) * wheelcircumfrence * 0.6 / 360);
      double derivative = distanceError - lasterror;
      double forwardVelocity = distanceError * p + derivative * d;
      double change = forwardVelocity - previous;
      if (change > slewUP) {
        change -= change - 20;
      }
      if (change < -slewDOWN) {
        change -= change + slewDOWN;
      }
      Leftside.spin(reverse, forwardVelocity + turnVelocity, pct);
      Rightside.spin(reverse, forwardVelocity - turnVelocity, pct);
      lasterror = distanceError;
      
      wait(1, msec);
      previous = Drivetrain.velocity(pct);
    }
  }
}

void pid::drivepid(double target, double p, double pp, double d, double c,
                   double maxvelocity) {
  Leftside.resetPosition();
  Rightside.resetPosition();
  double derror = 1;
  double aerror = 1;
  double lasterror = 0;
  double base = 0;
  double currentyaw = c;
  double lastV = 0;
  while (fabs(derror) > 0.1 || fabs(aerror) > 0.5) {
    double nowyaw = Inertial.yaw();
    double cdistance = ((Leftside.position(deg) + Rightside.position(deg) / 2) *
                        wheelcircumfrence * 0.6 / 360);
    derror = target - cdistance;
    double derivative = derror - lasterror;
    double speed = derror * p + derivative * d;

    double anglespeed = aerror * pp;
    // printf("%f, %f\n", derror, cdistance);
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
    lastV = Drivetrain.velocity(pct);
  }
  Drivetrain.stop(hold);
}

void pid::driveturn(double target, double p, double d) {
  double error = 1;
  double lasterror = 0;
  double totalerror = 0;
  double base = 0;

  double currentyaw;
  while (fabs(error) > 0.2) {
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
    // printf("%f\n", error);
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
