#include "pid_pursuit.h"
odome odometry;
position odom;
void odomthread() {
  // odometry.reset();
  Inertial9.resetRotation();
  while (true) {
    // printf("%f, %f\n", odom.x, odom.y);
    odom = odometry.odomoutputs();
    vex::this_thread::sleep_for(5);
  }
}
static double wheelcircumfrence = 3.25 * M_PI;
void Onesidepursuit(bool backwards, int side, double x, double y, double p,
                    double d, double slewup, double slewdown, double finalangle,
                    double maxspeed, double turnmax) {}
void pursuit2(bool backwards, double targetX, double targetY, double endA,
              double slewUP, double slewDOWN, double turnmax, double maxspeed, bool broke) {
  static double p = 3.2;
  static double ap = 0.45;
  static double d = 0.02;
  static double ad = 0.1;
  if (!backwards) {
    double xError = targetX - odom.x;
    double yError = targetY - odom.y;
    double tAngle = atan2(xError, yError) * (180 / M_PI);
    double angleError = 100;
    double lasterror = 0;
    double previous = 0;
    double previoust = 0;
    double base = 0;
    double currentAngle;
    printf("%f\n", tAngle);
    while (abs(angleError) > 0.15) {
      xError = targetX - odom.x;
      yError = targetY - odom.y;
      //tAngle = atan2(xError, yError) * (180 / M_PI);
      currentAngle = Inertial9.rotation();
      while (currentAngle < -360) {
        currentAngle += 360;
      }
      while (currentAngle > 360) {
        currentAngle -= 360;
      }
    
      if (tAngle > 0 && currentAngle < 0 && broke) {
        tAngle = tAngle - 360;
      }
      angleError = tAngle - currentAngle;
      double derivative = angleError - lasterror;
      double turnVelocity = angleError * ap + derivative * ad;
      double change = turnVelocity - previous;

      if (currentAngle > tAngle) {
        base = -2;
        if (change > +3) {
          turnVelocity = turnVelocity - change + 3;
        }
      }
      if (currentAngle < tAngle) {
        base = 2;
        if (change < -3) {
          turnVelocity = turnVelocity + change - 3;
        }
      }
      if (turnVelocity > turnmax) {
        turnVelocity = turnmax;
      }
      if (turnVelocity < -turnmax) {
        turnVelocity = -turnmax;
      }
      //("%f\n", turnVelocity);
      Rightside.spin(reverse, base + turnVelocity, pct);
      Leftside.spin(forward, base + turnVelocity, pct);
      lasterror = angleError;
      previous = turnVelocity;
      previoust = tAngle;
      wait(1, msec);
    }
    Drivetrain.stop(hold);
    wait(0.1, sec);
    angleError = 100;
    lasterror = 0;
    previous = 0;
    double distanceError = 100;
    double travel = sqrt((pow(xError, 2) + pow(yError, 2)));
    double tldist = Leftside.position(deg);
    double trdist = Rightside.position(deg);
    double crdist = 0;
    double cldist = 0;
    if (currentAngle < -1080) {
        currentAngle = currentAngle + 1080;
      } else if (currentAngle < -720) {
        currentAngle = currentAngle + 720;
      } else if (currentAngle < -360) {
        currentAngle = currentAngle + 360;
      }
      if (currentAngle > 1080) {
        currentAngle = currentAngle - 1080;
      } else if (currentAngle > 720) {
        currentAngle = currentAngle - 720;
      } else if (currentAngle > 360) {
        currentAngle = currentAngle - 360;
      }
    double angle = currentAngle;
    while (abs(distanceError) > 0.15) {
      xError = targetX - odom.x;
      yError = targetY - odom.y;
      tAngle = angle;
      double currentAngle = Inertial9.rotation();
      if (currentAngle < -1080) {
        currentAngle = currentAngle + 1080;
      } else if (currentAngle < -720) {
        currentAngle = currentAngle + 720;
      } else if (currentAngle < -360) {
        currentAngle = currentAngle + 360;
      }
      if (currentAngle > 1080) {
        currentAngle = currentAngle - 1080;
      } else if (currentAngle > 720) {
        currentAngle = currentAngle - 720;
      } else if (currentAngle > 360) {
        currentAngle = currentAngle - 360;
      }
      if (tAngle > 0 && currentAngle < 0 && broke) {
        tAngle = tAngle - 360;
      }
      angleError = tAngle - currentAngle;
      double turnVelocity = angleError * 0.3;
      crdist = Rightside.position(deg) - trdist;
      cldist = Leftside.position(deg) - tldist;
      // distanceError = sqrt((pow(xError, 2) + pow(yError, 2)));
      distanceError =
          travel - (((crdist + cldist) / 2) * wheelcircumfrence * 0.6666 / 360);
      double derivative = distanceError - lasterror;
      double forwardVelocity = distanceError * p + derivative * d;
      double change = forwardVelocity - previous;
      if (change > slewUP) {
        forwardVelocity -= change - slewUP;
      }
      if (change < -slewDOWN) {
        forwardVelocity -= change + slewDOWN;
      }
      if (forwardVelocity > maxspeed) {
        forwardVelocity = maxspeed;
      }
      if (forwardVelocity < -maxspeed) {
        forwardVelocity = -maxspeed;
      }
      //printf("%f\n", distanceError);
      if (abs(distanceError) > 2) {
        Leftside.spin(forward, forwardVelocity + turnVelocity, pct);
        Rightside.spin(forward, forwardVelocity - turnVelocity, pct);

      } else {
        Leftside.spin(forward, forwardVelocity, pct);
        Rightside.spin(forward, forwardVelocity, pct);
      }
      lasterror = distanceError;
      previoust = tAngle;
      wait(1, msec);
      previous = Drivetrain.velocity(pct);
    }
  }

  if (backwards) {
    double xError = targetX - odom.x;
    double yError = targetY - odom.y;
    double tAngle = atan2(-xError, -yError) * (180 / M_PI);
    double angleError = 100;
    double lasterror = 0;
    double previous = 0;
    double previoust = 0;
    double base = 0;
    double currentAngle = 0;
    printf("%f\n", tAngle);
    while (abs(angleError) > 0.15) {
      xError = targetX - odom.x;
      yError = targetY - odom.y;
      // tAngle = atan2(-xError, -yError) * (180 / M_PI);
      currentAngle = Inertial9.rotation();
      if (currentAngle < -1080) {
        currentAngle = currentAngle + 1080;
      } else if (currentAngle < -720) {
        currentAngle = currentAngle + 720;
      } else if (currentAngle < -360) {
        currentAngle = currentAngle + 360;
      }
      if (currentAngle > 1080) {
        currentAngle = currentAngle - 1080;
      } else if (currentAngle > 720) {
        currentAngle = currentAngle - 720;
      } else if (currentAngle > 360) {
        currentAngle = currentAngle - 360;
      }
      angleError = tAngle - currentAngle;
      double derivative = angleError - lasterror;
      double turnVelocity = angleError * ap + derivative * ad;
      double change = turnVelocity - previous;
      if (turnVelocity > turnmax) {
        turnVelocity = turnmax;
      }
      if (turnVelocity < -turnmax) {
        turnVelocity = -turnmax;
      }
      if (currentAngle > tAngle) {
        base = -2;
        if (change > 2.5) {
          turnVelocity = turnVelocity - change + 3;
        }
      }
      if (currentAngle < tAngle) {
        base = 2;
        if (change < -2.5) {
          turnVelocity = turnVelocity + change - 3;
        }
      }
      Rightside.spin(reverse, turnVelocity + base, pct);
      Leftside.spin(forward, turnVelocity + base, pct);
      lasterror = angleError;
      previous = turnVelocity;
      previoust = tAngle;
      wait(1, msec);
    }
    Drivetrain.stop(hold);
    wait(0.1, sec);
    angleError = 100;
    lasterror = 0;
    previous = 0;
    double distanceError = 100;
    double travel = sqrt((pow(xError, 2) + pow(yError, 2)));
    double tldist = Leftside.position(deg);
    double trdist = Rightside.position(deg);
    double crdist = 0;
    double cldist = 0;
    currentAngle = Inertial9.rotation();
    if (currentAngle < -1080) {
        currentAngle = currentAngle + 1080;
      } else if (currentAngle < -720) {
        currentAngle = currentAngle + 720;
      } else if (currentAngle < -360) {
        currentAngle = currentAngle + 360;
      }
      if (currentAngle > 1080) {
        currentAngle = currentAngle - 1080;
      } else if (currentAngle > 720) {
        currentAngle = currentAngle - 720;
      } else if (currentAngle > 360) {
        currentAngle = currentAngle - 360;
      }
    double angle = currentAngle;
    while (abs(distanceError) > 0.2) {
      xError = targetX - odom.x;
      yError = targetY - odom.y;
      tAngle = angle;
      double currentAngle = Inertial9.rotation();
      if (currentAngle < -1080) {
        currentAngle = currentAngle + 1080;
      } else if (currentAngle < -720) {
        currentAngle = currentAngle + 720;
      } else if (currentAngle < -360) {
        currentAngle = currentAngle + 360;
      }
      if (currentAngle > 1080) {
        currentAngle = currentAngle - 1080;
      } else if (currentAngle > 720) {
        currentAngle = currentAngle - 720;
      } else if (currentAngle > 360) {
        currentAngle = currentAngle - 360;
      }

      angleError = tAngle - currentAngle;
      double turnVelocity = angleError * 0.3;
      crdist = Rightside.position(deg) - trdist;
      cldist = Leftside.position(deg) - tldist;
      distanceError = -travel - (((crdist + cldist) / 2) * wheelcircumfrence *
                                 0.6666 / 360);
      double derivative = distanceError - lasterror;
      double forwardVelocity = distanceError * p + derivative * d;
      double change = forwardVelocity - previous;
      if (change > slewUP) {
        forwardVelocity -= change - slewUP;
      }
      if (change < -slewDOWN) {
        if (!(forwardVelocity < 0)) {
          forwardVelocity -= change + slewDOWN;
        }
      }
      if (forwardVelocity < -maxspeed) {
        forwardVelocity = -maxspeed;
      }

      Leftside.spin(forward, forwardVelocity + turnVelocity, pct);
      Rightside.spin(forward, forwardVelocity - turnVelocity, pct);

      lasterror = distanceError;
      previoust = tAngle;
      wait(1, msec);
      previous = Drivetrain.velocity(pct);
    }
  }
  Drivetrain.stop(hold);
  if (endA != 69) {
    pid g;
    g.driveturn(endA, 0.44, 0.12);
  }
}

void pid::drivepid(double target, double p, double pp, double d, double c,
                   double maxvelocity, double lim) {
  Leftside.resetPosition();
  Rightside.resetPosition();
  double derror = 1;
  double aerror = 1;
  double lasterror = 0;
  double base = 0;
  double currentyaw = c;
  double lastV = 0;
  timer mald;
  while (fabs(derror) > 0.2 || fabs(aerror) > 0.5) {
    double nowyaw = Inertial9.yaw();
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
      if (Inertial9.yaw() < 0) {
        nowyaw = 180 + (180 + Inertial9.yaw());
      }
    }

    aerror = currentyaw - nowyaw;
    Leftside.spin(forward, base + speed + anglespeed, pct);
    Rightside.spin(forward, base + speed - anglespeed, pct);
    lasterror = derror;
    lastV = Drivetrain.velocity(pct);
    if (mald.time(msec) > lim) {
      break;
    }
  }
  Drivetrain.stop(hold);
}

void pid::driveturn(double target, double p, double d, double timeout) {
  double error = 1;
  double lasterror = 0;
  double totalerror = 0;
  double base = 0;

  double currentyaw;
  timer i;
  i.reset();
  while (fabs(error) > 0.15) {
    currentyaw = Inertial9.yaw();
    if (target == 180) {
      if (Inertial9.yaw() < 0) {
        currentyaw = 180 + (180 + Inertial9.yaw());
        p = 0.35;
      }
    }
    if (target >= 180) {
      if (Inertial9.yaw() < 0) {
        currentyaw = 180 + (180 + Inertial9.yaw());
        p = 0.35;
      }
    }
    if (target <= -180) {
      if (Inertial9.yaw() > 0) {
        currentyaw = -180 - (180 - Inertial9.yaw());
        p = 0.35;
      }
    }
    error = target - currentyaw;
    double derivative = error - lasterror;
    double speed = error * p + derivative * d;
    totalerror += error;
    base = 0;
    Leftside.spin(forward, base + speed, pct);
    Rightside.spin(reverse, base + speed, pct);
    lasterror = error;
    if (i.time(msec) > timeout) {
      break;
    }
  }

  Drivetrain.stop(hold);
}