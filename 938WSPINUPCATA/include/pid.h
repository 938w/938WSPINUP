#include "vex.h"
class pid {
  public: 
  void driveturn(double target, double p, double d);
  void drivepid(double target, double p, double pp, double d = 0.05, double c = Inertial.yaw(), bool slow = false);
};
int intakeoutake ();