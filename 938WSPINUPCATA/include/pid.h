#include "vex.h"
#include "ogeometry.h"
void pursuit(double targetX, double targetY, double targetA, double P,
             double max, double maxd, bool constant);
void pursuit2(bool backwards, double targetX, double targetY, double endA,
              double slewUP, double slewDOWN, bool stack = false);
class pid {
public:
  void driveturn(double target, double p, double d);
  void drivepid(double target, double p, double pp, double d = 0.05,
                double c = Inertial.yaw(), double maxvelocity = 100);
};
int intakeoutake();
int odomthread();
extern odome odometry;