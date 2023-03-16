
#include "vex.h"
#include "odom.h"
void odomthread ();
void pursuit2(bool backwards, double targetX, double targetY, double endA,
              double slewUP, double slewDOWN,  double turnmax, double maxspeed);
extern odome odometry;
class pid {
public:
  void driveturn(double target, double p, double d);
  void drivepid(double target, double p, double pp, double d = 0.05,
                double c = Inertial9.yaw(), double maxvelocity = 100);
};