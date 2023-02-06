#include "ogeometry.h"
#include "vex.h"


static double wheelcircumfrence = 3.25 * M_PI;

position odom::odomoutputs() {
  static double tdistr = 0;
  static double tdistl = 0;
  static double theta = 0;
  double dleft = Leftside.position(deg) * wheelcircumfrence * 0.6 / 360;
  double dright = Rightside.position(deg) * wheelcircumfrence * 0.6 / 360;
  dleft -= tdistl;
  dright -= tdistr;
  tdistl += dleft;
  tdistr += dright;
  double dist = (dright + dleft) / 2;
  theta = Inertial.yaw() * (M_PI / 180);
  //
  double addY = dist * cos(theta);
  double addX = dist * sin(theta);
  x += addX;
  y += addY;

  position p;
  p.x = x;
  p.y = y;

  return p;
}
void odom::reset() {
  x = 0;
  y = 0;
  Leftside.resetPosition();
  Rightside.resetPosition();
}
void odom::setStarting(double ax, double ay) {
  x += ax;
  y += ay;
}
