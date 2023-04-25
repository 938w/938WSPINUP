#include "odom.h"
#include "vex.h"


static double wheelcircumfrence = 3.25 * M_PI;

position odome::odomoutputs() {
  static double tdistr = 0;
  static double tdistl = 0;
  static double theta = 0;
  //double dleft = Leftside.position(deg) * wheelcircumfrence * 0.6666 / 360;
  //double dright = Rightside.position(deg) * wheelcircumfrence * 0.6666 / 360;
  double dleft = Rotation20.position(deg) * wheelcircumfrence/ 360;
  double dright = Rotation19.position(deg) * wheelcircumfrence/ 360;
  dleft -= tdistl;
  dright -= tdistr;
  tdistl += dleft;
  tdistr += dright;
  double dist = (dright + dleft) / 2;
  theta = Inertial9.yaw() * (M_PI / 180);
  //printf("%f, %f\n", x, y);
  double addY = dist * cos(theta);
  double addX = dist * sin(theta);
  x += addX;
  y += addY;

  position p;
  p.x = x;
  p.y = y;

  return p;
}
void odome::reset() {
  x = -Distance21.objectDistance(inches);
  y = 0;
  Leftside.resetPosition();
  Rightside.resetPosition();
  Rotation19.resetPosition();
  Rotation20.resetPosition();
}
void odome::setStarting(double ax, double ay) {
  x = ax;
  y = ay;
}
