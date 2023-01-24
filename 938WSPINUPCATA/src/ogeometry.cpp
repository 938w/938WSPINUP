#include "vex.h"
#include "ogeometry.h"

static double wheelcircumfrence = 3.25 * M_PI;
position odomoutputs () {
  double theta = 0;
  static double x = 0;
  static double y = 0;
  double dleft = Leftside.position(deg) * wheelcircumfrence * 0.6 / 360;
  double dright = Rightside.position(deg) * wheelcircumfrence * 0.6 / 360;
  double dist = (dright + dleft) / 2;
  theta = Inertial.yaw() * (M_PI/180);
  //
  double addY = dist * cos(theta);
  double addX = dist * sin(theta);
  x += addX;
  y += addY;

  struct position p;
  p.x = x;
  p.y = y;

  return p;
}