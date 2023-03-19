#include "vex.h"
bool iscatagoingdown = false;
vex::color colour = vex::red;
void setCata() {
  Rotation11.resetPosition();
  this_thread::sleep_for(1);
  //printf("%f\n", Rotation11.position(deg));
  Catapult.setMaxTorque(1000, Nm);
  iscatagoingdown = true;
  while (Rotation11.position(deg) > -63 && !CataLimit.pressing()) {
     //printf("%f\n", Rotation11.position(deg));
    Catapult.spin(forward, 200, pct);
    if (Controller1.ButtonX.pressing()) {
      break;
    }
    this_thread::sleep_for(1);
  }
  Catapult.stop(hold);
  iscatagoingdown = false;
}
void launchCata() {
  Catapult.spinFor(forward, 80, deg, 200, rpm);
  this_thread::sleep_for(10);
  Rotation11.resetPosition();
  Rotation11.setPosition(0, degrees);
  setCata();
}