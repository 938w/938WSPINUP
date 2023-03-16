#include "vex.h"
bool iscatagoingdown = false;
vex::color colour = vex::red;
void setCata() {
  Catapult.setMaxTorque(1000, Nm);
  iscatagoingdown = true;
  while (!CataLimit.pressing()) {
    Catapult.spin(forward, 200, pct);
    if (Controller1.ButtonX.pressing()) {
      break;
    }
    this_thread::sleep_for(1);
  }
  Catapult.stop(coast);
  iscatagoingdown = false;
}
void launchCata() {
  Catapult.spinFor(forward, 40, deg);
  setCata();
}