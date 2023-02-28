#include "vex.h"
// Color for selection
color colour;
// Roller Code
int spinroller () {
  timer t1;
  while (!(Optical1.color() == colour)) {
    Intake.spin(reverse, 440, rpm);
    vex::this_thread::sleep_for(1);
    if (Controller1.ButtonX.pressing()){
      break;
    }
    if(t1.time(sec) > 3) {
      break;
    }
  }
  Intake.stop(hold);
  t1.clear();
  vex::this_thread::sleep_for(50);
  return 0;
}