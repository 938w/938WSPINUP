using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor Catapult;
extern motor Intake;
extern controller Controller1;
extern rotation Rotation3;
extern motor RightM;
extern motor LeftM;
extern motor RightB;
extern motor LeftB;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );