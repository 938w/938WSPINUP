using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor FrontLeft;
extern motor LeftBack;
extern motor RightFront;
extern motor RightBack;
extern motor Flywheel1;
extern motor FlywheelReversed;
extern controller Controller1;
extern controller Controller2;
extern motor_group INtake;
extern digital_out Piston;
extern smartdrive maindrive;
extern inertial Inertial;
extern digital_out endgame;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );