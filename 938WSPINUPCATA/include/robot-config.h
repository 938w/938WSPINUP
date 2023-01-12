using namespace vex;

extern brain Brain;

// VEXcode devices
extern optical Optical1;
extern motor Catapult;
extern motor Intake;
extern controller Controller1;
extern rotation Rotation3;
extern motor RightM;
extern motor LeftM;
extern motor RightB;
extern motor LeftB;
extern motor LeftF;
extern motor RightF;
extern inertial Inertial; 
extern smartdrive Drivetrain; 
extern motor_group Leftside; 
extern motor_group Rightside; 
extern digital_out boost;
extern digital_out endgame;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );