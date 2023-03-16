using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor Catapult;
extern limit CataLimit;
extern controller Controller1;
extern motor Intake;
extern motor BR;
extern motor OR;
extern motor TR;
extern motor OL;
extern motor BL;
extern motor TL;
extern digital_out Endgame;
extern digital_out IntakeP;
extern rotation Rotation19;
extern rotation Rotation20;
extern inertial Inertial9;
extern distance Distance21;
extern motor_group Leftside; 
extern motor_group Rightside; 
extern smartdrive Drivetrain;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );