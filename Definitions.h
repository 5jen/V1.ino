#ifndef Definitions_h
#define Definitions_h

/*
// ultrasonics
*/
#define SAFE_DISTANCE 20

const int trigger_pin = 22;
const int echo_pin =  23;

unsigned int frontWallDistance;

NewPing sonar(trigger_pin, echo_pin);


/*
//IRs
*/
const int front_ir_pin = A3;
const int rear_ir_pin = A4;

volatile  int frontDist;
volatile  int rearDist;
volatile double sideWallDistance;


double sideWallAngle;

const int OpenIrValue = 30 ;
volatile const int sideLimit = 10;

DistanceGP2Y0A21YK frontIR;
DistanceGP2Y0A21YK rearIR;

int y1,y2;
/*
//light sensors
*/
const int light_sensor_pin = A0;
const int lightSensorVal = 500;
/*
//stepper
*/
// const int stepper_step = 2;
// const int stepper_dir = 3;

AccelStepper stepper(AccelStepper::DRIVER,53,52); 

/*
//flame Sensor
*/
const int l_flame_sensor_pin = A5;
const int r_flame_sensor_pin = A6;
const int flameVal = 200;
int high;
int low;
 int distanceToFlame;
int theta;
SpinningFlameThing flame(&stepper, -55, 220, 200*4, A1, A2, 28);


/*
//encoders
*/

long l, r, reference_l, reference_r = 0;		//angle turned in degrees, distance traveled in mm


 
/*
//time check
*/
int lasthb = 0;
int lastRightTurn = 0;
int lastEncoderSample = 0;
int lastPing = 0;
int lastir = 0;
int lastcc = 0;
int lastfc = 0;




//int irMap[2][11] ={{5,6,7,8,9,10,11,12,13,14,15},{399,340,287,260,240,212,189,175,165,153,146}};


/*
//state machine
*/
enum dirveStates{goStraight, turnLeft_90, turnRight_90, turnToCandle, brake, followWall, alignWall, backup} driveState;

bool facingCliff, nearFrontWall, rightIsOpen, atCliff, getReferencePos 
, stop_move, flameDetected ,backUp, facingCandle, wallBreak= false;

long  x, y = 0;

/*
//motor controller
*/
// RegulatedMotor m1(&encoder1,7,6,10);
// RegulatedMotor m2(&encoder2,8,11,9);
MotorControllerMaster c;
// KinematicController kc(&m1,&m2,1,-1,225,75,64*50);

LiquidCrystal lcd(40,41,42,43,44,45);

void getCoordinate(){
	c.getGlobalPosition(&x, &y);
}
L3G gyro;



#endif