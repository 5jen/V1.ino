#include <SpinningFlameThing.h>

#include <MiniMotorControllerDefinitions.h>
#include <MotorControllerMaster.h>


#include <AccelStepper.h>
#include <NewPing.h>
#include <Wire.h>
#include <math.h>
#include <DistanceGP2Y0A21YK.h>
#include <DistanceGP2Y0A21YK_LUTs.h>
#include <LiquidCrystal.h>
#include <L3G.h>
#include <MemoryFree.h>
#include <LSM303.h>


#include "SpinningFlameThing.h"

#include "Definitions.h"
#include "flame.h"
#include "rangeFinder.h"
long time;
void setup()
{	

	compass.init();
	compass.enableDefault();

	compass.m_min = (LSM303::vector<int16_t>){ +1762.5,   -722, -25055.5};
	compass.m_max = (LSM303::vector<int16_t>){ +3138.5,   +488, -23394.5};

	flameSetup();
	Wire.begin();
	c.begin();

	frontIR.begin(front_ir_pin);
	rearIR.begin(rear_ir_pin);
	s.begin(A5);
	Serial.begin(9600);
	
	c.setAcceleration(2000,2000,2000,2000);
	c.brake();
	
	lcd.begin(16,2);
	startButton = false;
	driveState = goStraight;
	pinMode(9, OUTPUT);
	pinMode(29, OUTPUT);
	digitalWrite(29, LOW);
	digitalWrite(9, LOW);
	pinMode(19, INPUT_PULLUP);
	attachInterrupt(4, isStart, FALLING);
	
}

void loop()

{
	
	if(!startButton){

	} //if start button is not pressed, do nothing

 	else if (angleError){ //if IMU angle does not match odometry, stop  
 		driveState = brake;
 	}
	else if(fc ){	//  if fc is true, do flame check
		checkFlame();
		driveState = goStraight;
	}
	
	else
	{ //navigation

		sendHb();
		checkCliff();
		pingSonar();
		echoCheck();
		getReferencePosition();
		getCurrentPosition();
		Go();
		checkSideWall();
		checkIMU();


		if(flameDetected && !complete)// if flame is detected and flame has not been put off
		{
			digitalWrite(29, HIGH);//triget the alarm
			flameNavigator();	//navigate to flame
		}

		else if(!complete) // if flame has not been put off
		{
			wallFollowNavigator();// flollow the wall
		}

		else if (complete)//if flame has been put off 
		{	
			if(millis() - lastCoord > 500) //get robot's coordinates for every 500 ms
			getCoordinate();

			digitalWrite(29, LOW); //disable the alarm

			if (reference_r - r < theta + 20  && !backToOrigin) //if still facing the candle, turn to the wall(which is guranteed to be the rightslde)
			{driveState = turnRight;				
			}

			else if (!backToOrigin){// if just turned from candle to wall, go straight
				driveState = brake;
				if(c.isStandby()){
					backToOrigin = true;
					driveState = goStraight;
				}
			}

			else if(abs(x) <20){ //if the x coordinate is close to the origin position, stop
				driveState = brake;
				arrivedOrigin = true;
			}

			else if(!arrivedOrigin){
				// if at the middle of nowhere, just go straight unless there's somthing at front
				if(!nearFrontWall&& !facingCliff && sideWallDistance > 10 && !findWall){

				}
				//follw the wall
				else {
					findWall = true;
					wallFollowNavigator();

				}
			}			
		}
	}
}


// detect obstacle every 100 ms
void pingSonar()
{
	if (millis() - lastPing > 100)
	{
		lastPing = millis();
		sonar.ping_timer(echoCheck);
	}		
}


void echoCheck()
{	
	if(sonar.check_timer())
	{
		frontWallDistance = sonar.ping_result / US_ROUNDTRIP_CM;

		//if too close to front wall, stop and turn left
		if(frontWallDistance < SAFE_DISTANCE )	
		{	
			if(nearFrontWall == false)
			{	
				stop_move = true;
				
				getReferencePos = true;
			}
			nearFrontWall = true;
		}	
	}	
}

void checkCliff()
{
	if (millis() - lastcc > 20)
	{
		lastcc = millis();

		//if at cliff, backUp and then turn left
		if(analogRead(light_sensor_pin) > lightSensorVal)
		{
			if(facingCliff == false) 
			{
				stop_move = true;
				getReferencePos = true;
				backUp = true;
			}
			facingCliff = true;
			atCliff = true;
		}
	}
}

//sent hb message to mini
void sendHb()
{
	if (millis() - lasthb > 4000){
		c.heartbeat();
		lasthb = millis();	
	}
}

//get the reference position(angle) before turns
void getReferencePosition()
{
	if(getReferencePos)
	{
		c.getEncoder(&l, &r);
		reference_l = l;
		reference_r = r;
		getReferencePos = false;
	} 
}

//get the current robot position(angle)
void getCurrentPosition()
{
	if(millis() - lastEncoderSample > 20 ) 
	{
		c.getEncoder(&l, &r);	
		lastEncoderSample = millis();	
	}
}


void Go()
{
	switch (driveState) {
		case goStraight:

		c.goVelocity(100, 0);
		break;

		case turnLeft_90:

	    	c.goVelocity(0, 20);			//turn left          
	    	getCurrentPosition();

 			//if current angle is not 90 degree from reference, turn left
 			if (r - reference_r  > 90)
 			{
 				c.goVelocity(0,0);
 				if(c.isStandby())			
 				{

                 	fc = true;			//look for flame each time it turns left
                 	cnt = 0;
                 	_start = false;

                 	if(facingCliff) atCliff =true;
                 	nearFrontWall = false;
                 	facingCliff = false;
                 }

             }	
             break;

             case turnRight_90:
             c.goVelocity(0, -20);    
	    	//if current angle is not 90 degree from reference, turn left
	    	if (reference_r - r > 90)
	    	{
	    		c.goVelocity(0,0);
	    		if(c.isStandby())
	    		{
	    			driveState = goStraight;
	    			wallBreak = false;
	    		}
	    	}
	    	break;


	    	case turnToCandle:
	    	lcd.setCursor(0,0);
	    	lcd.println(theta);

 			//if current angle does not match the angle measured from , turn left
 			if (r - reference_r < theta){
 				c.goVelocity (0,10);

 			}
			//stop when it's facing the candle
			else{
				driveState = brake;
				facingCandle = true;

			}
		



		break;
		
		case brake:
		c.goVelocity(0,0);
		break;
		
		case followWall:			//turn to or away the wall, speed is relatvie to the distance from wall
		if(abs(reference_r - r) > 5)
		{
			driveState = alignWall;

		}
		else 
		{
			if(sideWallDistance - sideLimit < -1.5)
			c.goVelocity(90, map(sideWallDistance - sideLimit, -10,-1, 20,10));

			else if(sideWallDistance - sideLimit > 1.5)
			c.goVelocity(90, map(sideWallDistance - sideLimit, 10,1, -20, -10));
		}



		break;

		case alignWall:	//turn untill the robot is parallel to the wall
		if(frontDist - rearDist <= -1 )
		c.goVelocity(90, map(frontDist - rearDist,-8, -1, 20,10));

		else if(frontDist - rearDist >= 1 )
		c.goVelocity(90, map(frontDist - rearDist, 8, 1, -20, -10));
		else
		driveState = goStraight;
		break;

		case turnRight:
		c.goVelocity(0,-30);
		break;

		case turnLeft:
		c.goVelocity(0,30);
		break;

		case backup:
		c.goVelocity(-50,0);
		break;


		default:
		c.brake();
	}

}

void wallFollowNavigator() 
{

	if(facingCliff || nearFrontWall)
	{
		//stop completely before it turns
		if(stop_move){
			driveState = brake;
			if(c.isStandby())
			{
				stop_move = false;
			}
		}
		//in case of flame, back up before it turns
		else if(backUp)
		{	
			if(reference_l - l < 80)//5cm
			driveState = backup;	
			else
			{ 
				backUp = false;
				stop_move = true;
			}
		}
		else
		//turn left_90;
		driveState = turnLeft_90;
	}


	
	if(!_start && !facingCliff && !atCliff && !nearFrontWall && !rightIsOpen && driveState != alignWall){
		
		//if robot does not parallel with wall, drivestate set to alignwall
		if(abs(frontDist - rearDist) != 0 ){
			driveState = alignWall;
		}
		//if robot is too far or too close to the wall, turn to or away fromwall
		else if(abs(sideWallDistance - sideLimit) > 1.5 )
		{
			if(driveState != followWall) getReferencePos = true;
			driveState = followWall;
		}	

		else driveState = goStraight;
	}
}

void flameNavigator() 
{
	//stop completely 
	if(stop_move){
		driveState = brake;
		if(c.isStandby()) {
			stop_move = false;
		}
	}
	//if the robot is facing candle, drive to it
	else if(driveState == brake && !nearFrontWall && facingCandle){
		driveState = goStraight;
	}

	//if it is not facing candle, turn to candle
	else if(!facingCandle){
		
		if (driveState != turnToCandle && !facingCandle)		
			getReferencePos = true;

		driveState = turnToCandle;
	}

	
	//if reach the candle base, and didn't spined
	if(nearFrontWall && !spin){
		//turn on the fan
		digitalWrite(9, HIGH);

		if (fcnt == 0)
		fanTime = millis();
		driveState = brake;

		getCoordinate();
		xCoord = x;
		yCoord = y;

		//estimate candle hight with intensity
		
		zCoord = map(high,300, 700, 170, 300);

		//print candle location
		lcd.setCursor(0,0);
		lcd.print("(");
		lcd.print(xCoord + cos(theta*3.14/180) * 270 );

		lcd.print(",");

		lcd.print(yCoord + sin(theta * 3.14/180) *270);
		lcd.print(",");
		lcd.setCursor(0,1);
		lcd.print(zCoord);

		lcd.print(")");
		spin = true;
		getReferencePos = true;
		getReferencePosition();
		candleAngle = reference_r;
		fcnt ++;

		}

	if(spin){
		//turn 20 degree in case the fan does not face the flame directly
		if (candleAngle - r < 20 && !spinComplete){
			driveState = turnRight;
		}		
		else {
			driveState = brake;
			spinComplete = true;
		}

		//if the fan is turned on for 5 sec, turn off the fan. assume flame has been put off.
		if (millis() - fanTime > 5000)
		{
			digitalWrite(9, LOW);
			complete = true;
			getReferencePos = true;
		}
	}
}


void checkIMU(){
	if(millis() - lastimu > 1000){
		lastimu = millis();
		
		compass.read();					//read compass value
		angle = compass.heading();
		getCurrentPosition();
		
		if (abs(angle - r ) > 50)	//if IMU value is 50 degree off with odometry, somthing is wrong
		angleError = true;	
	}
	

}

