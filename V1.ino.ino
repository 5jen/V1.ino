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

#include "SpinningFlameThing.h"

#include "Definitions.h"
#include "flame.h"
#include "rangeFinder.h"
long time;
void setup()
{	

	// if (!gyro.init())
	// {
	// 	Serial.println("Failed to autodetect gyro type!");
	// 	while (1);
	// }
	// gyro.init();

	// gyro.enableDefault();
  //gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
	//gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz

	flameSetup();
	Wire.begin();
	c.begin();

	frontIR.begin(front_ir_pin);
	rearIR.begin(rear_ir_pin);
	s.begin(A5);
	Serial.begin(9600);
	
	c.setAcceleration(2000,2000,2000,2000);
	c.brake();
	//c.goVelocity(100, 0);
	//delay(200);
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
	// Serial.println(millis() - time);
	// 	time = millis();
	if(!startButton){} //if start button is not pressed, do nothing
 
	else if(fc ){	//  do flame check
		checkFlame();
		driveState = goStraight;


	}
	
	else
	{ //navigation

	// Serial.println(millis() - time);
	// 	time = millis();

	// gyro.read();
	// getCoordinate();
		sendHb();
		checkCliff();
		pingSonar();
		echoCheck();
		getReferencePosition();
		getCurrentPosition();
		Go();
		checkSideWall();
	// lcd.setCursor(0,1);
	// 	  lcd.print(r);

	// lcd.print(" ,");

	//   lcd.print(reference_r);
	 // lcd.print(" ,");
	 //lcd.println((int)gyro.g.z);
	  // lcd.setCursor(0,1);
	  // lcd.println(analogRead(light_sensor_pin) > lightSensorVal);
	//Serial.println(analogRead(A5));
	//Serial.println( 1 );
	//c.goVelocity(-100,0);

		if(flameDetected && !complete)
		{
			digitalWrite(29, HIGH);
		//prevState = driveState;
			flameNavigator();
		// lcd.setCursor(0,1);
		// lcd.print("flameDetected");	
		}
		else if(!complete)
		{
		//prevState = driveState;
			wallFollowNavigator();
		}
		else if (complete)
		{	
			digitalWrite(29, LOW);
			// if (l > distToCandle) driveState = backup;
			// else driveState = brake;


		}
	}
}

void pingSonar()
{
	if (millis() - lastPing > 100)
	{
		//frontWallDistance = s.getDistanceCentimeter();

		lastPing = millis();
		sonar.ping_timer(echoCheck);
	}		
}

void echoCheck()
{	
	if(sonar.check_timer())
	{
		frontWallDistance = sonar.ping_result / US_ROUNDTRIP_CM;
		
		if(frontWallDistance < SAFE_DISTANCE )//and if flame is not close by!! 
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
		//Serial.println(analogRead(light_sensor_pin));
		lastcc = millis();
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

void sendHb()
{
	if (millis() - lasthb > 4000){
		//Serial.println("inhb");
		c.heartbeat();
		lasthb = millis();	
	}
}

void getReferencePosition()
{
	if(getReferencePos)
	{
		//Serial.println("getref");
		c.getEncoder(&l, &r);
		reference_l = l;
		reference_r = r;
		getReferencePos = false;
	} 
}


void getCurrentPosition()
{
	//Serial.println("getcur");
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
	    //Serial.println("straight");
	    lcd.setCursor(0,0);
			lcd.println("goStraight");
		    c.goVelocity(100, 0);
	    	break;
	    
	    case turnLeft_90:
	    	//Serial.println("im turning");
	    	lcd.setCursor(0,0);
			lcd.println("turnLeft");
	    	c.goVelocity(0, 20);          
        	getCurrentPosition();
			if (r - reference_r  > 90)
			{
			//Serial.println("complete turn");

				 c.goVelocity(0,0);
                 if(c.isStandby())
                 {
                 	//driveState = goStraight;
                 	fc = true;
                 	cnt = 0;
                 	_start = false;

                 	if(facingCliff) atCliff =true;
					nearFrontWall = false;
					facingCliff = false;
                 }
				
			}	
	      	break;
	    
	    case turnRight_90:
	    lcd.setCursor(0,0);
			lcd.println("turnRight");
	    	c.goVelocity(0, -20);          
			if (reference_r - r > 80)
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
			if(theta > 0) 
			{
				if (r - reference_r < theta){
					c.goVelocity (0,10);
					
				}
				else{
					driveState = brake;
					facingCandle = true;

				}
			}
			else if(theta < 0){
				if(reference_r  - r < theta)
					c.goVelocity(0,-10);			    
				else{
					driveState = brake;
					facingCandle = true;
					l = distToCandle;
				}		    
			}
			

			break;
		
		case brake:
			c.goVelocity(0,0);
			break;
		
		case followWall:
			// lcd.setCursor(0,0);
			// lcd.println("followWall");
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

		case alignWall:
			// lcd.setCursor(0,0);
			// lcd.println("alignWall");
			if(frontDist - rearDist <= -1 )
			 	c.goVelocity(90, map(frontDist - rearDist,-8, -1, 20,10));
			 
			else if(frontDist - rearDist >= 1 )
			 	c.goVelocity(90, map(frontDist - rearDist, 8, 1, -20, -10));
			else
			driveState = goStraight;
			 
			break;
		case turnRight:
			c.goVelocity(0,-40);
			break;
		case turnLeft:
			c.goVelocity(0,40);
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
        if(stop_move){
   //      	lcd.setCursor(0,1);
			// lcd.println("brake");
        	driveState = brake;
			if(c.isStandby())
			{
				stop_move = false;
			}
		}

		else if(backUp)
		{	
			// lcd.setCursor(0,1);
			// lcd.println("backUp");
			if(reference_l - l < 50)//5cm
				driveState = backup;	
			else
			{ 
				backUp = false;
				stop_move = true;
				//getReferencePos = true;
			}
		}
		else
		    driveState = turnLeft_90;
      }	
	
	if(!_start && !facingCliff && !atCliff && !nearFrontWall && !rightIsOpen && driveState != alignWall){
		

	    if(abs(frontDist - rearDist) != 0 ){
			driveState = alignWall;

		}
		else if(abs(sideWallDistance - sideLimit) > 1.5 )
		{
			// lcd.setCursor(0,1);
			// lcd.println("enterFollw wall");
			if(driveState != followWall) getReferencePos = true;
			driveState = followWall;
		}
		else if (sideWallDistance > 30){
			// lcd.setCursor(0,1);
			// lcd.println("farFromwall, turnR");
			driveState = turnRight_90;
		}		

		else driveState = goStraight;
	}
}

void flameNavigator() 
{
	if(stop_move){

		driveState = brake;
		if(c.isStandby()) {
			//fc = true;
			stop_move = false;
		}

	}
	else if(driveState == brake && !nearFrontWall && facingCandle){
				driveState = goStraight;
	}
	else if(!facingCandle){
		if (driveState != turnToCandle && !facingCandle){
			// lcd.setCursor(0,1);
			// lcd.println("hahahaha");
			getReferencePos = true;
		}
			
		driveState = turnToCandle;
	}

	

	if(nearFrontWall && !spin){
		digitalWrite(9, HIGH);
		if (fcnt == 0)
			fanTime = millis();
		driveState = brake;

		getCoordinate();
		lcd.setCursor(0,1);

		lcd.print("(");
		lcd.print(xCoord + cos(theta*3.14/180) * 270 );

		lcd.print(",");

	  	lcd.print(yCoord + sin(theta * 3.14/180) *270);
	  	lcd.print(",");

	  	lcd.print(245);

	  	lcd.print(")");
	  	spin = true;
	  	getReferencePos = true;
	  	getReferencePosition();
	  	candleAngle = reference_r;
	  	fcnt ++;

	}
	
	if(spin){

		
		 if (candleAngle - r < 30 && !spinComplete){
			driveState = turnRight;
		}
		
		else {
			driveState = brake;
			spinComplete = true;

		}

	  	if (millis() - fanTime > 5000)
	  	{
	  		digitalWrite(9, LOW);
			complete = true;
	  		
	  	}

	}



}
	
