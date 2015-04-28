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

	Serial.begin(115200);
	
	c.setAcceleration(1000,1000,1000,1000);
	c.brake();
	//c.goVelocity(100, 0);
	//delay(200);
	lcd.begin(16,2);
	driveState = goStraight;
	Serial.println(1);

	
}

void loop()
{	
	Serial.println(millis() - time);
		time = millis();

	// gyro.read();
	// getCoordinate();
	sendHb();
	checkCliff();
	pingSonar();
	checkFlame();
	//getReferencePosition();
	//getCurrentPosition();
	//Go();
	//checkSideWall();
	lcd.setCursor(0,1);
	 lcd.print(flameDetected);
	// lcd.print(" ,");
	//c.goVelocity(300,0);
	//flame.getFlamePosition(&high, &low, &distanceToFlame, &theta);

	 // lcd.print(y);
	 // lcd.print(" ,");
	 //lcd.println((int)gyro.g.z);
	 // lcd.setCursor(0,1);
	 //  lcd.println(analogRead(light_sensor_pin) > lightSensorVal);
	//Serial.println(sideWallAngle);
	//Serial.println( 1 );
	//c.goVelocity(-100,0);

	if(flameDetected)
	{
		flameNavigator();
	}
	else
	{
		//wallFollowNavigator();
	}
	
}

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
			if (r - reference_r  > 80)
			{
			//Serial.println("complete turn");
				 c.goVelocity(0,0);
                 if(c.isStandby())
                 {
                 	driveState = goStraight;

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
			lcd.println("turnToCandle");
			if(theta > 0) 
			{
				if (reference_r < theta)
					c.goVelocity (0,10);
				else{
					driveState = brake;
					facingCandle = true;
				}
			}
			else if(theta < 0){
				if(reference_r > theta)
					c.goVelocity(0,-10);			    
				else{
					driveState = brake;
					facingCandle = true;
				}		    
			}
			

			break;
		
		case brake:
			c.goVelocity(0,0);
			break;
		
		case followWall:
			lcd.setCursor(0,0);
			lcd.println("followWall");
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
			lcd.setCursor(0,0);
			lcd.println("alignWall");
			if(frontDist - rearDist <= -1 )
			 	c.goVelocity(90, map(frontDist - rearDist,-8, -1, 20,10));
			 
			else if(frontDist - rearDist >= 1 )
			 	c.goVelocity(90, map(frontDist - rearDist, 8, 1, -20, -10));
			else
			driveState = goStraight;
			 
			break;
		case backup:
			c.goVelocity(-100,0);
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
        	lcd.setCursor(0,1);
			lcd.println("brake");
        	driveState = brake;
			if(c.isStandby())
			{
				stop_move = false;
			}
		}

		else if(backUp)
		{	
			lcd.setCursor(0,1);
			lcd.println("backUp");
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
	
	if(!facingCliff && !atCliff && !nearFrontWall && !wallBreak){
		

	    if(abs(frontDist - rearDist) != 0  ){
			driveState = alignWall;

		}
		else if(abs(sideWallDistance - sideLimit) > 1.5 )
		{
			lcd.setCursor(0,1);
			lcd.println("enterFollw wall");
			if(driveState != followWall) getReferencePos = true;
			driveState = followWall;
		}
			

		else driveState = goStraight;
	}
	if(wallBreak)
	{

	 	if (sideWallDistance > 30){
			lcd.setCursor(0,1);
			lcd.println("wallBreak");
			if(stop_move){
				driveState = brake;
				if(c.isStandby()){
					stop_move = false;
					driveState = turnRight_90;
				}
			}
		}	
	}
}

void flameNavigator() 
{
	if(stop_move){
		driveState = brake;
		if(c.isStandby()) stop_move = false;
	}
	else {
		if (driveState != turnToCandle)
			getReferencePos = true;
		driveState = turnToCandle;
	}

	if(driveState == brake && !nearFrontWall && facingCandle){
		//put off the candle
		driveState = goStraight;
	}
	if(nearFrontWall)
	driveState = brake;


}
	
