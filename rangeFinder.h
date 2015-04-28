// void getSideAngle() 
// {
// 	sideWallAngle = atan((frontDist - rearDist + frontIRXpos -rearIRXpos)/(rearDist*sin(0)- frontDist*sin(0) + rearIRYpos - frontIRYpos));
// }

void getSideDistance()

	{
		//getSideAngle();
	// 	 y1 = frontDist*cos(-sideWallAngle) + frontIRXpos*cos(sideWallAngle) + frontIRYpos*sin(sideWallAngle);
	// 	 y2 = rearDist*cos(-sideWallAngle) + rearIRXpos*cos(sideWallAngle) + rearIRYpos*sin(sideWallAngle);
		
	// 	sideWallDistance = (y1 + y2 )/2;
		
		 if(frontDist != rearDist)
		{
			if(abs(frontDist - rearDist) < 10)
			{	
				sideWallAngle = atan (11.0 / abs(frontDist - rearDist));
				sideWallDistance = (sin(sideWallAngle) * frontDist + sin(sideWallAngle) * rearDist) / 2.0; 
			}
			else
			{
				if(!wallBreak) stop_move = true;
				
				wallBreak = true;
			}
		}
		else
			sideWallDistance = frontDist;
	}




void checkSideWall() {
	
	if(millis() - lastir > 20)
	{
		//Serial.println("readIR");
		lastir = millis();
			
		frontDist = frontIR.getDistanceCentimeter();
		rearDist = rearIR.getDistanceCentimeter();
		getSideDistance();
	
		if(frontDist < 25 && rearDist < 25 )
		// {
		// 	if(!rightIsOpen)
		// 	{
		// 		//getReferencePos = true;
		// 		stop_move = true;
		// 	}
		// 	rightIsOpen = true;
		// }
		// else
			atCliff = false;

	}

}