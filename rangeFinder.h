

void getSideDistance()

{	
		//if two shrap sensor have different value, calculate the distance from the middle of the robot to wall
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
		lastir = millis();
		
		frontDist = frontIR.getDistanceCentimeter();
		rearDist = rearIR.getDistanceCentimeter();
		getSideDistance();
		
		if(frontDist < 25 && rearDist < 25 )
			//if side wall detected, must not at cliff
			atCliff = false;
	}

}