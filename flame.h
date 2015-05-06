
void flameSetup() {

	stepper.setAcceleration(1500);
	flame.zero();
}

void checkFlame() {
	flame.run();

	if (flame.isDone()){
			//scan twice for flame
		cnt ++;
		if(cnt == 2)
			fc = false;

		flame.getFlamePosition(&high, &low, &distanceToFlame, &theta);
		flame.scan(-55, 200, 300);
	}


	//if the  value from the flame sensor is high enough, flame must been detected
	if(high  > flameVal){
		if(!flameDetected) stop_move = true;
		flameDetected = true;
	}
}

