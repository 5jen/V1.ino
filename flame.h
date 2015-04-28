//#include "Definitions.h"

void flameSetup() {

	stepper.setAcceleration(1500);
		flame.zero();
}

void checkFlame() {
	// lcd.print("CF");
	 flame.run();
		
	if (flame.isDone()){
		// lcd.setCursor(0,0);
		// lcd.print("DONE");
		
		flame.getFlamePosition(&high, &low, &distanceToFlame, &theta);
		flame.scan(-55, 200, 300);
	}

	if(high  > flameVal){
			if(!flameDetected) stop_move = true;
			flameDetected = true;
	}
}

