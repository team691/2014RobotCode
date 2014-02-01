//Meccanum.cpp

//Includes
#include <cmath>
#include "PIDVelocityMotor.h"
#include "Meccanum.h"

//Constructor
Meccanum::Meccanum(PIDVelocityMotor &_fr,
				   PIDVelocityMotor &_fl,
				   PIDVelocityMotor &_br,
				   PIDVelocityMotor &_bl) : fr(_fr),
											fl(_fl),
											br(_br),
											bl(_bl),
											frVel(0.0),
											flVel(0.0),
											brVel(0.0),
											blVel(0.0),
											topVel(0.0)
{
	printf("Meccanum drive object created!\n");
}

//Destructor
Meccanum::~Meccanum() {}

//Functions
void Meccanum::moveDual(Joystick rjoy, Joystick ljoy) {
	update(ljoy.GetRawAxis(2), ljoy.GetRawAxis(1), rjoy.GetRawAxis(1));
}

void Meccanum::move(Joystick joy) {
	update(joy.GetRawAxis(2), joy.GetRawAxis(1), joy.GetRawAxis(3));
}

void Meccanum::update(double forward, double right, double clockwise) {
	//printf("Meccanum::update() called!\n");
	//Figure out motor speeds
	frVel = -(forward + right + clockwise); //0--
	flVel = (forward - right - clockwise);  //0++
	brVel = -(forward - right + clockwise); //0+-
	blVel = (forward + right - clockwise);  //0-+

	//Set the motor objects to the output value for motor control and additional scaling
	//Find the maximum speed
	topVel = 0.0;
	if(topVel < fabs(frVel)) {topVel = fabs(frVel);}
	if(topVel < fabs(flVel)) {topVel = fabs(flVel);}
	if(topVel < fabs(brVel)) {topVel = fabs(brVel);}
	if(topVel < fabs(blVel)) {topVel = fabs(blVel);}
	
	//Scale motor power if it is above 100%
	if(topVel > 1.0) {
		frVel /= topVel;
		flVel /= topVel;
		brVel /= topVel;
		blVel /= topVel;
	}
	
	//printf("Updating motors...\nFR: %f, FL: %f, BR: %f, BL: %f, TopVel: %f\n", frVel, flVel, brVel, blVel, topVel);
	fr.run(frVel);
	fl.run(flVel);
	br.run(brVel);
	bl.run(blVel);
}

void Meccanum::stop() {
		fr.run(0.0);
		fl.run(0.0);
		br.run(0.0);
		bl.run(0.0);
}
