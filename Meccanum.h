//Meccanum.h

#ifndef MECCANUM_H
#define MECCANUM_H

//Includes
#include "PIDVelocityMotor.h"
#include "WPILib.h"

//Declaration
class Meccanum {
public:
	//Constructor
	Meccanum(PIDVelocityMotor &_fr, PIDVelocityMotor &_fl, PIDVelocityMotor &_br, PIDVelocityMotor &_bl);

	//Destructor
	~Meccanum();

	//Functions
	void moveDual(Joystick rjoy, Joystick ljoy);
	void move(Joystick joy);
	void update(double forward, double right, double clockwise);
	void stop();

private:
	//Init Data
	//PIDMotors
	PIDVelocityMotor &fr;
	PIDVelocityMotor &fl;
	PIDVelocityMotor &br;
	PIDVelocityMotor &bl;
	//Meccanum control
	double frVel;
	double flVel;
	double brVel;
	double blVel;
	double maxVel;
	double topVel;
};

#endif //MECCANUM_H
