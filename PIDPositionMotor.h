//PIDPositionMotor.h

#ifndef PIDPOSITIONMOTOR_H
#define PIDPOSITIONMOTOR_H

//Includes
#include "PIDVelocityMotor.h"
#include "WPILib.h"

//Declaration
class PIDPositionMotor {
public:
	//Constructor
	PIDPositionMotor(char * _name,
					 Victor &_vic,
					 Encoder &_encoder,
					 const double _velPID[],
					 const double _posPID[]);

	//Destructor
	~PIDPositionMotor();

	//Functions
	void run();
	void run(double pos);
	char * getName();
	
private:
	//Init data
	char * name;
	PIDVelocityMotor motor;
	Encoder &encoder;
	//PIDMotor input
	double position;
	double target;
	double error;
	double deltaTime;
	//PIDMotor scale
	double kp;
	double ki;
	double kd;
	double kf;
	double scalar;
	//PIDMotor out
	double proportional;
	double integral;
	double derivative;
	double feedForward;
	//PIDMotor out
	double pout;
	double iout;
	double dout;
	double fout;
	double out;
	//PIDMotor loop
	double lastError;
	double lastTime;
};

#endif //PIDPOSITIONMOTOR_H
