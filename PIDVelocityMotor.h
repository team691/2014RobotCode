//PIDVelocityMotor.h

#ifndef PIDVELOCITYMOTOR_H
#define PIDVELOCITYMOTOR_H

//Includes
#include "WPILib.h"

//Declaration
class PIDVelocityMotor {
public:
	//Constructor
	PIDVelocityMotor(char *_name,
					 Victor &_motor,
					 Encoder &_encoder,
					 const double _pid[]);

	//Destructor
	~PIDVelocityMotor();

	//Functions
	void run();
	void run(double rpm);
	bool atTarget();
	char * getName();

	private:
	//Init data
	char * name;
	Victor &motor;
	Encoder &encoder;
	//PIDMotor input
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

#endif //PIDVELOCITYMOTOR_H
