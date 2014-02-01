//PIDVelocityMotor.cpp

//Includes
#include <cmath>
#include "PIDVelocityMotor.h"

//Constructor
PIDVelocityMotor::PIDVelocityMotor(char *_name,
								   Victor &_motor,
								   Encoder &_encoder,
								   const double _pid[]) : name(_name),
														  motor(_motor),
														  encoder(_encoder),
														  target(0.0),
														  error(0.0),
														  deltaTime(0.0),
														  kp(_pid[0]),
														  ki(_pid[1]),
														  kd(_pid[2]),
														  kf(_pid[3]),
														  scalar(_pid[4]),
														  proportional(0.0),
														  integral(0.0),
														  derivative(0.0),
														  feedForward(0.0),
														  pout(0.0),
														  iout(0.0),
														  dout(0.0),
														  fout(0.0),
														  out(0.0),
														  lastError(0.0),
														  lastTime(0)
{
	printf("PIDVelocityMotor %s created!\n", name);
}

//Destructor
PIDVelocityMotor::~PIDVelocityMotor() {}

//Functions

//PIDMotor control
void PIDVelocityMotor::run() {
	if(GetTime() - 0.01 > lastTime) {
		error = target - ((encoder.GetRate() / 360 * 60) / scalar);
		if(target == 0.0) {
			error = 0.0;
			integral = 0.0;
			lastError = 0.0;
		}
		deltaTime = GetTime() - lastTime;

		proportional = error;
		integral += error * deltaTime;
		derivative = (error - lastError) / deltaTime;
		feedForward = target;

		pout = kp * proportional;
		iout = ki * integral;
		dout = kd * derivative;
		fout = kf * feedForward;
		if(pout > 1.0) {
			pout = 1.0;
		} else if(pout < -1.0) {
			pout = -1.0;
		}
		if(iout > 1.0) {
			iout = 1.0;
			integral = iout / ki;
		} else if(iout < -1.0) {
			iout = -1.0;
			integral = iout / ki;
		}
		if(dout > 1.0) {
			dout = 1.0;
		} else if(dout < -1.0) {
			dout = -1.0;
		}

		out = pout + iout + dout + fout;
		if(out > 1.0) {
			out = 1.0;
		} else if(out < -1.0) {
			out = -1.0;
		}
		motor.SetSpeed(out);
		printf("Name: %s KP: %f Target: %f CurrentRPM: %f, ScaledRPM: %f Error: %f Get(): %ld Out: %f\n", name, kp, target, (encoder.GetRate() / 360 * 60), ((encoder.GetRate() / 360 * 60) / scalar), error, encoder.Get(), out);
	
		lastError = error;
		lastTime += deltaTime;
	} else {
		//printf("PIDVelocityMotor %s waiting... Time: %f\n", name, GetTime());
	}
}

//PIDMotor control
void PIDVelocityMotor::run(double speed) {
	target = speed;
	run();
}

bool PIDVelocityMotor::atTarget() {
	if(abs(error - target) <= 5) {
		return true;
	} else {
		return false;
	}
}

char * PIDVelocityMotor::getName() {
	return name;
}
