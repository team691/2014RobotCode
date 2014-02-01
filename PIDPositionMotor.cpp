//PIDPositionMotor.cpp

//Includes
#include <cmath>
#include "PIDPositionMotor.h"

//Constructor
PIDPositionMotor::PIDPositionMotor(char * _name,
								   Victor &_vic,
								   Encoder &_encoder,
								   const double _velPID[],
								   const double _posPID[]) : name(_name),
															 motor(_name, _vic, _encoder, _velPID),
															 encoder(_encoder),
															 position(0.0),
															 target(0.0),
															 error(0.0),
															 deltaTime(0.0),
															 kp(_posPID[0]),
															 ki(_posPID[1]),
															 kd(_posPID[2]),
															 kf(_posPID[3]),
															 scalar(_velPID[4]),
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
	printf("PIDPositionMotor %s created!\n", name);
}

//Destructor
PIDPositionMotor::~PIDPositionMotor() {}

//Functions

//PIDPositionMotor control
void PIDPositionMotor::run() {
	if(GetTime() - 0.01 > lastTime) {
		position = encoder.GetDistance();
		while(position > 360 || position < -360) {
			position /= 360;
		}
		error = target - encoder.GetDistance();
		if(target == encoder.GetDistance()) {
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
		motor.run(out);
		printf("Name: %s KP: %f Target: %f CurrentPos: %f Error: %f Get(): %ld Out: %f\n", name, kp, target, encoder.GetDistance(), error, encoder.Get(), out);

		lastError = error;
		lastTime = GetTime();
	} else {
		//printf("PIDPositionMotor %s waiting... Time: %f\n", name, GetTime());
	}
}

//PIDPositionMotor control
void PIDPositionMotor::run(double pos) {
	target = pos;
	run();
}

char * PIDPositionMotor::getName() {
	return name;
}
