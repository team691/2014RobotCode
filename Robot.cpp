//Robot.cpp

//Includes
#include <cmath>
#include "WPILib.h"
#include "Values.h"
#include "Meccanum.h"

//Main Class
class Robot : public SimpleRobot {
private:
	Joystick joy;
	DriverStationLCD * dslcd;

	Victor frMotor;
	Victor flMotor;
	Victor brMotor;
	Victor blMotor;
	Encoder frEnc;
	Encoder flEnc;
	Encoder brEnc;
	Encoder blEnc;
	PIDVelocityMotor fr;
	PIDVelocityMotor fl;
	PIDVelocityMotor br;
	PIDVelocityMotor bl;

	Meccanum drive;
	double forward;
	double right;
	double clockwise;
	double scalar;

	Victor shooter;
	Victor rIntake;
	Victor lIntake;

	bool check;
	bool shoot;
	bool move;
	double time;
	AnalogTrigger photosensor;

public:
	Robot(void): joy(JOYSTICK),
				 dslcd(DriverStationLCD::GetInstance()),
				 frMotor(DRIVE_VICTOR_SIDECARS[0], FR_DRIVE_VICTOR),
				 flMotor(DRIVE_VICTOR_SIDECARS[1], FL_DRIVE_VICTOR),
				 brMotor(DRIVE_VICTOR_SIDECARS[2], BR_DRIVE_VICTOR),
				 blMotor(DRIVE_VICTOR_SIDECARS[3], BL_DRIVE_VICTOR),
				 frEnc(FR_DRIVE_ENCODER_SIDECAR, FR_DRIVE_ENCODER_A, FR_DRIVE_ENCODER_SIDECAR, FR_DRIVE_ENCODER_B, FR_DRIVE_ENCODER_B),
				 flEnc(FL_DRIVE_ENCODER_SIDECAR, FL_DRIVE_ENCODER_A, FL_DRIVE_ENCODER_SIDECAR, FL_DRIVE_ENCODER_B, FL_DRIVE_ENCODER_B),
				 brEnc(BR_DRIVE_ENCODER_SIDECAR, BR_DRIVE_ENCODER_A, BR_DRIVE_ENCODER_SIDECAR, BR_DRIVE_ENCODER_B, BR_DRIVE_ENCODER_B),
				 blEnc(BL_DRIVE_ENCODER_SIDECAR, BL_DRIVE_ENCODER_A, BL_DRIVE_ENCODER_SIDECAR, BL_DRIVE_ENCODER_B, BL_DRIVE_ENCODER_B),
				 fr("FR", frMotor, frEnc, FR_DRIVE_PID),
				 fl("FL", flMotor, flEnc, FL_DRIVE_PID),
				 br("BR", brMotor, brEnc, BR_DRIVE_PID),
				 bl("BL", blMotor, blEnc, BL_DRIVE_PID),
				 drive(fr, fl, br, bl),
				 forward(0.0),
				 right(0.0),
				 clockwise(0.0),
				 scalar(1.0),
				 shooter(SHOOTER_VICTOR_SIDECAR, SHOOTER_VICTOR),
				 rIntake(INTAKE_VICTOR_SIDECARS[0], R_INTAKE_VICTOR),
				 lIntake(INTAKE_VICTOR_SIDECARS[1], L_INTAKE_VICTOR),
				 check(false),
				 shoot(false),
				 move(false),
				 time(GetTime()),
				 photosensor(GOAL_PHOTOSENSOR_MODULE, GOAL_PHOTOSENSOR)
	{
		frEnc.SetDistancePerPulse(FR_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		flEnc.SetDistancePerPulse(FL_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		brEnc.SetDistancePerPulse(BR_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		blEnc.SetDistancePerPulse(BL_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		frEnc.SetReverseDirection(FR_DRIVE_ENCODER_REVERSE);
		flEnc.SetReverseDirection(FL_DRIVE_ENCODER_REVERSE);
		brEnc.SetReverseDirection(BR_DRIVE_ENCODER_REVERSE);
		blEnc.SetReverseDirection(BL_DRIVE_ENCODER_REVERSE);
		frEnc.Start();
		flEnc.Start();
		brEnc.Start();
		blEnc.Start();
		photosensor.SetLimitsVoltage(GOAL_PHOTOSENSOR_LOWER_BOUND, GOAL_PHOTOSENSOR_UPPER_BOUND);
	}

	/**
	 * Kill the watchdog so it doesn't interfere with the rest of the program.
	 */
	void RobotInit(void) {
		Watchdog().SetEnabled(false);
	}

	/**
	 * Print a message stating that the robot is disabled.
	 */
	void Disabled(void) {
		printf("Robot is disabled!\n");
	}

	/**
	 * Wait for the hot goal, shoot, then drive forward for 2 seconds.
	 */
	void Autonomous(void) {
		printf("Autonomous mode enabled!\n");
		check = true;
		shoot = false;
		move = false;
		time = GetTime();
		printf("Check: %s, Shoot: %s, Move: %s, LastTime: %f, CurrentTime: %f\n", check ? "T" : "F", shoot ? "T" : "F", move ? "T" : "F", time, GetTime());
		while(IsEnabled() && IsAutonomous()) {
			if(check) {
				if(photosensor.GetInWindow() == false) {
					Wait(5.5);
				}
				check = false;
				shoot = true;
				time = GetTime();
				printf("Check: %s, Shoot: %s, Move: %s, LastTime: %f, CurrentTime: %f\n", check ? "T" : "F", shoot ? "T" : "F", move ? "T" : "F", time, GetTime());
			}
			if(shoot) {
				shooter.SetSpeed(1.0);
				if(GetTime() - time >= 2.0) {
					shooter.SetSpeed(0.0);
					shoot = false;
					move = true;
					time = GetTime();
					printf("Check: %s, Shoot: %s, Move: %s, LastTime: %f, CurrentTime: %f\n", check ? "T" : "F", shoot ? "T" : "F", move ? "T" : "F", time, GetTime());
				} else {
					Wait(0.005);
				}
			}
			if(move) {
				drive.update(0.5, 0.0, 0.0);
				if(GetTime() - time >= 2.0) {
					Wait(0.01);
					drive.update(0.0, 0.0, 0.0);
					move = false;
					printf("Check: %s, Shoot: %s, Move: %s, LastTime: %f, CurrentTime: %f\n", check ? "T" : "F", shoot ? "T" : "F", move ? "T" : "F", time, GetTime());
				} else {
					Wait(0.005);
				}
			}
		}
	}

	/**
	 * Run PID Meccanum drive and shooter controls.
	 */
	void OperatorControl(void) {
		printf("Operator control enabled!\n");
		while(IsEnabled() && IsOperatorControl()) {
			if(fabs(joy.GetRawAxis(2)) < 0.2) {
				forward = 0.0;
			} else {
				forward = joy.GetRawAxis(2);
				forward *= fabs(forward);
				forward *= scalar;
			}
			if(fabs(joy.GetRawAxis(1)) < 0.2) {
				right = 0.0;
			} else {
				right = joy.GetRawAxis(1);
				right *= fabs(right);
				right *= scalar;
			}
			if(fabs(joy.GetRawAxis(3)) < 0.2) {
				clockwise = 0.0;
			} else {
				clockwise = joy.GetRawAxis(3);
				if(fabs(clockwise) <= 0.5) {
					clockwise *= 0.5;
				} else {
					clockwise *= fabs(clockwise);
				}
				clockwise *= scalar;
			}
			if(joy.GetRawButton(4)) {
				forward = 0.0;
				right = 0.0;
				clockwise = 0.0;
			}
			drive.update(forward, right, clockwise);
						//Forward  Right  Clockwise
			//printf("Forward: %f,\tRight: %f,\tClockwise: %f\n", forward, right, clockwise);
			dslcd->PrintfLine(DriverStationLCD::kUser_Line1, "Forward: %f", forward);
			dslcd->PrintfLine(DriverStationLCD::kUser_Line2, "Right: %f", right);
			dslcd->PrintfLine(DriverStationLCD::kUser_Line3, "Clockwise: %f", clockwise);
			dslcd->PrintfLine(DriverStationLCD::kUser_Line4, "Time: %f", GetTime());
			dslcd->UpdateLCD();

			if(joy.GetRawButton(1)) {
				shooter.SetSpeed(1.0);
			} else {
				shooter.SetSpeed(0.0);
			}
			if(joy.GetRawButton(5)) {
				rIntake.SetSpeed(-1.0);
				lIntake.SetSpeed(1.0);
			} else if(joy.GetRawButton(3)) {
				rIntake.SetSpeed(1.0);
				lIntake.SetSpeed(-1.0);
			} else {
				rIntake.SetSpeed(0.0);
				lIntake.SetSpeed(0.0);
			}

			Wait(0.005);
		}
	}

	/**
	 * Update SmartDashboard while in test mode.
	 */
	void Test() {
		printf("Test mode enabled!\n");
		while(IsTest()) {
			LiveWindow::GetInstance()->Run();
		}
	}
};

START_ROBOT_CLASS(Robot);
