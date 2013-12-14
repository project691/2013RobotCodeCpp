//Robot.cpp

//Includes
#include <cmath>
#include <ctime>
#include "WPILib.h"
#include "Values.h"
#include "PIDVelocityMotor.h"
#include "PIDPositionMotor.h"
#include "Meccanum.h"
#include "Shooter.h"
#include "Uptake.h"

//Main Class
class Robot : public SimpleRobot
{
private:
	Joystick driveJoy;
	Joystick shooterJoy;
	
	//Meccanum Drive
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
	//double scalar;

	//Shooter, Uptake, & Intake
	Victor shooterMotor;
	Victor shooterAccelMotor;
	Victor tiltMotor;
	Encoder tiltEnc;
	PIDPositionMotor tilt;	
	Shooter shooter;
	
	Relay spiral;
	Servo gatekeeper;
	Uptake uptake;
	
	//Intake intake;
	    
	//Autonomous
	long autoStart;

public:
	Robot(void):
		driveJoy(DRIVE_JOYSTICK),
		shooterJoy(SHOOTER_JOYSTICK),
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
		shooterMotor(SHOOTER_VICTOR_SIDECARS[0], SHOOTER_VICTOR),
		shooterAccelMotor(SHOOTER_VICTOR_SIDECARS[1], SHOOTER_ACCEL_VICTOR),
		tiltMotor(SHOOTER_TILT_VICTOR_SIDECAR, SHOOTER_TILT_VICTOR),
		tiltEnc(SHOOTER_TILT_ENCODER_SIDECAR, SHOOTER_TILT_ENCODER_A, SHOOTER_TILT_ENCODER_SIDECAR, SHOOTER_TILT_ENCODER_B, SHOOTER_TILT_ENCODER_REVERSE),
		tilt("Tilt", tiltMotor, tiltEnc, SHOOTER_TILT_PID, SHOOTER_TILT_POSITION_PID),
		shooter(shooterMotor, shooterAccelMotor, tilt),
		spiral(UPTAKE_RELAY_SIDECAR, UPTAKE_RELAY),
		gatekeeper(UPTAKE_GATEKEEPER_SIDECAR, UPTAKE_GATEKEEPER),
		uptake(spiral, gatekeeper),
		autoStart(0)
	{
		frEnc.SetDistancePerPulse(FR_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		flEnc.SetDistancePerPulse(FL_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		brEnc.SetDistancePerPulse(BR_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		blEnc.SetDistancePerPulse(BL_DRIVE_ENCODER_DISTANCE_PER_PULSE);
		
		tiltEnc.SetDistancePerPulse(SHOOTER_TILT_ENCODER_DISTANCE_PER_PULSE);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */
	void Autonomous(void)
	{
		autoStart = time(NULL);
		while(IsEnabled() && IsAutonomous()) {
			//while(!shooter.resetTilt()) {}
			
			if ((time(NULL) - autoStart) >= 2000 && (time(NULL) - autoStart) < 3000) {
				shooter.shoot(SHOOTER_RPM, SHOOTER_ACCEL_RPM);	//Full Speed
			} else if((time(NULL) - autoStart) >= 3000) {
				uptake.feed();									//Feed Disc
				shooter.shoot(SHOOTER_RPM, SHOOTER_ACCEL_RPM);	//Full Speed
			}
			
			/*if ((time(NULL) - autoStart) < 1000) {
				intake.reach(-0.5);
			} else if ((time(NULL) - autoStart) < 2000) {
				shooter.tilt(-0.25);
			} else if ((time(NULL) - autoStart) < 10000) {
				shooter.shoot(SHOOTER_RPM, SHOOTER_ACCEL_RPM); //Full Speed
				uptake.feed();                                               //Feed Disc
			} else {
				shooter.shoot(SHOOTER_RPM_IDLE, SHOOTER_ACCEL_RPM_IDLE); //Idle
			}*/
			
			/*if(((time(NULL) - autoStart) > 12000) && ((time(NULL) - autoStart) < 14000)) {
				drive.update(-0.25, 0.0, 0.0);  //Drive Forward
			} else {
				drive.update(0.0, 0.0, 0.0);    //Stop
			}*/
		}
	}

	/**
	 * Runs the motors with PID Meccanum drive. 
	 */
	void OperatorControl(void) {
		while(IsEnabled() && IsOperatorControl()) {
			if(fabs(driveJoy.GetRawAxis(2)) < 0.2) {
				forward = 0.0;
			} else {
				forward = driveJoy.GetRawAxis(2);
				forward *= fabs(forward);
				//forward *= scalar;
			}
			if(fabs(driveJoy.GetRawAxis(1)) < 0.2) {
				right = 0.0;
			} else {
				right = driveJoy.GetRawAxis(1);
				right *= fabs(right);
				//right *= scalar;
			}
			if(fabs(driveJoy.GetRawAxis(3)) < 0.2) {
				clockwise = 0.0;
			} else {
				clockwise = driveJoy.GetRawAxis(3);
				if(clockwise <= 0.5) {
					clockwise *= 0.5;
				} else {
					clockwise *= fabs(clockwise);
				}
				//clockwise *= scalar;
			}
			drive.update(forward, right, -clockwise);
						//Forward  Right  Clockwise
			
			/*if(driveJoy.GetRawButton(1)) {
				drive.update(1.0, 0.0, 0.0);
			} else if(driveJoy.GetRawButton(3)) {
				drive.update(0.75, 0.0, 0.0);
			} else if(driveJoy.GetRawButton(2)) {
				drive.update(0.5, 0.0, 0.0);
			} else {
				drive.update(0.0, 0.0, 0.0);
			}*/
			
			if(shooterJoy.GetRawButton(1)) {
				shooter.shoot(SHOOTER_RPM, SHOOTER_ACCEL_RPM);            //Full Speed
			} else if(shooterJoy.GetRawButton(2)) {
				shooter.stop();                                                         //Stop
			} else {
				shooter.shoot(SHOOTER_RPM_IDLE, SHOOTER_ACCEL_RPM_IDLE);  //Idle
			}
			/*if(shooterJoy.GetRawButton(10)) {
				while(!shooter.resetTilt()) {}
			}*/
			shooter.setAngle(shooterJoy.GetRawAxis(2), shooterJoy.GetRawButton(11));//* SHOOTER_TILT_POSITION_SCALAR);
			
			if(shooterJoy.GetRawButton(3)) {
				uptake.feed();                      //Feed Disc
			} else if(shooterJoy.GetRawButton(4)) {
				uptake.run(Relay.kForward);   //Forward
			} else if(shooterJoy.GetRawButton(5)) {
				uptake.run(Relay.kReverse);   //Backward
			} else {
				uptake.stop();                      //Stop
			}
			
			Wait(0.005);
		}
	}
	
	/**
	 * Runs during test mode
	 */
	void Test() {
		while(IsTest()) {
			LiveWindow::GetInstance()->Run();
			Wait(0.1);
		}
	}
};

START_ROBOT_CLASS(Robot);
