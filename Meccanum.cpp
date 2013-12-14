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
												maxVel(0.0),
												topVel(0.0)
{

}

/*
Meccanum::Meccanum(SpeedController &_frMotor,
					SpeedController &_flMotor,
					SpeedController &_brMotor,
					SpeedController &_blMotor) : frMotor(_frMotor),
													flMotor(_flMotor),
													brMotor(_brMotor),
													blMotor(_blMotor),
													fr(NULL),
													fl(NULL),
													br(NULL),
													bl(NULL),
													frVel(0.0),
													flVel(0.0),
													brVel(0.0),
													blVel(0.0),
													maxVel(0.0),
													topVel(0.0),
													useEncoders(false)
{

}
*/

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
	//Figure out motor speeds
	frVel = -(forward + right - clockwise); //0--
	flVel = (forward - right + clockwise);  //0++
	brVel = -(forward - right - clockwise); //0+-
	blVel = (forward + right + clockwise);  //0-+

	//Set the motor objects to the output value for motor control and additional scaling
	//Find the maxVelimum speed
	if(topVel < fabs(frVel)) {topVel = frVel;}
	else if(topVel < fabs(flVel)) {topVel = flVel;}
	else if(topVel < fabs(brVel)) {topVel = brVel;}
	else if(topVel < fabs(blVel)) {topVel = blVel;}

	//if(useEncoders) {   
		//Scale motor power if it is above 100%
		if(fabs(topVel) > maxVel) {
			frVel /= topVel;
			flVel /= topVel;
			brVel /= topVel;
			blVel /= topVel;
			frVel *= maxVel;
			flVel *= maxVel;
			brVel *= maxVel;
			blVel *= maxVel;
		}
		fr.run(frVel);
		fl.run(flVel);
		br.run(brVel);
		bl.run(blVel);
	/*} else {
		//Scale motor power if it is above 100%
		if(fabs(topVel) > 1.0) {
			frVel /= topVel;
			flVel /= topVel;
			brVel /= topVel;
			blVel /= topVel;
		}
		frMotor.Set(frVel);
		flMotor.Set(flVel);
		brMotor.Set(brVel);
		blMotor.Set(blVel);
	}*/
}

/*
void Meccanum::tank(double right, double left) {
	if(useEncoders) {
		fr.run(right);
		fl.run(left);
		br.run(right);
		bl.run(left);
	} else {
		frMotor.Set(right);
		flMotor.Set(left);
		brMotor.Set(right);
		blMotor.Set(left);
	}
}
*/

void Meccanum::stop() {
	//if(useEncoders) {
		fr.run(0.0);
		fl.run(0.0);
		br.run(0.0);
		bl.run(0.0);
	/*} else {
		frMotor.Set(0.0);
		flMotor.Set(0.0);
		brMotor.Set(0.0);
		blMotor.Set(0.0);
	}*/
}
