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

    
    //Meccanum(SpeedController &_frMotor, SpeedController &flMotor, SpeedController &brMotor, SpeedController &_blMotor);
          
    //Destructor
    ~Meccanum();
    
    //Functions
    void moveDual(Joystick rjoy, Joystick ljoy);
    void move(Joystick joy);
    void update(double forward, double right, double clockwise);
    //void tank(double right, double left);
    void stop();
    
private:
    //Init Data
/*
    //Speed Controllers
	SpeedController &frMotor;
	SpeedController &flMotor;
	SpeedController &brMotor;
	SpeedController &blMotor;
*/
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
	//Fallback in case of meccanum code failure
	//bool useEncoders;
};

#endif //MECCANUM_H
