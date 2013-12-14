//Shooter.h

#ifndef SHOOTER_H
#define SHOOTER_H

//Declaration
class Shooter {
public:
    //Constructor
	Shooter(PIDVelocityMotor &_shooter,
			PIDVelocityMotor &_shooterAccel,
			PIDPositionMotor &_tilt,
			DigitalInput *_tiltLimits[]);
	
	Shooter(Victor &_shooterMotor,
			Victor &_shooterAccelMotor,
			Victor &_tiltMotor,
			DigitalInput *_tiltLimits[],
			Counter &_tachometer,
			double _targetRPM);
	
	Shooter(Victor &_shooterMotor,
			Victor &_shooterAccelMotor,
			Victor &_tiltMotor,
			DigitalInput *_tiltLimits[]);
	
	Shooter(Victor &_shooterMotor,
			Victor &_shooterAccelMotor,
			PIDPositionMotor &_tilt);
	
	Shooter(Victor &_shooterMotor,
			Victor &_shooterAccelMotor,
			Victor &_tiltMotor);
          
    //Destructor
    ~Shooter();
    
    //Functions
    void update(double shooterSpeed, double accelSpeed, double tiltPos);
    void shoot(double shooterSpeed, double accelSpeed);
    void setAngle(double tiltPos, bool force);
    bool isReady();
    bool resetAngle();
    void stop();
    
private:
    //Init Data
    //Speed Controllers
	Victor &shooterMotor;
    Victor &shooterAccelMotor;
    Victor &tiltMotor;
    //PIDMotors
    PIDVelocityMotor &shooter;
    PIDVelocityMotor &shooterAccel;
    PIDPositionMotor &tilt;
    //Limit Switches
    DigitalInput &topLimit;
    DigitalInput &bottomLimit;
    //Tachometer
    Counter &tachometer;
    double tachTarget;
    long tachTime;
    double lastTiltPos;
    int tiltResetCounter;
    //Shooter Control
    bool useSpeedEncoders;
    bool useTiltEncoder;
    bool useLimits;
};

#endif //SHOOTER_H
