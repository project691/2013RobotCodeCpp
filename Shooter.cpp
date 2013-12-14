//Shooter.cpp

//Includes
#include "Shooter.h"

//Constructors
Shooter::Shooter(PIDVelocityMotor &_shooter,
					PIDVelocityMotor &_shooterAccel,
					PIDPositionMotor &_tilt,
					DigitalInput *_tiltLimits[]) : shooter(_shooter),
													shooterAccel(_shooterAccel),
													tilt(_tilt),
													topLimit(_tiltLimits[0]),
													bottomLimit(_tiltLimits[1]),
													useSpeedEncoders(true),
													useTiltEncoder(true),
													useLimits(true)
{

}

Shooter::Shooter(Victor &_shooterMotor,
					Victor &_shooterAccelMotor,
					PIDPositionMotor &_tilt,
					DigitalInput *_tiltLimits[],
					Counter &_tachometer,
					double _targetRPM) : shooter(_shooter),
											shooterAccel(_shooterAccel),
											tilt(_tilt),
											topLimit(_tiltLimits[0]),
											bottomLimit(_tiltLimits[1]),
											tachometer(_tachometer),
											tachTarget(_targetRPM),
											tachTime(0),
											lastTiltPos(0.0),
											tiltResetCounter(0),
											useSpeedEncoders(false),
											useTiltEncoder(true),
											useLimits(true)
{

}

Shooter::Shooter(Victor &_shooterMotor,
					Victor &_shooterAccelMotor,
					Victor &_tiltMotor,
					DigitalInput &_tiltLimits[],
					Counter &_tachometer,
					double _targetRPM) : shooter(_shooter),
											shooterAccel(_shooterAccel),
											tiltMotor(_tiltMotor),
											topLimit(_tiltLimits[0]),
											bottomLimit(_tiltLimits[1]),
											tachometer(_tachometer),
											tachTarget(_targetRPM),
											tachTime(0),
											lastTiltPos(0.0),
											tiltResetCounter(0),
											useSpeedEncoders(false),
											useTiltEncoder(false),
											useLimits(true)
{

}

Shooter::Shooter(Victor &_shooterMotor,
					Victor &_shooterAccelMotor,
					Victor &_tiltMotor,
					DigitalInput *_tiltLimits[]) : shooterMotor(_shooterMotor),
													shooterAccelMotor(_shooterAccelMotor),
													tiltMotor(_tiltMotor),
													topLimit(_tiltLimits[0]),
													bottomLimit(_tiltLimits[1]),
													useSpeedEncoders(false),
													useTiltEncoder(true),
													useLimits(true)
{

}


Shooter::Shooter(Victor &_shooterMotor,
					Victor &_shooterAccelMotor,
					PIDPositionMotor &_tilt) : shooter(_shooter),
												shooterAccel(_shooterAccel),
												tilt(_tilt),
												tachometer(_tachometer),
												tachTarget(_targetRPM),
												tachTime(0),
												lastTiltPos(0.0),
												tiltResetCounter(0),
												useSpeedEncoders(false),
												useTiltEncoder(true),
												useLimits(false)
{

}

Shooter::Shooter(Victor shooterMotor,
					Victor shooterAccelMotor,
					Victor tiltMotor) : shooterMotor(_shooterMotor),
													shooterAccelMotor(_shooterAccelMotor),
													tiltMotor(_tiltMotor),
													useSpeedEncoders(false),
													useTiltEncoder(true),
													useLimits(false)
{

}
    
//Destructor
Shooter::~Shooter() {}

//Functions
void Shooter::update(double shooterSpeed, double accelSpeed, double tiltPos) {
	if(useSpeedEncoders) {
		shooter.run(shooterSpeed);
		shooterAccel.run(accelSpeed);
		if(useTiltEncoder) {
			tilt.run(tiltPos);
		} else {
			tiltMotor.Set(tiltPos);
		}
	} else {
		shooterMotor.set(shooterSpeed);
		shooterAccelMotor.set(accelSpeed);
		if(useTiltEncoder) {
			tilt.run(tiltPos);
		} else {
			tiltMotor.Set(tiltPos);
		}
	}
}

void Shooter::shoot(double shooterSpeed, double accelSpeed) {
	if(useSpeedEncoders) {
		shooter.run(shooterSpeed);
		shooterAccel.run(accelSpeed);
	} else {
		shooterMotor.set(shooterSpeed);
		shooterAccelMotor.set(accelSpeed);
	}
}

void Shooter::setAngle(double tiltPos, bool force) {
	if(useTiltEncoder) {
		if(useLimits) {
			if(tiltPos >= 0 && !topLimit.get()) {
				tilt.run(tiltPos);
			} else if(tiltPos < 0 && !bottomLimit.get()) {
				tilt.run(tiltPos);
			} else if(force) {
				tilt.run(tiltPos);
			}
		} else {
			tilt.run(tiltPos);
		}
	} else {
		if(useLimits) {
			if(tiltPos >= 0 && !topLimit.get()) {
				tiltMotor.set(tiltPos);
			} else if(tiltPos < 0 && !bottomLimit.get()) {
				tiltMotor.set(tiltPos);
			} else if(force) {
				tiltMotor.set(tiltPos);
			}
		} else {
			tiltMotor.set(tiltPos);
		}
	}
}

bool Shooter::isReady() {
	if(useSpeedEncoders && shooter.atTarget() && shooterAccel.atTarget()) {
		return true;
	} else if(useSpeedEncoders) {
		return false;
	} else if(tachometer != null && fabs((tachometer.get() / (System.currentTimeMillis() - tachTime)) - tachTarget) <= 0.05) {
		tachTime = System.currentTimeMillis();
		tachometer.reset();
		return true;
	} else if (tachometer != null) {
		tachTime = System.currentTimeMillis();
		tachometer.reset();
		return false;
	} else {
		return true;
	}
}

bool Shooter::resetAngle() {
	//TODO: Fix!
	if(useSpeedEncoders) {
		tilt.run(tiltEncoder.getDistance());
		tilt.set(0.1);

		if(Math.abs(tiltEncoder.getDistance() - lastTiltPos) <= 5) {
			tiltResetCounter++;
		} else {
			tiltResetCounter = 0;
		}        
		lastTiltPos = tiltEncoder.getDistance();

		if(tiltResetCounter >= 5) {
			tiltMotor.set(0.0);
			tiltEncoder.reset();
			tiltResetCounter = 0;
			return true;
		} else {
			return false;
		}
	} else {
		return true;
	}
}

void Shooter::stop() {
	if(useSpeedEncoders) {
		shooter.run(0.0);
		shooterAccel.run(0.0);
	} else {
		shooterMotor.set(0.0);
		shooterAccelMotor.set(0.0);
	}
}
