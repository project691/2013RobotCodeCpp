//PIDVelocityMotor.cpp

//Includes
#include <cmath>
#include <ctime>
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
                                                   maxVel(_pid[3]),
                                                   integral(0.0),
												   derivative(0.0),
												   out(0.0),
												   lastError(0.0),
												   lastTime(0)
{

}

//Destructor
PIDVelocityMotor::~PIDVelocityMotor() {}

//Functions

//PIDMotor control
void PIDVelocityMotor::run() {
    if(time(NULL) - 10 > lastTime) {
        error = target - (encoder.GetRate() / 60);
        if(target == 0.0) {
            integral = 0.0;
        }
        deltaTime = time(NULL) - lastTime;

        integral += error * deltaTime;
        derivative = (error - lastError) / deltaTime;
        out = (kp * error) + (ki * integral) + (kd * derivative);
        //TODO: Fix!
        //motor.SetSpeed(out / maxVel);
        printf("Name: %s KP: %f Target: %f CurrentRPM: %f Error: %f Get(): %d Out: %f\n", name, kp, target, (encoder.GetRate() / 60), error, encoder.Get(), out);

        lastError = error;
        lastTime = time(NULL);
    }
}

//PIDMotor control
void PIDVelocityMotor::run(double rpm) {
    target = rpm;
    run();
}

bool PIDVelocityMotor::atTarget() {
    if(abs(error - target) <= 5) {
        return true;
    } else {
        return false;
    }
}

void PIDVelocityMotor::Set(double speed, UINT8 syncGroup) {
	run(speed);
}

double PIDVelocityMotor::Get() {
	return target;
}

void PIDVelocityMotor::Disable() {}
