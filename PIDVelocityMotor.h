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
    void Set(double speed, UINT8 syncGroup = 0);
    double Get();
    void Disable();
    
private:
    //Init data
    char * name;
    SpeedController &motor;
    Encoder &encoder;
    //PIDMotor input
    double target;
    double error;
    double deltaTime;
    //PIDMotor scale
    double kp;
    double ki;
    double kd;
    double maxVel;
    //PIDMotor out
    double integral;
    double derivative;
    double out;
    //PIDMotor loop
    double lastError;
    unsigned long lastTime;
};

#endif //PIDVELOCITYMOTOR_H
