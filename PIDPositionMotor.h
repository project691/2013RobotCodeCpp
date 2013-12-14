//PIDPositionMotor.h

#ifndef PIDPOSITIONMOTOR_H
#define PIDPOSITIONMOTOR_H

//Includes
#include "PIDVelocityMotor.h"
#include "WPILib.h"

//Declaration
class PIDPositionMotor {
public:
    //Constructor
    PIDPositionMotor(char * _name,
    				 Victor &_vic,
    				 Encoder &_encoder,
    				 const double _velPID[],
                     const double _posPID[]);
          
    //Destructor
    ~PIDPositionMotor();
    
    //Functions
    void run();
    void run(double pos);  
    
private:
    //Init data
    char * name;
    PIDVelocityMotor motor;
    Encoder &encoder;
    //PIDMotor input
    double position;
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

#endif //PIDPOSITIONMOTOR_H
