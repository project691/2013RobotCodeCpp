//Uptake.h

#ifndef UPTAKE_H
#define UPTAKE_H

//Includes
#include "WPILib.h"

//Declaration
class Uptake {
public:
    //Constructor
	Uptake::Uptake(Relay &_spiral, Servo &_gatekeeper);
          
    //Destructor
    ~Uptake();
    
    //Functions
    void Uptake::update(int direction, bool open);
    void Uptake::feed();
    void Uptake::run(int direction);
    void Uptake::open();
    void Uptake::close();
    void Uptake::stop();
    
private:
	Relay &sprial;
	Servo &gatekeeper;
};

#endif //UPTAKE_H
