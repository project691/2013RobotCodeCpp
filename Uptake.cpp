//Uptake.cpp

//Includes
#include "Uptake.h"

//Constructor
Uptake::Uptake(Relay &_spiral,
				Servo &_gatekeeper) : spiral(_spiral),
										gatekeeper(_gatekeeper)
{

}

//Destructor
Uptake::~Uptake() {}

//Functions
void Uptake::update(Relay::Value direction, bool open) {
	spiral.set(direction);
	if(open) {
		gatekeeper.setAngle(70.0);
	} else {
		gatekeeper.setAngle(180.0);
	}
}

void Uptake::feed() {
	gatekeeper.setAngle(70.0);
	spiral.set(Relay.Value.kForward);
}

void Uptake::run(Relay::Value direction) {
	spiral.set(direction);
}

void Uptake::open() {
	gatekeeper.setAngle(70.0);
}

void Uptake::close() {
	gatekeeper.setAngle(180.0);
}

void Uptake::stop() {
	spiral.set(Relay.Value.kOff);
	gatekeeper.setAngle(180.0);
}
