#include "physics.hpp"

physics_t::physics_t(mass_t mass, bool immobile, vector2d_t& velocity) {
	this->mass = mass;
	this->velocity = velocity;
	this->immobile = immobile;
}

void physics_t::apply_force(vector2d_t& force, real_t dt) {
	if (!this->immobile) {
		vector2d_t& dv = force / ((real_t)this->mass) * dt;

		this->velocity += dv;
	}
	else {
		//TODO? "take damage?"
	}
}
