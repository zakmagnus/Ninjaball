#ifndef NB_PHYSICS_H
#define NB_PHYSICS_H

#include "vector.hpp"

typedef unsigned mass_t;

static vector2d_t default_vel;

class physics_t {
	public:
		mass_t mass;
		vector2d_t velocity;
		bool immobile;

		physics_t(mass_t mass=1, bool immobile=false,
				vector2d_t& velocity=default_vel);
		void apply_force(vector2d_t& force, real_t dt);
};

static physics_t immobile_physics(1, true);

#endif
