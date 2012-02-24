#ifndef NB_POLYWALL_H
#define NB_POLYWALL_H

#include "SDL/SDL.h"
#include "physics.hpp"
#include "poly.hpp"

class polywall : public poly {
	public:
		physics_t *physics;
		polywall(real_t x=0.0, real_t y=0.0,
				real_t h=1.0, real_t w=1.0);
		void show(SDL_Surface *screen, SDL_Rect *camera);
};

#endif

