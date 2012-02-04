#ifndef NB_WALL_H
#define NB_WALL_H

#include "SDL/SDL.h"
#include "physics.hpp"
#include "rect.hpp"

class wall : public rect {
	private:
		SDL_Surface *img;
	public:
		physics_t *physics;
		wall(SDL_Surface *img, real_t x=0.0, real_t y=0.0,
				real_t h=0.0, real_t w=0.0);
		void show(SDL_Surface *screen, SDL_Rect *camera);
};

#endif
