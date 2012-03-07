#ifndef NB_MOVEABLE_H
#define NB_MOVEABLE_H

#include "SDL/SDL.h"
#include "physics.hpp"
#include "solid.hpp"
#include "visible.hpp"

//TODO handle_input is too specific, it needs a more generic way to perform actions
class Moveable : public visible {
	protected:
		vector2d_t total_force;
		vector2d_t tmp_force;
		SDL_Surface *img;
		void apply_normal_forces(void);

	public:
                physics_t *physics;
		solid *s;

		Moveable(solid *s, void *visible_data=NULL);

		virtual void handle_input(SDL_Event&, real_t dt);
		void add_force(vector2d_t& f);
		void add_tmp_force(vector2d_t& f);
		void slow_down(real_t factor);
		virtual void move(real_t dt);
		virtual void show(SDL_Surface *screen, SDL_Rect *camera);
		virtual void verify_onbases(void);
};

#endif
