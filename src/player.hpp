#ifndef NB_PLAYER_H
#define NB_PLAYER_H

#include "SDL/SDL.h"
#include "physics.hpp"
#include "wall.hpp"
#include "polywall.hpp"
#include "solid.hpp"
#include "rope.hpp"
#include "Moveable.hpp"
#include "test_utils.hpp"

enum {
	NB_ROPE_STATE_NONE = 0,
	NB_ROPE_STATE_EXTD,
	NB_ROPE_STATE_RETR,
	NB_NUM_ROPE_STATES
};
class player : public Moveable {
	private:
		void update_rope(void);
		void change_rope_len(real_t dt);
		list<solid::onbase_data>::iterator ropepos;
		bool rope_taut;
		int rope_state;
		real_t pointer_angle;
		real_t pointer_vel;
		hook *flyhook;
		bool hook_flying;
		void attach_rope(void);
		void remove_rope(void);
		bool rope_attached;

	public:
		player(SDL_Surface *img, solid *s);
		virtual void handle_input(SDL_Event&, real_t dt);
		//TODO choose_action should be a Moveable thing
		//virtual void choose_action(vector<wall *> *walls, real_t dt);
		virtual void choose_action(vector<polywall *> *walls, real_t dt);
		virtual void move(real_t dt);
		virtual void verify_onbases(void);
		virtual void show(SDL_Surface *screen, SDL_Rect *camera);
		ropedata *rope;
};

#endif
