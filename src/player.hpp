#ifndef NB_PLAYER_H
#define NB_PLAYER_H

#include "SDL/SDL.h"
#include "physics.hpp"
#include "solid.hpp"
#include "rope.hpp"
#include "Moveable.hpp"

#define NB_HOOK_SPD (1800.0)
#define NB_ROPE_MAX_LEN (600)

class hook : public solid {
	public:
		hook() {new_seg(this, true, 0, 0, 1, 1, false);};
		void init(Moveable *m, vector2d_t& dir);
		bool advance(Moveable *m, real_t dt);
};

void render_rope(SDL_Surface *srf, ropedata& rope, SDL_Rect *camera);
void render_pointer(SDL_Surface *srf, solid *b, vector2d_t& dir, SDL_Rect *camera);

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
		vector2d_t pointer_dir;
		hook *flyhook;
		bool hook_flying;
		void attach_rope(void);
		void remove_rope(void);
		bool rope_attached;

		bool a_on, d_on, q_on, e_on;

	public:
		player();
		virtual void handle_input(SDL_Event&, real_t dt);
		//TODO choose_action should be a Moveable thing
		virtual void choose_action(vector<solid *> *walls, real_t dt);
		virtual void move(real_t dt);
		virtual void verify_onbases(void);
		virtual void show(SDL_Surface *screen, SDL_Rect *camera);
		void mouse_at(int mx, int my, char button);
		ropedata *rope;
		bool alive;
};

void player_collided(solid *player, solid& other);

#endif
