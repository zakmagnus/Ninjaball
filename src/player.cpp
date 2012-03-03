#include <math.h>
#include <vector>
#include "player.hpp"
#include "test_utils.hpp"
#include "vector.hpp"

#define MOVERATE (200)
#define NB_ROPE_EXTEND_RATE (175)
#define NB_ROPE_FORCE (300)
#define NB_POINTER_TURN_SPEED (M_PI / 2)

static vector2d_t rightforce = rightvec * MOVERATE;
static vector2d_t leftforce = leftvec * MOVERATE;

using namespace std;

player::player(SDL_Surface *img, solid *s) : Moveable(s, img) {
	this->rope_state = NB_ROPE_STATE_NONE;
	this->rope_taut = false;
	this->pointer_angle = 0;
	this->pointer_vel = 0;
	this->hook_flying = false;
	this->rope_attached = false;
	this->flyhook = new hook();
	this->rope = new ropedata[1];
}

void player::handle_input(SDL_Event& event, real_t dt) {
	if (event.type == SDL_KEYDOWN) {
		switch (event.key.keysym.sym) {
			case SDLK_w: 
			case SDLK_UP: 
				//this->total_force += upforce;
				if (this->rope_state != NB_ROPE_STATE_EXTD)
					this->rope_state = NB_ROPE_STATE_RETR;
				break;
			case SDLK_s: 
			case SDLK_DOWN:
				if (this->rope_state != NB_ROPE_STATE_RETR)
					this->rope_state = NB_ROPE_STATE_EXTD;
				break;
			case SDLK_a: 
			case SDLK_LEFT:
				this->total_force += leftforce;
				break;
			case SDLK_d: 
			case SDLK_RIGHT:
				total_force += rightforce;
				break;
			case SDLK_e:
				this->pointer_vel +=
					NB_POINTER_TURN_SPEED;
				break;
			case SDLK_q:
				this->pointer_vel -=
					NB_POINTER_TURN_SPEED;
				break;
			case SDLK_f:
				if (rope_attached) {
					this->remove_rope();
				}
				else if (!hook_flying) {
					hook_flying = true;
					vector2d_t hookdir =
						angle_to_dir
						(pointer_angle);
					flyhook->init(hookdir);
				}
				break;
		}
	}
	else if (event.type == SDL_KEYUP) {
		switch (event.key.keysym.sym) {
			case SDLK_w: 
			case SDLK_UP: 
				if (this->rope_state == NB_ROPE_STATE_RETR)
					this->rope_state = NB_ROPE_STATE_NONE;
				break;
			case SDLK_s: 
			case SDLK_DOWN:
				if (this->rope_state == NB_ROPE_STATE_EXTD)
					this->rope_state = NB_ROPE_STATE_NONE;
				break;
			case SDLK_a: 
			case SDLK_LEFT:
				this->total_force += rightforce;
				break;
			case SDLK_d: 
			case SDLK_RIGHT:
				this->total_force += leftforce;
				break;
			case SDLK_e:
				this->pointer_vel -=
					NB_POINTER_TURN_SPEED;
				break;
			case SDLK_q:
				this->pointer_vel +=
					NB_POINTER_TURN_SPEED;
				break;
		}
	}
}

void player::update_rope(void) {
	if (this->rope_attached) {
		vector2d_t ropedir(rope->x2 - rope->x1, rope->y2 - rope->y1);
		real_t d = ropedir.norm();
		if (d < this->rope->length) { /* loose rope */
			if (this->rope_taut) {
				this->s->stop_being_on(ropepos);
				this->rope_taut = false;
				//printf("rope became loose\n");
			}
			return;
		}
		/* else, taut rope */
		ropedir /= d; //TODO d = 0 ??
		if (!rope_taut) {
			/*TODO the player is not really "on" the solid it's
			 * attached to, so maybe NULL should be allowed as
			 * an onbase? */
			this->ropepos = this->s->become_on(s, ropedir);
			this->rope_taut = true;
			//printf("rope became taut\n");
		}
		else {
			(*ropepos).normal = ropedir;
		}
	}
}

void player::choose_action(vector<poly *> *walls, real_t dt) {
	if (hook_flying)
		hook_flying = flyhook->advance(this, dt);
	if (hook_flying) {
		this->rope->x1 = this->s->x;
		this->rope->y1 = this->s->y;
		this->rope->x2 = flyhook->x +
			flyhook->solid_data->seg_data.dir->x;
		this->rope->y2 = flyhook->y +
			flyhook->solid_data->seg_data.dir->y;
		this->rope->length = flyhook->solid_data->seg_data.dir->norm();

		for (int i = 0; i < walls->size(); i++) {
			if (solids_collide(*walls->at(i), *flyhook, NULL)) {
				hook_flying = false;
				this->attach_rope();
				break;
			}
		}
	}
}

void player::move(real_t dt) {
	this->pointer_angle += this->pointer_vel * dt;
	if (this->pointer_angle < 0)
		this->pointer_angle += 2 * M_PI;
	else if (this->pointer_angle > 2 * M_PI)
		this->pointer_angle -= 2 * M_PI;

	//TODO calculate ropedist once and pass to the following functions
	this->change_rope_len(dt);
	this->update_rope();
	Moveable::move(dt);
}

void player::change_rope_len(real_t dt) {
	if (this->rope_attached) {
		rope->x1 = this->s->x;
		rope->y1 = this->s->y;
		if (this->rope_state == NB_ROPE_STATE_EXTD) {
			//TODO this does not give smooth movement...
			real_t old_len = this->rope->length;
			this->rope->length += NB_ROPE_EXTEND_RATE * dt;
			if (this->rope->length > NB_ROPE_MAX_LEN)
				this->rope->length = old_len;
		}
		else if (this->rope_state == NB_ROPE_STATE_RETR) {
			//TODO have a maximum retraction rate?
			//printf("URL on retracting state, ");
			if (!this->rope_taut) {
				real_t old_len = this->rope->length;
				this->rope->length -= NB_ROPE_EXTEND_RATE * dt;
				if (this->rope->length <= 0)
					this->rope->length = old_len;
				//printf("loose rope, changing from %g to %g\n",old_len,this->rope->length);
			}
			else {
				real_t d = vector2d_t(rope->x2 - rope->x1,
						rope->y2 - rope->y1).norm();
				//printf("taut rope of length %g sepped by %g so ",rope->length,d);
				if (this->rope->length > d) {
					/*
					printf("setting ropelen %g to dist %g\n",
					this->rope->length,d);
					*/
					this->rope->length = d;
				}
				else {
					//printf("no-op\n");
				}

				this->tmp_force += (*ropepos).normal *
					NB_ROPE_FORCE;
			}
		}
	}
}

void player::attach_rope(void) {
	this->rope_attached = true;
	this->rope_taut = false;
	this->rope_state = NB_ROPE_STATE_NONE;
	this->update_rope(); //XXX this does not set rope->x1,y1
}

void player::remove_rope(void) {
	if (rope_taut)
		this->s->stop_being_on(ropepos);
	this->rope_attached = false;
}

void player::verify_onbases(void) {
	list<solid::onbase_data>::iterator I = this->s->onbases->begin();
	while (I != s->onbases->end()) {
		if ((rope && (ropepos == I)) || is_still_on(*I, s)) {
			I++;
		}
		else {
			I = this->s->stop_being_on(I);
		}
	}
}

void player::show(SDL_Surface *screen, SDL_Rect *camera) {
	Moveable::show(screen, camera);

	vector2d_t hookdir = angle_to_dir(this->pointer_angle);
	render_pointer(screen, (ball *)this->s, hookdir, camera);
	if (this->hook_flying || this->rope_attached) {
		render_rope(screen, *this->rope, camera);
	}
}
