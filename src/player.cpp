#include <math.h>
#include <vector>
#include "SDL/SDL_gfxPrimitives.h"
#include "player.hpp"
#include "vector.hpp"
#include "level_utils.hpp"

#define MOVERATE (200)
#define NB_ROPE_EXTEND_RATE (175)
#define NB_ROPE_FORCE (300)

static vector2d_t rightforce = rightvec * MOVERATE;
static vector2d_t leftforce = leftvec * MOVERATE;

using namespace std;

player::player() : Moveable() {
	this->rope_state = NB_ROPE_STATE_NONE;
	this->rope_taut = false;
	this->pointer_dir = upvec;
	this->hook_flying = false;
	this->rope_attached = false;
	this->flyhook = new hook();
	this->rope = new ropedata[1];
	this->a_on = this->d_on = this->q_on = this->e_on = false;
	this->alive = true;
	this->points = 0;

	put_solid_prop(NB_PLAYER, this->props);
	this->collision_callback_func = player_collided;
}

void player_collided(solid *p, solid& other, real_t para_vel) {
	bool deadly = (bool) get_solid_prop(NB_DEADLY, other.props);
	if (deadly) {
		((player *)p)->alive = false;
	}
	bool win = (bool) get_solid_prop(NB_VICTORY, other.props);
	if (win) {
		((player *)p)->points++;
		other.visible_data = (void *) 0x1111FFff;
	}
	if (fabs(para_vel) >= 50)
		Mix_PlayChannel(2, soft_bump, 0);
}

//TODO some sort of "getkeystate()" might be better
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
				this->a_on = true;
				//this->total_force += leftforce;
				break;
			case SDLK_d: 
			case SDLK_RIGHT:
				this->d_on = true;
				//total_force += rightforce;
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
				this->a_on = false;
				//this->total_force += rightforce;
				break;
			case SDLK_d: 
			case SDLK_RIGHT:
				this->d_on = false;
				//this->total_force += leftforce;
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
				this->stop_being_on(ropepos);
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
			this->ropepos = this->become_on(this, ropedir);
			this->rope_taut = true;
			//printf("rope became taut\n");
		}
		else {
			(*ropepos).normal = ropedir;
		}
	}
}

void player::choose_action(vector<solid *> *walls, real_t dt) {
	if (a_on)
		this->add_tmp_force(leftforce);
	if (d_on)
		this->add_tmp_force(rightforce);

	if (hook_flying)
		hook_flying = flyhook->advance(this, dt);
	if (hook_flying) {
		this->rope->x1 = this->x;
		this->rope->y1 = this->y;
		this->rope->x2 = flyhook->x + flyhook->dir.x;
		this->rope->y2 = flyhook->y + flyhook->dir.y;
		this->rope->length = vector2d_t(this->rope->x2 - this->rope->x1,
				this->rope->y2 - this->rope->y1).norm();

		for (int i = 0; i < walls->size(); i++) {
			if (walls->at(i)->get_solid_type() == NB_SLD_POLY) {
				real_t cx, cy;
				bool hit = seg_poly_intersection(walls->at(i),
						flyhook, cx, cy);
				if (hit) {
					this->hook_flying = false;
					if (!get_solid_prop(NB_UNSTICKABLE,
								walls->at(i)->props)) {
						Mix_PlayChannel(1, hook_hit, 0);
						this->rope->x2 = cx;
						this->rope->y2 = cy;
						this->attach_rope();
					}
					else {
						Mix_PlayChannel(1, hook_clank, 0);
					}
					break;
				}
			}
			else {
				printf("skipping hook intersection with non-poly wall\n");
				//TODO ??? :^(
			}
			/*
			if (solids_collide(*walls->at(i), *flyhook, NULL)) {
				hook_flying = false;
				if (!get_solid_prop(NB_UNSTICKABLE,
							walls->at(i)->props))
					this->attach_rope();
				break;
			}
			*/
		}
	}
}

void player::mouse_at(int mx, int my, char button) {
	vector2d_t pv(mx - this->x, my - this->y);
	if (pv.normalize())
		this->pointer_dir = pv;

	if (button & SDL_BUTTON_LEFT) {
		if (!(rope_attached || hook_flying)) {
			hook_flying = true;
			flyhook->init(this, this->pointer_dir);
			Mix_PlayChannel(0, hook_shot, 0);
		}
	}
	else {
		hook_flying = false;
		if (rope_attached)
			this->remove_rope();
	}
}

void player::move(real_t dt) {
	//TODO calculate ropedist once and pass to the following functions
	this->change_rope_len(dt);
	this->update_rope();
	Moveable::move(dt);
}

void player::change_rope_len(real_t dt) {
	if (this->rope_attached) {
		rope->x1 = this->x;
		rope->y1 = this->y;
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
	this->update_rope(); //XXX this does not set rope->x1,y1
}

void player::remove_rope(void) {
	if (rope_taut)
		this->stop_being_on(ropepos);
	this->rope_attached = false;
}

void player::verify_onbases(void) {
	list<solid::onbase_data>::iterator I = this->onbases->begin();
	while (I != this->onbases->end()) {
		if ((rope && (ropepos == I)) || is_still_on(*I, this)) {
			I++;
		}
		else {
			I = this->stop_being_on(I);
		}
	}
}

void player::show(SDL_Surface *screen, SDL_Rect *camera) {
	Moveable::show(screen, camera);

	render_pointer(screen, this, this->pointer_dir, camera);
	if (this->hook_flying || this->rope_attached) {
		render_rope(screen, *this->rope, camera);
	}
}
//TODO use a visible here
void render_pointer(SDL_Surface *srf, solid *b, vector2d_t& dir, SDL_Rect *camera) {
	vector2d_t endpt;
	endpt.x = b->x;
	endpt.y = b->y;
	endpt += dir * (b->solid_data->ball_data.r + 2); //TODO magic!
	real_t x1 = b->x, x2 = endpt.x, y1 = b->y, y2 = endpt.y;
	if (camera) {
		x1 -= camera->x;
		x2 -= camera->x;
		y1 -= camera->y;
		y2 -= camera->y;
	}
	lineColor(srf, x1, y1, x2, y2, 0xFFffFFff);
}

//TODO move this?
void hook::init(Moveable *m, vector2d_t& dir) {
	this->x = m->x;
	this->y = m->y;
	this->dir = dir;
	this->flight_dir = dir;
}

bool hook::advance(Moveable *m, real_t dt) {
	real_t d = vector2d_t(m->x - (this->x + this->dir.x),
			m->y - (this->y + this->dir.y)).norm();
	if (d >= NB_ROPE_MAX_LEN)
		return false;

	this->x += this->dir.x;
	this->y += this->dir.y;
	this->dir = this->flight_dir * (dt * NB_HOOK_SPD);
	return true;
}

//TODO use a visible here
void render_rope(SDL_Surface *srf, ropedata& rope, SDL_Rect *camera) {
	//TODO render loose rope hanging loosely
	real_t x1 = rope.x1, x2 = rope.x2, y1 = rope.y1, y2 = rope.y2;
	if (camera) {
		x1 -= camera->x;
		x2 -= camera->x;
		y1 -= camera->y;
		y2 -= camera->y;
	}
	real_t d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
	unsigned color = NB_ROPE_COLOR;
	if (d < rope.length) {
		real_t looseness = 1 - (d / rope.length);
		unsigned tight_comp_r = ((unsigned)
			((NB_ROPE_COLOR&0xff000000) * (1 - looseness)))
			& 0xFF000000;
		unsigned loose_comp_r = ((unsigned)
			((NB_ROPE_LOOSE_COLOR&0xff000000) * looseness))
			& 0xFF000000;
		unsigned tight_comp_g = ((unsigned)
			((NB_ROPE_COLOR&0x00ff0000) * (1 - looseness)))
			& 0x00ff0000;
		unsigned loose_comp_g = ((unsigned)
			((NB_ROPE_LOOSE_COLOR&0x00ff0000) * looseness))
			& 0x00ff0000;
		unsigned tight_comp_b = ((unsigned)
			((NB_ROPE_COLOR&0x0000FF00) * (1 - looseness)))
			& 0x0000FF00;
		unsigned loose_comp_b = ((unsigned)
			((NB_ROPE_LOOSE_COLOR&0x0000FF00) * looseness))
			& 0x0000FF00;
		color = loose_comp_r + tight_comp_r + loose_comp_g +
			tight_comp_g + loose_comp_b + tight_comp_b;
		color |= 0xff;
	}
	lineColor(srf, x1, y1, x2, y2, color);
}

