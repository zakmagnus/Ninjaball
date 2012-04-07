#include <math.h>
#include <vector>
#include "SDL/SDL_gfxPrimitives.h"
#include "Moveable.hpp"
#include "vector.hpp"
#include "visible.hpp"

using namespace std;

Moveable::Moveable() {
	vector2d_t nullforce = vector2d_t(0, 0);
	this->total_force = nullforce;
	this->tmp_force = nullforce;
}

void Moveable::add_force(vector2d_t& f) {
	this->total_force += f;
}

void Moveable::add_tmp_force(vector2d_t& f) {
	this->tmp_force += f;
}

//TODO wow this is stupid
void Moveable::slow_down(real_t factor) {
	this->velocity = this->velocity * factor;
}

void Moveable::handle_input(SDL_Event& event, real_t dt) {
}


void Moveable::move(real_t dt) {
	this->apply_force(this->total_force, dt);
	this->apply_force(this->tmp_force, dt);
	this->tmp_force.x = 0;
	this->tmp_force.y = 0;
	apply_normal_forces();

	//printf("%p Fy: %g, Vy: %g\n",this,this->total_force.y,this->velocity.y);
	this->x += this->velocity.x * dt;
	this->y += this->velocity.y * dt;
}

void Moveable::show(SDL_Surface *screen, SDL_Rect *camera) {
	show_visible(this, screen, camera);
}

