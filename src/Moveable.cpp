#include <math.h>
#include <vector>
#include "SDL/SDL_gfxPrimitives.h"
#include "Moveable.hpp"
#include "test_utils.hpp"
#include "vector.hpp"
#include "seg.hpp"

using namespace std;

Moveable::Moveable(SDL_Surface *img, solid *s) { //TODO mass variable
	this->img = img;
	this->physics = new physics_t(1);
	vector2d_t nullforce = vector2d_t(0, 0);
	this->total_force = nullforce;
	this->tmp_force = nullforce;
	this->s = s;
}

void Moveable::add_force(vector2d_t& f) {
	this->total_force += f;
}

//TODO wow this is stupid
void Moveable::slow_down(real_t factor) {
	this->physics->velocity = this->physics->velocity * factor;
}

void Moveable::handle_input(SDL_Event& event, real_t dt) {
}

//TODO move!!
struct ang_range {
	real_t start_ang;
	real_t end_ang;
};
/* requires the normals in the onbase_data are normalized direction vectors */
void Moveable::apply_normal_forces(void) {
	vector2d_t& v = this->physics->velocity;
	//printf("starting v is %g,%g\n",v.x,v.y);

	if (v.x == 0 && v.y == 0)
		return;

	vector<ang_range> deadzones;
	list<solid::onbase_data>::iterator I, J;
	for (I = this->s->onbases->begin(); I != this->s->onbases->end(); I++) {
		real_t ang1 = dir_to_angle(-(*I).normal);
		J = I;
		J++;
		for (; J != this->s->onbases->end(); J++) {
			if ((*I).normal.x == -(*J).normal.x &&
					(*I).normal.y == -(*J).normal.y)
				continue;
			real_t ang2 = dir_to_angle(-(*J).normal);
			ang_range range;
			range.start_ang = ang1;
			range.end_ang = ang2;
			if (ang1 < ang2) {
				if (ang2 - ang1 > M_PI) {
					range.start_ang = ang2;
					range.end_ang = ang1;
				}
			}
			else {
				if (ang1 - ang2 <= M_PI) {
					range.start_ang = ang2;
					range.end_ang = ang1;
				}
			}
			//TODO find min-dist order
			deadzones.push_back(range);
			/*
			printf("deadzone: %g,%g[%g] to %g,%g[%g]\n",-(*I).normal.x,
			-(*I).normal.y, ang1,-(*J).normal.x, -(*J).normal.y,ang2);
			*/
		}
	}

	real_t v_ang = dir_to_angle(v / v.norm());

	/* check deadzones: here, v will get reduced to 0 */
	if (!deadzones.empty()) {
		for (int i = 0; i < deadzones.size(); i++) {
			//printf("%g in [%g, %g]?\n",v_ang,deadzones[i].start_ang,deadzones[i].end_ang);
			if (deadzones[i].end_ang < deadzones[i].start_ang) {
				if (v_ang <= deadzones[i].end_ang ||
						v_ang >= deadzones[i].start_ang) {
					v.x = 0;
					v.y = 0;
					//printf("deadzoned\n");
					return;
				}
			}
			else {
				if (v_ang <= deadzones[i].end_ang &&
						v_ang >= deadzones[i].start_ang) {
					v.x = 0;
					v.y = 0;
					//printf("deadzoned\n");
					return;
				}
			}
		}
	}

	for (I = this->s->onbases->begin(); I != this->s->onbases->end(); I++) {
		vector2d_t& normal = (*I).normal;

		real_t para = v.dot(normal);

		//printf("oldv: %g,%g\n",v.x,v.y);
		//printf("projection onto normal vector %g,%g is %g\n",
				//normal.x,normal.y,para);

		if (para < 0) { /* moving towards the onbase */
			v -= (normal * para); /* set v to perp component */
			vector2d_t p = normal * para;
			//printf("removed %g,%g [%g]\n",p.x,p.y,para);
		}
	}
	/*
	printf("ending v is %g,%g\n",physics->velocity.x,
	physics->velocity.y);
	*/
}

void Moveable::move(real_t dt) {
	this->physics->apply_force(this->total_force, dt);
	this->physics->apply_force(this->tmp_force, dt);
	this->tmp_force.x = 0;
	this->tmp_force.y = 0;
	apply_normal_forces();

	//printf("%p Fy: %g, Vy: %g\n",this,this->total_force.y,this->physics->velocity.y);
	this->s->x += this->physics->velocity.x * dt;
	this->s->y += this->physics->velocity.y * dt;
}

void Moveable::show(SDL_Surface *screen, SDL_Rect *camera) {
	//TODO make this suck less
	if (this->img) {
		apply_surface(this->img, screen, this->s->left_edge(),
				this->s->top_edge(), camera);
	}
	else {
		real_t x1, x2, y1, y2;
		seg **segs;
		switch (this->s->get_solid_type()) {
		case NB_SLD_POLY:
			segs = (seg **) this->s->solid_data->poly_data.segs;
			for (int i = 0; i < this->s->solid_data->
					poly_data.num_segs; i++) {
				x1 = this->s->x + segs[i]->x;
				y1 = this->s->y + segs[i]->y;
				if (camera) {
					x1 -= camera->x;
					y1 -= camera->y;
				}
				x2 = x1 + segs[i]->solid_data->seg_data.dir->x;
				y2 = y1 + segs[i]->solid_data->seg_data.dir->y;
				//TODO magic number
				lineColor(screen, x1, y1, x2, y2, 0xFFffFFff);
			}
			break;
		case NB_SLD_SEG:
			x1 = this->s->x;
			y1 = this->s->y;
			if (camera) {
				x1 -= camera->x;
				y1 -= camera->y;
			}
			x2 = x1 + this->s->solid_data->seg_data.dir->x;
			y2 = y1 + this->s->solid_data->seg_data.dir->y;
			//TODO magic number
			lineColor(screen, x1, y1, x2, y2, 0xFFffFFff);
			break;
		default:
			break;
		}
		//TODO poly/seg rendering
	}
}

void Moveable::verify_onbases(void) {
	list<solid::onbase_data>::iterator I = this->s->onbases->begin();
	while (I != s->onbases->end()) {
		if (!is_still_on(*I, s)) {
			I = this->s->stop_being_on(I);
		}
		else {
			I++;
		}
	}
}
