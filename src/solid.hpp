#ifndef NB_SOLID_H
#define NB_SOLID_H

#include <assert.h>
#include <list>
#include "vector.hpp"
#include "physics.hpp"
#include "solid_types.hpp"
#include "visible.hpp"
using namespace std;
//TODO this file is full of so much crap. break it up into smaller files

#define NB_NUM_COLL_FUNCS ((NB_NUM_SLD_TYPES * (NB_NUM_SLD_TYPES + 1)) / 2)

struct ball_data_t {
	real_t r;
};
struct poly_data_t {
	segment **segs;
	unsigned num_segs;
};

union solid_data_t {
	struct ball_data_t ball_data;
	struct poly_data_t poly_data;
};

typedef unsigned solid_props_t;
typedef unsigned solid_prop_id;
enum {
	NB_DEADLY = 0,
	NB_PLAYER,
	NB_UNSTICKABLE,
	NB_VICTORY,
	NB_NUM_SLD_PROPS /* not a property, just a count */
};
void init_solid_props(solid_props_t& props);
void set_solid_props(solid_props_t& dst, solid_props_t& src);
void add_solid_props(solid_props_t& dst, solid_props_t& src);
void *get_solid_prop(solid_prop_id prop, solid_props_t& src);
void put_solid_prop(solid_prop_id prop, solid_props_t& src);
void remove_solid_prop(solid_prop_id prop, solid_props_t& src);

class solid : public visible, public physics_t {
	protected:
		unsigned solid_type;
		void (*collision_callback_func)(solid *collidee, solid& collider,
				real_t para_vel);
		void apply_normal_forces(void);
	public:
		struct onbase_data {
			solid *onbase;
			vector2d_t normal;
		};
		struct on_backref_data {
			solid *backref;
			vector2d_t normal;
			list<onbase_data>::iterator I;
		};

		//TODO make position a vector?
		real_t x, y, elasticity;
		solid_data_t *solid_data;
		list<onbase_data> *onbases;
		list<on_backref_data> *on_backrefs;
		solid_props_t props;

		unsigned get_solid_type(void);
		solid(real_t x=0.0, real_t y=0.0, real_t e=0.0,
				bool immobile=false, mass_t mass=1);
		list<onbase_data>::iterator become_on(solid *onbase,
				vector2d_t& normal);
		void become_onbase(solid *backref, vector2d_t& normal,
				list<onbase_data>::iterator I);
		list<onbase_data>::iterator
			stop_being_on(list<onbase_data>::iterator I);
		virtual void verify_onbases(void);
		void collision_callback(solid& s, real_t para_vel);

		friend solid *new_ball(solid *buf, bool immobile, real_t x, real_t y, real_t r,
				SDL_Surface *img, real_t e);
		friend solid *new_seg(solid *buf, bool immobile, real_t x, real_t y, real_t dx,
				real_t dy, bool directed,
				real_t e);
		friend solid *new_poly(solid *buf, bool immobile, real_t *points, unsigned num_pts,
				real_t x, real_t y, real_t e, unsigned color);
		friend solid *new_poly(solid *buf, bool immobile, real_t x, real_t y, real_t e,
				int num_points, ...);
};

/* Collision function: takes two solids and returns whether they're colliding.
 * If they are, writes the vector along which the collision occurred, pointing
 * FROM s1 INTO s2. Also writes overlap between solids, which is the minimum
 * distance they need to be separated along the collision axis. Does not write
 * if the passed-in buffer argument is NULL.
 */
struct collision_data {
	vector2d_t dir;
	real_t overlap;
};
typedef bool (*collision_function) (solid& s1, solid& s2,
		struct collision_data *data);

/* collision function list: one per pair of types */
void init_coll_funcs(void);
extern collision_function coll_funcs[NB_NUM_COLL_FUNCS];
bool ball_ball_coll (solid& s1, solid& s2, struct collision_data *data);
bool ball_poly_coll (solid& s1, solid& s2, struct collision_data *data);
bool poly_poly_coll (solid& s1, solid& s2, struct collision_data *data);

#include <stdio.h>
/* requires t1 <= t2 and both are solid types */
static unsigned get_coll_func_ind(unsigned t1, unsigned t2) {
	assert(t1 <= t2);
	assert(t2 < NB_NUM_SLD_TYPES);
	unsigned base = ((2 * NB_NUM_SLD_TYPES - t1 + 1) * t1) / 2;
	return base + t2 - t1;
}

/* requires t1 and t2 are actual solid types */
static collision_function get_coll_func(unsigned t1, unsigned t2) {
	if (t1 <= t2)
		return coll_funcs[get_coll_func_ind(t1, t2)];
	return coll_funcs[get_coll_func_ind(t2, t1)];
}

static bool solids_collide(solid& s1, solid& s2, struct collision_data *data) {
	collision_function f = get_coll_func(s1.get_solid_type(),
			s2.get_solid_type());
	if (s1.get_solid_type() <= s2.get_solid_type()) {
		return f(s1, s2, data);
	}
	else {
		bool ret = f(s2, s1, data);

		/* dir must go from first to second */
		if (data)
			data->dir = -(data->dir);

		return ret;
	}
}

/* adjust velocities due to collision, in the directions of
 * the given vectors */
void resolve_collision(solid& s1, solid& s2, struct collision_data& data);
/* check is a solid is actually still on one of its onbases */
//TODO make this a member function?
bool is_still_on(solid::onbase_data& data, solid *s);

//TODO move these declarations?
solid *new_ball(solid *buf, bool immobile, real_t x=0.0, real_t y=0.0, real_t r=0.0,
		SDL_Surface *img=NULL, real_t e=0.0);
solid *new_poly(solid *buf, bool immobile, real_t *points, unsigned num_pts,
		real_t x=0.0, real_t y=0.0, real_t e=0.0,
		unsigned color=0xFFffFFff);
solid *new_poly(solid *buf, bool immobile, real_t x, real_t y, real_t e,
		int num_points, ...);

//TODO move this

bool seg_poly_intersection(solid *p, segment *s, real_t& cx, real_t& cy,
		real_t sx, real_t sy);
#endif
