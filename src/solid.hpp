#ifndef NB_SOLID_H
#define NB_SOLID_H

#include <assert.h>
#include <list>
#include "vector.hpp"
#include "physics.hpp"
using namespace std;

enum { /* solid types */
	NB_SLD_RECT = 0,
	NB_SLD_BALL,
	NB_SLD_SEG,
	NB_SLD_POLY,
	NB_NUM_SLD_TYPES /* not a type, just a count */
};

#define NB_NUM_COLL_FUNCS ((NB_NUM_SLD_TYPES * (NB_NUM_SLD_TYPES + 1)) / 2)

struct rect_data_t {
	real_t h;
	real_t w;
};
struct ball_data_t {
	real_t r;
};
struct seg_data_t {
	vector2d_t *dir;
	/* if directed is true, it indicates that this segment is part of
	 * an ensemble with an "inside" which is at a negative rotation
	 * from the dir vector */
	bool directed;
};
struct poly_data_t {
	/* TODO the intention is that these segs are just used for collisions,
	 * and don't otherwise have physical properties of their own
	 * like elasticity and onbases, etc. Maybe factor out the
	 * strictly collision-relevant data to its own container? */
	void **segs;
	unsigned num_segs;
};

union solid_data_t {
	struct rect_data_t rect_data;
	struct ball_data_t ball_data;
	struct seg_data_t seg_data;
	struct poly_data_t poly_data;
};

class solid { //TODO shouldn't all solids have physics?
	protected:
		unsigned solid_type;
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

		unsigned get_solid_type(void);
		solid(real_t x=0.0, real_t y=0.0, real_t e=0.0);
		list<onbase_data>::iterator become_on(solid *onbase,
				vector2d_t& normal);
		void become_onbase(solid *backref, vector2d_t& normal,
				list<onbase_data>::iterator I);
		list<onbase_data>::iterator
			stop_being_on(list<onbase_data>::iterator I);
		virtual real_t left_edge(void);
		virtual real_t top_edge(void);
};

/* Collision function: takes two solids and returns whether they're colliding.
 * If they are, writes the vector along which the collision occurred, pointing
 * FROM s1 INTO s2. Does not do this if dir is null.
 */
typedef bool (*collision_function) (solid& s1, solid& s2, vector2d_t *dir);

/* collision function list: one per pair of types */
extern collision_function coll_funcs[NB_NUM_COLL_FUNCS];
void init_coll_funcs(void);
bool rect_rect_coll (solid& s1, solid& s2, vector2d_t *dir);
bool ball_ball_coll (solid& s1, solid& s2, vector2d_t *dir);
bool rect_ball_coll (solid& s1, solid& s2, vector2d_t *dir);
bool ball_seg_coll (solid& s1, solid& s2, vector2d_t *dir);
bool seg_seg_coll (solid& s1, solid& s2, vector2d_t *dir);
bool ball_poly_coll (solid& s1, solid& s2, vector2d_t *dir);
bool seg_poly_coll (solid& s1, solid& s2, vector2d_t *dir);
bool poly_poly_coll (solid& s1, solid& s2, vector2d_t *dir);

/* requires t1 <= t2 and both are solid types */
static unsigned get_coll_func_ind(unsigned t1, unsigned t2) {
	assert(t1 <= t2);
	assert(t2 < NB_NUM_SLD_TYPES);
	unsigned base = (t1 * (t1 + 1)) / 2;
	return base + t2;
}

/* requires t1 and t2 are actual solid types */
static collision_function get_coll_func(unsigned t1, unsigned t2) {
	if (t1 <= t2)
		return coll_funcs[get_coll_func_ind(t1, t2)];
	return coll_funcs[get_coll_func_ind(t2, t1)];
}

static bool solids_collide(solid& s1, solid& s2, vector2d_t *dir) {
	collision_function f = get_coll_func(s1.get_solid_type(),
			s2.get_solid_type());
	if (s1.get_solid_type() <= s2.get_solid_type()) {
		return f(s1, s2, dir);
	}
	else {
		bool ret = f(s2, s1, dir);

		/* dir must go from first to second */
		if (dir)
			*dir = -*dir;

		return ret;
	}
}

/* adjust velocities due to collision, in the directions of
 * the given vectors */
void resolve_collision(solid& s1, solid& s2, physics_t *p1, physics_t *p2,
		vector2d_t& dir);
/* check is a solid is actually still on one of its onbases */
bool is_still_on(solid::onbase_data& data, solid *s);

#endif
