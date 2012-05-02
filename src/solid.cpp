#include <math.h>
#include <vector>
#include "solid.hpp"
#include "math_utils.hpp"

using namespace std;

collision_function coll_funcs[NB_NUM_COLL_FUNCS];

/* Absolute parallel velocity threshold such that the mover
 * starts being "on" the solid it's colliding with. */
#define NB_COLL_V_THRESH  (7.0)
/* Distance to move an object towards something it's on in order
 * to check for a collision and decide if it's really still on. */
#define NB_ONCHECK_P_DIST (1.5)

solid::solid(real_t x, real_t y, real_t e, bool immobile, mass_t mass)
	: physics_t(mass, immobile),
	visible(NB_NUM_VIS_TYPES) {
	this->x = x;
	this->y = y;
	this->elasticity = e;

	this->onbases = new list<onbase_data>();
	this->on_backrefs = new list<on_backref_data>();
	//TODO make abstract?
	this->solid_type = NB_NUM_SLD_TYPES;
	this->solid_data = new solid_data_t;
	this->collision_callback_func = NULL;

	init_solid_props(this->props);
}

void solid::collision_callback(solid& s) {
	if (this->collision_callback_func)
		this->collision_callback_func(this, s);
}

void init_solid_props(solid_props_t& props) {
	props = 0;
}

void set_solid_props(solid_props_t& dst, solid_props_t& src) {
	dst = src;
}

void add_solid_props(solid_props_t& dst, solid_props_t& src) {
	dst |= src;
}

void *get_solid_prop(solid_prop_id prop, solid_props_t& src) {
	assert(prop < NB_NUM_SLD_PROPS);

	return (void *) (src & (1 << prop));
}

void put_solid_prop(solid_prop_id prop, solid_props_t& src) {
	assert(prop < NB_NUM_SLD_PROPS);

	src |= (1 << prop);
}

void remove_solid_prop(solid_prop_id prop, solid_props_t& src) {
	assert(prop < NB_NUM_SLD_PROPS);

	src &= ~(1 << prop);
}

unsigned solid::get_solid_type(void) {
	return this->solid_type;
}

#include <stdio.h>

bool ball_ball_coll (solid& s1, solid& s2, struct collision_data *data) {
	assert(s1.get_solid_type() == NB_SLD_BALL);
	assert(s2.get_solid_type() == NB_SLD_BALL);
	vector2d_t c_axis;
	c_axis.x = s2.x - s1.x;
	c_axis.y = s2.y - s1.y;
	real_t r1 = s1.solid_data->ball_data.r;
	real_t r2 = s2.solid_data->ball_data.r;
	real_t dsq = c_axis.dot(c_axis);

	if (dsq > (r1 + r2) * (r1 + r2))
		return false;

	if (data) {
		data->dir = c_axis;
		data->overlap = r1 + r2 - sqrt(dsq);
	}

	return true;
}

typedef void (*proj_func) (solid& s, vector2d_t& axis, real_t& min,
		real_t& max);
static void project_poly(solid& s, vector2d_t& axis, real_t& min,
		real_t& max) {
	const struct poly_data_t& pd = s.solid_data->poly_data;
	for (int i = 0; i < pd.num_segs; i++) {
		vector2d_t point(pd.segs[i]->x + s.x, pd.segs[i]->y + s.y);
		real_t proj = point.dot(axis);
		if ((i == 0) || proj < min)
			min = proj;
		if ((i == 0) || proj > max)
			max = proj;
	}
}
static void project_ball (solid& s, vector2d_t& axis, real_t& min,
		real_t& max) {
	const struct ball_data_t& bd = s.solid_data->ball_data;
	vector2d_t center(s.x, s.y);
	real_t mid = center.dot(axis);
	min = mid - bd.r;
	max = mid + bd.r;
}

real_t get_separation (solid& s1, solid& s2, proj_func p1, proj_func p2,
		vector2d_t& axis) {
	real_t s1min, s1max, s2min, s2max;
	p1(s1, axis, s1min, s1max);
	p2(s2, axis, s2min, s2max);
	/*
	   printf("s1 projects onto %g,%g as %g,%g\n",axis.x,axis.y,s1min,s1max);
	   printf("s2 projects onto %g,%g as %g,%g\n",axis.x,axis.y,s2min,s2max);
	 */
	real_t overlap = SINGLE_DIM_OVERLAP(s1min, s1max, s2min, s2max);
	if (overlap <= 0) { /* found separating axis; no collision */
		//printf("separate!\n");
		return 0;
	}
	if (s2min < s1min) /* pointing the wrong way */
		return -overlap;
}

/* requires s1 is the ball and s2 is a sane poly (no nulls) */
bool ball_poly_coll (solid& s1, solid& s2, struct collision_data *data) {
	assert(s1.get_solid_type() == NB_SLD_BALL);
	assert(s2.get_solid_type() == NB_SLD_POLY);

	const struct poly_data_t& pd = s2.solid_data->poly_data;
	assert(pd.segs);
	const struct ball_data_t& bd = s1.solid_data->ball_data;

#define GET_GLOBAL_SEG(i) do {\
	assert(pd.segs[(i)]);\
	global_seg = *pd.segs[(i)];\
	global_seg.x += s2.x;\
	global_seg.y += s2.y;\
} while (0)
	segment global_seg; /* global coordinates, not poly-centric ones */
	real_t min_sep;
	vector2d_t min_axis;
	vector2d_t corner_axis;
	real_t min_corner_dist = -1;
	for (int i = 0; i < pd.num_segs; i++) {
		GET_GLOBAL_SEG(i);
		vector2d_t axis = global_seg.dir;
		axis.turn_pos()->normalize();
		real_t sep = get_separation(s1, s2, project_ball, project_poly,
				axis);
		if (sep == 0)
			return false;
		if (sep < 0) {
			axis = -axis;
			sep = -sep;
		}

		if ((i == 0) || sep < min_sep) {
			min_axis = axis;
			min_sep = sep;
		}

		vector2d_t caxis(global_seg.x - s1.x, global_seg.y - s1.y);
		real_t caxis_dist = caxis.norm();
		if (min_corner_dist < 0 || caxis_dist < min_corner_dist) {
			corner_axis = caxis;
			min_corner_dist = caxis_dist;
		}
	}
	corner_axis.normalize();
	real_t sep = get_separation(s1, s2, project_ball, project_poly,
		corner_axis);
	if (sep == 0)
		return false;
	if (sep < 0) {
		corner_axis = -corner_axis;
		sep = -sep;
	}

	if (sep < min_sep) {
		min_axis = corner_axis;
		min_sep = sep;
	}

	if (data) {
		data->dir = min_axis;
		data->overlap = min_sep;
	}

	return true;
}

/* requires s1 and s2 are sane polygons (no nulls) */
bool poly_poly_coll (solid& s1, solid& s2, struct collision_data* data) {
	assert(s1.get_solid_type() == NB_SLD_POLY);
	assert(s1.get_solid_type() == NB_SLD_POLY);

	const struct poly_data_t& pd1 = s1.solid_data->poly_data;
	const struct poly_data_t& pd2 = s2.solid_data->poly_data;

	assert(pd1.segs);
	assert(pd2.segs);
	
	real_t min_sep;
	vector2d_t min_axis;
	//TODO these loops are similar; factor out?
	for (int i = 0; i < pd1.num_segs; i++) {
		vector2d_t axis = pd1.segs[i]->dir;
		axis.turn_pos();
		axis.normalize();

		real_t sep = get_separation(s1, s2, project_poly, project_poly, axis);
		if (sep == 0)
			return false;
		if (sep < 0) {
			axis = -axis;
			sep = -sep;
		}

		if ((i == 0) || sep < min_sep) {
			min_axis = axis;
			min_sep = sep;
		}
	}
	for (int i = 0; i < pd2.num_segs; i++) {
		vector2d_t axis = pd2.segs[i]->dir;
		axis.turn_pos();
		axis.normalize();

		real_t sep = get_separation(s1, s2, project_poly, project_poly, axis);
		if (sep == 0)
			return false;
		if (sep < 0) {
			axis = -axis;
			sep = -sep;
		}

		if (sep < min_sep) {
			min_axis = axis;
			min_sep = sep;
		}
	}
	//printf("min overlap is %g on %g,%g\n",min_overlap,min_axis.x,min_axis.y);

	if (data) {
		data->dir = min_axis;
		data->overlap = min_sep;
	}
	
	return true;
}

void solid::verify_onbases(void) {
	list<solid::onbase_data>::iterator I = this->onbases->begin();
	while (I != this->onbases->end()) {
		if (!is_still_on(*I, this)) {
			I = this->stop_being_on(I);
		}
		else {
			I++;
		}
	}
}

//TODO move!!
struct ang_range {
	real_t start_ang;
	real_t end_ang;
};
/* requires the normals in the onbase_data are normalized direction vectors */
void solid::apply_normal_forces(void) {
	vector2d_t& v = this->velocity;
	//printf("starting v is %g,%g\n",v.x,v.y);

	if (v.x == 0 && v.y == 0)
		return;

	vector<ang_range> deadzones;
	list<solid::onbase_data>::iterator I, J;
	for (I = this->onbases->begin(); I != this->onbases->end(); I++) {
		real_t ang1 = dir_to_angle(-(*I).normal);
		J = I;
		J++;
		for (; J != this->onbases->end(); J++) {
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

	for (I = this->onbases->begin(); I != this->onbases->end(); I++) {
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

/* Sets up record keeping so a solid knows which normal forces are exerted
 * on it. Also notifies the onbase of this.
 * requires onbase be non-null */
list<solid::onbase_data>::iterator solid::become_on(solid *onbase,
		vector2d_t& normal) {
	assert(onbase);
	onbase_data data;
	data.onbase = onbase;
	data.normal = normal;
	this->onbases->push_front(data);
	printf("a solid is now on, with normal %g,%g\n",normal.x,normal.y);

	onbase->become_onbase(this, normal, this->onbases->begin());

	return this->onbases->begin();
}

/* Installs a backreference in the onbase when something becomes on.
 * Only meant to be called from become_on(). If called from elsewhere,
 * this is likely an ERROR! */
void solid::become_onbase(solid *backref, vector2d_t& normal,
		list<onbase_data>::iterator I) {
	on_backref_data data;
	data.backref = backref;
	data.normal = normal;
	data.I = I;
	this->on_backrefs->push_back(data);
}

list<solid::onbase_data>::iterator
solid::stop_being_on(list<solid::onbase_data>::iterator I) {
	vector2d_t& normal = (*I).normal;
	list<solid::onbase_data>::iterator ret = this->onbases->erase(I);

	bool normal_still_applies = false;
	for (list<on_backref_data>::iterator J = this->on_backrefs->begin();
			J != this->on_backrefs->end(); J++) {
		if ((*J).normal == normal) {
			normal_still_applies = true;
			break;
		}
	}

	if (!normal_still_applies) {
		/* any solid that uses this as an onbase with this normal
		 * is no longer on; notify them */

		list<on_backref_data>::iterator J =
			this->on_backrefs->begin();
		while (J != this->on_backrefs->end()) {
			if ((*J).normal == normal) {
				(*J).backref->stop_being_on((*J).I);
				J = this->on_backrefs->erase(J);
			}
			else {
				J++;
			}
		}
	}

	return ret;
}

/* MUST be called before collisions are used! */
void init_coll_funcs(void) {
	//XXX make sure to put the types in ascending order!
	coll_funcs[get_coll_func_ind(NB_SLD_BALL, NB_SLD_BALL)] =
		ball_ball_coll;
	coll_funcs[get_coll_func_ind(NB_SLD_BALL, NB_SLD_POLY)] =
		ball_poly_coll;
	coll_funcs[get_coll_func_ind(NB_SLD_POLY, NB_SLD_POLY)] =
		poly_poly_coll;
}

/* Requires dir is a direction and not a 0 vector, pointing FROM s1 INTO s2. */
void resolve_collision(solid& s1, solid& s2, struct collision_data& data) {
	/*TODO This artificial immobility is fucked! If the onbase is
	 * actually moving in the direction of its normal, then that
	 * collision should be treated as a mobile one. */
	/* break velocities into parallel and perpendicular components */
	vector2d_t para_c1, para_c2, perp_c1, perp_c2;
	real_t c1, c2, new_c1, new_c2;

	real_t norm_d = data.dir.norm();
	assert(norm_d > 0.0);
	data.dir = data.dir / norm_d;

	//TODO when should this happen? what if these adjust velocities?
	s1.collision_callback(s2);
	s2.collision_callback(s1);

	vector2d_t& v1 = s1.velocity;
	c1 = 0.0;
	if (v1.x != 0 || v1.y != 0)
		c1 = v1.dot(data.dir);
	vector2d_t& v2 = s2.velocity;
	c2 = 0.0;
	if (v2.x != 1 || v1.y != 0)
		c2 = v2.dot(data.dir);

	bool immobile1 = s1.immobile;
	if ((!immobile1) && c1 <= 0) {
		vector2d_t& normal = data.dir;
		for (list<solid::onbase_data>::iterator I = s1.onbases->begin();
				I != s1.onbases->end(); I++) {
			if ((*I).normal == normal) {
				immobile1 = true;
				break;
			}
		}
	}
	bool immobile2 = s2.immobile;
	if (!immobile2 && c2 >= 0) {
		vector2d_t& normal = -data.dir; /* from s2 into s1 */
		for (list<solid::onbase_data>::iterator I = s2.onbases->begin();
				I != s2.onbases->end(); I++) {
			if ((*I).normal == normal) {
				immobile2 = true;
				break;
			}
		}
	}

	/* use parallels to calculate new parallels */
	if ((!immobile1) && (!immobile2)) {
		mass_t total_mass = s1.mass + s2.mass;
		new_c1 = (c1 * (s1.mass - s2.mass) + 2 * s2.mass * c2)
			/ total_mass;
		new_c2 = (c2 * (s2.mass - s1.mass) + 2 * s1.mass * c1)
			/ total_mass;
	}
	else {
		if (!immobile1)
			new_c1 = -c1;
		if (!immobile2)
			new_c2 = -c2;
	}

	new_c1 *= s1.elasticity;
	new_c2 *= s2.elasticity;

	if (immobile2 && fabs(new_c1) < NB_COLL_V_THRESH) {
		vector2d_t normal = -data.dir; /* from s2 into s1 */

		if (!s2.immobile)
			printf("mobile onbase!\n");
		printf("s1 is on (%g); dir = %g,%g\n",new_c1,data.dir.x,data.dir.y);
		s1.become_on(&s2, normal); //TODO double onness??
		new_c1 = 0;
	}
	if (immobile1 && fabs(new_c2) < NB_COLL_V_THRESH) {
		vector2d_t normal = data.dir;

		if (!s1.immobile)
			printf("mobile onbase!\n");
		printf("s2 is on; dir = %g,%g\n",data.dir.x,data.dir.y);
		s2.become_on(&s1, normal); //TODO double onness??
		new_c2 = 0;
	}

	real_t delta_c1 = new_c1 - c1;
	/* solids' velocities should adjust AWAY from each other */
	if (delta_c1 > 0)
		delta_c1 = -delta_c1;
	real_t delta_c2 = new_c2 - c2;
	if (delta_c2 < 0)
		delta_c2 = -delta_c2;
	para_c1 = data.dir * (c1 + delta_c1);
	para_c2 = data.dir * (c2 + delta_c2);

	/* use new parallels to calculate new velocities */
	if (!immobile1) {
		//printf("delta-c1: %g\n",new_c1-c1);
		//printf("v1,1 = %g,%g\n",s1.velocity.x,s1.velocity.y);
		//printf("para1,1 = %g,%g[%g]\n",(dir*c1).x,(dir*c1).y,c1);
		//printf("para1,2 = %g,%g [%g <- %g]\n",para_c1.x,para_c1.y,c1+delta_c1,new_c1);
		perp_c1 = v1 - (data.dir * c1);
		//printf("perp1 = %g,%g\n",perp_c1.x,perp_c1.y);
		s1.velocity = para_c1 + perp_c1;
		//printf("v1,2 = %g,%g\n",s1.velocity.x,s1.velocity.y);
	}
	if (!immobile2) {
		//printf("delta-c2: %g\n",new_c2-c2);
		//printf("v2,1 = %g,%g\n",s2.velocity.x,s2.velocity.y);
		//printf("para2,1 = %g,%g[%g]\n",(dir*c2).x,(dir*c2).y,c2);
		//printf("para2,2 = %g,%g [%g <- %g]\n",para_c2.x,para_c2.y,c2+delta_c2,new_c2);
		perp_c2 = v2 - (data.dir * c2);
		s2.velocity = para_c2 + perp_c2;
		//printf("v2,2 = %g,%g\n",s2.velocity.x,s2.velocity.y);
	}
}

/* requires s non-null */
/* reqires that s is on the solid referred to in data */
bool is_still_on(solid::onbase_data& data, solid *s) {
	assert(s);

	/* Advance s's position by a constant along the onbase normal
	 * and check for collision. s is still on iff there is a collision.
	 */
	real_t old_x = s->x, old_y = s->y;
	vector2d_t& normal = data.normal;
	vector2d_t dp = (-normal) * NB_ONCHECK_P_DIST;

	vector2d_t p_vec;
	p_vec.x = s->x;
	p_vec.y = s->y;
	p_vec += dp;
	s->x = p_vec.x;
	s->y = p_vec.y;

	bool ret = solids_collide(*s, *data.onbase, NULL);
	if (!ret)
		printf("adjust pos by %g,%g and the object does not collide with onbase\n", dp.x,dp.y);

	s->x = old_x;
	s->y = old_y;
	return ret;
}

//TODO move these initializers
solid *new_ball(solid *buf, bool immobile, real_t x, real_t y, real_t r, SDL_Surface *img, real_t e) {
	assert(r >= 0.0);
	solid *ret = buf;
	if (!ret) {
		ret = new solid(x, y, e, immobile);
	}
	else {
		ret->x = x;
		ret->y = y;
		ret->elasticity = e;
		ret->immobile = immobile;
	}
	ret->solid_type = NB_SLD_BALL;
	ret->solid_data->ball_data.r = r;

	ret->visible_type = NB_SLD_BALL;
	ret->visible_data = img;
	return ret;
}

solid *new_poly(solid *buf, bool immobile, real_t *points, unsigned num_pts, real_t x, real_t y,
		real_t e, unsigned color) {
	assert(points);
	assert(num_pts >= 3);
	solid *ret = buf;
	if (!ret) {
		ret = new solid(x, y, e, immobile);
	}
	else {
		ret->x = x;
		ret->y = y;
		ret->elasticity = e;
		ret->immobile = immobile;
	}
	ret->solid_type = NB_SLD_POLY;

#define X_COORD(i) (points[(2 * (i))])
#define Y_COORD(i) (points[(2 * (i)) + 1])
	segment **segs = new segment *[num_pts];
	for (int i = 0; i < num_pts; i++) {
		int next_i = (i + 1) % num_pts;
		segs[i] = new segment(X_COORD(i), Y_COORD(i),
				X_COORD(next_i), Y_COORD(next_i));
	}

	ret->solid_data->poly_data.segs = segs;
	ret->solid_data->poly_data.num_segs = num_pts;

	ret->visible_type = NB_SLD_POLY;
	ret->visible_data = (void *) color;

	return ret;
}

solid *new_poly(solid *buf, bool immobile, real_t x, real_t y, real_t e, int num_points, ...) {
	assert(num_points >= 3);
	solid *ret = buf;
	if (!ret) {
		ret = new solid(x, y, e, immobile);
	}
	else {
		ret->x = x;
		ret->y = y;
		ret->elasticity = e;
		ret->immobile = immobile;
	}
	ret->solid_type = NB_SLD_POLY;

	segment **segs = new segment *[num_points];
	va_list args;
	va_start(args, num_points);
	/* varargs create doubles :^/ */
	real_t prev_x = va_arg(args, double);
	real_t prev_y = va_arg(args, double);
	for (int i = 0; i < num_points - 1; i++) {
		real_t x = va_arg(args, double);
		real_t y = va_arg(args, double);
		segs[i] = new segment(prev_x, prev_y, x, y);

		prev_x = x;
		prev_y = y;
	}
	va_end(args);
	segs[num_points - 1] = new segment(prev_x, prev_y, segs[0]->x,
			segs[0]->y);
	ret->solid_data->poly_data.segs = segs;
	ret->solid_data->poly_data.num_segs = num_points;

	return ret;
}

//TODO get source point, to find NEAREST intersection
bool seg_poly_intersection(solid *p, segment *s, real_t& cx, real_t& cy) {
	assert(p->get_solid_type() == NB_SLD_POLY);

	real_t ax1, ay1, adx, ady;
	ax1 = s->x;
	ay1 = s->y;
	adx = s->dir.x;
	ady = s->dir.y;
	int i;
	for (i = 0; i < p->solid_data->poly_data.num_segs; i++) {
		segment *pseg = p->solid_data->poly_data.segs[i];
		real_t bx1, by1, bdx, bdy;
		bx1 = pseg->x + p->x;
		by1 = pseg->y + p->y;
		bdx = pseg->dir.x;
		bdy = pseg->dir.y;

		real_t den = adx * bdy - bdx * ady;
		if (den == 0) {
			printf("WARNING!! avoiding dbz in seg-intersection!!\n");
			continue;
		}

		real_t at = (adx * (ay1 - by1) - ady * (ax1 - bx1)) / den;
		real_t bt = (bdx * (ay1 - by1) - bdy * (ax1 - bx1)) / den;

		if (between_ord(0, at, 1) && between_ord(0, bt, 1)) {
			cx = ax1 + bt * adx;
			cy = ay1 + bt * ady;
			return true;
		}
	}
	return false;
}
